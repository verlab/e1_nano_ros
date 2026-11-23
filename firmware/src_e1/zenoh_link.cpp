// E1 Zenoh link for the CF-Drone flight controller (ESP32-S3, zenoh-pico).
//
// - Runs Zenoh on core 0 (FreeRTOS task), the flight loop keeps core 1: the control loop never
//   blocks on the network. Data is exchanged through a spinlock-protected mailbox.
// - Publishes  e1/<id>/state (50 Hz), e1/<id>/log (console output), liveliness e1/<id>/alive
// - Subscribes e1/<id>/cmd/** and e1/all/cmd/**  (attitude, manual, motors, arm, mode, param, console)
// - Failsafe: in AUTO, if no attitude/motor command arrives for ZN_CMD_TIMEOUT seconds the drone
//   runs CF-Drone's descend() (slow landing then disarm).
//
// Console commands (serial / web / Zenoh):
//   zn                         status
//   zn id <1..254>             drone id (key space e1/<id>)
//   zn connect tcp/IP:7447     Zenoh router locator
//   zn on | zn off             enable / disable the link (reboot to apply)
// WiFi must be in station mode:  sta <ssid> <password>   then   p WIFI_MODE 2   and reboot.
//
// SPDX-License-Identifier: GPL-3.0-only (links with CF-Drone, see firmware/README.md)

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <zenoh-pico.h>
#include "vector.h"
#include "quaternion.h"
extern float t, dt;   // CF-Drone time globals, used inline by pid.h
#include "pid.h"
#include "e1_protocol.h"
#include "e1_zenoh_link.h"

// CF-Drone functions used by the link (defined in the .ino files)
void print(const char *format, ...);
void doCommand(String str, bool echo);
bool setParameter(const char *name, const float value);
void descend();

#ifndef E1_ZENOH_DEFAULT_LOCATOR
#define E1_ZENOH_DEFAULT_LOCATOR "tcp/192.168.1.100:7447"
#endif
#ifndef E1_ZENOH_DEFAULT_ON
#define E1_ZENOH_DEFAULT_ON 1
#endif
#define E1_STATE_HZ 50
#define E1_LOG_BUF 1024

extern Quaternion attitude, attitudeTarget;
extern Vector rates, acc, ratesTarget, ratesExtra, torqueTarget;
extern float motors[4];
extern float thrustTarget, batteryVoltage, t, dt;
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode, controlTime;
extern bool armed, landed, imuOK;
extern int mode;
extern int wifiMode;
extern Preferences storage;
extern PID rollRatePID, pitchRatePID, yawRatePID, rollPID, pitchPID, yawPID;

// ----------------------------------------------------------------------------- shared state
static portMUX_TYPE znMux = portMUX_INITIALIZER_UNLOCKED;
static struct {
	bool hasAtt, hasMan, hasMot, hasArm, hasMode, hasParam, hasConsole;
	E1CmdAttitude att; E1CmdManual man; E1CmdMotors mot; E1CmdArm arm; E1CmdMode mode; E1CmdParam param;
	char console[96];
} znIn;
static E1State znState;
static char znLog[E1_LOG_BUF];
static size_t znLogLen = 0;

static bool znEnabled = false, znConnected = false;
static int znId = 1;
static String znLocator;
static float znLastAutoCmd = -1;
static float znCmdTimeout = 0.5f;
static uint16_t znSeq = 0;
static uint32_t znRx = 0, znTx = 0, znErr = 0;

// ----------------------------------------------------------------------------- receive (zenoh read task)
static void znOnSample(z_loaned_sample_t *sample, void *arg) {
	z_view_string_t ks;
	z_keyexpr_as_view_string(z_sample_keyexpr(sample), &ks);
	const char *k = z_string_data(z_view_string_loan(&ks));
	size_t kl = z_string_len(z_view_string_loan(&ks));
	const char *cmd = NULL;  // suffix after ".../cmd/"
	for (size_t i = 0; i + 4 < kl; i++) {
		if (memcmp(k + i, "/cmd/", 5) == 0) { cmd = k + i + 5; kl = kl - (i + 5); break; }
	}
	if (!cmd) return;
	uint8_t buf[128];
	const z_loaned_bytes_t *pl = z_sample_payload(sample);
	size_t n = z_bytes_len(pl);
	if (n > sizeof(buf) - 1) { znErr++; return; }
	z_bytes_reader_t rd = z_bytes_get_reader(pl);
	z_bytes_reader_read(&rd, buf, n);
	buf[n] = 0;
	#define IS(s) (kl == strlen(s) && memcmp(cmd, s, kl) == 0)
	if (IS("console")) {
		portENTER_CRITICAL(&znMux);
		strlcpy(znIn.console, (const char *)buf, sizeof(znIn.console));
		znIn.hasConsole = true;
		portEXIT_CRITICAL(&znMux);
		znRx++;
		return;
	}
	if (n < sizeof(E1Header) || buf[0] != E1_PROTOCOL_VERSION) { znErr++; return; }
	uint8_t type = buf[1];
	portENTER_CRITICAL(&znMux);
	if (IS("attitude") && type == E1_CMD_ATTITUDE && n >= sizeof(E1CmdAttitude)) { memcpy(&znIn.att, buf, sizeof(znIn.att)); znIn.hasAtt = true; }
	else if (IS("manual") && type == E1_CMD_MANUAL && n >= sizeof(E1CmdManual)) { memcpy(&znIn.man, buf, sizeof(znIn.man)); znIn.hasMan = true; }
	else if (IS("motors") && type == E1_CMD_MOTORS && n >= sizeof(E1CmdMotors)) { memcpy(&znIn.mot, buf, sizeof(znIn.mot)); znIn.hasMot = true; }
	else if (IS("arm") && type == E1_CMD_ARM && n >= sizeof(E1CmdArm)) { memcpy(&znIn.arm, buf, sizeof(znIn.arm)); znIn.hasArm = true; }
	else if (IS("mode") && type == E1_CMD_MODE && n >= sizeof(E1CmdMode)) { memcpy(&znIn.mode, buf, sizeof(znIn.mode)); znIn.hasMode = true; }
	else if (IS("param") && type == E1_CMD_PARAM && n >= sizeof(E1CmdParam)) { memcpy(&znIn.param, buf, sizeof(znIn.param)); znIn.hasParam = true; }
	else znErr++;
	portEXIT_CRITICAL(&znMux);
	znRx++;
	#undef IS
}

// ----------------------------------------------------------------------------- zenoh task (core 0)
static void znPut(z_owned_session_t &s, const char *key, const uint8_t *data, size_t len) {
	z_view_keyexpr_t ke;
	if (z_view_keyexpr_from_str(&ke, key) < 0) return;
	z_owned_bytes_t p;
	z_bytes_copy_from_buf(&p, data, len);
	if (z_put(z_session_loan(&s), z_view_keyexpr_loan(&ke), z_bytes_move(&p), NULL) < 0) znErr++;
}

static void znTask(void *) {
	char key[48];
	for (;;) {
		while (WiFi.status() != WL_CONNECTED) vTaskDelay(pdMS_TO_TICKS(500));

		z_owned_config_t cfg;
		z_config_default(&cfg);
		zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_MODE_KEY, "client");
		zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_CONNECT_KEY, znLocator.c_str());
		z_owned_session_t s;
		if (z_open(&s, z_config_move(&cfg), NULL) < 0) {
			print("[zn] cannot reach router %s, retrying\n", znLocator.c_str());
			vTaskDelay(pdMS_TO_TICKS(2000));
			continue;
		}
		zp_start_read_task(z_session_loan_mut(&s), NULL);
		zp_start_lease_task(z_session_loan_mut(&s), NULL);

		z_owned_publisher_t pubState;
		z_view_keyexpr_t ke;
		snprintf(key, sizeof(key), "e1/%d/state", znId);
		z_view_keyexpr_from_str(&ke, key);
		z_declare_publisher(z_session_loan(&s), &pubState, z_view_keyexpr_loan(&ke), NULL);

		z_owned_subscriber_t subMine, subAll;
		z_owned_closure_sample_t cb1, cb2;
		z_closure_sample(&cb1, znOnSample, NULL, NULL);
		z_closure_sample(&cb2, znOnSample, NULL, NULL);
		snprintf(key, sizeof(key), "e1/%d/cmd/**", znId);
		z_view_keyexpr_from_str(&ke, key);
		z_declare_subscriber(z_session_loan(&s), &subMine, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cb1), NULL);
		z_view_keyexpr_from_str(&ke, "e1/all/cmd/**");
		z_declare_subscriber(z_session_loan(&s), &subAll, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cb2), NULL);

		z_owned_liveliness_token_t token;
		snprintf(key, sizeof(key), "e1/%d/alive", znId);
		z_view_keyexpr_from_str(&ke, key);
		z_liveliness_declare_token(z_session_loan(&s), &token, z_view_keyexpr_loan(&ke), NULL);

		znConnected = true;
		print("[zn] connected to %s as e1/%d\n", znLocator.c_str(), znId);

		char logKey[24];
		snprintf(logKey, sizeof(logKey), "e1/%d/log", znId);
		TickType_t last = xTaskGetTickCount();
		while (WiFi.status() == WL_CONNECTED && !z_session_is_closed(z_session_loan(&s))) {
			E1State st;
			char logCopy[E1_LOG_BUF];
			size_t logLen;
			portENTER_CRITICAL(&znMux);
			st = znState;
			logLen = znLogLen;
			memcpy(logCopy, znLog, logLen);
			znLogLen = 0;
			portEXIT_CRITICAL(&znMux);

			z_owned_bytes_t p;
			z_bytes_copy_from_buf(&p, (const uint8_t *)&st, sizeof(st));
			if (z_publisher_put(z_publisher_loan(&pubState), z_bytes_move(&p), NULL) < 0) znErr++; else znTx++;
			if (logLen) znPut(s, logKey, (const uint8_t *)logCopy, logLen);

			vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / E1_STATE_HZ));
		}
		znConnected = false;
		z_liveliness_token_drop(z_liveliness_token_move(&token));
		z_subscriber_drop(z_subscriber_move(&subMine));
		z_subscriber_drop(z_subscriber_move(&subAll));
		z_publisher_drop(z_publisher_move(&pubState));
		z_session_drop(z_session_move(&s));
		print("[zn] session lost, reconnecting\n");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

// ----------------------------------------------------------------------------- hooks called by CF-Drone
void setupZenohLink() {
	znEnabled = storage.getBool("ZN_EN", E1_ZENOH_DEFAULT_ON);
	znId = storage.getInt("ZN_ID", 1);
	znLocator = storage.getString("ZN_LOC", E1_ZENOH_DEFAULT_LOCATOR);
	znCmdTimeout = storage.getFloat("ZN_TIMEOUT", 0.5f);
	if (!znEnabled) { print("[zn] Zenoh link disabled (zn on)\n"); return; }
	if (wifiMode != 2) { print("[zn] Zenoh link needs WiFi station mode: sta <ssid> <pass>; p WIFI_MODE 2\n"); return; }
	xTaskCreatePinnedToCore(znTask, "zenoh", 8192, NULL, 1, NULL, 0);
	print("[zn] e1/%d -> %s\n", znId, znLocator.c_str());
}

static bool znCanArm() {
	if (!imuOK) { print("[zn] IMU error, arming refused\n"); return false; }
	if (batteryVoltage > 1.0f && batteryVoltage < 3.5f) { print("[zn] battery low (%.2f V), arming refused\n", batteryVoltage); return false; }
	return true;
}

static void znResetPIDs() {
	rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
	rollPID.reset(); pitchPID.reset(); yawPID.reset();
}

// Apply received commands. Called from loop() right after readRC(), before estimate()/control().
void zenohLinkApply() {
	if (!znEnabled) return;
	bool hasAtt, hasMan, hasMot, hasArm, hasMode, hasParam, hasConsole;
	E1CmdAttitude att; E1CmdManual man; E1CmdMotors mot; E1CmdArm arm; E1CmdMode md; E1CmdParam prm;
	char console[96];
	portENTER_CRITICAL(&znMux);
	hasAtt = znIn.hasAtt; hasMan = znIn.hasMan; hasMot = znIn.hasMot; hasArm = znIn.hasArm;
	hasMode = znIn.hasMode; hasParam = znIn.hasParam; hasConsole = znIn.hasConsole;
	att = znIn.att; man = znIn.man; mot = znIn.mot; arm = znIn.arm; md = znIn.mode; prm = znIn.param;
	memcpy(console, znIn.console, sizeof(console));
	znIn.hasAtt = znIn.hasMan = znIn.hasMot = znIn.hasArm = znIn.hasMode = znIn.hasParam = znIn.hasConsole = false;
	portEXIT_CRITICAL(&znMux);

	if (hasArm) {
		if (arm.arm) { if (znCanArm()) armed = true; }
		else { armed = false; }
	}
	if (hasMode && md.mode <= 4) mode = md.mode;
	if (hasMan) {   // same semantics as MAVLink MANUAL_CONTROL
		controlRoll = man.roll; controlPitch = man.pitch; controlYaw = man.yaw; controlThrottle = man.throttle;
		controlMode = NAN; controlTime = t;
	}
	if (hasAtt) {   // same semantics as MAVLink SET_ATTITUDE_TARGET (frame already FLU)
		if (mode != 4) { mode = 4; znResetPIDs(); }
		attitudeTarget.w = att.q[0]; attitudeTarget.x = att.q[1]; attitudeTarget.y = att.q[2]; attitudeTarget.z = att.q[3];
		if (att.type_mask & E1_IGNORE_ATTITUDE) attitudeTarget.invalidate();
		if (att.type_mask & E1_IGNORE_RATES) ratesExtra = Vector(0, 0, 0);
		else ratesExtra = Vector(att.rates[0], att.rates[1], att.rates[2]);
		thrustTarget = constrain(att.thrust, 0.0f, 1.0f);
		if (att.thrust > 0 && !armed) { if (znCanArm()) armed = true; }
		else if (att.thrust <= 0) armed = false;
		znLastAutoCmd = t;
	}
	if (hasMot) {   // same semantics as MAVLink SET_ACTUATOR_CONTROL_TARGET
		mode = 4;
		attitudeTarget.invalidate(); ratesTarget.invalidate(); torqueTarget.invalidate();
		for (int i = 0; i < 4; i++) motors[i] = constrain(mot.duty[i], 0.0f, 1.0f);
		armed = motors[0] > 0 || motors[1] > 0 || motors[2] > 0 || motors[3] > 0;
		znLastAutoCmd = t;
	}
	if (hasParam) {
		char name[17]; memcpy(name, prm.name, 16); name[16] = 0;
		if (setParameter(name, prm.value)) print("%s = %g\n", name, prm.value);
		else print("[zn] unknown parameter %s\n", name);
	}
	if (hasConsole) doCommand(String(console), true);

	// command-stream failsafe (only for flights driven over Zenoh)
	if (mode == 4 && armed && znLastAutoCmd >= 0 && t - znLastAutoCmd > znCmdTimeout) {
		static float lastWarn = -10;
		if (t - lastWarn > 2) { print("[zn] command timeout, descending\n"); lastWarn = t; }
		descend();
	}
}

// Snapshot telemetry. Called from loop() right after sendMotors().
void zenohLinkSnapshot() {
	if (!znEnabled) return;
	E1State s;
	s.h.version = E1_PROTOCOL_VERSION; s.h.type = E1_STATE; s.h.seq = ++znSeq; s.h.t_ms = (uint32_t)(t * 1000);
	s.q[0] = attitude.w; s.q[1] = attitude.x; s.q[2] = attitude.y; s.q[3] = attitude.z;
	s.rates[0] = rates.x; s.rates[1] = rates.y; s.rates[2] = rates.z;
	s.acc[0] = acc.x; s.acc[1] = acc.y; s.acc[2] = acc.z;
	for (int i = 0; i < 4; i++) s.motors[i] = motors[i];
	s.thrust = thrustTarget; s.battery_v = batteryVoltage;
	s.mode = (uint8_t)mode; s.armed = armed;
	s.flags = (landed ? E1_F_LANDED : 0) | (!imuOK ? E1_F_IMU_ERROR : 0) |
	          (batteryVoltage > 1.0f && batteryVoltage < 3.5f ? E1_F_LOW_BATTERY : 0) |
	          (mode == 4 && znLastAutoCmd >= 0 && t - znLastAutoCmd > znCmdTimeout ? (E1_F_CMD_TIMEOUT | E1_F_FAILSAFE) : 0);
	portENTER_CRITICAL(&znMux);
	znState = s;
	portEXIT_CRITICAL(&znMux);
}

// Mirror console output to e1/<id>/log. Called from print().
void zenohLinkLog(const char *text) {
	if (!znEnabled || !znConnected) return;
	size_t n = strlen(text);
	portENTER_CRITICAL(&znMux);
	if (znLogLen + n < E1_LOG_BUF) { memcpy(znLog + znLogLen, text, n); znLogLen += n; }
	portEXIT_CRITICAL(&znMux);
}

// Console commands. Returns true when handled.
bool zenohLinkCommand(String &command, String &arg0, String &arg1) {
	if (command != "zn") return false;
	if (arg0 == "") {
		print("zenoh: %s, %s, id=%d, router=%s, timeout=%.2fs, rx=%u tx=%u err=%u, wifi=%s\n",
		      znEnabled ? "enabled" : "disabled", znConnected ? "connected" : "not connected", znId, znLocator.c_str(),
		      znCmdTimeout, znRx, znTx, znErr, WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "down");
	} else if (arg0 == "id") {
		int id = arg1.toInt();
		if (id < 1 || id > 254) { print("id must be 1..254\n"); return true; }
		storage.putInt("ZN_ID", id); print("ZN_ID=%d (reboot to apply)\n", id);
	} else if (arg0 == "connect") {
		storage.putString("ZN_LOC", arg1); print("ZN_LOC=%s (reboot to apply)\n", arg1.c_str());
	} else if (arg0 == "timeout") {
		znCmdTimeout = constrain(arg1.toFloat(), 0.1f, 5.0f); storage.putFloat("ZN_TIMEOUT", znCmdTimeout);
		print("ZN_TIMEOUT=%.2f\n", znCmdTimeout);
	} else if (arg0 == "on" || arg0 == "off") {
		storage.putBool("ZN_EN", arg0 == "on"); print("Zenoh link %s (reboot to apply)\n", arg0.c_str());
	} else {
		print("usage: zn | zn id <n> | zn connect tcp/<ip>:7447 | zn timeout <s> | zn on|off\n");
	}
	return true;
}
