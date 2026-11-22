// E1 wire protocol v1 (C mirror of e1_core/e1_core/protocol.py) - little-endian, packed.
// Key space: e1/<id>/{state,pose,log,alive}, e1/<id>/cmd/{attitude,manual,motors,arm,mode,param,console}, e1/all/cmd/*
#pragma once
#include <stdint.h>

#define E1_PROTOCOL_VERSION 1
enum E1MsgType : uint8_t {
	E1_STATE = 1, E1_POSE = 2, E1_CMD_ATTITUDE = 3, E1_CMD_MANUAL = 4,
	E1_CMD_MOTORS = 5, E1_CMD_ARM = 6, E1_CMD_MODE = 7, E1_CMD_PARAM = 8
};
// STATE flags
#define E1_F_LANDED      (1 << 0)
#define E1_F_LOW_BATTERY (1 << 1)
#define E1_F_CMD_TIMEOUT (1 << 2)
#define E1_F_FAILSAFE    (1 << 3)
#define E1_F_IMU_ERROR   (1 << 4)
#define E1_F_POSE_VALID  (1 << 5)
// CMD_ATTITUDE type_mask
#define E1_IGNORE_RATES    1
#define E1_IGNORE_ATTITUDE 2

#pragma pack(push, 1)
struct E1Header { uint8_t version; uint8_t type; uint16_t seq; uint32_t t_ms; };
struct E1State {                      // 76 bytes
	E1Header h;
	float q[4];                       // w x y z, body FLU -> world
	float rates[3];                   // rad/s body
	float acc[3];                     // m/s^2 body
	float motors[4];                  // duty, RL RR FR FL
	float thrust;                     // collective thrust target (duty)
	float battery_v;
	uint8_t mode, armed; uint16_t flags;
};
struct E1Pose { E1Header h; float pos[3]; float q[4]; float vel[3]; };
struct E1CmdAttitude { E1Header h; float q[4]; float rates[3]; float thrust; uint8_t type_mask; };
struct E1CmdManual { E1Header h; float roll, pitch, yaw, throttle, mode; };
struct E1CmdMotors { E1Header h; float duty[4]; };
struct E1CmdArm { E1Header h; uint8_t arm; };
struct E1CmdMode { E1Header h; uint8_t mode; };
struct E1CmdParam { E1Header h; char name[16]; float value; };
#pragma pack(pop)

static_assert(sizeof(E1State) == 76, "E1State size must match protocol.py");
static_assert(sizeof(E1Pose) == 48, "E1Pose size");
static_assert(sizeof(E1CmdAttitude) == 41, "E1CmdAttitude size");
static_assert(sizeof(E1CmdParam) == 28, "E1CmdParam size");
