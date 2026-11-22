#include "Arduino.h"
float t, dt;
#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "Preferences.h"
void print(const char *fmt, ...) {}
void doCommand(String, bool) {}
bool setParameter(const char *, const float) { return true; }
void descend() {}
Quaternion attitude, attitudeTarget;
Vector rates, acc, ratesTarget, ratesExtra, torqueTarget;
float motors[4], thrustTarget, batteryVoltage, controlRoll, controlPitch, controlYaw, controlThrottle, controlMode, controlTime;
bool armed, landed, imuOK;
int mode, wifiMode;
Preferences storage;
PID rollRatePID(1,0,0), pitchRatePID(1,0,0), yawRatePID(1,0,0), rollPID(1,0,0), pitchPID(1,0,0), yawPID(1,0,0);
#include "zenoh_link.ino"
int main() { setupZenohLink(); zenohLinkApply(); zenohLinkSnapshot(); zenohLinkLog("x"); String a("zn"), b, c; zenohLinkCommand(a, b, c); }
