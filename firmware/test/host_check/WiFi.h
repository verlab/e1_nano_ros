#pragma once
#include "Arduino.h"
#define WL_CONNECTED 3
struct WiFiClass { int status() { return WL_CONNECTED; } bool isConnected() { return true; } IPAddress localIP() { return {}; } };
static WiFiClass WiFi;
