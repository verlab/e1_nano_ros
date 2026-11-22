#pragma once
#include "Arduino.h"
struct Preferences {
	bool getBool(const char *, bool d) { return d; } int getInt(const char *, int d) { return d; }
	float getFloat(const char *, float d) { return d; } String getString(const char *, const char *d) { return d; }
	void putBool(const char *, bool) {} void putInt(const char *, int) {} void putFloat(const char *, float) {} void putString(const char *, const String &) {}
};
