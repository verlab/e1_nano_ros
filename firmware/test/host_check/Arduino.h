#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <string>
#define PI 3.14159265358979f
template<class T, class L, class H> T constrain(T x, L lo, H hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float radians(float d) { return d * PI / 180; }
inline float degrees(float r) { return r * 180 / PI; }
using std::isfinite; using std::isnan;
class Print { public: template<class T> size_t print(T, int = 0) { return 0; } }; class Printable { public: virtual size_t printTo(Print &p) const = 0; virtual ~Printable() {} };
class String : public std::string {
public:
	String(const char *s = "") : std::string(s) {}
	String(const std::string &s) : std::string(s) {}
	int toInt() const { return atoi(c_str()); }
	float toFloat() const { return atof(c_str()); }
	bool isEmpty() const { return empty(); }
	void toLowerCase() {}
};
struct IPAddress { String toString() const { return "1.2.3.4"; } };
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (void)(x)
#define portEXIT_CRITICAL(x) (void)(x)
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
inline void vTaskDelay(TickType_t) {}
inline void vTaskDelayUntil(TickType_t *, TickType_t) {}
inline TickType_t xTaskGetTickCount() { return 0; }
inline int xTaskCreatePinnedToCore(void (*)(void *), const char *, int, void *, int, void *, int) { return 1; }
