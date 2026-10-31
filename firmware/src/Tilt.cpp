// =============================================================================
//  Tilt.cpp - GY-521 / MPU6050 driver + gravity fusion. See Tilt.h.
// =============================================================================
#include "Tilt.h"
#include <Wire.h>
#include <Preferences.h>

TiltSensor Tilt;

static const uint8_t R_SMPLRT = 0x19, R_CONFIG = 0x1A, R_GYRO = 0x1B, R_ACCEL = 0x1C,
                     R_DATA = 0x3B, R_PWR1 = 0x6B, R_WHOAMI = 0x75;
static const float   ACC_LSB  = 1.0f / 8192.0f;              // +/-4 g
static const float   GYRO_LSB = (1.0f / 65.5f) * 0.0174533f; // +/-500 dps -> rad/s

static bool wr(uint8_t a, uint8_t r, uint8_t v) {
    Wire.beginTransmission(a); Wire.write(r); Wire.write(v);
    return Wire.endTransmission() == 0;
}
static inline void norm3(float v[3]) {
    float m = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (m > 1e-6f) { v[0] /= m; v[1] /= m; v[2] /= m; }
}
static inline float dot3(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static inline void cross3(const float a[3], const float b[3], float o[3]) {
    o[0] = a[1] * b[2] - a[2] * b[1]; o[1] = a[2] * b[0] - a[0] * b[2]; o[2] = a[0] * b[1] - a[1] * b[0];
}

bool TiltSensor::read(float a[3], float w[3]) {
    Wire.beginTransmission(addr); Wire.write(R_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr, 14) != 14) return false;
    int16_t v[7];
    for (int i = 0; i < 7; i++) { uint8_t hi = Wire.read(), lo = Wire.read(); v[i] = (int16_t)((hi << 8) | lo); }
    for (int i = 0; i < 3; i++) { a[i] = v[i] * ACC_LSB; w[i] = v[4 + i] * GYRO_LSB - _bias[i]; }
    return true;
}

bool TiltSensor::begin() {
    // Wire is already up on PIN_SDA/PIN_SCL (main) - never Wire.begin() here.
    _ok = false;
    for (uint8_t a : {(uint8_t)0x68, (uint8_t)0x69}) {
        Wire.beginTransmission(a); Wire.write(R_WHOAMI);
        if (Wire.endTransmission(false) != 0) continue;
        if (Wire.requestFrom((int)a, 1) != 1) continue;
        uint8_t who = Wire.read();
        if (who == 0 || who == 0xFF) continue;    // 0x68 genuine; clones answer 0x70/0x72/0x98
        addr = a; _ok = true; break;
    }
    if (!_ok) return false;
    wr(addr, R_PWR1, 0x01);       // wake, clock off the X gyro PLL
    delay(50);
    wr(addr, R_SMPLRT, 9);        // 1 kHz / 10 = 100 Hz
    wr(addr, R_CONFIG, 3);        // DLPF ~44 Hz: engine/ESC vibration out
    wr(addr, R_GYRO, 0x08);       // +/-500 dps
    wr(addr, R_ACCEL, 0x08);      // +/-4 g
    delay(20);
    load();
    // gyro bias: the truck is sitting still at power-up
    float sum[3] = {0, 0, 0}, acc[3] = {0, 0, 0}, a[3], w[3];
    int n = 0;
    for (int i = 0; i < 100; i++) {
        if (read(a, w)) { for (int k = 0; k < 3; k++) { sum[k] += w[k]; acc[k] += a[k]; } n++; }
        delay(5);
    }
    if (n > 50) {
        for (int k = 0; k < 3; k++) { _bias[k] = sum[k] / n; _g[k] = acc[k] / n; }
        norm3(_g);
    }
    _lastUs = micros();
    return true;
}

void TiltSensor::update() {
    if (!_ok) return;
    uint32_t now = micros();
    float dt = (now - _lastUs) * 1e-6f;
    if (dt < 0.010f) return;      // ~100 Hz
    _lastUs = now;
    if (dt > 0.1f) dt = 0.1f;
    float a[3], w[3];
    if (!read(a, w)) return;

    // gravity in the board frame turns opposite to the body: dg/dt = -w x g
    float c[3]; cross3(w, _g, c);
    for (int k = 0; k < 3; k++) _g[k] -= c[k] * dt;
    // the accelerometer only votes when the truck is not accelerating hard
    float am = sqrtf(dot3(a, a));
    if (am > 0.85f && am < 1.15f) {
        float k = 0.02f;          // ~0.5 s to settle back onto the accelerometer
        for (int i = 0; i < 3; i++) _g[i] += (a[i] / am - _g[i]) * k;
    }
    norm3(_g);

    // the truck's axes: forward = the chosen board axis, flattened against level
    float f[3] = {0, 0, 0};
    f[axis < 2 ? 0 : 1] = (axis == 0 || axis == 2) ? 1.0f : -1.0f;
    float u0[3] = {up[0], up[1], up[2]}; norm3(u0);
    float d = dot3(f, u0);
    for (int k = 0; k < 3; k++) f[k] -= u0[k] * d;
    norm3(f);
    float r[3]; cross3(f, u0, r);   // right

    // _g points UP (the accelerometer reads the reaction to gravity)
    _pitch = asinf(fmaxf(-1.0f, fminf(1.0f, dot3(_g, f)))) * 57.2958f;
    _roll  = -asinf(fmaxf(-1.0f, fminf(1.0f, dot3(_g, r)))) * 57.2958f;
    _tilt  = acosf(fmaxf(-1.0f, fminf(1.0f, dot3(_g, u0)))) * 57.2958f;
    // what is left of the accelerometer after gravity = the truck's own motion
    float lin[3] = {a[0] - _g[0], a[1] - _g[1], a[2] - _g[2]};
    float kA = fminf(1.0f, dt / 0.05f);
    _aF += (dot3(lin, f) - _aF) * kA;
    _aL += (dot3(lin, r) - _aL) * kA;
    float gTgt = sinf(_pitch * 0.0174533f);
    _gradeF += (gTgt - _gradeF) * fminf(1.0f, dt * 3.0f);   // ~0.3 s, no twitching on bumps

    // rollover: past 55 deg for half a second; back under 35 deg clears it
    uint32_t ms = millis();
    if (_tilt > 55.0f) { if (!_overMs) _overMs = ms; if (ms - _overMs > 500) _over = true; }
    else { _overMs = 0; if (_tilt < 35.0f) _over = false; }
}

float TiltSensor::grade() const {
    if (!_ok) return 0.0f;
    float g = _gradeF * hill;
    return g > 0.35f ? 0.35f : (g < -0.35f ? -0.35f : g);
}

void TiltSensor::learnLevel() {
    if (!_ok) return;
    for (int k = 0; k < 3; k++) up[k] = _g[k];
    save();
}

void TiltSensor::load() {
    Preferences p;
    if (!p.begin("tilt", true)) return;
    axis = p.getUChar("axis", axis);
    hill = p.getFloat("hill", hill);
    if (p.getBytesLength("up") == sizeof(up)) p.getBytes("up", up, sizeof(up));
    p.end();
    if (axis > 3) axis = 0;
}
void TiltSensor::save() {
    Preferences p; p.begin("tilt", false);
    p.putUChar("axis", axis); p.putFloat("hill", hill); p.putBytes("up", up, sizeof(up));
    p.end();
}
void TiltSensor::print() {
    if (!_ok) { Serial.println("[tilt] no GY-521 on the accessory header (0x68/0x69)"); return; }
    static const char *AX[4] = {"+X", "-X", "+Y", "-Y"};
    Serial.printf("[tilt] @0x%02X  pitch %+.1f  roll %+.1f  tilt %.1f  grade %+.3f  hill %.2f  front %s  %s\n",
                  addr, _pitch, _roll, _tilt, grade(), hill, AX[axis], _over ? "ROLLED OVER" : "upright");
}
