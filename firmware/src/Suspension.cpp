// =============================================================================
//  Suspension.cpp - active front suspension. See Suspension.h.
// =============================================================================
#include "Suspension.h"
#include <Preferences.h>
#include "ServoOut.h"
#include "Tilt.h"

SuspensionClass Susp;

static const uint8_t SLOT_L = 5, SLOT_R = 6;
static inline float clampf_s(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

void SuspensionClass::begin() {
    Preferences p;
    if (p.begin("susp", true)) {
        if (p.getBytesLength("p") == sizeof(P)) p.getBytes("p", &P, sizeof(P));
        p.end();
    }
}
void SuspensionClass::save() {
    Preferences p; p.begin("susp", false); p.putBytes("p", &P, sizeof(P)); p.end();
}
void SuspensionClass::test(int mode) {
    _test = mode; _testUntil = millis() + 2500;      // holds 2.5 s, then back to live
}

void SuspensionClass::update(float dt, float steer, float mph, float simAccG, float engTq01) {
    if (!P.on) {                                      // give the headers back
        Servos.claim(SLOT_L, -1, 1500); Servos.claim(SLOT_R, -1, 1500);
        _why = ""; _r = _rv = _p = _pv = 0; _usL = _usR = 0; return;
    }
    if (dt <= 0 || dt > 0.1f) dt = 0.02f;

    // ---- what the body FEELS (g): sideways and fore/aft ----
    float lat, fwd;
    if (Tilt.present()) { lat = Tilt.accLat(); fwd = Tilt.accFwd(); }
    else {
        // no gyro: the sim's own acceleration, and a turn ~ steer x speed^2
        // (full lock at 25 mph ~ 0.4 g, which is where a loaded tractor leans hard)
        float v = mph / 25.0f;
        lat = -steer * v * v * 0.4f;                  // turning LEFT (steer < 0) pushes the body RIGHT (+)
        fwd = simAccG;
    }
    lat = clampf_s(lat, -1.0f, 1.0f); fwd = clampf_s(fwd, -1.0f, 1.0f);
    // torque twist, as g-equivalent roll so it rides the same spring + damper
    lat += P.twist / fmaxf(P.roll, 50.0f) * clampf_s(engTq01, 0.0f, 3.0f);

    // test positions (setting up the linkage on the page)
    if (_test && millis() < _testUntil) {
        lat = (_test == 1) ? -0.4f : (_test == 2) ? 0.4f : 0.0f;
        fwd = (_test == 3) ? -0.4f : (_test == 4) ? 0.4f : 0.0f;
    } else _test = 0;

    // ---- the body: spring + damper towards what it feels ----
    // x'' = w^2 (target - x) - 2 z w x'   (sub-stepped: the loop can be slow)
    const float w = 6.2832f * clampf_s(P.freq, 0.3f, 6.0f), z = clampf_s(P.damp, 0.05f, 1.5f);
    int steps = (int)(dt / 0.004f) + 1; float h = dt / steps;
    for (int i = 0; i < steps; i++) {
        _rv += (w * w * (lat - _r) - 2.0f * z * w * _rv) * h; _r += _rv * h;
        _pv += (w * w * (fwd - _p) - 2.0f * z * w * _pv) * h; _p += _pv * h;
    }

    // ---- to the shock mounts ----
    // roll: body pushed RIGHT -> right corner down, left corner up
    // pitch: slowing (fwd < 0) -> nose DOWN on both; pulling (fwd > 0) -> nose up
    // "down" = the mount moves to compress the spring; inv flips a mirrored servo
    float rollUs  = _r * P.roll;
    float pitchUs = _p * P.pitch;
    float dL = clampf_s(-rollUs + pitchUs, -P.travel, P.travel);
    float dR = clampf_s( rollUs + pitchUs, -P.travel, P.travel);
    if (P.invL) dL = -dL;
    if (P.invR) dR = -dR;
    int usL = (int)clampf_s(P.ctrL + dL, 900, 2100);
    int usR = (int)clampf_s(P.ctrR + dR, 900, 2100);
    const char *a = Servos.claim(SLOT_L, P.chL, usL);
    const char *b = Servos.claim(SLOT_R, P.chR, usR);
    _why = a[0] ? a : b;
    if (!_why[0]) {
        // claim() quietly refuses a header the truck reads - say which
        const char *lk = Servos.lockedBy(P.chL); if (!lk[0]) lk = Servos.lockedBy(P.chR);
        if (lk[0]) _why = lk;
        else if (Servos.owner(P.chL) != SLOT_L || Servos.owner(P.chR) != SLOT_R) _why = "header in use";
    }
    _usL = _why[0] ? 0 : usL; _usR = _why[0] ? 0 : usR;
    _lastMph = mph;
}

String SuspensionClass::json() const {
    char b[320];
    snprintf(b, sizeof(b),
        "{\"on\":%d,\"chL\":%d,\"chR\":%d,\"ctrL\":%d,\"ctrR\":%d,\"invL\":%d,\"invR\":%d,\"travel\":%.0f,"
        "\"roll\":%.0f,\"pitch\":%.0f,\"twist\":%.0f,\"freq\":%.2f,\"damp\":%.2f,\"why\":\"%s\",\"r\":%.3f,\"p\":%.3f,\"gyro\":%d}",
        P.on, P.chL, P.chR, P.ctrL, P.ctrR, P.invL, P.invR, P.travel, P.roll, P.pitch, P.twist, P.freq, P.damp,
        _why, _r, _p, Tilt.present() ? 1 : 0);
    return String(b);
}
