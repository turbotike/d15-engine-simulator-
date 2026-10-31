// =============================================================================
//  Hydraulics.cpp - the fluid side of the truck (port of the Teensy version).
// =============================================================================
#include "Hydraulics.h"
#include <Preferences.h>

Hydraulics Hyd;

static const float AMBIENT = 75.0f;

void Hydraulics::begin() {
    // Cylinders stay where you left them: a real crane does not spring back to
    // centre when you switch off, and a servo that jumped on every power-up
    // would slam the boom.
    Preferences p;
    if (p.begin("rclc_hyd", true)) {
        if (p.getBytesLength("pos") == sizeof(_pos)) p.getBytes("pos", _pos, sizeof(_pos));
        p.end();
    }
    for (float& x : _pos) x = constrain(x, 0.0f, 1.0f);
}

void Hydraulics::savePos() {
    if (!_moved) return;
    Preferences p;
    if (p.begin("rclc_hyd", false)) { p.putBytes("pos", _pos, sizeof(_pos)); p.end(); }
    _moved = false;
}

void Hydraulics::setOn(bool on) {
    if (on == _on) return;
    _on = on;
    Serial.printf("[hyd] work mode %s\n", on ? "ON (PTO engaged)" : "off");
    if (!on) { _press = 0; _ptoLoad = 0; _demand = 0; savePos(); }
}

void Hydraulics::tick(float dt, float rpm, bool engineRunning, const float cmdIn[4]) {
    int n = nFn();
    if (!_on || !engineRunning || n == 0) {
        _press  += (0 - _press) * fminf(1.0f, dt * 3.0f);
        _temp   += (AMBIENT - _temp) * fminf(1.0f, dt * 0.004f);
        _ptoLoad = 0; _demand = 0; _relief = 0;
        return;
    }

    // pump flow follows rpm: nothing below ~550, full at ~2000
    float flow = constrain((rpm - 550.0f) / 1450.0f, 0.0f, 1.0f);

    float cmd[4], demand = 0, relief = 0;
    for (int i = 0; i < 4; i++) {
        cmd[i] = (i < n) ? constrain(cmdIn[i], -1.0f, 1.0f) : 0.0f;
        if (fabsf(cmd[i]) < 0.05f) { cmd[i] = 0; continue; }
        demand += fabsf(cmd[i]);
        // end stop + still commanding into it = dumping over the relief valve
        if ((_pos[i] >= 1.0f && cmd[i] > 0) || (_pos[i] <= 0.0f && cmd[i] < 0)) relief += fabsf(cmd[i]);
    }
    // flow sharing: many functions at once = everything slows down
    float share = (demand > 0.01f) ? fminf(1.0f, (flow * 1.6f) / demand) : 1.0f;

    for (int i = 0; i < n; i++) {
        float c = cmd[i];
        if (c == 0) continue;
        // raising (positive) fights gravity: slower than lowering
        float rate = (c > 0 ? 0.22f : 0.34f) * ((_mode == 2) ? 0.6f : 1.0f);   // dump box = one big slow ram
        // LOWERING IS GRAVITY: the load pushes the oil back through the valve,
        // so it still comes down at idle - it does not wait for pump flow.
        float f = (c > 0) ? flow * share : fmaxf(flow * share, 0.5f);
        _pos[i] = constrain(_pos[i] + c * f * rate * dt, 0.0f, 1.0f);
        _moved = true;
    }

    // pressure: standby ~250, working with load, pinned at relief when stalled
    float tgt = 250.0f + 2100.0f * fminf(1.0f, demand) * flow + (relief > 0 ? 650.0f : 0.0f);
    _press += (tgt - _press) * fminf(1.0f, dt * 5.0f);

    // fluid temp: relief dumping is the heater; honest work warms it slowly
    float heat = relief * 9.0f + demand * flow * 1.4f;
    _temp += heat * dt / 6.0f;
    _temp += (AMBIENT - _temp) * fminf(1.0f, dt * 0.006f);
    if (_temp > 240.0f) _temp = 240.0f;

    // the engine feels the pump: idle bump + drag under work
    _ptoLoad = 0.12f + 0.55f * fminf(1.0f, demand) * flow + (relief > 0 ? 0.15f : 0.0f);
    _demand  = fminf(1.0f, demand);
    _relief  = fminf(1.0f, relief);

    // save positions a while after the last move, not on every frame
    static uint32_t lastMoveMs = 0;
    if (demand > 0) lastMoveMs = millis();
    else if (_moved && millis() - lastMoveMs > 3000) savePos();
}
