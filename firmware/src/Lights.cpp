// =============================================================================
//  Lights.cpp - PCA9685 driver + light behaviours for the S3 board
// =============================================================================
#include "Lights.h"
#include "Config.h"
#include <Wire.h>

Lights Lite;

static const uint8_t MODE1 = 0x00, MODE2 = 0x01, PRESCALE = 0xFE, LED0_ON_L = 0x06;

void Lights::wreg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool Lights::begin(uint8_t i2cAddr, float pwmHz) {
    _addr = i2cAddr;

    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) { _ok = false; return false; }

    int pre = (int)lroundf(25000000.0f / (4096.0f * pwmHz)) - 1;
    if (pre < 3)   pre = 3;
    if (pre > 255) pre = 255;

    wreg(MODE1, 0x10);                  // sleep - PRESCALE is writable only asleep
    wreg(PRESCALE, (uint8_t)pre);
    wreg(MODE1, 0x00);                  // wake
    delayMicroseconds(500);
    wreg(MODE1, 0xA0);                  // restart + auto-increment
    // Totem-pole outputs. The PCA runs off 3V3 and has to SOURCE current into
    // each ULN2803's 2.7k base resistor; in open-drain mode it can only sink,
    // and every light would stay dark with no error anywhere.
    wreg(MODE2, 0x04);

    for (int i = 0; i < LT_COUNT; i++) { setDuty((uint8_t)i, 0); _last[i] = 0; }
    _haveLast = true;
    _ok = true;
    return true;
}

void Lights::setDuty(uint8_t ch, uint16_t duty12) {
    if (ch >= LT_COUNT) return;                  // keeps us off channel 15
    if (_haveLast && _last[ch] == duty12) return; // don't re-send an unchanged channel
    _last[ch] = duty12;

    uint16_t off = duty12 & 0x0FFF;
    bool full = (duty12 >= 4095);
    Wire.beginTransmission(_addr);
    Wire.write(LED0_ON_L + 4 * ch);
    Wire.write(0x00);                            // ON_L
    Wire.write(full ? 0x10 : 0x00);              // ON_H bit4 = full on
    Wire.write(full ? 0x00 : (off & 0xFF));      // OFF_L
    Wire.write(full ? 0x00 : ((off >> 8) & 0x0F)); // OFF_H
    Wire.endTransmission();
}

void Lights::setF(uint8_t ch, float f) {
    if (f <= 0) { setDuty(ch, 0); return; }
    if (f >= 1) { setDuty(ch, 4095); return; }
    setDuty(ch, (uint16_t)(f * 4095.0f));
}

float Lights::auxLevel(AuxFunc fn, const LightState& s, bool blink,
                       bool bflash2, bool markers, float trim) const {
    switch (fn) {
        case AUX_HIGHBEAM: return s.highBeam ? trim : 0.0f;
        case AUX_FOG:      return s.fog      ? trim : 0.0f;
        case AUX_ROOF:     return s.roof     ? markLevel * trim : 0.0f;
        case AUX_BRAKE:    return s.braking  ? br3rd : 0.0f;   // its own slider: 3rd Brake Light
        case AUX_BEACON2:  return (s.beacon && bflash2) ? trim : 0.0f;
        case AUX_CABIN:    return s.cabin    ? trim : 0.0f;
        case AUX_OFF:
        default:           return 0.0f;
    }
    (void)blink; (void)markers;
}

void Lights::update(const LightState& s, uint32_t nowMs) {
    // 50 Hz is plenty for lights and leaves the bus quiet for the accessory
    // header, which shares it.
    static uint32_t lastMs = 0;
    if (nowMs - lastMs < 20) return;
    lastMs = nowMs;

    // !_ok FIRST. With the test path ahead of it, a board whose PCA9685 was not
    // found still got hammered with failing I2C writes every 20 ms forever the
    // moment anyone hit the web light test - each one a NACK plus a bus timeout,
    // slowing loop() permanently with no symptom that points at the cause.
    if (!_ok) return;
    if (_testOn) {
        for (int i = 0; i < LT_COUNT; i++) setF((uint8_t)i, _test[i] / 255.0f);
        return;
    }

    bool blink = (nowMs % 800) < 400;                 // ~1.25 Hz, as the truck runs
    uint32_t bp = nowMs % 1000;
    bool bflash1 = (bp < 60) || (bp > 120 && bp < 180);
    bool bflash2 = (bp > 500 && bp < 560) || (bp > 620 && bp < 680);

    bool markers = s.park || s.head || s.running;

    // Hazard is both indicators together. The Teensy board had a dedicated
    // hazard line; this one does not, and it does not need one - that IS how a
    // truck wires it.
    bool wantL = (s.indicate < 0) || s.hazard;
    bool wantR = (s.indicate > 0) || s.hazard;

    // If an AUX is carrying the brake lamp, TAIL stops doubling as one and just
    // sits at marker level.
    bool auxHasBrake = (aux1 == AUX_BRAKE) || (aux2 == AUX_BRAKE);

    setF(LT_HEAD,  s.head ? headLevel * brHead : 0.0f);
    setF(LT_TURNL, (wantL && blink) ? brSig : 0.0f);
    setF(LT_TURNR, (wantR && blink) ? brSig : 0.0f);
    setF(LT_CAB,   s.cabin ? brCabin : 0.0f);
    setF(LT_RUN,   markers ? markLevel * brRun : 0.0f);
    setF(LT_TAIL,  (s.braking && !auxHasBrake) ? brBrake
                                               : (markers ? tailLevel * brTail : 0.0f));
    setF(LT_REV,   s.reverse ? brRev : 0.0f);
    setF(LT_BCON,  (s.beacon && bflash1) ? brBeacon : 0.0f);
    setF(LT_AUX1,  auxLevel(aux1, s, blink, bflash2, markers, brAux1));
    setF(LT_AUX2,  auxLevel(aux2, s, blink, bflash2, markers, brAux2));

    // REAR INDICATORS. Normal: blink with the fronts. American combined rear: the
    // rear lamp is a tail light (marker level), full on the brake, and only the
    // side being indicated blinks - so braking while turning left blinks the left
    // rear and holds the right rear bright, as a US trailer does.
    float tailLv = markers ? tailLevel * brTail : 0.0f;
    float stopLv = (s.braking && !auxHasBrake) ? brBrake : tailLv;
    float rl, rr;
    if (s.combinedRear) {
        rl = wantL ? (blink ? brSig : 0.0f) : stopLv;
        rr = wantR ? (blink ? brSig : 0.0f) : stopLv;
    } else {
        rl = (wantL && blink) ? brSig : 0.0f;
        rr = (wantR && blink) ? brSig : 0.0f;
    }
    setF(LT_TURNL_R, rl);
    setF(LT_TURNR_R, rr);
}
