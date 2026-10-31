// =============================================================================
//  Lights.h - ten-channel light engine on the S3 board's single PCA9685
// =============================================================================
//  The Teensy board had fourteen light lines across two PCA9685s. This board has
//  ONE chip at 0x5F and exactly ten of its outputs reach a connector, so this is
//  a reduced engine, not a copy.
//
//  Two consequences worth knowing before you wire a truck:
//
//  1. THE CHANNEL ORDER IS MIRRORED. A ULN2803's input N drives output 19-N, so
//     PCA channel 0 comes out of U5 pin 18, channel 1 out of pin 17, and so on.
//     The numbers below were read off the rev3 board. Do not infer them from the
//     silkscreen or from the connector order.
//
//  2. THERE IS NO SEPARATE BRAKE CHANNEL. Real trucks share the filament, so by
//     default TAIL glows at marker level and goes full on the pedal. If you want
//     a dedicated brake lamp, assign it to AUX1 or AUX2 below.
//
//  Channel 15 is absent from this enum on purpose: its ULN2803 output is the
//  track that grounds the PCA9685.
// =============================================================================
#pragma once
#include <Arduino.h>

enum {
    LT_HEAD = 0,   // U5 pin 18
    LT_TURNL,      // U5 pin 17
    LT_TURNR,      // U5 pin 16
    LT_CAB,        // U5 pin 15
    LT_RUN,        // U5 pin 14   running / marker / side
    LT_TAIL,       // U5 pin 13   tail, and brake unless an AUX takes it
    LT_REV,        // U5 pin 12
    LT_BCON,       // U5 pin 11
    LT_AUX1,       // U6 pin 18
    LT_AUX2,       // U6 pin 17
    // Public V1: REAR indicators on their own channels (PCA LED10/LED11 -> U6 O3/O4
    // -> J8.3 rear left / J8.5 rear right). On a rev3 board these outputs reach no
    // connector and the rears stay ganged to the fronts, so driving them is harmless.
    LT_TURNL_R,    // U6 pin 16
    LT_TURNR_R,    // U6 pin 15
    LT_COUNT
};

// What the two spare channels do. The board calls them spare, so this is a
// choice rather than a fixed mapping.
enum AuxFunc : uint8_t {
    AUX_OFF = 0,
    AUX_HIGHBEAM,
    AUX_FOG,
    AUX_ROOF,
    AUX_BRAKE,      // dedicated brake lamp; also stops TAIL doubling as one
    AUX_BEACON2,    // second beacon, offset from the first for a rotating look
    AUX_CABIN,
};

struct LightState {
    bool running   = false;   // engine on
    bool park      = false;   // side / tail / marker on
    bool head      = false;
    bool highBeam  = false;
    bool braking   = false;
    bool reverse   = false;
    int  indicate  = 0;       // -1 left, 0 none, +1 right
    bool hazard    = false;
    bool fog       = false;
    bool roof      = false;
    bool beacon    = false;
    bool cabin     = false;
    bool combinedRear = false; // US: rear indicators = tail/brake + turn
};

class Lights {
public:
    bool begin(uint8_t i2cAddr, float pwmHz);
    void update(const LightState& s, uint32_t nowMs);
    bool present() const { return _ok; }
    uint8_t addr() const { return _addr; }

    // Spare channel assignment. Defaults put high beam on AUX1 and leave AUX2
    // free, which suits the most common build.
    AuxFunc aux1 = AUX_HIGHBEAM;
    AuxFunc aux2 = AUX_OFF;

    // per-channel brightness trims, 0..1
    float tailLevel = 0.28f, headLevel = 1.0f, markLevel = 0.85f;
    float brHead = 1.0f, brTail = 1.0f, brBrake = 1.0f, brRev = 1.0f;
    float brSig  = 1.0f, brRun  = 1.0f, brBeacon = 1.0f, brCabin = 1.0f;
    float brAux1 = 1.0f, brAux2 = 1.0f;
    float br3rd  = 1.0f;        // an AUX set to 3rd brake light uses this, not its own trim
    // 0..255 level a channel is showing right now (for the wireless trailer)
    uint8_t level8(uint8_t ch) const { return (ch < LT_COUNT && _haveLast) ? (uint8_t)(_last[ch] >> 4) : 0; }

    // bench / web light test - takes over every channel until cleared
    void testLight(uint8_t ch, uint8_t pwm) { if (ch < LT_COUNT) { _test[ch] = pwm; _testOn = true; } }
    void testAll(uint8_t pwm)               { for (int i = 0; i < LT_COUNT; i++) _test[i] = pwm; _testOn = true; }
    void testClear()                        { _testOn = false; }
    bool testActive() const                 { return _testOn; }

    void setDuty(uint8_t ch, uint16_t duty12);       // 0..4095, public for the test menu

private:
    uint8_t _addr = 0x5F;
    bool    _ok = false;
    uint8_t _test[LT_COUNT] = {0};
    bool    _testOn = false;
    uint16_t _last[LT_COUNT];
    bool    _haveLast = false;

    void wreg(uint8_t reg, uint8_t val);
    void setF(uint8_t ch, float f);
    float auxLevel(AuxFunc f, const LightState& s, bool blink,
                   bool bflash2, bool markers, float trim) const;
};

extern Lights Lite;
