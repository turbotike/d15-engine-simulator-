// =============================================================================
//  RcInput.h - one input driver, three protocols:
//    iBUS  (FlySky, 115200 8N1, 14ch)   - hand-rolled parser
//    SBUS  (FrSky/Futaba, 100k 8E2, inverted or not, 16ch)
//    PWM   (classic servo pulses on the CH1-8 headers, EIGHT channels here)
//  Auto-detects on boot, or force a mode with RC_FORCE_MODE in Config.h.
//  All protocols normalise to 1000..2000us per channel.
// =============================================================================
//  Differences from the Teensy build:
//    * EIGHT PWM channels, not six - this board brings out CH1-CH8.
//    * The serial receiver lands on GPIO12, so Serial1 is remapped onto it.
//      Serial itself is the native USB CDC on GPIO19/20, which is why flashing
//      and tuning over USB does not disturb the receiver.
//    * SBUS inversion is a begin() argument on ESP32, not a config constant -
//      there is no SERIAL_8E2_RXINV here.
// =============================================================================
#pragma once
#include <Arduino.h>

enum RcMode : uint8_t {
    RC_DETECT = 0,
    RC_IBUS   = 1,
    RC_SBUS   = 2,      // inverted (standard) SBUS
    RC_SBUS_N = 3,      // non-inverted SBUS (some receivers)
    RC_PWM    = 4,
};

class RcInput {
public:
    void begin();
    void poll();                         // call every loop
    uint16_t ch(uint8_t i) const { return (i < 16) ? _ch[i] : 1500; }
    // _everSaw matters: without it, _lastFrame starts at 0 and millis() starts
    // at 0, so this reads TRUE for the first 500 ms with no receiver attached.
    bool   active() const { return _everSaw && (millis() - _lastFrame) < 500; }
    bool   everSaw() const { return _everSaw; }
    void   failsafe();          // force every channel to neutral
    // CONTROL tab: hand a CH header to ServoOut (no ISR, so the PWM
    // auto-detect cannot see its own servo pulses) and take it back.
    void   releasePin(uint8_t i);
    void   claimPin(uint8_t i);
    RcMode mode()  const { return _mode; }
    const char* modeName() const;

private:
    void enterIbus();
    void enterSbus(bool inverted);
    void pollIbus();
    void pollSbus();
    void detect();

    uint16_t _ch[16];
    uint8_t  _buf[32];
    uint8_t  _idx = 0;
    uint32_t _lastByteUs = 0;
    uint32_t _lastFrame = 0;
    bool     _everSaw = false;
    RcMode   _mode = RC_DETECT;
    uint8_t  _try = 0;                  // 0=iBUS 1=SBUS 2=SBUS non-inv
    uint32_t _tryStart = 0;
};

extern RcInput RC;
