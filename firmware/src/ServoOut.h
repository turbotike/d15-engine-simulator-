// =============================================================================
//  ServoOut.h - the CONTROL tab: turn spare CH headers into servo OUTPUTS.
//
//  Every CH header is wired  header pin -> 330R -> plain ESP32 GPIO  (no
//  buffer), with the header + pin on BEC_5V - so any of them can drive a servo
//  signal instead of reading a receiver. Each header has TWO sockets on the
//  same signal; a channel used as an output must have no receiver lead on it.
//  The 330R limits a mistake to ~10 mA, so a clash is harmless.
//
//  Rules:
//    * Every channel boots OFF until switched on from the UI.
//    * In PWM receiver mode a channel the truck reads (steering, throttle,
//      gear, engine, lights, horn, jake, aux) is LOCKED - it can never be
//      taken over, so you cannot remove your own throttle by accident. In
//      iBUS/SBUS mode the radio comes in on one wire and every header is free.
//    * At most 7 at once: the S3 has 8 LEDC channels and the ESC uses one.
//    * An enabled pin is released from the RC input side first - the PWM
//      auto-detect watches every header and would otherwise "detect" its own
//      servo pulses and latch the board into PWM mode.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "Config.h"

class ServoOutClass {
public:
    static const uint8_t N = DC_PWM_CHANNELS;    // CH1..CH8 headers
    static const uint16_t US_MIN = 1000, US_MAX = 2000, US_MID = 1500;

    void begin();                 // restore saved outputs (after RC.begin)
    void poll();                  // lock enforcement + deferred save

    // returns "" on success, else why not (shown in the UI)
    const char* setOn(uint8_t i, bool on);
    void        setUs(uint8_t i, int us);

    bool        on(uint8_t i) const { return i < N && _on[i]; }
    uint16_t    us(uint8_t i) const { return i < N ? _us[i] : US_MID; }
    const char* lockedBy(uint8_t i) const;   // truck function holding it, or ""

    // STEERING SERVO OUTPUT (iBUS/SBUS only). Called every loop: header i
    // (0-7, or -1 = off) follows `us`. In PWM mode it is ignored - there the
    // servo plugs into the receiver, and taking a header would stop the board
    // reading it. Not saved here; Cfg.steerOut is the setting.
    void        steer(int8_t i, int us);

    // OWNED HEADERS. A truck feature (steering = slot 0, hydraulic functions
    // 1..4 = slots 1..4) can hold a header and drive it every loop. Refused on
    // a header the truck READS in PWM mode (same rule as the CONTROL tab).
    // idx -1 releases the slot. Returns "" or why not.
    static const uint8_t NSLOT = 7;   // 0 steering, 1-4 hydraulics, 5-6 active suspension L/R
    const char* claim(uint8_t slot, int8_t idx, int us);
    int8_t      owner(uint8_t i) const;       // slot holding header i, or -1
    String      json() const;

private:
    void  attach(uint8_t i);
    void  detach(uint8_t i);
    void  write(uint8_t i);
    void  save();
    void  dirty() { _dirtyMs = millis(); _dirty = true; }

    bool     _on[N]   = {};
    uint16_t _us[N]   = { US_MID, US_MID, US_MID, US_MID, US_MID, US_MID, US_MID, US_MID };
    int8_t   _ledc[N] = { -1, -1, -1, -1, -1, -1, -1, -1 };
    int8_t   _slotIdx[NSLOT] = { -1, -1, -1, -1, -1, -1, -1 };
    uint16_t _slotUs[NSLOT]  = { US_MID, US_MID, US_MID, US_MID, US_MID, US_MID, US_MID };
    bool     _dirty = false;
    uint32_t _dirtyMs = 0;
};

extern ServoOutClass Servos;
