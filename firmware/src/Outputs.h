// =============================================================================
//  Outputs.h - ESC on the S3 board. The motor follows the sim's road speed.
// =============================================================================
//  The Teensy build used the Servo library. On the S3 that library is a wrapper
//  over LEDC anyway, so this drives LEDC directly: one 50 Hz channel at 16-bit
//  resolution, which gives ~0.3 us of pulse-width granularity. Plenty - the
//  slew limiter below moves in 12 us steps.
//
//  There is no steering output and there are no GPIO light pins on this board.
//  Steering is a hardware pass-through at the receiver, and every light is on
//  the PCA9685 (see Lights.cpp).
// =============================================================================
#pragma once
#include <Arduino.h>

class Outputs {
public:
    void begin();
    // fwdFrac / revFrac are 0..1 from the sim; `reverse` picks direction.
    // brake01 0..1 is a REAL drag brake - it commands the ESC below neutral so
    // the motor holds the truck back on a grade.
    void drive(float fwdFrac, float revFrac, bool reverse, float brake01);
    void lights(bool head, bool brake, bool rev);   // no-op, kept for callers
    float escMicros() const { return _cur; }        // for the serial heartbeat

private:
    void writeMicros(float us);
    float _cur = 1500.0f;
    bool  _ready = false;
};

extern Outputs Out;
