// =============================================================================
//  Tilt.h - GY-521 (MPU6050) on the accessory header: the truck's REAL hill.
//
//  ACC1: 1 3V3 -> VCC, 3 GND -> GND, 4 SDA -> SDA, 5 SCL -> SCL (AD0 open =
//  0x68; tied high = 0x69, both are probed). Optional: the board can go in any
//  way up and any way round - '~tiltlevel=1' on flat ground learns which way is
//  UP, '~tiltaxis=0..3' says which board axis points to the FRONT (+X -X +Y -Y,
//  the arrows printed on the GY-521).
//
//  What it does:
//   * gravity is tracked with the gyro and corrected by the accelerometer
//     only while the truck is not accelerating hard (|a| near 1 g), so
//     braking, launches and engine vibration do not read as a hill;
//   * pitch along the truck becomes the sim's road GRADE (x hill feel), so a
//     real ramp loads the engine and the sound for real;
//   * past ~55 deg for half a second it reports a ROLLOVER: main.cpp cuts the
//     throttle and holds the brakes until the truck is back on its wheels.
//  Raw I2C, no library.
// =============================================================================
#pragma once
#include <Arduino.h>

class TiltSensor {
public:
    bool  begin();                 // probe 0x68/0x69, configure, measure gyro bias, load settings
    void  update();                // call every loop (reads at ~100 Hz)
    bool  present()  const { return _ok; }
    float pitchDeg() const { return _pitch; }   // + = nose up
    float rollDeg()  const { return _roll; }    // + = right side down
    float tiltDeg()  const { return _tilt; }    // total angle from level
    float grade()    const;                     // sim grade: sin(pitch) x hill feel
    bool  rolledOver() const { return _over; }
    // LINEAR acceleration in the truck's frame, g (gravity removed, ~50 ms smoothing):
    // + fwd = speeding up, + lat = pushed to the RIGHT (a left turn pushes the body right)
    float accFwd() const { return _aF; }
    float accLat() const { return _aL; }

    // settings (NVS "tilt")
    uint8_t axis = 0;              // forward = 0 +X, 1 -X, 2 +Y, 3 -Y
    float   hill = 0.5f;           // hill feel: 1 = true grade, 0.5 = half (RC ramps are steep)
    float   up[3] = {0, 0, 1};     // which way is up in board axes (learned by tiltlevel)
    void  learnLevel();            // the truck is parked on flat ground: this is level
    void  save();
    void  print();
    uint8_t addr = 0x68;

private:
    bool     _ok = false, _over = false;
    uint32_t _lastUs = 0, _overMs = 0;
    float    _g[3] = {0, 0, 1};    // fused gravity direction, board axes
    float    _bias[3] = {0, 0, 0}; // gyro bias, rad/s
    float    _pitch = 0, _roll = 0, _tilt = 0, _gradeF = 0, _aF = 0, _aL = 0;
    bool     read(float a[3], float w[3]);
    void     load();
};

extern TiltSensor Tilt;
