// =============================================================================
//  Suspension.h - ACTIVE FRONT SUSPENSION, SuperScale-style (Mat 2026-10-31:
//  "it uses 2 servos as the front suspension, they act off the gyro and make
//  it look real ... the last 2 servo ch").
//
//  Two servos move the front shock mounts. The body is a spring-mass-damper:
//    * cornering (sideways g) ROLLS it - the outside corner goes down
//    * slowing down DIVES the nose, pulling away SQUATS it (nose up)
//    * with a little overshoot and settle, so it reads as weight, not a motor
//  Driven by the GY-521's real accelerations (Tilt.accFwd/accLat); without a
//  gyro it falls back on the sim's acceleration and steering x speed.
//
//  Headers: CH7 / CH8 by default (ServoOut slots 5 and 6). In PWM-receiver
//  mode a header the truck READS is locked (CH7 = lights, CH8 = horn by
//  default) - remap those in RC CHANNEL MAP, or run iBUS/SBUS. The page says
//  which function is holding a header.
//  Every setting is in NVS "susp"; travel is hard-limited so a bad trim can
//  never drive a servo into its linkage stop.
// =============================================================================
#pragma once
#include <Arduino.h>

struct SuspParams {
    uint8_t on    = 0;       // off until set up on the page
    int8_t  chL   = 6;       // header index (0 = CH1 .. 7 = CH8): left front
    int8_t  chR   = 7;       //                                    right front
    int16_t ctrL  = 1500;    // ride-height trim, us
    int16_t ctrR  = 1500;
    uint8_t invL  = 0;       // flip a side if its servo moves the wrong way
    uint8_t invR  = 1;       // mirrored mounting is the usual case
    float   travel = 250;    // max us either way from centre (linkage protection)
    float   roll  = 700;     // us per g sideways
    float   pitch = 600;     // us per g fore/aft (dive / squat)
    float   freq  = 1.6f;    // body bounce frequency, Hz (lower = softer, floatier)
    float   damp  = 0.35f;   // 0.2 = bouncy, 0.7 = no overshoot
    float   twist = 80;      // TORQUE TWIST: us of roll at the stock engine's peak torque. Under power the
                             // chassis twists against the engine - left front lifts, right front squats -
                             // and rocks back when you lift. Grows with the build (3000 hp twists ~3x).
                             // Negative flips it.
};

class SuspensionClass {
public:
    SuspParams P;
    void  begin();                                    // load NVS
    // steer -1..1 (stick), mph + accel from the sim for the no-gyro fallback
    void  update(float dt, float steer, float mph, float simAccG, float engTq01);
    void  test(int mode);                             // 0 off, 1 lean L, 2 lean R, 3 dive, 4 squat, 5 centre
    void  save();
    const char* status() const { return _why; }       // "" ok, else why a header is not ours
    String json() const;
    float rollNow() const { return _r; }
    float pitchNow() const { return _p; }
    int   usL() const { return _usL; }               // what the servos were last told (0 = not driving)
    int   usR() const { return _usR; }
    float cornerL() const { return _cornerL; }        // -1..1 of travel, + = that corner goes UP (servo reverse undone)
    float cornerR() const { return _cornerR; }
private:
    float _r = 0, _rv = 0, _p = 0, _pv = 0;          // body roll / pitch state (g-equivalent) + velocity
    int   _test = 0; uint32_t _testUntil = 0;
    const char* _why = "";
    float _lastMph = 0; int _usL = 0, _usR = 0; float _cornerL = 0, _cornerR = 0;
};

extern SuspensionClass Susp;
