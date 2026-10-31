// =============================================================================
//  Hydraulics.h - WORK MODE: PTO pump + hydraulic cylinders on the servo outputs
// =============================================================================
//  Ported from the Teensy DieselCore Hydraulics (which took its sticks from a
//  PS4 pad - the S3 cannot pair one, so here the sticks are RC channels).
//
//  A servo is treated as a CYLINDER, not a position servo: the stick sets the
//  SPEED a function moves, centre stick = it holds where it is. How fast it
//  moves comes from the pump, and the pump hangs off the PTO:
//    * flow follows engine rpm (nothing below ~550, full by ~2000)
//    * several functions at once share the flow - everything slows down
//    * raising (stick +) fights gravity, lowering is quicker
//    * holding a function into its end stop dumps over the relief valve:
//      pressure pins, the fluid heats, and the engine lugs (PTO load)
//
//  Work mode only runs in PARK with the engine on. The ESC stays at neutral -
//  the throttle and steering sticks belong to the crane while it is on.
//  Modes: 0 none, 1 HIAB crane (4 functions), 2 dump box (function 1 only).
// =============================================================================
#pragma once
#include <Arduino.h>

class Hydraulics {
public:
    void begin();                                   // restore cylinder positions
    // cmd[i] -1..1 per function; call every loop
    void tick(float dt, float rpm, bool engineRunning, const float cmd[4]);
    void setOn(bool on);
    bool on() const          { return _on; }
    int  nFn() const         { return _mode == 1 ? 4 : (_mode == 2 ? 1 : 0); }
    void setMode(int m)      { _mode = constrain(m, 0, 2); }

    float pressPsi() const   { return _press; }
    float tempF()    const   { return _temp; }
    float pos(int fn) const  { return (fn >= 0 && fn < 4) ? _pos[fn] : 0; }
    float ptoLoad()  const   { return _ptoLoad; }   // 0..1 extra engine load
    float demand()   const   { return _demand; }    // 0..1 how hard the sticks are pushed
    float relief()   const   { return _relief; }    // 0..1 dumping over the relief valve

private:
    void  savePos();
    bool  _on = false;
    int   _mode = 0;
    float _pos[4] = { 0.5f, 0.2f, 0.0f, 0.5f };     // parked-ish
    float _press = 0, _temp = 75.0f, _ptoLoad = 0, _demand = 0, _relief = 0;
    bool  _moved = false;
};

extern Hydraulics Hyd;
