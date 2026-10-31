// =============================================================================
//  Battery.h - real pack monitoring + "battery = fuel" (after DIYGuy999's
//  BATTERY_PROTECTION, adapted to the sim).
//
//  Hardware (rev3 and Public V1): R11 100k / R12 47k + C7 100nF from the
//  SWITCHED pack rail into IO4 (ADC1_CH3): 8.4 V -> 2.69 V. R13/R14 100k/100k
//  from the ESC's BEC into IO5: 5 V -> 2.5 V.
//
//  What it does:
//   * pack volts, averaged, x 147/47, x a user trim (calibrate once to a meter)
//   * cell count at the first good reading (this board is 2S; anything else warns)
//   * fuel gauge = state of charge from a LiPo curve (not linear - a LiPo sits
//     flat at 3.7-3.9 V and a straight line reads half-empty on a full pack)
//   * cutoff 3.30 V/cell, filtered ~1 s and held 3 s (a launch sag must not
//     trip it): the engine STARVES - sputters for ~4 s - then dies like it ran
//     dry, the truck coasts down on the ESC, hazards on. Latched until a fresh
//     pack goes in (the sim refuses to start on dry tanks).
//   * ONLY ACTIVE WHEN THE ESC'S BEC IS PRESENT. On USB alone the pack pin reads
//     ~5.7 V (USB back-feeds through the bucks into the divider - measured at
//     rev3 bring-up), which would look like a flat 2S pack.
//
//  DIYGuy's recovery test is `v > cutoff + full` (15 V on 2S) - it can never
//  pass, so his protection is a latch by accident. Here the latch is on purpose.
// =============================================================================
#pragma once
#include <Arduino.h>

enum BattState : uint8_t { BATT_NONE = 0, BATT_OK, BATT_LOW, BATT_EMPTY };

class BatteryClass {
public:
    void  begin();
    void  poll();                       // call every loop; self-paces to 10 Hz
    float volts()    const { return _v; }         // filtered pack volts
    float bec()      const { return _bec; }       // BEC volts
    uint8_t cells()  const { return _cells; }
    float soc()      const { return _soc; }       // 0..1, for the fuel gauge
    BattState state() const { return _st; }
    bool  active()   const { return _active; }    // BEC present + protection on
    float starve()   const { return _starve; }    // 0..1 fuel-cut amount for the sim
    bool  hazard()   const { return _st == BATT_EMPTY; }
    const char* stateName() const;
private:
    float readPackRaw();
    float _v = 0, _vFast = 0, _bec = 0, _soc = 1;
    uint8_t _cells = 0;
    BattState _st = BATT_NONE;
    bool  _active = false;
    float _starve = 0;
    uint32_t _last = 0, _lowSince = 0, _emptyAt = 0;
};
extern BatteryClass Batt;
