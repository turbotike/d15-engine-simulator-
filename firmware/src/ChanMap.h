// =============================================================================
//  ChanMap.h - runtime RC channel assignment. The CH_* names in Config.h used to
//  BE the channel number, hardcoded at compile time - which meant lights/horn/
//  jake/aux lived on channels 7-10 and were unreachable on a 6-channel PWM radio.
//  Now CH_* is a FUNCTION index and this maps each function to whatever channel
//  the user's transmitter actually puts it on. Web-settable, saved to EEPROM.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "Config.h"

#define CM_N 10          // one slot per CH_* function index

class ChanMap {
public:
    void begin();                       // load from EEPROM (defaults if unwritten)
    void save();
    uint8_t of(uint8_t fn) const { return fn < CM_N ? _ch[fn] : 0; }   // function -> RC channel
    bool set(uint8_t fn, uint8_t chan);  // returns false on a bad index
    uint8_t raw(uint8_t fn) const { return of(fn); }
private:
    // defaults reproduce the old hardcoded behaviour exactly
    // Defaults match the beier firmware's CHANNELS.md, converted to the 0-based
    // index RC.ch() wants. They were a straight 0..9 identity map, which meant
    // every function read one channel too low: gear landed on CH4 - the left
    // stick on a typical radio - instead of the CH7 switch.
    //
    //   index  function     beier channel   RC.ch() index
    //     0    STEERING     CH1             0
    //     1    THROTTLE     CH3             2
    //     2    (unused)     -               2
    //     3    GEAR         CH7             6
    //     4    BRAKE        (unused)        3
    //     5    ENGINE       CH5             4
    //     6    LIGHTS       CH8             7
    //     7    HORN         CH6             5
    //     8    JAKE         (spare)         8
    //     9    AUX          (spare)         9
    uint8_t _ch[CM_N] = { 0, 2, 2, 6, 3, 4, 7, 5, 8, 9 };
};
extern ChanMap Chan;
