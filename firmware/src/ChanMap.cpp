// =============================================================================
//  ChanMap.cpp - see header.
// =============================================================================
#include "ChanMap.h"
#include "DcEeprom.h"     // ESP32: NVS-backed, Teensy-compatible API

ChanMap Chan;

// EEPROM: parts @500, servo map @512-675. This sits clear of both.
static const int      CM_EE_ADDR  = 690;
// BUMPED when the defaults changed. A saved map silently wins over a new
// default otherwise - the exact trap the beier README documents costing
// them three settings that never reached the truck.
static const uint16_t CM_MAGIC    = 0xC4A2;

void ChanMap::begin() {
    uint16_t magic; EEPROM.get(CM_EE_ADDR, magic);
    if (magic != CM_MAGIC) return;                      // unwritten -> keep defaults
    uint8_t tmp[CM_N]; EEPROM.get(CM_EE_ADDR + 2, tmp);
    for (uint8_t i = 0; i < CM_N; i++)
        if (tmp[i] < 16) _ch[i] = tmp[i];               // validate: never drive on garbage
}

void ChanMap::save() {
    EEPROM.put(CM_EE_ADDR, CM_MAGIC);
    EEPROM.put(CM_EE_ADDR + 2, _ch);
}

bool ChanMap::set(uint8_t fn, uint8_t chan) {
    if (fn >= CM_N || chan >= 16) return false;         // RcInput holds 16 channels
    _ch[fn] = chan;
    // No save() here: hSave commits once. This fired a full NVS blob write
    // on every channel dropdown the user touched.
    return true;
}
