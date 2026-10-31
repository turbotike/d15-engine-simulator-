// =============================================================================
//  DcEeprom.h - Teensy-compatible EEPROM API backed by the ESP32's NVS
// =============================================================================
//  EngineSim and ChanMap were written against the Teensy EEPROM library, where
//  a write lands immediately and `update()` exists. Neither is true on ESP32:
//  the Arduino EEPROM shim there is deprecated flash emulation, it has no
//  `update()`, and nothing is persisted until you call `commit()`. Code that
//  "works" without the commit silently loses everything on the next boot.
//
//  So rather than scatter commits through the physics files - which are copies
//  of the Teensy product and want to stay diffable against it - this presents
//  the API they already use, over Preferences/NVS, which is what you are
//  supposed to use on this chip.
//
//  The whole store is one NVS blob held in RAM. Reads never touch flash. A
//  write only reaches flash if the byte actually changed, so the repeated
//  put() calls in saveParts() cost one erase between them, not three.
//
//  Address map inherited from the Teensy build - do not reshuffle, saved units
//  in the field already use it:
//      500 - 510   engine parts (magic + Parts + unlocks)
//      512 - 675   servo map    (UNUSED on S3: the servo bank is gone)
//      690 - ...   channel map
// =============================================================================
#pragma once
#include <Arduino.h>

#define DC_EE_SIZE 1024

class DcEepromClass {
public:
    void begin(size_t sz = DC_EE_SIZE);
    void commit();                       // flush if dirty; safe to call often

    uint8_t read(int addr) const {
        return (addr >= 0 && (size_t)addr < _sz) ? _buf[addr] : 0;
    }
    void write(int addr, uint8_t v) { poke(addr, v); }

    // Teensy's update() - write only if different. Kept because the physics
    // files call it and because on NVS it is the difference between an erase
    // cycle and nothing at all.
    void update(int addr, uint8_t v) { poke(addr, v); }

    template <typename T> T& get(int addr, T& t) {
        if (addr >= 0 && (size_t)(addr + sizeof(T)) <= _sz) {
            memcpy((void *)&t, _buf + addr, sizeof(T));
        }
        return t;
    }

    template <typename T> const T& put(int addr, const T& t) {
        if (addr >= 0 && (size_t)(addr + sizeof(T)) <= _sz) {
            const uint8_t *p = (const uint8_t *)&t;
            for (size_t i = 0; i < sizeof(T); i++) poke(addr + (int)i, p[i]);
            // NO commit() here. commit() rewrites the whole 1 KB NVS blob, so a
            // put()-per-field meant hSave did ~50 full blob writes in one
            // synchronous handler - blocking loop() AND stalling core 0, because
            // a flash write disables the cache and the render task lives in
            // flash. That was an audible dropout every time you pressed Save.
            // Callers commit once when they are done.
        }
        return t;
    }

    size_t length() const { return _sz; }

private:
    void poke(int addr, uint8_t v) {
        if (addr < 0 || (size_t)addr >= _sz) return;
        if (_buf[addr] == v) return;            // no change, no wear
        _buf[addr] = v;
        _dirty = true;
    }

    uint8_t *_buf = nullptr;
    size_t   _sz  = 0;
    bool     _dirty = false;
    bool     _open  = false;
};

extern DcEepromClass EEPROM;
