#include "DcEeprom.h"
#include <Preferences.h>

DcEepromClass EEPROM;

static Preferences s_prefs;
// Renamed with the project (RCLC D15 -> RCLC D15). The OLD namespace is read once
// if the new one is empty, so a board that ran the old firmware keeps its tune,
// channel map, servos, air and web knobs instead of silently resetting.
static const char *NS     = "rclc";
static const char *OLD_NS = "dieselcore";
static const char *KEY = "store";

void DcEepromClass::begin(size_t sz) {
    if (_open) return;
    _sz  = sz;
    _buf = (uint8_t *)calloc(_sz, 1);
    if (!_buf) { _sz = 0; return; }

    // read-write namespace; false = not read-only
    if (!s_prefs.begin(NS, false)) {
        // No NVS partition, or it is full. Run with a RAM-only store rather
        // than refusing to boot - the truck still drives, it just forgets its
        // parts and channel map at power-off.
        _open = true;
        return;
    }
    size_t have = s_prefs.getBytesLength(KEY);
    if (have > 0) {
        if (have > _sz) have = _sz;
        s_prefs.getBytes(KEY, _buf, have);
    } else {
        // first boot of the renamed firmware: migrate the old store once
        Preferences old;
        if (old.begin(OLD_NS, true)) {
            size_t o = old.getBytesLength(KEY);
            if (o > 0) {
                if (o > _sz) o = _sz;
                old.getBytes(KEY, _buf, o);
                s_prefs.putBytes(KEY, _buf, _sz);          // now lives under "rclc"
                Serial.printf("[nvs] migrated %u bytes of settings from \"%s\" to \"%s\"\n", (unsigned)o, OLD_NS, NS);
            }
            old.end();
        }
    }
    _open = true;
}

void DcEepromClass::commit() {
    if (!_dirty || !_open || !_buf) return;
    s_prefs.putBytes(KEY, _buf, _sz);
    _dirty = false;
}
