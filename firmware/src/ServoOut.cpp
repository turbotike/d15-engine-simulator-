// =============================================================================
//  ServoOut.cpp - see ServoOut.h.
// =============================================================================
#include "ServoOut.h"
#include "RcInput.h"
#include "ChanMap.h"
#include "DcEeprom.h"

ServoOutClass Servos;

static const uint8_t PINS[ServoOutClass::N] = {
    PIN_CH1, PIN_CH2, PIN_CH3, PIN_CH4, PIN_CH5, PIN_CH6, PIN_CH7, PIN_CH8
};

// Same 50 Hz / 14-bit as the ESC. LEDC channels pair up on a timer (0+1,
// 2+3, ...), so matching the ESC's config lets channel 1 share its timer.
static const uint32_t SV_HZ = 50;
static const uint8_t  SV_BITS = 14;
static const float    SV_COUNTS_PER_US = (float)(1UL << SV_BITS) / (1000000.0f / SV_HZ);

// Storage: clear of parts @500, chan map @690, air @710, web knobs @720-910.
static const int      SV_EE_ADDR = 940;
static const uint16_t SV_MAGIC   = 0x5E71;

// Functions readRc() actually reads. CH_BRAKE is not one - nothing reads it.
static const struct { uint8_t fn; const char* name; } TRUCK_FN[] = {
    { CH_STEERING, "STEERING" }, { CH_THROTTLE, "THROTTLE" }, { CH_GEAR, "GEAR" },
    { CH_ENGINE,   "ENGINE"   }, { CH_LIGHTS,   "LIGHTS"   }, { CH_HORN, "HORN" },
    { CH_JAKE,     "JAKE"     }, { CH_AUX,      "AUX"      },
};

const char* ServoOutClass::lockedBy(uint8_t i) const {
    // Only a PWM receiver reads the headers. On iBUS/SBUS the radio arrives on
    // the RX header and Chan.of() indexes the serial stream, not a header.
    if (RC.mode() != RC_PWM) return "";
    for (auto& f : TRUCK_FN) if (Chan.of(f.fn) == i) return f.name;
    return "";
}

void ServoOutClass::begin() {
    uint16_t magic = 0;
    EEPROM.get(SV_EE_ADDR, magic);
    if (magic != SV_MAGIC) return;                 // never saved: all OFF
    bool on[N]; uint16_t us[N];
    EEPROM.get(SV_EE_ADDR + 2, on);
    EEPROM.get(SV_EE_ADDR + 2 + (int)sizeof(on), us);
    for (uint8_t i = 0; i < N; i++) {
        _us[i] = constrain(us[i], US_MIN, US_MAX);
        // The receiver mode is not known yet at boot. poll() hands a channel
        // back the moment PWM mode latches with a truck function on it.
        if (on[i]) setOn(i, true);
    }
    _dirty = false;                                // restoring is not a change
}

void ServoOutClass::poll() {
    // A PWM receiver can latch AFTER boot (or be plugged in later). If it
    // claims a channel we are driving, give the pin back to the truck at once.
    for (uint8_t i = 0; i < N; i++)
        if (_on[i] && lockedBy(i)[0]) { detach(i); _on[i] = false; dirty(); }
    for (uint8_t s = 0; s < NSLOT; s++)
        if (_slotIdx[s] >= 0 && lockedBy(_slotIdx[s])[0]) claim(s, -1, US_MID);
    // Holding an arrow sends a stream of moves; commit() rewrites the whole
    // 1 KB NVS blob, so save once the arrows have been still for 1.5 s.
    if (_dirty && millis() - _dirtyMs > 1500) { save(); _dirty = false; }
}

const char* ServoOutClass::setOn(uint8_t i, bool on) {
    if (i >= N) return "bad channel";
    if (on == _on[i]) return "";
    if (on) {
        if (owner(i) >= 0) return owner(i) == 0 ? "STEER OUT" : "HYDRAULICS";
        const char* lk = lockedBy(i);
        if (lk[0]) return lk;                      // UI shows "used by <fn>"
        uint8_t used = 0;
        for (uint8_t k = 0; k < N; k++) used += _on[k];
        if (used >= 7) return "max 7 servos";
        _on[i] = true;
        attach(i);
    } else {
        detach(i);
        _on[i] = false;
    }
    dirty();
    return "";
}

void ServoOutClass::setUs(uint8_t i, int us) {
    if (i >= N) return;
    _us[i] = (uint16_t)constrain(us, (int)US_MIN, (int)US_MAX);
    if (_on[i]) write(i);
    dirty();
}

void ServoOutClass::attach(uint8_t i) {
    // pick a free LEDC channel 1..7 (0 is the ESC)
    int8_t ch = -1;
    for (int8_t c = 1; c <= 7 && ch < 0; c++) {
        bool taken = false;
        for (uint8_t k = 0; k < N; k++) if (_ledc[k] == c) taken = true;
        if (!taken) ch = c;
    }
    if (ch < 0) { _on[i] = false; return; }
    RC.releasePin(i);                              // stop reading it first
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(PINS[i], SV_HZ, SV_BITS, ch);
#else
    ledcSetup(ch, SV_HZ, SV_BITS);
    ledcAttachPin(PINS[i], ch);
#endif
    _ledc[i] = ch;
    write(i);
}

void ServoOutClass::detach(uint8_t i) {
    if (_ledc[i] < 0) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcDetach(PINS[i]);
#else
    ledcDetachPin(PINS[i]);
#endif
    _ledc[i] = -1;
    RC.claimPin(i);                                // back to a receiver input
}

int8_t ServoOutClass::owner(uint8_t i) const {
    for (uint8_t s = 0; s < NSLOT; s++) if (_slotIdx[s] == (int8_t)i) return (int8_t)s;
    return -1;
}

const char* ServoOutClass::claim(uint8_t slot, int8_t idx, int us) {
    if (slot >= NSLOT) return "bad slot";
    if (idx >= (int8_t)N) idx = -1;
    if (idx >= 0) {
        if (RC.mode() == RC_DETECT) idx = -1;                    // wait until we know the radio
        else if (lockedBy(idx)[0]) idx = -1;                     // the truck reads that header
        else { int8_t o = owner(idx); if (o >= 0 && o != (int8_t)slot) idx = -1; }   // first come
    }
    if (idx != _slotIdx[slot]) {
        int8_t old = _slotIdx[slot];
        _slotIdx[slot] = -1;
        if (old >= 0 && !_on[old] && owner(old) < 0) detach(old);   // hand it back
        if (idx >= 0) {
            _slotIdx[slot] = idx;
            if (_on[idx]) { _on[idx] = false; dirty(); }         // the feature takes the header over
            if (_ledc[idx] < 0) attach(idx);
            if (_ledc[idx] < 0) { _slotIdx[slot] = -1; return "max 7 servos"; }
        }
    }
    if (_slotIdx[slot] < 0) return idx < 0 ? "" : "not available";
    _slotUs[slot] = (uint16_t)constrain(us, 800, 2200);
    write(_slotIdx[slot]);
    return "";
}

// Steering is slot 0 and only exists with a serial receiver: in PWM mode the
// servo plugs into the receiver and never sees this board.
void ServoOutClass::steer(int8_t i, int us) {
    RcMode m = RC.mode();
    bool serial = (m == RC_IBUS || m == RC_SBUS || m == RC_SBUS_N);
    claim(0, serial ? i : -1, us);
}

void ServoOutClass::write(uint8_t i) {
    if (_ledc[i] < 0) return;
    int8_t o = owner(i);
    uint16_t us = (o >= 0) ? _slotUs[o] : _us[i];
    uint32_t duty = (uint32_t)(us * SV_COUNTS_PER_US + 0.5f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PINS[i], duty);
#else
    ledcWrite(_ledc[i], duty);
#endif
}

void ServoOutClass::save() {
    EEPROM.put(SV_EE_ADDR, SV_MAGIC);
    EEPROM.put(SV_EE_ADDR + 2, _on);
    EEPROM.put(SV_EE_ADDR + 2 + (int)sizeof(_on), _us);
    EEPROM.commit();
}

String ServoOutClass::json() const {
    String j; j.reserve(400);
    j = "{\"rc\":\""; j += RC.modeName(); j += "\",\"ch\":[";
    for (uint8_t i = 0; i < N; i++) {
        if (i) j += ',';
        int8_t o = owner(i);
        j += "{\"on\":"; j += (_on[i] || o >= 0) ? '1' : '0';
        j += ",\"us\":"; j += (o >= 0) ? _slotUs[o] : _us[i];
        j += ",\"lock\":\""; j += (o == 0) ? "STEER OUT" : (o > 0 ? "HYDRAULICS" : lockedBy(i)); j += "\"}";
    }
    j += "]}";
    return j;
}
