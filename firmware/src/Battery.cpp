// =============================================================================
//  Battery.cpp - see Battery.h
// =============================================================================
#include "Battery.h"
#include "Config.h"
#include "Settings.h"

BatteryClass Batt;

static const float CUTOFF_CELL = 3.30f;     // DIYGuy's default; never below 3.2
static const float FULL_CELL   = 4.20f;
static const float BEC_PRESENT = 4.0f;      // volts at the BEC rail
static const uint32_t HOLD_MS  = 3000;      // below cutoff this long before it trips
static const uint32_t STARVE_MS = 4000;     // sputter before it dies

// resting LiPo state of charge per cell (volts, %)
static const float SOC_V[] = { 3.30f, 3.45f, 3.60f, 3.70f, 3.74f, 3.78f, 3.82f, 3.86f, 3.92f, 4.00f, 4.10f, 4.20f };
static const float SOC_P[] = { 0.00f, 0.05f, 0.10f, 0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f, 0.80f, 0.90f, 1.00f };
static float socOf(float vc) {
    if (vc <= SOC_V[0]) return 0;
    for (int i = 1; i < 12; i++)
        if (vc <= SOC_V[i]) return SOC_P[i - 1] + (SOC_P[i] - SOC_P[i - 1]) * (vc - SOC_V[i - 1]) / (SOC_V[i] - SOC_V[i - 1]);
    return 1;
}

static float readMv(int pin) {
    // analogReadMilliVolts uses the chip's eFuse ADC calibration - the same job
    // DIYGuy's ESP32AnalogRead library does. Average a short burst.
    uint32_t s = 0;
    for (int i = 0; i < 8; i++) s += analogReadMilliVolts(pin);
    return s / 8.0f;
}

float BatteryClass::readPackRaw() {
    return readMv(PIN_VBAT_SENSE) / 1000.0f * DC_VBAT_RATIO * DC_VBAT_TRIM * Cfg.battTrim;
}

void BatteryClass::begin() {
    analogSetPinAttenuation(PIN_VBAT_SENSE, ADC_11db);
    analogSetPinAttenuation(PIN_BEC_SENSE, ADC_11db);
    _v = _vFast = readPackRaw();
    _bec = readMv(PIN_BEC_SENSE) / 1000.0f * 2.0f;
}

const char* BatteryClass::stateName() const {
    switch (_st) { case BATT_OK: return "ok"; case BATT_LOW: return "low"; case BATT_EMPTY: return "empty"; default: return "none"; }
}

void BatteryClass::poll() {
    uint32_t now = millis();
    if (now - _last < 100) return;
    float dt = (now - _last) / 1000.0f; _last = now;
    if (dt > 1.0f) dt = 1.0f;

    float raw = readPackRaw();
    _bec += (readMv(PIN_BEC_SENSE) / 1000.0f * 2.0f - _bec) * 0.5f;
    _vFast += (raw - _vFast) * fminf(1.0f, dt / 1.0f);     // ~1 s: cutoff decisions
    _v     += (raw - _v)     * fminf(1.0f, dt / 8.0f);     // ~8 s: gauge and display

    bool protect = Cfg.battProtect > 0.5f;
    _active = protect && _bec > BEC_PRESENT;
    if (!_active) {
        // USB bench / protection off: show the reading, but never cut the engine
        if (_st != BATT_EMPTY) _st = BATT_NONE;
        if (!protect) _st = BATT_NONE;
        _starve = 0; _lowSince = 0;
        _cells = 0;
        return;
    }
    if (_cells == 0) {
        // first good reading with the ESC on: snap the filters, count cells
        _v = _vFast = raw;
        _cells = (raw > 9.0f) ? 3 : 2;                     // board is designed for 2S
        Serial.printf("[batt] %.2f V -> %uS pack detected%s\n", raw, _cells,
                      _cells == 2 ? "" : "  WARNING: this board is designed for 2S");
    }
    float perCell = _vFast / _cells;
    _soc = socOf(_v / _cells);

    if (_st == BATT_EMPTY) {
        // latched: starve, then dry. A fresh pack (clearly above 3.9 V/cell) clears it.
        uint32_t t = now - _emptyAt;
        _starve = (t < STARVE_MS) ? 0.35f + 0.55f * t / (float)STARVE_MS : 1.0f;
        _soc = 0;
        if (perCell > 3.90f) {
            Serial.printf("[batt] fresh pack %.2f V - protection cleared\n", _vFast);
            _st = BATT_OK; _starve = 0; _lowSince = 0;
        }
        return;
    }
    if (perCell < CUTOFF_CELL) {
        if (!_lowSince) _lowSince = now;
        if (now - _lowSince > HOLD_MS) {
            _st = BATT_EMPTY; _emptyAt = now;
            Serial.printf("[batt] CUTOFF %.2f V (%.2f V/cell < %.2f) - starving the engine, unplug the pack\n",
                          _vFast, perCell, CUTOFF_CELL);
        }
    } else {
        _lowSince = 0;
    }
    if (_st != BATT_EMPTY) _st = (_soc < 0.125f) ? BATT_LOW : BATT_OK;
    _starve = 0;
}
