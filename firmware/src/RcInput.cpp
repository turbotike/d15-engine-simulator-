// =============================================================================
//  RcInput.cpp - iBUS / SBUS / PWM with boot-time auto-detect.
//
//  Detection: cycle 400 ms listening windows - iBUS (115200 8N1), SBUS
//  (100000 8E2 inverted), SBUS non-inverted - while PWM edge interrupts watch
//  the CH1-8 headers the whole time. First protocol to deliver a valid frame,
//  or 20+ clean PWM pulses, wins and the mode latches.
// =============================================================================
#include "RcInput.h"
#include "Config.h"

RcInput RC;

// ---- PWM capture (interrupt driven, CH1-8 headers) -------------------------
static const uint8_t PWM_PINS[DC_PWM_CHANNELS] = {
    PIN_CH1, PIN_CH2, PIN_CH3, PIN_CH4, PIN_CH5, PIN_CH6, PIN_CH7, PIN_CH8
};
static volatile uint32_t pwmRise[DC_PWM_CHANNELS];
static volatile uint16_t pwmUs[DC_PWM_CHANNELS];
static volatile uint32_t pwmLastMs[DC_PWM_CHANNELS];
static volatile uint16_t pwmCount = 0;

// IRAM_ATTR matters on ESP32: without it the handler lives in flash, and an
// interrupt that fires during a flash operation (an NVS commit, a pack read)
// takes the whole board down with a cache-disabled panic.
template <int I> static void IRAM_ATTR pwmIsr() {
    if (digitalRead(PWM_PINS[I])) {
        pwmRise[I] = micros();
    } else {
        uint32_t w = micros() - pwmRise[I];
        if (w >= 800 && w <= 2300) {
            pwmUs[I] = (uint16_t)w;
            pwmLastMs[I] = millis();
            pwmCount++;
        }
    }
}

static void (*const PWM_ISR[DC_PWM_CHANNELS])() = {
    pwmIsr<0>, pwmIsr<1>, pwmIsr<2>, pwmIsr<3>, pwmIsr<4>, pwmIsr<5>, pwmIsr<6>, pwmIsr<7>
};
void RcInput::releasePin(uint8_t i) {
    if (i >= DC_PWM_CHANNELS) return;
    detachInterrupt(digitalPinToInterrupt(PWM_PINS[i]));
    pwmLastMs[i] = 0;                  // goes stale: PWM mode keeps the last value
}
void RcInput::claimPin(uint8_t i) {
    if (i >= DC_PWM_CHANNELS) return;
    pinMode(PWM_PINS[i], INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PWM_PINS[i]), PWM_ISR[i], CHANGE);
}

void RcInput::begin() {
    for (int i = 0; i < 16; i++) _ch[i] = 1500;
    for (int i = 0; i < DC_PWM_CHANNELS; i++) { pwmUs[i] = 1500; pwmLastMs[i] = 0; }

    // PULLDOWN, not plain INPUT. A floating CH pin picks up enough mains hum and
    // crosstalk to produce edges that look like servo pulses - on the bench that
    // was enough to trip the >20-pulse detector and latch RC_PWM with nothing
    // connected, after which readRc() overwrote every bench keypress and the
    // engine could not be started at all. A pulldown makes an unconnected
    // channel read a solid low and generate nothing.
    for (int i = 0; i < DC_PWM_CHANNELS; i++) pinMode(PWM_PINS[i], INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_CH1), pwmIsr<0>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_CH2), pwmIsr<1>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_CH3), pwmIsr<2>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_CH4), pwmIsr<3>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_CH5), pwmIsr<4>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_CH6), pwmIsr<5>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_CH7), pwmIsr<6>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_CH8), pwmIsr<7>, CHANGE);

#if RC_FORCE_MODE == 1
    _mode = RC_IBUS;  enterIbus();
#elif RC_FORCE_MODE == 2
    _mode = RC_SBUS;  enterSbus(true);
#elif RC_FORCE_MODE == 3
    _mode = RC_PWM;
#else
    _mode = RC_DETECT; _try = 0; _tryStart = millis(); enterIbus();
#endif
}

// The receiver signal arrives on GPIO12 through R10's 330R series resistor.
// RX only - nothing on this board talks back to the receiver, so TX is -1.
void RcInput::enterIbus() {
    RC_SERIAL.end();
    RC_SERIAL.begin(115200, SERIAL_8N1, PIN_RC_SERIAL_RX, -1, false);
    _idx = 0;
}

void RcInput::enterSbus(bool inv) {
    RC_SERIAL.end();
    // On ESP32 inversion is the 5th begin() argument. There is no
    // SERIAL_8E2_RXINV constant like the Teensy core has.
    RC_SERIAL.begin(100000, SERIAL_8E2, PIN_RC_SERIAL_RX, -1, inv);
    _idx = 0;
}

void RcInput::failsafe() {
    // 1500 reads as zero throttle, zero brake, and neither drive nor reverse.
    for (int i = 0; i < 16; i++) _ch[i] = 1500;
}

const char* RcInput::modeName() const {
    switch (_mode) {
        case RC_IBUS:   return "iBUS";
        case RC_SBUS:   return "SBUS";
        case RC_SBUS_N: return "SBUS(non-inv)";
        case RC_PWM:    return "PWM";
        default:        return "detecting";
    }
}

// ---- iBUS: 0x20 0x40 + 14x u16le + checksum --------------------------------
void RcInput::pollIbus() {
    while (RC_SERIAL.available()) {
        uint8_t b = RC_SERIAL.read();
        if (_idx == 0 && b != 0x20) continue;
        if (_idx == 1 && b != 0x40) { _idx = 0; continue; }
        _buf[_idx++] = b;
        if (_idx >= 32) {
            _idx = 0;
            uint16_t sum = 0xFFFF;
            for (int i = 0; i < 30; i++) sum -= _buf[i];
            uint16_t chk = _buf[30] | (_buf[31] << 8);
            if (sum == chk) {
                for (int c = 0; c < 14; c++)
                    _ch[c] = _buf[2 + c * 2] | (_buf[3 + c * 2] << 8);
                _lastFrame = millis(); _everSaw = true;
                if (_mode == RC_DETECT) _mode = RC_IBUS;
            }
        }
    }
}

// ---- SBUS: 25 bytes, 0x0F header, 16x 11-bit, frame gap >= 2.5ms -----------
void RcInput::pollSbus() {
    while (RC_SERIAL.available()) {
        uint32_t nowUs = micros();
        uint8_t b = RC_SERIAL.read();
        if (nowUs - _lastByteUs > 2500) _idx = 0;      // inter-frame gap resync
        _lastByteUs = nowUs;
        if (_idx == 0 && b != 0x0F) continue;
        _buf[_idx++] = b;
        if (_idx >= 25) {
            _idx = 0;
            // The receiver is telling us it lost the transmitter. Dropping the
            // frame left _ch[] holding the last good values and stopped updating
            // _lastFrame - so the one case that most needs the ESC neutralised
            // was handled worst of all. Neutralise instead.
            if (_buf[23] & 0x08) { failsafe(); _lastFrame = millis(); continue; }
            uint16_t raw[16];
            const uint8_t *d = _buf + 1;
            raw[0]  = (uint16_t)((d[0]        | d[1]  << 8)               & 0x07FF);
            raw[1]  = (uint16_t)((d[1]  >> 3  | d[2]  << 5)               & 0x07FF);
            raw[2]  = (uint16_t)((d[2]  >> 6  | d[3]  << 2 | d[4] << 10)  & 0x07FF);
            raw[3]  = (uint16_t)((d[4]  >> 1  | d[5]  << 7)               & 0x07FF);
            raw[4]  = (uint16_t)((d[5]  >> 4  | d[6]  << 4)               & 0x07FF);
            raw[5]  = (uint16_t)((d[6]  >> 7  | d[7]  << 1 | d[8] << 9)   & 0x07FF);
            raw[6]  = (uint16_t)((d[8]  >> 2  | d[9]  << 6)               & 0x07FF);
            raw[7]  = (uint16_t)((d[9]  >> 5  | d[10] << 3)               & 0x07FF);
            raw[8]  = (uint16_t)((d[11]       | d[12] << 8)               & 0x07FF);
            raw[9]  = (uint16_t)((d[12] >> 3  | d[13] << 5)               & 0x07FF);
            raw[10] = (uint16_t)((d[13] >> 6  | d[14] << 2 | d[15] << 10) & 0x07FF);
            raw[11] = (uint16_t)((d[15] >> 1  | d[16] << 7)               & 0x07FF);
            raw[12] = (uint16_t)((d[16] >> 4  | d[17] << 4)               & 0x07FF);
            raw[13] = (uint16_t)((d[17] >> 7  | d[18] << 1 | d[19] << 9)  & 0x07FF);
            raw[14] = (uint16_t)((d[19] >> 2  | d[20] << 6)               & 0x07FF);
            raw[15] = (uint16_t)((d[20] >> 5  | d[21] << 3)               & 0x07FF);
            for (int c = 0; c < 16; c++)               // 172..1811 -> ~988..2012us
                _ch[c] = (uint16_t)(raw[c] * 0.625f + 880.0f);
            _lastFrame = millis(); _everSaw = true;
            if (_mode == RC_DETECT) _mode = (_try == 2) ? RC_SBUS_N : RC_SBUS;
        }
    }
}

void RcInput::detect() {
    // PWM wins when pulses are genuinely streaming in. The count alone is not
    // enough - noise on a floating pin can reach it - so also require that at
    // least one channel is sitting in the valid servo band right now.
    if (pwmCount > 20) {
        bool real = false;
        for (int i = 0; i < DC_PWM_CHANNELS; i++)
            if (millis() - pwmLastMs[i] < 200 && pwmUs[i] >= 900 && pwmUs[i] <= 2200) real = true;
        if (real) { _mode = RC_PWM; RC_SERIAL.end(); return; }
        pwmCount = 0;                       // noise, not a receiver - keep looking
    }
    if (millis() - _tryStart < 400) return;             // stay in this window
    _try = (uint8_t)((_try + 1) % 3);
    _tryStart = millis();
    if (_try == 0)      enterIbus();
    else if (_try == 1) enterSbus(true);
    else                enterSbus(false);
}

void RcInput::poll() {
    switch (_mode) {
        case RC_IBUS:   pollIbus(); break;
        case RC_SBUS:
        case RC_SBUS_N: pollSbus(); break;
        case RC_PWM: {
            bool any = false;
            for (int i = 0; i < DC_PWM_CHANNELS; i++) {
                if (millis() - pwmLastMs[i] < 500) {
                    _ch[i] = pwmUs[i];
                    _lastFrame = millis(); _everSaw = true;
                    any = true;
                }
            }
            // Unplugging the receiver used to leave this latched in PWM forever,
            // reading floating pins. After 3 s of nothing, go back to detecting
            // so serial receivers are found again - and so the bench keys work.
            if (!any && (millis() - _lastFrame) > 3000) {
                _mode = RC_DETECT; _try = 0; _tryStart = millis(); pwmCount = 0;
                enterIbus();
            }
            break;
        }
        default: {
            if (_try == 0) pollIbus(); else pollSbus();
            if (_mode == RC_DETECT) detect();
            break;
        }
    }
}
