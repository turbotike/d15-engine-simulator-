// =============================================================================
//  Outputs.cpp
// =============================================================================
#include "Outputs.h"
#include "Config.h"

Outputs Out;

static const float NEUTRAL = 1500, FWD_FLOOR = 55, FWD_SPAN = 410, REV_SPAN = 200;
// NO below-neutral braking. On a forward/reverse ESC below-neutral IS reverse,
// so sending it while rolling forward spun the motor backwards under braking.
// In a drive gear the pulse never goes under NEUTRAL: the air brake in
// EngineSim bleeds road speed, the ESC follows that speed down, and while
// braking it is allowed to come down faster. Real wheel braking comes from the
// ESC's own drag-brake setting (set it on the ESC / its program card).
static const float SLEW_US       = 12.0f;   // normal step per call
static const float BRAKE_SLEW_US = 30.0f;   // faster ramp-down while braking

// One PWM period is (1 << BITS) counts and lasts 1e6/HZ microseconds, so this is
// counts per microsecond. DERIVED from Config.h rather than hardcoded: it was
// written as 65536/20000 for 16 bits, and when the S3 forced the resolution down
// to 14 (its LEDC tops out there, unlike the original ESP32's 20) a literal would
// have quietly made every ESC pulse four times too long - full throttle at rest.
static const float COUNTS_PER_US =
    (float)(1UL << DC_ESC_LEDC_BITS) / (1000000.0f / (float)DC_ESC_LEDC_HZ);

void Outputs::writeMicros(float us) {
    if (!_ready) return;
    uint32_t duty = (uint32_t)(us * COUNTS_PER_US + 0.5f);
    ledcWrite(DC_ESC_LEDC_CH, duty);
}

void Outputs::begin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    // Core 3.x attaches the pin and picks a channel itself.
    _ready = ledcAttachChannel(PIN_ESC, DC_ESC_LEDC_HZ, DC_ESC_LEDC_BITS, DC_ESC_LEDC_CH);
#else
    ledcSetup(DC_ESC_LEDC_CH, DC_ESC_LEDC_HZ, DC_ESC_LEDC_BITS);
    ledcAttachPin(PIN_ESC, DC_ESC_LEDC_CH);
    _ready = true;
#endif
    _cur = NEUTRAL;
    writeMicros(NEUTRAL);
    // Hold clean neutral so the ESC arms. Do this before the audio task starts,
    // or the arming beeps fight the render for the I2S DMA.
    delay(1200);
}

void Outputs::drive(float fwdFrac, float revFrac, bool reverse, float brake01) {
    float target;
    if (reverse) {
        target = NEUTRAL - (FWD_FLOOR + revFrac * REV_SPAN);    // below neutral = reverse
    }
    else if (fwdFrac > 0.002f) {
        target = NEUTRAL + FWD_FLOOR + fwdFrac * FWD_SPAN;
    }
    else {
        target = NEUTRAL;
    }

    // Slew-limit so current spikes don't sag the rail. On this board that matters
    // more than it did on the Teensy one: the pack feeds two bucks through a
    // single P-FET, and the audio rail is one of them. Braking only speeds up
    // the ramp DOWN; it never changes where the ramp ends.
    const float up   = SLEW_US;
    const float down = (brake01 > 0.01f) ? BRAKE_SLEW_US : SLEW_US;
    if (_cur < target - up)          _cur += up;
    else if (_cur > target + down)   _cur -= down;
    else                             _cur = target;

    // Hard floor: outside reverse gear the ESC is NEVER sent below neutral,
    // whatever the caller or a ramp coming out of reverse asks for.
    if (!reverse && target >= NEUTRAL && _cur < NEUTRAL) _cur = NEUTRAL;
    if (_cur < 1000.0f) _cur = 1000.0f;
    if (_cur > 2000.0f) _cur = 2000.0f;
    writeMicros(_cur);
}

void Outputs::lights(bool, bool, bool) {
    // Every light is on the PCA9685 light engine. Kept as a no-op so callers
    // ported from the Teensy build still link.
}
