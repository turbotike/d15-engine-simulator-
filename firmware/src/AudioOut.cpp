// =============================================================================
//  AudioOut.cpp
// =============================================================================
#include "AudioOut.h"
#include "Config.h"
#include "SoundPack.h"
#include "EngineAudio.h"
#include <driver/i2s.h>

AudioOut Audio;

static const i2s_port_t PORT = I2S_NUM_0;

bool AudioOut::begin() {
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = (uint32_t)DC_OUT_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    // Eight buffers of 256 frames at 44.1k is ~46ms of slack. Generous, but the
    // main loop does SD reads and NVS commits, and an audio dropout is far more
    // noticeable than 46ms of control latency.
    cfg.dma_buf_count        = 8;
    cfg.dma_buf_len          = DC_AUDIO_FRAMES;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;   // emit silence on underrun, not the last
                                       // block over and over - that buzz was a
                                       // known failure mode on the Teensy board

    if (i2s_driver_install(PORT, &cfg, 0, NULL) != ESP_OK) return false;

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = I2S_PIN_NO_CHANGE;
    pins.bck_io_num   = PIN_I2S_BCLK;
    pins.ws_io_num    = PIN_I2S_LRCK;
    pins.data_out_num = PIN_I2S_DOUT;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;
    if (i2s_set_pin(PORT, &pins) != ESP_OK) { i2s_driver_uninstall(PORT); return false; }

    _stop = false;
    // Core 0 by default. The Arduino loop runs on core 1, and so does WiFi when
    // it is up - keeping the renderer off that core is what stops a scan or a
    // reconnect from chopping the exhaust.
    BaseType_t ok = xTaskCreatePinnedToCore(taskTrampoline, "dcaudio", 8192, this,
                                            configMAX_PRIORITIES - 3, &_task,
                                            DC_AUDIO_CORE);
    if (ok != pdPASS) { _task = nullptr; i2s_driver_uninstall(PORT); return false; }
    return true;
}

void AudioOut::stop() {
    if (!_task) return;
    _stop = true;
    // let the task notice and leave its i2s_write
    vTaskDelay(pdMS_TO_TICKS(50));
    _task = nullptr;
    i2s_driver_uninstall(PORT);
}

void AudioOut::setState(float rpm, float load01, float boostPsi, bool jake, bool run) {
    _rpm = rpm; _load = load01; _boost = boostPsi; _jake = jake; _run = run;
}

void AudioOut::setMasterVol(float v) {
    _vol = constrain(v, 0.0f, 1.0f);
    EngAudio.masterVol = _vol;
}

void AudioOut::setHorn(bool on) {
    _horn = on;                  // synth fallback
    EngAudio.setHorn(on);        // sampled engine
}

void AudioOut::setBackup(bool on) {
    _backup = on;
    EngAudio.setBackup(on);
}

void AudioOut::taskTrampoline(void *arg) { ((AudioOut *)arg)->taskLoop(); }

void AudioOut::taskLoop() {
    static int16_t buf[DC_AUDIO_FRAMES * 2];
    while (!_stop) {
        // With a pack resident in PSRAM, the sampled engine renders. Without one
        // - no card, or a card with no pack on it - fall back to the synth rather
        // than to silence, so the truck still makes a noise and you can still
        // tell the board is alive.
        uint32_t t0 = micros();
        if (Pack.loaded()) {
            EngAudio.render(buf, DC_AUDIO_FRAMES);
            float p = EngAudio.takePeak();
            if (p > _peak) _peak = p;
        } else {
            render(buf, DC_AUDIO_FRAMES);
        }
        // One block is DC_AUDIO_FRAMES/DC_OUT_RATE of real time. Smoothed so the
        // heartbeat shows a trend rather than one unlucky block.
        float budget = (float)DC_AUDIO_FRAMES * 1e6f / DC_OUT_RATE;
        _cpu += (((float)(micros() - t0) / budget) - _cpu) * 0.05f;

        size_t wrote = 0;
        esp_err_t e = i2s_write(PORT, buf, sizeof(buf), &wrote, pdMS_TO_TICKS(200));
        if (e != ESP_OK || wrote != sizeof(buf)) _under++;   // bus died, not an underrun
        _blocks++;
        // NEVER STARVE THE WATCHDOG. If the render runs over real time, i2s_write
        // never blocks, this task never sleeps, IDLE0 never runs and the task
        // watchdog REBOOTS the truck (measured twice on 2026-10-30). An overrun
        // block now costs a dropout, never a reboot.
        if ((float)(micros() - t0) > budget * 0.97f) vTaskDelay(1);

        // A REAL underrun is the DMA running dry because the producer was late,
        // and i2s_write returns ESP_OK for that - immediately and happily. The
        // check above only fires if the write TIMED OUT, i.e. the consumer
        // stopped, so `xrun` read zero through every genuine dropout: useless
        // for the one diagnosis it exists for. Compare wall-clock against the
        // audio we should have produced instead.
        uint32_t nowMs = millis();
        if (_t0 == 0) { _t0 = nowMs; _blocks0 = _blocks; }
        else if (nowMs - _t0 >= 2000) {
            uint32_t produced = _blocks - _blocks0;
            float expect = (float)(nowMs - _t0) * DC_OUT_RATE / (1000.0f * DC_AUDIO_FRAMES);
            // more than one block behind over two seconds = the DMA went dry
            if ((float)produced < expect - 1.5f) _under++;
            _t0 = nowMs; _blocks0 = _blocks;
        }
    }
    vTaskDelete(NULL);
}

// =============================================================================
//  PLACEHOLDER RENDERER - see the note in AudioOut.h
// =============================================================================
//  A diesel is a pulse train: one combustion event per cylinder per two
//  revolutions. For a six, that is rpm/20 Hz. Everything below is built around
//  that pulse - harmonics for the body, a short noise burst per event for the
//  clatter, and a load-dependent growl. It is not the sampled engine and it is
//  not meant to be; it is enough to prove the chain and to tell you at a glance
//  whether rpm is reaching the audio side.
// =============================================================================
void AudioOut::render(int16_t *out, size_t frames) {
    const float sr = DC_OUT_RATE;
    const float dt = 1.0f / sr;

    for (size_t n = 0; n < frames; n++) {
        // smooth the control inputs - they update at loop rate, not sample rate
        _rpmS   += (_rpm   - _rpmS)   * 0.0008f;
        _loadS  += (_load  - _loadS)  * 0.0005f;
        _boostS += (_boost - _boostS) * 0.0003f;

        float s = 0.0f;
        if (_run && _rpmS > 50.0f) {
            float fireHz = _rpmS / 20.0f;              // six-cylinder four-stroke
            _firePh += fireHz * dt;
            if (_firePh >= 1.0f) _firePh -= 1.0f;

            // body: the firing fundamental plus two harmonics
            float p = _firePh * 2.0f * (float)PI;
            s  = sinf(p)        * 0.55f;
            s += sinf(p * 2.0f) * 0.28f;
            s += sinf(p * 3.0f) * 0.14f;

            // combustion clatter: a decaying noise burst at the start of each event
            float burst = expf(-_firePh * 26.0f);
            s += frand() * burst * (0.18f + 0.30f * _loadS);

            // turbo whistle, pitch off the shaft, level off boost
            float whine = sinf(_firePh * 2.0f * (float)PI * 11.0f);
            s += whine * 0.05f * constrain(_boostS / 25.0f, 0.0f, 1.0f);

            // jake brake: harsher, louder, and drops the body
            if (_jake) { s = s * 0.75f + frand() * 0.22f; }

            // grunt with load
            s *= 0.35f + 0.55f * _loadS;
        }

        if (_horn) {
            _hornPh += 320.0f * dt;
            if (_hornPh >= 1.0f) _hornPh -= 1.0f;
            float h = sinf(_hornPh * 2.0f * (float)PI)
                    + 0.6f * sinf(_hornPh * 2.0f * (float)PI * 1.5f);
            s += h * 0.28f;
        }

        if (_backup) {                                  // reverse beeper, 1Hz 50%
            _beepCtr++;
            if (_beepCtr >= (uint32_t)sr) _beepCtr = 0;
            if (_beepCtr < (uint32_t)(sr * 0.5f)) {
                _beepPh += 1000.0f * dt;
                if (_beepPh >= 1.0f) _beepPh -= 1.0f;
                s += (_beepPh < 0.5f ? 0.22f : -0.22f);
            }
        }

        s *= _vol;
        // soft clip. tanh rather than a hard limit: the MAX98357A's full scale IS
        // digital full scale here, and a hard clip on a class-D sounds like
        // crackle, which is the exact fault that dogged the Teensy board.
        s = tanhf(s * 1.3f);

        float a = fabsf(s);
        if (a > _peak) _peak = a;

        int16_t v = (int16_t)(s * 30000.0f);
        out[2 * n + 0] = v;        // LEFT  -> U4 / SPK1
        out[2 * n + 1] = v;        // RIGHT -> U9 / SPK2
    }
}
