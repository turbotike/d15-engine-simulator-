// =============================================================================
//  AudioOut.h - I2S output stage + the render task
// =============================================================================
//  This replaces the Teensy Audio Library entirely. On that board, AudioStream
//  called update() from an interrupt every 128 samples and the library owned the
//  DMA. Here we own it: one task, pinned to a core, filling a block and handing
//  it to the I2S driver, which blocks until there is DMA room. That is the whole
//  scheduler - no interrupt context, so the renderer is allowed to touch PSRAM
//  and do float maths freely, which the Teensy version could not.
//
//  Both MAX98357As sit on this one bus. Which channel each plays is fixed in
//  HARDWARE by its SD_MODE resistor (U4/R24 220k = LEFT, U9/R25 1M = RIGHT), so
//  we just render stereo and the board splits it.
//
//  STATUS: the renderer below is a PLACEHOLDER. It is a synthesised diesel -
//  a firing-order pulse train with harmonics and a little noise - driven by the
//  sim's rpm and load. It exists so the board makes a recognisable engine noise
//  the day it arrives, and so the I2S/task/DMA plumbing is proven before the
//  real engine lands on top of it. The sampled engine (the port of the Teensy
//  EngineAudio: rpm-crossfaded coast/load exhaust ladders, lope idle handoff,
//  synth turbo, jake ladder, rev-limiter chop) replaces render() and nothing
//  else in this file has to change.
// =============================================================================
#pragma once
#include <Arduino.h>

class AudioOut {
public:
    bool begin();
    void stop();
    bool running() const { return _task != nullptr; }

    // Called from the main loop each tick. These are plain floats written by one
    // task and read by another: a 32-bit aligned float store is atomic on this
    // core, and a render block landing one tick stale is inaudible, so no lock.
    void setState(float rpm, float load01, float boostPsi, bool jake, bool running);

    // These reach whichever renderer is live, so main.cpp does not have to know
    // whether a pack loaded.
    void setMasterVol(float v);
    void setHorn(bool on);
    void setBackup(bool on);

    // diagnostics for the serial heartbeat
    float peak()      { float p = _peak; _peak = 0; return p; }
    uint32_t blocks() const { return _blocks; }
    uint32_t underruns() const { return _under; }
    // Fraction of real time the renderer is using, 0..1+. Above 1.0 means it
    // cannot keep up and you will hear it. Worth watching: the cubic resampler
    // reads four taps per sample per layer out of PSRAM, which has far worse
    // random-access latency than the internal RAM the Teensy build read from.
    // Playback does advance monotonically through each loop, so the access is
    // effectively a linear scan and should sit inside one cache line for ~16
    // samples at a time - but that is reasoning, not a measurement, so here is
    // the measurement.
    float cpuFrac() const { return _cpu; }

private:
    static void taskTrampoline(void *arg);
    void taskLoop();
    void render(int16_t *out, size_t frames);   // <- swap for the sampled engine

    TaskHandle_t _task = nullptr;
    volatile bool _stop = false;

    // state published by the main loop
    volatile float _rpm = 0, _load = 0, _boost = 0;
    volatile bool  _jake = false, _run = false;
    volatile bool  _horn = false, _backup = false;
    float _vol = 0.92f;

    // render state
    float _firePh = 0, _rpmS = 0, _loadS = 0, _boostS = 0;
    float _hornPh = 0, _beepPh = 0;
    uint32_t _beepCtr = 0;
    uint32_t _rng = 0x2468ace;
    volatile float _peak = 0;
    volatile float _cpu = 0;
    volatile uint32_t _blocks = 0, _under = 0;
    uint32_t _t0 = 0, _blocks0 = 0;   // wall-clock underrun detector

    inline float frand() {
        _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
        return (int32_t)_rng * (1.0f / 2147483648.0f);
    }
};

extern AudioOut Audio;
