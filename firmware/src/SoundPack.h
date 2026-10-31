// =============================================================================
//  SoundPack.h - loads an SD sound pack resident into PSRAM
// =============================================================================
//  This is where the S3 board diverges most from the Teensy build, and entirely
//  in our favour.
//
//  On a Teensy 4.0 there are 488 KB of usable RAM, so the pack could not be held
//  and the ladder had to be STREAMED off the card while playing - which meant
//  SdLoop, two voices per ladder, per-rung handover, a shared file handle, and
//  four separate failure modes around opens that starve the other voices. All of
//  that machinery exists to work around a memory limit.
//
//  This board has 8 MB of octal PSRAM and a 506 KB pack. So: read the whole
//  thing into PSRAM once at boot, index it directly, and never touch the card
//  again while running. SdLoop is gone. There are no voices, no handover, and no
//  underruns to count.
//
//  The flash-baked C15 pack is also gone. c15_ats.h is 3.7 MB of const arrays
//  and the app partition is 3 MB, so it could never fit - and with PSRAM there
//  is no reason to want it. No card means no samples, and AudioOut falls back to
//  its synthesised engine rather than silence.
// =============================================================================
#pragma once
#include <Arduino.h>

// One rpm-labelled loop (or one-shot). PCM is 16-bit mono at the pack rate; the
// render pitches it live.
struct Layer {
    int16_t *data  = nullptr;   // left (or mono)
    int16_t *dataR = nullptr;   // right (null = mono, render duplicates L)
    uint32_t len   = 0;         // frames
    float    rate  = 44100.0f;  // native sample rate of this clip
    float    rpm   = 0;         // rpm this loop was recorded at (0 = one-shot)
    // FIRING-PULSE GRID (loops only, found at load): a pulse every pulsePer
    // samples, the first peak at pulseOff, pulsePpl pulses per loop.
    float    pulsePer = 0, pulseOff = 0; int pulsePpl = 0;
    bool ok()     const { return data && len > 8; }
    bool stereo() const { return dataR != nullptr; }
};

#define DC_MAX_LAYERS 16        // per role

class SoundPack {
public:
    // Reads /packs/<name>/ into PSRAM - manifest.txt + WAVs for the full pack,
    // falling back to dieselcore.pak. SD_MMC must already be mounted. Pass
    // nullptr for the default pack name.
    bool loadFromSD(const char *packName = nullptr);
    void release();

    const char *name() const { return _name; }
    bool loaded() const { return _loaded; }
    uint32_t bytesResident() const { return _bytes; }

    // Roles, sorted by rpm ascending.
    Layer idle;                                   // aliases eng[0]
    Layer horn, air;                              // action one-shots
    Layer turbo, lope;
    Layer eng  [DC_MAX_LAYERS]; int nEng   = 0;   // mechanical engine ladder
    Layer coast[DC_MAX_LAYERS]; int nCoast = 0;   // off-throttle exhaust ladder
    Layer pull [DC_MAX_LAYERS]; int nPull  = 0;   // under-load exhaust ladder
    Layer jake [DC_MAX_LAYERS]; int nJake  = 0;

    // ---- action one-shots ------------------------------------------------
    // Separate named members rather than an array: each is fired by a different
    // named event and nothing ever iterates them. All PSRAM-resident like every
    // other layer - the Teensy streamed these off the card only because a 4.0
    // has 488 KB, which is not a constraint here.
    Layer shStart, shStop, shBov;          // engine events
    // Three-part START, matching the browser's startSequence():
    //   shStarter  starter motor engaging   (1.28 s)
    //   shCrank    cranking, LOOPED while it churns (0.66 s)
    //   shFireup   the catch / light-off     (1.70 s)
    // start.wav stays as the fallback for packs that only ship one file.
    Layer shStarter, shCrank, shFireup;
    Layer shTick[3]; int nTick = 0;        // cooling ticks, round-robin
    Layer shAirDry, shPbOn, shPbOff;       // air system
    // optional, by file name: hydraulic pump + relief loops for WORK mode, and
    // the clip played when the battery protection cuts in. A build with them
    // baked in (HydSamples.h / DangerClip.h present) uses the card's copy first.
    Layer hydPump, hydFlow, shDanger;

private:
    bool loadManifest(const char *dir);
    bool loadPak(const char *dir);
    bool loadWav(const char *dir, const char *file, Layer &L, float rpm);
    int16_t *psAlloc(uint32_t bytes);
    void     psFree(int16_t *p);

    char  _name[48] = {0};
    bool  _loaded = false;
    uint32_t _bytes = 0;
    // One allocation per clip. ~15 of them in 8 MB does not meaningfully
    // fragment PSRAM, and it means a single missing file just skips that rung
    // instead of failing the whole pack.
    // 128, not 64: a stereo pack is two allocations per clip, and ~30 clips
    // plus any failed attempts was close enough to the old ceiling to matter.
    int16_t *_allocs[128] = {nullptr};
    int      _nAlloc = 0;
};

extern SoundPack Pack;
