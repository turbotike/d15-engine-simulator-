// =============================================================================
//  EngineAudio.h - renders the engine from the PSRAM-resident SoundPack and the
//  EngineSim state. rpm-crossfaded coast/loaded exhaust, lope idle handoff,
//  synth turbo, jake ladder, diesel pop, rev-limiter chop.
// =============================================================================
//  Ported from the Teensy build. Two structural changes, both because of the
//  hardware:
//
//  1. NOT an AudioStream. There it was driven from the audio interrupt every 128
//     samples, which is why it could never touch the card and had to publish
//     "rungs I want" for the main loop to service. Here it is a plain renderer
//     called from a task (see AudioOut), so it may block, touch PSRAM and do
//     float maths freely.
//
//  2. NO STREAMING. SdLoop, the two voices per ladder, the per-rung handover and
//     the underrun counters are all gone. The pack is resident in PSRAM, so a
//     rung change is a pointer, not a seek.
//
//  The DSP itself - the cubic resampler, the rung picker, the resonators, the
//  chop and limiter ramps - is unchanged, including the comments explaining why
//  each one is the way it is. Those were paid for in measurement.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "Config.h"
#include "SoundPack.h"

// Playback position, split into whole samples plus a fraction in [0,1).
//
// A single float cannot do this job. Counting up to ~55000 samples puts it in a
// binade where the smallest representable step is about 0.004, so the fraction
// the interpolator sees has only ~256 distinct values, and adding the pitch
// increment SNAPS to that grid - so the increment actually applied wobbles by
// roughly 0.2% from sample to sample. That is phase jitter, not noise, and
// jitter smears the harmonics of something as tonal as a diesel.
struct Pos { uint32_t i = 0; float f = 0.0f; };

// Was 4, which is fewer than the number of things that can legitimately sound
// at once: start/stop, a cooling tick, the air brake, the park brake, the horn
// and the blow-off. When all four were busy playShot() STOLE the voice furthest
// through its clip - and on shutdown that was often stop.wav itself, because the
// shutdown path deliberately schedules a tick 1.6 s into it. Six voices means
// every slot can sound without stealing. Each voice is a pointer, a position and
// a few floats, so this costs tens of bytes.
#define DC_SHOT_VOICES 6

// One PSRAM-resident action one-shot in flight. The Teensy streamed these
// through SdLoop because a 4.0 has 488 KB; the pack is resident here, so a
// voice is just a pointer, a position and a gain - no ring, no refill, no
// underrun counter, and polyphony costs nothing.
struct ShotVoice {
    const Layer *buf = nullptr;   // nullptr = free
    Pos      pos;
    float    gain = 1.0f;
    float    env  = 1.0f;         // release ramp - see the mix block
    bool     rel  = false;
    uint8_t  slot = 0;
    // LOOP: wrap at the end instead of finishing. Needed for the cranking
    // sample, which has to churn for as long as the starter is turning - the
    // browser does this with crankLoopSrc.loop = true.
    bool     loop = false;
    bool active() const { return buf != nullptr; }
};

// One sound per slot may sound at a time: a retrigger steals its OWN slot, so
// holding the horn cannot stack blasts into a wall of gain, but never steals a
// different sound. The Teensy's single _vShot had no such protection - a horn
// stab killed a 5.6 s park-brake release dead.
// One slot per INDEPENDENT sound. playShot() releases anything already sounding
// in the same slot, which is what stops a held button stacking copies of itself -
// but it means two UNRELATED sounds must never share a slot.
//
// DC_SHOT_ENG used to carry start, stop AND the blow-off valve. The engine coasts
// down through a let-off on shutdown, which raises a BOV event, which then
// released stop.wav a few ms after it started - so the shutdown sound never
// played. Startup had the same failure any time a BOV landed during cranking.
// start and stop are also mutually exclusive by nature, so they can share.
enum : uint8_t { DC_SHOT_ENG = 0,     // start.wav / stop.wav (never both at once)
                 DC_SHOT_TICK, DC_SHOT_AIR, DC_SHOT_PARK,
                 DC_SHOT_HORN,
                 DC_SHOT_BOV,         // its own slot: must not cut start or stop
                 DC_SHOT_CRANK,       // the LOOPED cranking sample
                 // The FIREUP needs its own slot. It shared DC_SHOT_ENG with
                 // the starter, and playShot() releases anything sounding in
                 // the same slot - so a quick catch fired the fireup while the
                 // 1.28 s starter was still playing and chopped it off
                 // mid-sample. That is the weird noise on startup.
                 DC_SHOT_FIRE,
                 DC_SHOT_NSLOT };

// THE PULSE LOPE + LIMITER POPS (Mat 2026-10-30, after the synth chug: "it
// sounds horrible, there's got to be a better way"). Nothing synthesised: the
// loops are trains of real firing pulses, and these lift / drop / fatten whole
// pulses in time with the engine. Shaped on tools/lope_designer.html; Mat's
// settings are the defaults. Big-plunger injectors only, whole rev range.
// Serial: '~key=val ...', '~get=1', '~save=1' (NVS), '~reset=1'.
struct LopeParams {
    float chug   = 5.0f;   // CHUG: lope cylinder's pulse x this
    float chu    = 4.0f;   // chu (small): x this
    float miss   = 1.0f;   // x (dead hole): pulse drops by this (1 = gone)
    float width  = 1.07f;  // how much of a firing interval is 'the pulse'
    float len    = 4.0f;   // CHUG spans up to this many pulses (random 2..len)
    float tailK  = 1.0f;   // strength of the 2nd/3rd/4th pulse of a CHUG
    float bass   = 4.0f;   // extra low end on lifted pulses
    float rest   = 0.15f;  // other pulses of a CHUG cycle pulled down by this
    float kick   = 50.0f;  // rpm (pitch) kick on a CHUG
    float kickMs = 180.0f; // kick fade
    float life   = 0.5f;   // chance a step is swapped (on top of the random walk)
    float eng    = 1.5f;   // engine layer level while loping
    float exh    = 1.5f;   // exhaust layer level while loping
    float limCut = 0.55f;  // limiter: chance a pulse is cut (more gaps = more pops between)
    float limPop = 4.5f;   // limiter: a pulse that fires comes in this fat (was 3 - "not popping enough")
    float limBass= 3.0f;   // limiter: bass on the pops (was 2)
    float body   = 0.4f;   // the WHOLE engine sound follows the rhythm: up this much on a CHUG cycle
    float bodyMs = 120.0f; // ...and settles back over this
    char  pattern[17] = "BmBmBmmssBmss";  // the feel: B CHUG, s chu, x miss, m normal
};
extern LopeParams Lope;
bool lopeSet(const char *key, const char *val);   // false = unknown key
void lopePrint();
void lopeSave();
void lopeLoad();

class EngineAudio {
public:
    // Renders `frames` interleaved stereo int16 frames. Called from the audio
    // task only.
    void render(int16_t *out, size_t frames);

    // ---- live tunables (mirror the shipped truck sound) ----
    float revPitch    = 1.0f;
    // Master for ALL THREE turbo voices - the turbo.wav sample, the
    // synthesised whistle and the whoosh. 0.62 -> 0.48 is about -2.2 dB,
    // enough to seat the turbo behind the engine instead of on top of it
    // without losing the spool.
    float turboLevel  = 0.48f;
    // Pre-softclip drive. On this board the amp cannot clip before the tanh does
    // - digital full scale IS the MAX98357A's full scale, there is no analog
    // stage to misjudge - so this buys loudness through compression only.
    // Note dm = makeup * grunt, and grunt already rises to ~2.4x with load and
    // coolant temp, so this is a trim on top of an automatic drive.
    // 1.6 came from the Teensy board, where it was SAFE because an analog
    // attenuator sat after it: lineOutLevel=26 was chosen so digital full scale
    // landed exactly on the PAM8403's clip point, and its own note says "the
    // only thing that can limit is the tanh, which is smooth".
    //
    // This board has no such stage. The MAX98357A is digital - full-scale PCM IS
    // full output. Porting 1.6 without the attenuator meant `dm` reached 2.9
    // under load (grunt alone rises to 1.99x, 2.43x with hot injectors), driving
    // ~1.8x into the tanh and flattening peaks at 0.94 of full scale. That held
    // near-square top is what crackles on a class-D, and it only appears at high
    // rpm because `grunt` tracks load - which is exactly the symptom reported.
    //
    // 0.9 keeps the loudest case just under the knee, so the tanh shapes peaks
    // instead of parking on them. Raise it from the phone (`audioDrive`) if you
    // want more compression; that control is now safe in the way the Teensy's
    // was, because nothing downstream can hard-clip before the tanh does.
    // The Teensy ran 1.6 and drove dm up to 3.89 under load - that density IS
    // part of the shipped sound, not an accident. 0.9 was set while masterVol
    // was still wrongly inside dm, so it was correcting for a drive that was
    // itself inflated. With volume moved out, 1.4 puts the hot case at
    // tanh(0.602*3.40)=0.966, within 0.15 dB of the Teensy, with the tanh
    // shaping peaks instead of parking on them. Do not go below ~1.2: that
    // starves the amp, which is what made it grainy in the first place.
    float makeup      = 1.4f;   // FIXED - see WebUI.cpp, no longer a slider
    // 0.50 was HALF OF A POWER SUM that no longer has its other half.
    // engG computes sqrt(engLevel^2 + w0^2), which made sense when idle was a
    // separate second stream. It is not: idleG is forced to 0, and above
    // 1080 rpm w0 is 0 too - so the sqrt collapses to engLevel alone and the
    // engine ladder just loses 6 dB with nothing compensating. Measured as the
    // second-largest loss in the whole chain.
    float engLevel    = 1.00f;  // FIXED - was the orphaned power-sum leg at 0.50
    float fanLevel    = 0.0f;     // fan-clutch roar: firmware-only, no browser
                                  // counterpart, so it stays off
    // THESE TWO WERE ZERO, which multiplied two entire browser layers out of
    // existence: the induction whoosh and the diesel knock/clatter. Both are
    // always live in the browser (turboWhooshG and knockGain are assigned every
    // frame in updateMix) and they are two of its three turbo/clatter layers.
    // Nothing downstream could compensate - the gain was multiplied by 0.
    float whooshLevel = 1.0f;     // broadband induction air
    float knockLevel  = 1.0f;     // diesel clatter
    float coastQuiet  = 1.0f, loadHot = 1.0f;
    // Low-air warning buzzer. FIXED. 0.20 was loud enough to eat real headroom
    // from the engine while it sounded - and since it only sounds below 55 psi,
    // that is exactly when you want to still hear the truck. It is a warning,
    // so it stays deliberately annoying, just not dominant.
    float lowAirLevel = 0.10f;
    float hornLevel   = 1.0f;   // scales horn sample or synth reeds
    float beepLevel   = 1.0f;   // reverse beeper
    float hydLevel    = 1.0f;   // hydraulic pump whine + relief squeal (web: volHydPump)
    // Speaker-protection high-pass. FIXED, not a slider - there is one right
    // answer for a given speaker and exposing it only invited mis-setting.
    //
    // Measured on this pack: at idle (exh650) **45% of the exhaust energy sits
    // below 80 Hz** and 28% more between 80-150. A small RC cone cannot move
    // that much air, so all of it becomes mechanical distortion that gets WORSE
    // the louder you drive it. At 1800 rpm only 0.5% is below 80 Hz, so cutting
    // there costs nothing at speed.
    //
    // 120 Hz removes the part that can only distort, keeps the 80-150 band that
    // gives idle its body, and is inaudible up top. Raise it toward 170 for a
    // very small speaker; drop it to ~80 if you ever fit a real one.
    // Speaker-protection high-pass. WAS 120 Hz, which fought the exhaust low
    // shelf directly: the shelf exists to add +5 dB below 130 Hz and this was
    // removing that exact band, so the board sounded thin next to the browser
    // no matter how the ladders were balanced. The browser has NO high-pass at
    // all - its only master filter is a 20 kHz lowpass, which is transparent.
    //
    // Not deleted, because it is real speaker protection: a small driver cannot
    // produce 60 Hz and trying only costs excursion and distortion. Moved to
    // 55 Hz, below the shelf corner, so it still catches subsonic content
    // without eating the chest of the sound.
    float hpHz        = 55.0f;
    float masterVol   = DC_MASTER_VOL;

    // outPeak is the signal BEFORE the drive and the tanh - useful for seeing
    // what the render produces, useless for judging whether the limiter is
    // being abused. These two answer the question that actually matters:
    //   drivePeak = |oL * dm|  - how hard the tanh is being driven. Above 1.0
    //               it is compressing; much above ~1.5 it is flattening peaks,
    //               which on a class-D reads as crackle.
    //   clipFrac  = fraction of samples driven past 1.0 in the last window.
    volatile float outPeak = 0;
    volatile float drivePeak = 0;
    // Smoothed. This was computed per 256-frame block = 5.8 ms, so clipping on
    // 2% of samples averaged over a second read 0% on most polls and 30% on the
    // unlucky one - a number that flickers and invites the wrong conclusion.
    volatile float clipFrac = 0;
    float takePeak()  { float p = outPeak;   outPeak = 0;   return p; }
    // peekDrive() does NOT clear. takeDrive() is destructive and BOTH the
    // serial heartbeat and the phone's /status poll were calling it, so
    // whichever ran first won and the heartbeat's `drive` read near zero
    // whenever a phone was connected - while `drive` is the number documented
    // as the one to tune the audio by. That actively misled debugging.
    float peekDrive() const { return drivePeak; }
    float takeDrive() { float p = drivePeak; drivePeak = 0; return p; }
    float clipping() const { return clipFrac; }
    float chopDepthDbg() const { return chopDbg; }
    // start-sequence probe (bench Q): peak of the one-shot voices since the last read,
    // and each live voice as slot:position-in-clip
    float takeShotPeak() { float p = shotPeakDbg; shotPeakDbg = 0; return p; }
    int   voiceDbg(char *b, int n) const { int k = 0; b[0] = 0;
        for (int i = 0; i < DC_SHOT_VOICES; i++) { const Layer *L = _sv[i].buf; if (!L || !L->len) continue;
            k += snprintf(b + k, n - k, " %u:%.2f%s", _sv[i].slot, (float)_sv[i].pos.i / L->len, _sv[i].rel ? "r" : ""); if (k >= n) break; }
        return k; }
    // Envelope measurement - takeChopStats clears so each read covers the
    // interval since the last one.
    void takeChopStats(float &mn, float &mx, uint32_t &hits) {
        mn = (chopMinDbg > 8.0f) ? 1.0f : chopMinDbg;
        mx = chopMaxDbg; hits = chopHitsDbg;
        chopMinDbg = 9.0f; chopMaxDbg = 0.0f; chopHitsDbg = 0;
    }

    // ---- action sounds ----
    // HORN - matched to the poor-mans-beier firmware, which is the reference.
    //
    // What beier does (DrivingBrain.cpp:1070, AudioEngine.cpp:224/294):
    //   press   -> playOneShot() from sample 0, envCurrent = 1.0 INSTANTLY,
    //              then loop = true so it sounds for as long as you hold it
    //   release -> fadeOutAndStop(voice, cfHornOut) = a 600 ms linear ramp,
    //              and the sample KEEPS PLAYING through that ramp
    //
    // My previous version got two of the three wrong: it latched at the end of
    // the clip instead of looping, and it FROZE the playback position on
    // release instead of letting the horn keep sounding while it faded. A real
    // air horn blows for as long as the valve is open and then bleeds down -
    // it does not stop dead partway through, and it does not run out.
    //
    // Attack is instant, NOT ramped. beier sets envCurrent = 1.0 directly and
    // never uses cfHornIn on this path; a horn valve opening really is a step.
    void setHorn(bool on) {
        if (on && !hornOn) { hornPos.i = 0; hornPos.f = 0.0f; hornGate = 1.0f; }
        hornOn = on;
    }
    void setBackup(bool on) { backupOn = on; }
    void triggerAir()       { if (Pack.air.ok()) { aoBuf = &Pack.air; aoPos = Pos(); aoActive = true; aoGain = 0.9f; } }
    // on = PTO engaged, demand01 = how hard the sticks are pushed, relief01 = dumping over the relief valve
    void setHydraulics(bool on, float demand01, float relief01) { hydOn = on; hydLoad = demand01; hydPress = relief01; }

    // Fire a one-shot. Safe from the main loop: it writes the position and gain
    // BEFORE the buffer pointer, so the render task on the other core can never
    // see a live voice with a stale position.
    void playShot(const Layer *clip, float gain, uint8_t slot, bool loop = false);
    // Release every voice in a slot - used to end the looped crank on the catch.
    void stopSlot(uint8_t slot);
    bool shotBusy() const;

    // battery protection cut in: play Mat's clip once (stopDanger cuts it short)
    void playDanger() { dangerReq = true; }
    void stopDanger() { dangerStopReq = true; }
    bool dangerPlaying() const { return dangerOn; }
private:
    Pos pEng[DC_MAX_LAYERS];
    Pos pIdle, pCoast[DC_MAX_LAYERS], pLoad[DC_MAX_LAYERS], pJake[DC_MAX_LAYERS];
    Pos pTurbo, pLope;
    float knA1 = 0, knA2 = 0, knA1R = 0, knA2R = 0, knPh = 0;
    float loopsLevel = 0, workLvl = 0, boost = 0, wander = 0, pitchWob = 1;
    float firePhase = 0, fireAmp = 1;
    float flP = 0;
    float yA1 = 0, yA2 = 0, yB1 = 0, yB2 = 0, whooshLP = 0;
    float yA1R = 0, yA2R = 0, yB1R = 0, yB2R = 0, whooshLPR = 0;
    float fanG = 0, fanLP = 0, fanLPR = 0;
    float limPh = 0, limEnv = 0; uint32_t limSeed = 771;
    float chopPh = 0, chopDur = 0, chopAmp = 0;
    uint32_t lastEvSeq = 0; bool chopBarkNow = false;   // lope events read from EngineSim
    // pulse lope + limiter pops (EngineAudio.cpp "THE PULSE LOPE")
    int      pulseMaster = -1, pulseLastK = 0;
    uint32_t pulseCnt = 0, pulseRng = 0;
    char     pulseStep = 'm';
    float    pulseSlotG[6] = {1,1,1,1,1,1}, pulseSlotB[6] = {0,0,0,0,0,0};
    float    pulseCurG = 1, pulseCurB = 0, pulseKick = 0, plE = 0, plX = 0;
    float    pulseBody = 1, pulseBodyTgt = 1;   // the engine behind the chug, swelling with the rhythm
    float    pulseRnd() { uint32_t r = pulseRng; r ^= r << 13; r ^= r >> 17; r ^= r << 5; pulseRng = r; return (r >> 8) * (1.0f / 16777216.0f); }
    char     pulseNextStep();
    // "Shake Hands with Danger" (DangerClip.h): 16 kHz IMA ADPCM, decoded a block at a time
    volatile bool dangerReq = false, dangerStopReq = false;
    bool     dangerOn = false;
    uint32_t dangerPos = 0;        // next ADPCM sample to decode
    int32_t  dangerPred = 0; int dangerIdx = 0;
    float    dangerPh = 0, dangerA = 0, dangerB = 0;   // resampler: previous / next decoded sample
    int16_t  dangerNext();
    void     pulseNewCycle(float a);
    void     pulseNewPulse(float a, float lim);
    float chopG = 1.0f, limG = 1.0f, popG = 1.0f;
    float chopDbg = 0, shotPeakDbg = 0;
    float chopMinDbg = 9.0f, chopMaxDbg = 0.0f;
    uint32_t chopHitsDbg = 0;
    bool  chopInEvDbg = false;
    // Slow average of rpm, for the lope's pitch swing. The sim keeps this as
    // window._rAvg and uses the DEVIATION from it, not rpm itself, to decide how
    // far the exhaust pitch is thrown on each misfire.
    float rAvg = 0;
    // Exhaust low-shelf state (browser _exLS: lowshelf 130 Hz, +5 dB). One-pole
    // low band per channel; the shelf adds a scaled copy of it back.
    float lsL = 0, lsR = 0;
    // Compressor detector envelope (browser DynamicsCompressor, -6dB 3:1).
    float compEnv = 0;
    // Applied gain, ramped. Starts at 1.0 (not 0) so the first block is not
    // faded in from silence.
    float compG = 1.0f;
    // Headroom guard gain - rides the true peak of the summed buses so no
    // combination of layers can sit on the hard clamp. Starts at 1 (inert).
    float hdG = 1.0f;
    // Smoothed turbo whistle level - the compressor wheel's inertia. Without
    // this the whistle swung with instantaneous boost and audibly surged when
    // held at full throttle.
    float whnS = 0;
    // Rung SELECTION rpm. Frozen at the value a shift started from, so the
    // ladder does not hop across two rung boundaries while the revs slew to
    // the new gear. Pitch continues to follow the real rpm.
    float ladderRpm = 0, ladderHold = 0;
    bool  shiftPrevA = false;
    // Lagged boost, for the browser's _bChg gate on the sampled turbo.
    float bLag = 0;
    // Smoothed version of the boost-change gate that fades the turbo SAMPLE
    // in and out. Carried across render blocks like bLag, so it must live
    // here and not in renderBlock - a local would reset every block and the
    // slew would have no effect whatsoever.
    float bMoveS = 0;
    // Independent slow pitch wander PER RUNG. Without this the two crossfading
    // rungs keep a fixed rate ratio and realign every 3.5-5.5 s, which is the
    // audible 'looping'. See the note in render().
    float rWander[DC_MAX_LAYERS] = {0};
    // DC_MAX_LAYERS is 16, so a brace list of eight 1s would leave rungs 8-15
    // at ZERO and silently mute them on a pack with more rungs. Set in begin().
    float rWob[DC_MAX_LAYERS];
    bool  rWobInit = false;

    bool hornOn = false; Pos hornPos;
    // The synth horn needs its OWN phase. It used to borrow hornPos.f, which is
    // also the sampled horn's fractional playback position - so a synth blast
    // left hornPos.f mid-phase and a sampled horn would then start at a
    // fractional offset. hornGate ramps the level so release is not a square
    // edge; every other gain here is ramped for exactly that reason.
    float synHornPh = 0.0f, hornGate = 0.0f;
    // No hornDone latch any more: the horn LOOPS while held, like beier's
    // voice.loop = true, so there is no end-of-clip state to latch.
    const Layer *aoBuf = nullptr; Pos aoPos; float aoGain = 1; bool aoActive = false;
    bool backupOn = false; float beepPhase = 0; uint32_t beepCtr = 0;
    float laPh = 0, laPulse = 0, laGate = 0;
    float hpX1 = 0, hpX2 = 0, hpY1 = 0, hpY2 = 0;
    float hpX1R = 0, hpX2R = 0, hpY1R = 0, hpY2R = 0;
    float hpSet = -1, hb0 = 1, hb1 = 0, hb2 = 0, ha1 = 0, ha2 = 0;
    bool hydOn = false; float hydLoad = 0, hydPress = 0;
    float hydG = 0, hydPos = 0, relG = 0, relPos = 0;   // Cat 323 pump + flow loops (HydSamples.h)
    ShotVoice _sv[DC_SHOT_VOICES];
    uint32_t _clipHits = 0, _clipTotal = 0;
    uint32_t rng = 0x2468ace;
    inline float frand() {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        return (int32_t)rng * (1.0f / 2147483648.0f);
    }
};

extern EngineAudio EngAudio;
