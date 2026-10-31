// =============================================================================
//  EngineAudio.cpp - the engine render. Ported from the Teensy build; the DSP
//  and the reasoning behind it are unchanged, the plumbing is not.
// =============================================================================
#include "EngineAudio.h"
#include <Preferences.h>
#include "esp_system.h"
// cycle counter for the render profiler: ESP.getCycleCount()   // esp_random
// Baked-in extras are OPTIONAL. The public repo does not ship them (the pump is
// a third-party recording, the cut-off clip is the builder's own); without them
// the same sounds come from the SD pack (hydpump.wav / hydflow.wav / danger.wav)
// or stay silent.
#if __has_include("HydSamples.h")
#include "HydSamples.h"
#define HAVE_BAKED_HYD 1
#else
#define HAVE_BAKED_HYD 0
#endif
#if __has_include("DangerClip.h")
#include "DangerClip.h"
#define HAVE_BAKED_DANGER 1
#else
#define HAVE_BAKED_DANGER 0
#endif
#include "EngineSim.h"
#include "Config.h"
#include <math.h>

EngineAudio EngAudio;

// Loop-seam crossfade at READ time. This must be 0 for the baked packs.
//
// It used to blend the last CF samples over the head AND wrap at (len - CF)
// instead of len. That shortening is the problem: tools/bake_pack.py cuts every
// loop to an exact whole number of 720-degree firing cycles, and lopping 700
// samples off the playback length throws that away - eng650 came out as 5.92
// cycles as actually played, slipping 0.08 of a cycle every wrap. That is the
// exact firing-pattern smear the baker exists to prevent, reintroduced here.
static const uint32_t CF = 0;
static_assert(CF == 0, "the read path below assumes exact-length circular loops");

// Read a sample at a split position, CUBIC (Catmull-Rom) interpolated.
//
// This used to be linear, and linear is a bad resampler. Measured against a
// band-limited reference on our own eng1000 loop:
//
//     pitch    linear     cubic
//     1.13x   -43.5 dB   -52.3 dB
//     1.26x   -42.5 dB   -47.7 dB
//     0.83x   -43.2 dB   -50.2 dB
//
// -43dB of resampling error is not subtle: it is broadband, it tracks the
// signal, and it is a large part of why this never sounded as clean as playing a
// recording straight out. Linear also rolls the top off (-2.7dB at 13kHz), which
// is exactly the injector clatter that makes a diesel sound mechanical.
//
// The table stops helping past ~1.5x - beyond that the error is genuine aliasing,
// not interpolation, and no kernel fixes it. Our crossovers sit at 1.1-1.3x.
// LINEAR, not cubic. The continuous two-rung crossfade means 2 rungs x 2 ladders
// are always sounding, and the 4-tap Catmull-Rom was costing 16 PSRAM reads per
// output sample. Enabling the whoosh and knock layers on top of that took the
// render to 103% of real time with xrun climbing - i.e. audible dropouts, and the
// task watchdog has already killed this board once at 106%.
//
// Two taps instead of four. The loops are baked to exact whole firing cycles and
// the playback rate is clamped to 0.5..2.1, so the interpolation error here is
// small compared to the resampling error that is already present - and a dropout
// is far more audible than either.
static inline __attribute__((always_inline)) float readBufLin(const int16_t *d, uint32_t len, const Pos &p) {
    if (len < 2 || !d) return 0;
    uint32_t i = p.i; if (i >= len) i %= len;
    uint32_t i1 = (i + 1 >= len) ? 0 : i + 1;      // loops are circular
    float fr = p.f;
    if (!(fr >= 0.0f && fr < 1.0f)) fr = 0.0f;     // NaN-safe
    float y0 = (float)d[i], y1 = (float)d[i1];
    return (y0 + (y1 - y0) * fr) * (1.0f / 32768.0f);
}

static inline __attribute__((always_inline)) float readBuf(const int16_t *d, uint32_t len, const Pos &p) {
    if (len < 4 || !d) return 0;
    uint32_t i = p.i; if (i >= len) i = 0;
    float fr = p.f;
    if (!(fr >= 0.0f && fr < 1.0f)) fr = 0.0f;      // NaN-safe
    // The continuous two-rung crossfade doubled the number of taps this is
    // called for and took the render to 92% of real time. When the fractional
    // position is within 1/512 of a sample boundary the cubic kernel evaluates
    // to within ~0.2% of y0 anyway, so take y0 and skip three PSRAM reads and
    // the polynomial. That happens constantly: playback rate is rpm/rung_rpm,
    // so every time rpm sits near a rung centre the rate is near 1.0 and the
    // fraction barely moves.
    if (fr < 0.00195f || fr > 0.99805f) {
        uint32_t j = (fr > 0.5f) ? ((i + 1 >= len) ? i + 1 - len : i + 1) : i;
        return (float)d[j] * (1.0f / 32768.0f);
    }
    // four taps, wrapped - the baked loops are circular by construction, so the
    // kernel may reach across the seam without a discontinuity
    uint32_t im1 = i ? i - 1 : len - 1;
    uint32_t i1  = (i + 1 >= len) ? i + 1 - len : i + 1;
    uint32_t i2  = (i + 2 >= len) ? i + 2 - len : i + 2;
    float ym1 = (float)d[im1], y0 = (float)d[i], y1 = (float)d[i1], y2 = (float)d[i2];
    float c1 = 0.5f * (y1 - ym1);
    float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);
    return (((c3 * fr + c2) * fr + c1) * fr + y0) * (1.0f / 32768.0f);
}

static inline __attribute__((always_inline)) void advance(Pos &p, uint32_t len, float inc) {
    if (!isfinite(inc) || inc < 0) return;          // never let a bad rate run away
    p.f += inc;
    uint32_t whole = (uint32_t)p.f;
    p.i += whole;
    p.f -= (float)whole;
    if (len && p.i >= len) p.i %= len;
}

static float ladderW(const Layer *a, int n, float rpm, int idx) {
    if (n <= 0) return 0;
    if (n == 1) return 1;
    if (rpm <= a[0].rpm)     return idx == 0 ? 1 : 0;
    if (rpm >= a[n - 1].rpm) return idx == n - 1 ? 1 : 0;
    for (int i = 0; i < n - 1; i++)
        if (rpm >= a[i].rpm && rpm <= a[i + 1].rpm) {
            float den = a[i + 1].rpm - a[i].rpm;
            if (den <= 0.0f) return idx == i ? 1 : 0;   // duplicate rung rpm = 0/0 = NaN
            float t = (rpm - a[i].rpm) / den;
            if (idx == i) return 1 - t;
            if (idx == i + 1) return t;
            return 0;
        }
    return 0;
}

static inline float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

// ---- one rung at a time ----------------------------------------------------
// ladderW() crossfades adjacent rungs across the WHOLE gap between them, so at
// nearly every engine speed two different recordings sound together. Two
// recordings of the same engine can never be phase coherent, so summing them
// combs the spectrum, and because their measured rpm differs by a fraction of a
// percent that comb slowly sweeps - heard as the sound oscillating. Idle was the
// worst case: a permanent 50/50 blend of two recordings, forever.
//
// So pick the NEAREST rung and play it alone. Two rungs only ever sound together
// during a switch, for XFADE_S. Hysteresis keeps a hunting idle from flipping
// back and forth across a boundary every block.
#define XFADE_S 0.090f

// Horn release, matched to poor-mans-beier's cfHornOut = 600 ms. LINEAR, the way
// beier's envStepFor() is (step = 1/(rate*ms/1000) per sample), so the gate hits
// exactly zero after that time. A one-pole decay never actually arrives: it sounds
// slower than its nominal time AND leaves a tail hanging under the engine.
// There is no attack constant because there is no attack ramp - beier sets the
// envelope to 1.0 instantly, and a horn valve opening really is a step.
// Exhaust low shelf, matching the browser's _exLS (lowshelf, 130 Hz, +5 dB).
// LS_K is the one-pole coefficient for a 130 Hz corner at DC_OUT_RATE:
//   k = 1 - exp(-2*pi*fc/sr) = 1 - exp(-2*pi*130/44100) = 0.01818
// LS_BOOST is 10^(5/20) - 1 = 0.778, the amount of low band to ADD back.
// Compressor, matching the browser's DynamicsCompressor:
//   threshold -6 dB = 10^(-6/20) = 0.5012 linear
//   ratio 3:1, attack 4 ms, release 150 ms
// One-pole coefficients reach ~95% in 3/k samples, so k = 3/(sr*t):
// Attack is deliberately SLOWER than the browser's nominal 4 ms. Web Audio's
// DynamicsCompressor detects on a smoothed RMS internally; a bare one-pole peak
// detector at 4 ms follows individual waveform cycles instead and modulates the
// gain at the signal frequency. ~20 ms is slow enough that one cycle of a 40 Hz
// firing pulse cannot drag the detector across the threshold, and still fast
// enough to catch a horn or a jake stab.
//   attack  3/(44100*0.020) = 0.0034
//   release 3/(44100*0.150) = 0.00045
// Ceiling on the pre-clip drive. See the note at the dm computation.
// Ceiling on the pre-clip drive. I set this to 1.80 after reading drive 1.9-2.5
// and clip 13% on the bench and calling it hard clipping. That was wrong:
// tanhf() CANNOT exceed +/-1, so a large input is not clipping - it is exactly
// what the tanh is there to do, and a diesel exhaust pulse really is a
// compressed, saturated waveform. Cutting it to 1.80 removed ~5.6 dB and the
// board came out sounding like computer speakers: clean, bounded, lifeless.
// 3.20 lets grunt reach its designed ~2.4x on top of makeup 1.4.
#define DM_MAX 3.20f

#define COMP_THRESH 0.5012f
#define COMP_RATIO  3.0f
#define COMP_ATK    0.0034f
#define COMP_REL    0.00045f

#define LS_K     0.01818f
#define LS_BOOST 0.778f
#define PULSE_LP_K 0.0211f
#define DANGER_GAIN 1.4f    // straight onto the output: clip RMS ~0.31, peaks soft-limited to ~0.95 of full scale  // one-pole ~150 Hz at 44.1 kHz (pulse-lope bass)

#define HORN_FADE_MS   600.0f
#define HORN_FADE_STEP (1.0f / (DC_OUT_RATE * HORN_FADE_MS / 1000.0f))
// One rung playing flat replaced a two-rung blend whose INCOHERENT power sum was
// sqrt((1-t)^2 + t^2): 0.707 at the midpoint, 1.0 at a centre, ~0.81 averaged.
// So single-rung playback runs up to 3dB hotter for free, which measured as a
// mean output peak of 1.04 - sitting ABOVE the tanh continuously instead of
// occasionally touching it, heard as grit. This trims it back.
// Was 0.95, to trim a measured mean output peak of 1.04 that sat above the
// tanh. That measurement is stale: the board now measures 0.77 peak with 0% of
// samples over 1.0, so the condition this corrected no longer exists and it was
// simply costing 0.45 dB per stage, twice.
#define LADDER_TRIM 1.00f

struct RungSel { int cur = -1, prv = -1; float xf = 1.0f; };
static RungSel selEng, selExh;

// CONTINUOUS two-rung crossfade - exactly what the browser does:
//
//     if (eR >= CENTERS[i] && eR < CENTERS[i+1]) {
//         f = (eR-CENTERS[i]) / (CENTERS[i+1]-CENTERS[i]);
//         w[i] = 1-f;  w[i+1] = f;
//     }
//
// Two rungs are ALWAYS sounding, weighted by where rpm sits between them. The
// old code here picked the single nearest rung and only crossfaded during a
// handover, on the reasoning that blending two recordings of one engine combs
// the spectrum and the comb sweeps. That reasoning is sound in the abstract -
// but the browser does the full blend and the browser is the reference, so the
// comb is evidently not what matters next to having a continuous timbre.
//
// A single rung held flat across a 321 rpm gap is stretched up to 16% off the
// pitch it was recorded at and then swapped in 90 ms. That step is what made
// the board sound like a clone of the browser rather than the browser.
//
// RungSel's hysteresis/xf fields are now unused; kept so the struct and its
// call sites do not have to change.
static void pickRung(RungSel &S, const Layer *a, int n, float rpm, float dt, float *g) {
    (void)S; (void)dt;
    for (int i = 0; i < DC_MAX_LAYERS; i++) g[i] = 0;
    if (n <= 0) return;
    if (n == 1) { g[0] = 1.0f; return; }

    if (rpm <= a[0].rpm)          { g[0] = 1.0f; return; }
    if (rpm >= a[n - 1].rpm)      { g[n - 1] = 1.0f; return; }
    for (int i = 0; i < n - 1; i++) {
        if (rpm >= a[i].rpm && rpm < a[i + 1].rpm) {
            float den = a[i + 1].rpm - a[i].rpm;
            if (den <= 0.0f) { g[i] = 1.0f; return; }   // duplicate rung = 0/0
            float f = (rpm - a[i].rpm) / den;
            g[i]     = 1.0f - f;
            g[i + 1] = f;
            return;
        }
    }
}
static inline float pinc(float rpm, const Layer &L, float revPitch) {
    if (L.rpm < 1) return 0;
    // Browser clamps playbackRate to 0.5..2.1 on BOTH ladders. 0.35..2.6
    // let the outermost rungs stretch past where this resampler's own
    // note says the error becomes genuine aliasing.
    return clampf((rpm / L.rpm) * revPitch * (L.rate / DC_OUT_RATE), 0.5f, 2.1f);
}

// read a layer into L/R (mono layers duplicate L)
// Ladders use the LINEAR reader - they are the hot path (4 streams always
// live). One-shots still use the cubic readBuf(): they are at most a few
// voices and they are heard in isolation.
#define READ_LR(L, pos, gl, gr) do { \
    float _sl = readBufLin((L).data, (L).len, (pos)); \
    float _sr = (L).dataR ? readBufLin((L).dataR, (L).len, (pos)) : _sl; \
    (gl) += _sl; (gr) += _sr; } while (0)

// Release every voice in a slot. Used to end the looped crank the instant the
// engine catches - setting rel lets the 3 ms ramp take it out instead of
// cutting mid-waveform, which would be a click.
LopeParams Lope;

// ---- IMA ADPCM decode, one sample (DangerClip.h, low nibble first) ----
static const int16_t IMA_STEP[89] = {7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,
    107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,
    1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,10442,11487,
    12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767};
static const int8_t IMA_IDX[8] = {-1,-1,-1,-1,2,4,6,8};
static inline uint32_t dangerLen() {
    if (Pack.shDanger.ok()) return Pack.shDanger.len;
#if HAVE_BAKED_DANGER
    return DANGER_SAMPLES;
#else
    return 0;
#endif
}
static inline float dangerRate() {
    if (Pack.shDanger.ok()) return Pack.shDanger.rate;
#if HAVE_BAKED_DANGER
    return (float)DANGER_RATE;
#else
    return 16000.0f;
#endif
}
int16_t IRAM_ATTR EngineAudio::dangerNext() {
    if (dangerPos >= dangerLen()) return 0;
    if (Pack.shDanger.ok()) return Pack.shDanger.data[dangerPos++];   // the card's own clip, as recorded
#if !HAVE_BAKED_DANGER
    return 0;
#else
    uint8_t b = DANGER_ADPCM[dangerPos >> 1];
    uint8_t code = (dangerPos & 1) ? (b >> 4) : (b & 15);
    dangerPos++;
    int step = IMA_STEP[dangerIdx], dq = step >> 3;
    if (code & 4) dq += step;
    if (code & 2) dq += step >> 1;
    if (code & 1) dq += step >> 2;
    dangerPred += (code & 8) ? -dq : dq;
    if (dangerPred > 32767) dangerPred = 32767; else if (dangerPred < -32768) dangerPred = -32768;
    dangerIdx += IMA_IDX[code & 7];
    if (dangerIdx < 0) dangerIdx = 0; else if (dangerIdx > 88) dangerIdx = 88;
    return (int16_t)dangerPred;
#endif
}

// x / (1 + 0.3|x|) without the divide: 1/(1+a) ~ 1 - a + a^2 - a^3, a = 0.3|x| <= 0.9
static inline __attribute__((always_inline)) float pulseSoft(float x) {
    float a = 0.3f * fabsf(x); if (a > 0.9f) a = 0.9f;
    return x * (1.0f - a + a * a - a * a * a);
}

// ---- the pulse lope's random walk. "It has to be properly random": the
// pattern is not looped - it is the FEEL. Each new engine cycle picks what
// follows the current step somewhere in the pattern (a Markov chain learned
// from it), then `life` swaps it the way the designer page does. Every hit
// gets its own size (+/-25%), length (2..len pulses) and kick. Seeded from
// the hardware RNG, so no two idles are alike.
char EngineAudio::pulseNextStep() {
    const char *p = Lope.pattern; int n = (int)strlen(p); if (n < 1) return 'm';
    char cand[16]; int nc = 0;
    for (int i = 0; i < n; i++) if (p[i] == pulseStep && nc < 16) cand[nc++] = p[(i + 1) % n];
    char st = nc ? cand[(int)(pulseRnd() * nc) % nc] : p[(int)(pulseRnd() * n) % n];
    if (pulseRnd() < Lope.life) st = (st == 'B') ? 'B' : (st == 's') ? 'm' : (st == 'm') ? 's' : 'm';
    if (st != 'B' && st != 's' && st != 'x') st = 'm';
    return st;
}
void EngineAudio::pulseNewCycle(float a) {
    for (int k = 0; k < 6; k++) { pulseSlotG[k] = 1.0f; pulseSlotB[k] = 0.0f; }
    pulseBodyTgt = 1.0f;
    if (a <= 0.01f) return;
    pulseStep = pulseNextStep();
    const float j = 0.75f + 0.5f * pulseRnd();
    const float bk = Lope.bass / fmaxf(Lope.chug - 1.0f, 0.01f);
    if (pulseStep == 'B') {
        int L = (int)Lope.len; if (L < 1) L = 1; if (L > 6) L = 6;
        int lo = (L + 1) / 2; if (lo < 1) lo = 1;
        int hitLen = lo + (int)(pulseRnd() * (L - lo + 1)); if (hitLen > L) hitLen = L;
        float G0 = 1.0f + (Lope.chug - 1.0f) * j;
        for (int k = 0; k < 6; k++)
            pulseSlotG[k] = (k < hitLen) ? 1.0f + (G0 - 1.0f) * (k == 0 ? 1.0f : powf(Lope.tailK, (float)k))
                                         : 1.0f - Lope.rest;
        pulseKick += Lope.kick * j * a;
        pulseBodyTgt = 1.0f + Lope.body * j * a;
    } else if (pulseStep == 's') {
        pulseSlotG[0] = 1.0f + (Lope.chu - 1.0f) * j;
        pulseKick += 0.4f * Lope.kick * j * a;
        pulseBodyTgt = 1.0f + 0.4f * Lope.body * j * a;
    } else if (pulseStep == 'x') {
        pulseSlotG[0] = 1.0f - Lope.miss * (0.7f + 0.3f * pulseRnd());
        pulseBodyTgt = 1.0f - 0.6f * Lope.body * a;
    } else {
        pulseBodyTgt = 1.0f - 0.25f * Lope.body * a;      // the gaps sag a little
    }
    for (int k = 0; k < 6; k++) {
        pulseSlotG[k] = 1.0f + (pulseSlotG[k] - 1.0f) * a;
        pulseSlotB[k] = pulseSlotG[k] > 1.0f ? (pulseSlotG[k] - 1.0f) * bk : 0.0f;
    }
}
// one firing pulse: the lope's slot gain, or on the LIMITER a random cut / pop
// ("that's how the rev limiter pops should be") - the fuel is cut on some
// pulses and the ones that do fire come in fat.
void IRAM_ATTR EngineAudio::pulseNewPulse(float a, float lim) {
    int slot = (int)(pulseCnt % 6);
    if (slot == 0) pulseNewCycle(a);
    float G = pulseSlotG[slot], B = pulseSlotB[slot];
    if (lim > 0.01f) {
        float Gl, Bl;
        if (pulseRnd() < Lope.limCut) { Gl = 0.08f; Bl = 0.0f; }
        else { float j = 0.7f + 0.6f * pulseRnd(); Gl = 1.0f + (Lope.limPop - 1.0f) * j; Bl = Lope.limBass * j; }
        G = G * (1.0f - lim) + Gl * lim; B = B * (1.0f - lim) + Bl * lim;
    }
    pulseCurG = G; pulseCurB = B;
}

// ---- serial protocol: ~key=val key=val ...
struct LopeKey { const char *k; float *v; float lo, hi; };
static LopeKey LOPE_KEYS[] = {
    {"chug", &Lope.chug, 1, 8},       {"chu", &Lope.chu, 1, 8},         {"miss", &Lope.miss, 0, 1},
    {"width", &Lope.width, 0.3f, 1.6f}, {"len", &Lope.len, 1, 6},       {"tailk", &Lope.tailK, 0, 1},
    {"bass", &Lope.bass, 0, 6},       {"rest", &Lope.rest, 0, 0.8f},    {"kick", &Lope.kick, 0, 200},
    {"kickms", &Lope.kickMs, 20, 800}, {"life", &Lope.life, 0, 0.8f},   {"eng", &Lope.eng, 0, 2},
    {"exh", &Lope.exh, 0, 2},         {"limcut", &Lope.limCut, 0, 1},   {"limpop", &Lope.limPop, 1, 8},
    {"limbass", &Lope.limBass, 0, 6},  {"body", &Lope.body, 0, 1.5f},    {"bodyms", &Lope.bodyMs, 10, 600},
};
bool lopeSet(const char *key, const char *val) {
    if (!strcmp(key, "pat")) {
        int n = 0; for (; val[n] && n < 16; n++) Lope.pattern[n] = val[n];
        if (n == 0) return false;
        Lope.pattern[n] = 0; return true;
    }
    if (!strcmp(key, "save"))  { lopeSave(); return true; }
    if (!strcmp(key, "get"))   { lopePrint(); return true; }
    if (!strcmp(key, "reset")) { Lope = LopeParams(); return true; }
    for (auto &K : LOPE_KEYS) if (!strcasecmp(key, K.k)) {
        float v = atof(val); if (!isfinite(v)) return false;
        *K.v = constrain(v, K.lo, K.hi); return true;
    }
    return false;
}
void lopePrint() {
    Serial.print("[lope]");
    for (auto &K : LOPE_KEYS) Serial.printf(" %s=%g", K.k, *K.v);
    Serial.printf(" pat=%s\n", Lope.pattern);
}
void lopeSave() {
    Preferences p; p.begin("lope", false);
    p.putBytes("p", &Lope, sizeof(Lope)); p.end();
    Serial.println("[lope] saved");
}
void lopeLoad() {
    Preferences p;
    if (p.begin("lope", true)) {
        if (p.getBytesLength("p") == sizeof(Lope)) p.getBytes("p", &Lope, sizeof(Lope));
        p.end();
    }
    Lope.pattern[16] = 0;
}

void EngineAudio::stopSlot(uint8_t slot) {
    for (int i = 0; i < DC_SHOT_VOICES; i++)
        if (_sv[i].active() && _sv[i].slot == slot) { _sv[i].rel = true; _sv[i].loop = false; }
}
void EngineAudio::playShot(const Layer *clip, float gain, uint8_t slot, bool loop) {
    if (!clip || !clip->ok() || gain <= 0.01f) return;   // same mute gate the
                                                         // Teensy's playAct had
    // Release anything already sounding in this slot, so a held button cannot
    // stack copies of the same sample on top of each other.
    for (int i = 0; i < DC_SHOT_VOICES; i++)
        if (_sv[i].active() && _sv[i].slot == slot && !_sv[i].rel) _sv[i].rel = true;

    int v = -1;
    for (int i = 0; i < DC_SHOT_VOICES; i++) if (!_sv[i].active()) { v = i; break; }
    if (v < 0) {                       // all busy: steal whichever is furthest
        float best = -1.0f;            // through its clip, least left to lose
        v = 0;                         // never leave this -1: _sv[-1] is a wild write
        for (int i = 0; i < DC_SHOT_VOICES; i++) {
            // buf may have been cleared by the render task on the OTHER core
            // between the scan above and here - that check-then-use race was a
            // nullptr->len dereference and a watchdog reboot mid-drive.
            const Layer *b = _sv[i].buf;
            float f = (b && b->len) ? (float)_sv[i].pos.i / (float)b->len : 1.0f;
            if (f > best) { best = f; v = i; }
        }
    }
    // Position and gain first, pointer LAST - the render task runs on the other
    // core and must never see a live buf with a stale pos.
    _sv[v].pos = Pos();
    _sv[v].gain = gain; _sv[v].env = 1.0f; _sv[v].rel = false; _sv[v].slot = slot;
    _sv[v].loop = loop;
    _sv[v].buf = clip;
}

bool EngineAudio::shotBusy() const {
    for (int i = 0; i < DC_SHOT_VOICES; i++) if (_sv[i].active()) return true;
    return false;
}

// =============================================================================
// ---- render profiler (bench '~prof=1'): CPU cycles per stage, per block ----
static uint32_t s_pc[4] = {0,0,0,0}, s_pn = 0;
void renderProfPrint() {
    uint32_t n = s_pn ? s_pn : 1;
    const float budget = 240e6f * DC_AUDIO_FRAMES / DC_OUT_RATE;
    Serial.printf("[prof] %lu blocks  setup %.1f%%  ladder %.1f%%  main %.1f%%  (budget %.0f cyc)\n",
                  (unsigned long)s_pn, s_pc[0] / (float)n / budget * 100, s_pc[1] / (float)n / budget * 100,
                  s_pc[2] / (float)n / budget * 100, budget);
    s_pc[0] = s_pc[1] = s_pc[2] = 0; s_pn = 0;
}
// IRAM: the render ran from FLASH, and flash shares one cache with the PSRAM the
// loops stream from - code and samples evicting each other every block.
void IRAM_ATTR EngineAudio::render(int16_t *out, size_t frames) {
    if (!Pack.loaded()) { memset(out, 0, frames * 2 * sizeof(int16_t)); return; }
    const uint32_t pc0 = ESP.getCycleCount();
    uint32_t pc1 = pc0, pc2 = pc0;
    _clipHits = 0; _clipTotal = 0;

    float rpm    = Engine.rpm();

    // ---- LADDER rpm IS FROZEN THROUGH A SHIFT -----------------------------
    //
    // MEASURED, with the gearbox instrumented: every shift fires exactly once,
    // at wr 1950-1954, and engages once. The gear logic is correct. What is NOT
    // correct is what the LADDER does while the revs slew to the new gear.
    //
    // The float shift converges rpm at dt*3.4 over 1.4 s. On a low-gear shift
    // that is 1950 -> ~1350, and this pack's rungs sit at 601/866/980/1063/
    // 1508/1584/2009 - so in the first 376 ms the active PAIR changes twice:
    //      1584/2009  ->  1508/1584  ->  1063/1508
    // Three different pairs of recordings inside half a second. Each handover
    // is individually correct; hearing three in a row reads as extra shifts
    // inside the shift. It only happens in gears 1-5, which drop ~600 rpm,
    // where 6-13 drop ~280 - which is exactly where it was reported.
    //
    // The browser never gets this: its CENTERS are 100 rpm apart, so a sweep
    // crosses many boundaries and no single one stands out. With 445 rpm gaps
    // each crossing is a distinct change of timbre.
    //
    // So hold the rung SELECTION where it was when the shift started, and let
    // the PITCH follow the real revs. The engine still audibly drops - that is
    // the pitch - but it does not hop between recordings on the way down.
    {
        bool sh = Engine.inShift();
        if (sh && !shiftPrevA) ladderHold = rpm;      // latch on the rising edge
        shiftPrevA = sh;
        if (sh && ladderHold > 1.0f) ladderRpm = ladderHold;
        else                         ladderRpm = rpm;
    }
    // NOT while cranking. running() includes ES_CRANK, and the crank holds rpm
    // at 220 - over the 200 threshold - so loopsLevel began fading the ladders
    // up the moment the key turned and the sim was audible UNDER the starter
    // sample for the whole crank. The engine has not fired yet.
    bool  run    = Engine.running() && !Engine.cranking() && rpm > 200;
    float thr    = Engine.throttle();
    bool  jakeOn = Engine.jakeEngaged();
    bool  lim    = Engine.limiter();
    float invSR  = 1.0f / DC_OUT_RATE;

    // recompute the high-pass only when the corner changes
    if (hpHz != hpSet) {
        hpSet = hpHz;
        if (hpHz > 20.0f) {
            float w0 = 2.0f * (float)M_PI * hpHz / DC_OUT_RATE;
            float cw = cosf(w0), sw = sinf(w0);
            float alpha = sw / 1.41421356f;              // Q = 0.707, Butterworth
            float a0 = 1.0f + alpha;
            hb0 =  (1.0f + cw) * 0.5f / a0;
            hb1 = -(1.0f + cw)        / a0;
            hb2 =  (1.0f + cw) * 0.5f / a0;
            ha1 =  (-2.0f * cw)       / a0;
            ha2 =  (1.0f - alpha)     / a0;
        }
        hpX1 = hpX2 = hpY1 = hpY2 = hpX1R = hpX2R = hpY1R = hpY2R = 0;
    }

    // Asymmetric: 0.02 per block is ~290 ms, which is right for a shutdown
    // fade but far too slow for a light-off - the engine would swell in over a
    // third of a second after the fireup sample had already barked. Up fast
    // (~45 ms), down at the original rate.
    {   float tgt = run ? 1.0f : 0.0f;
        loopsLevel += (tgt - loopsLevel) * ((tgt > loopsLevel) ? 0.13f : 0.02f);
    }
    // Shutdown: mute the LADDERS fast so stop.wav plays into silence instead of
    // over the top of the sim tapering its own rpm down. loopsLevel alone is far
    // too slow for this (0.02 per block is ~1.5 s) and it is also what fades the
    // engine in on start, so it cannot simply be sped up.
    loopsLevel *= Engine.shutMute();
    // DOUBLE-CLUTCH LET-OFF between gears.
    //
    // loadT used to be max(thr, boost*0.7), and boost is held near 29 psi
    // through a shift now that the shaft coasts properly - so boost*0.7 pinned
    // loadT at 1.0 and the exhaust stayed at FULL LOAD across every gear change
    // even though the governor had cut fuel to zero. No let-off, no drop, no
    // sound of anyone shifting. (The old 0.55 s shaft stall hid this by
    // collapsing boost, which let load fall for the wrong reason.)
    //
    // A driver double-clutching lifts out of it, the revs fall, he blips and
    // goes back in. The LIFT is the half that was missing - main.cpp already
    // feeds the blip on a downshift. So: during a shift the boost term is
    // ignored and loadT follows the (zero) throttle, and the fall is made fast
    // so it is heard as a lift rather than a slow sag.
    bool inShift = Engine.inShift();
    float loadT = run ? (inShift ? thr : clampf(fmaxf(thr, boost * 0.7f), 0, 1)) : 0;
    // 0.05 up / 0.015 down is ~0.9 s to fall - far too slow to read as a lift
    // inside a 1.4 s shift. 0.10 during a shift is ~0.13 s, which is about how
    // fast a real engine drops when the fuel goes away.
    float kDown = inShift ? 0.10f : 0.015f;
    workLvl += (loadT - workLvl) * (loadT > workLvl ? 0.05f : kDown);

    wander += (frand() * 0.5f - wander) * 0.02f;
    pitchWob = 1.0f + wander * (0.003f + 0.003f * (1 - workLvl));

    // ---- PER-RUNG DECORRELATION ------------------------------------------
    // MEASURED: at a steady cruise the two active rungs have loop periods in a
    // fixed ratio, so they realign every 3.5-5.5 s - a slow, strong, perfectly
    // periodic beat, which is exactly the timescale the ear hears as 'looping'.
    //   rpm 1200: rungs 1063+1508, periods 1.100 s and 1.500 s -> realign 4.1 s
    //   rpm 1750: rungs 1584+2009, periods 1.097 s and 1.371 s -> realign 5.5 s
    //
    // The browser never gets this because its CENTERS are 100 rpm apart (600,
    // 700, ... 1800), so the crossfade is always moving to a new pair and no
    // single pair sounds long enough to establish a beat. This pack has 7 rungs
    // with gaps up to 445 rpm, so one pair holds for a long time.
    //
    // pitchWob is shared by every rung, so it cannot break this - it moves them
    // all together and the RATIO is unchanged. Each rung needs its OWN slow
    // wander, which makes the ratio drift continuously so the alignment never
    // repeats. +/-0.25% is far below pitch perception (4 cents) but it is
    // several samples per loop, which is all it takes to destroy the periodicity.
    if (!rWobInit) { for (int i = 0; i < DC_MAX_LAYERS; i++) rWob[i] = 1.0f; rWobInit = true; }
    for (int i = 0; i < DC_MAX_LAYERS; i++) {
        rWander[i] += (frand() * 0.5f - rWander[i]) * 0.013f;
        rWob[i] = 1.0f + rWander[i] * 0.005f;
    }

    float w0 = (rpm < 1080.0f) ? clampf(1 - (rpm - 720.0f) / 360.0f, 0, 1) : 0;
    w0 = sqrtf(w0);
    // The (1 - w0) fade muted this ladder approaching idle because a SEPARATE
    // engine bank used to carry idle. A single-ladder pack has no engine bank, so
    // that fade leaves the truck almost silent at idle - w0 is ~0.96 at 750rpm,
    // scaling the ladder to 0.04. When nEng is 0 this ladder IS the whole engine.
    float exS   = (Pack.nEng > 0) ? (1 - w0) * loopsLevel : loopsLevel;
    float loadW = workLvl;
    float pIdleInc = pinc(rpm, Pack.idle, 1.0f) * pitchWob;
    float idleG = w0 * loopsLevel;
    // The sim sets lope gain to 0 - the lope comes out of the cam physics
    // (chopMiss shaking the crank), not a faded-in sample. So don't play the lope
    // loop, and don't duck idle by 85% to make room for it either.
    // lope.wav is NOT played (Mat 2026-10-30: "get rid of lope wav", after
    // hearing it crossfaded in over the idle). The lope is the sim's chop only.
    // Browser: bk = (0.10 + bkTim^1.6*2.05) * _thr * ... * clamp((r-1100)/500, 0, 1)
    // The rpm gate was missing entirely, so the bark applied at EVERY rpm
    // including idle. The browser mutes it below 1100 and reaches full at 1600 -
    // i.e. the bark arrives as the engine comes on song and vanishes between
    // shifts, which is most of the rowing character.
    float barkGate = clampf((rpm - 1100.0f) / 500.0f, 0, 1);
    float barkV = 0.35f + Engine.barkFactor(rpm) * 1.6f * barkGate;

    // Chop depth comes from the PHYSICS, so audio and crank agree.
    static uint8_t chopCyl = 0;      // walks the per-cylinder pattern
    float chopD = Engine.chopDepth();
    const uint32_t evSeq = Engine.chopSeq();
    const float evAmp = Engine.chopAmpNow(), evDur = Engine.chopDurNow(), evHunt = Engine.huntNow();
    const bool evBark = Engine.chopBark();
    const bool evRhythm = Engine.chopRhythm();
    const float evDeep = Engine.chopDeep();
    // (rhythm events no longer exist - the sim's phrase is gone; kept harmless)
    if (evRhythm && evSeq != lastEvSeq && Engine.chopDepth() > 0.02f) { lastEvSeq = evSeq; chopDur = 0; }
    (void)evDeep;
    chopDbg = chopD;
    bool  chopHi = Engine.chopHi();
    // THE AUDIO HAD ITS OWN COPY OF THE CHOP TIMING, AND IT WENT STALE.
    //
    // This was `3.4 + chopD*1.4` - the same fixed rate EngineSim used to have.
    // When the sim was changed to beat at rpm/120 (one weak cylinder fires once
    // per two crank revolutions) THIS was not, so the physics galloped with the
    // revs while the sound carried on at a flat ~4 Hz. Two copies of one
    // number, and only one of them got fixed.
    //
    // Same formula as EngineSim::chopMiss now. If that changes, change this.
    // FIRE ON EVERY CYLINDER EVENT, NOT ONCE PER CYCLE.
    //
    // rpm/120 is the rate at which ONE given cylinder comes round - right for
    // a single lazy injector, wrong for a 6-entry pattern: one full pass of
    // the table would take six engine cycles (over a second at idle), far too
    // slow to hear as a rhythm. It read as occasional dips in a steady idle.
    //
    // A six-cylinder four-stroke fires rpm/20 times a second - 30 Hz at 600
    // rpm - and stepping the table once per FIRING puts the whole uneven
    // pattern inside one engine cycle (5 Hz at 600 rpm), which is the gallop
    // you actually hear from a cammed engine. Same number the research gives
    // for the pattern period: 2 crank revolutions.
    // BARK TIMING, MEASURED OFF THE REAL RECORDING.
    //
    // The bursts in lopeidle.wav are spaced 175-300 ms apart - roughly 3.5-5 Hz -
    // and are about 25 ms wide. NOT the firing rate. I had this stepping at
    // rpm/20 (30-50 Hz), which is why it buzzed instead of loping: it was firing
    // ten times too often for the thing being modelled.
    //
    // A lope burst spans ~17 firing events at 700 rpm, so it is a slow instability
    // in the fuel system, not one weak cylinder. That matches the research - a
    // competition diesel hunts through its injection pump - and it matches the
    // measurement, which found the bursts are not phase-locked to anything.
    //
    // 4.2 Hz centre with heavy jitter on each interval reproduces the irregular
    // 175-300 ms spacing. Still scaled by rpm so it speeds up with the revs, just
    // from a far lower base.
    float chopRate = chopHi ? 6.5f : ((rpm > 1.0f) ? (4.2f * (rpm / 700.0f)) * (1.0f + chopD * 0.10f) : 4.2f);

    // ---- LOPE PITCH SWING --------------------------------------------------
    // The chop below ducks GAIN, and gain alone is why the first two cells of
    // timing and fuel sounded wrong: a volume stutter on a steady pitch reads as
    // a tremolo or a fault, not as a cam. A real lumpy cam loses and regains
    // crank speed on every misfire, so the EXHAUST PITCH moves with it - that
    // pitch movement is most of what the ear identifies as lope.
    //
    // The sim does it like this:
    //     _rAvg += (r - _rAvg) * 0.08
    //     rX     = _rAvg + (r - _rAvg) * (1 + 1.4 * min(1, _chD))
    // i.e. keep a slow average and EXAGGERATE the instantaneous deviation from
    // it, by up to 2.4x at full chop depth. The physics already produces the
    // deviation - chopMiss() subtracts real torque from the crank every misfire -
    // so this amplifies something true rather than inventing a wobble.
    //
    // Note the sim smooths per audio FRAME; this runs once per render block, so
    // the coefficient is scaled by the block length to give the same time
    // constant regardless of DC_AUDIO_FRAMES.
    // rpmPitch drives PLAYBACK RATE ONLY. It must not drive rung selection:
    // pickRung() would then flap across a band boundary on every misfire, and
    // the boundary blend would wobble with it - trading a timbre step for a
    // warble, which is worse. Selection stays on the true, unswung rpm.
    float rpmPitch = rpm;
    {
        if (rAvg < 1.0f) rAvg = rpm;                      // first block, no ramp
        float k = 1.0f - expf(-(float)frames / DC_OUT_RATE / 0.072f);
        rAvg += (rpm - rAvg) * clampf(k, 0.0f, 1.0f);
        float swing = 1.0f + 1.4f * clampf(chopD, 0.0f, 1.0f);
        float rX = rAvg + (rpm - rAvg) * swing;
        if (isfinite(rX)) rpmPitch = clampf(rX, 120.0f, 2400.0f);
    }

    float gC[DC_MAX_LAYERS], iC[DC_MAX_LAYERS], gL[DC_MAX_LAYERS], iL[DC_MAX_LAYERS];
    float gJ[DC_MAX_LAYERS], iJ[DC_MAX_LAYERS];
    float gE[DC_MAX_LAYERS], iE[DC_MAX_LAYERS];
    float wE[DC_MAX_LAYERS], wX[DC_MAX_LAYERS];
    const float blockDt = (float)frames / DC_OUT_RATE;

    // ENGINE ladder. Pack.idle ALIASES the first rung, so folding the idle gain
    // into the one engine stream is deliberate: played through its own position
    // the identical recording sounded twice at unrelated phases - a flanger
    // sitting on idle, sweeping as the two positions drifted apart.
    pickRung(selEng, Pack.eng, Pack.nEng, ladderRpm, blockDt, wE);
    // sum in POWER, not amplitude: these used to be two independent streams, and
    // incoherent streams add as sqrt(a^2+b^2). Adding linearly now they are one
    // coherent stream would come out ~3dB hotter at idle.
    // ---- swell + jake duck, the browser's `dr` term -------------------------
    // Browser:
    //     rlf   = clamp((r-2000)/100, 0, 1)
    //     swell = 1 + rlf*0.4
    //     dr    = duck * swell * (jkOn ? 0.68 : 1)
    //     drE = drX = dr          <- applied to BOTH ladder gains
    //
    // Neither half of this existed here. `swell` appeared only in a comment, so
    // the engine did NOT lift +2.9 dB from 2000 rpm up to the limiter - and that
    // lift is the surge INTO every upshift point, which is a large part of what
    // makes the browser sound like it is being rowed. And `duck` was only ever
    // multiplied into gL[i], the pull ladder, which is hardcoded to zero - so the
    // jake duck multiplied nothing at all.
    float rlf   = clampf((rpm - 2000.0f) / 100.0f, 0, 1);
    float swell = 1.0f + rlf * 0.4f;
    float duck  = swell * (jakeOn ? 0.68f : 1.0f);
    // `duck` carries the swell and the jake duck - the browser's drE.
    float engG = loopsLevel * sqrtf(engLevel * engLevel + w0 * w0) * LADDER_TRIM * duck;
    for (int i = 0; i < Pack.nEng; i++) {
        // Browser: eG[i].gain *= (i===0 ? clamp((900-eR)/230,0,1) : 1)
        // The BOTTOM rung is faded out above ~900 rpm. It is the idle recording,
        // and holding it in once the engine is off idle muddies everything above
        // it. Nothing here was doing this.
        gE[i] = wE[i] * engG;
        if (i == 0) gE[i] *= clampf((900.0f - rpm) / 230.0f, 0, 1);
        iE[i] = pinc(rpmPitch, Pack.eng[i], 1.0f) * pitchWob * rWob[i];
    }
    idleG = 0.0f;                       // never a second stream of the same sample

    // EXHAUST ladder. coast[] and pull[] are drawn from the same recordings and
    // used to have their own positions too, so at any partial load the same
    // sample flanged against itself. One rung, one stream; coast/load character
    // is a gain on it.
    pickRung(selExh, Pack.coast, Pack.nCoast, ladderRpm, blockDt, wX);
    float exA = (1 - loadW) * coastQuiet, exB = loadW * loadHot * barkV;
    float exhG = exS * sqrtf(exA * exA + exB * exB) * LADDER_TRIM * duck;   // browser drX
    for (int i = 0; i < Pack.nCoast; i++) {
        // Same bottom-rung fade on the exhaust ladder (browser xG[i]).
        gC[i] = wX[i] * exhG;
        if (i == 0) gC[i] *= clampf((900.0f - rpm) / 230.0f, 0, 1);
        // The browser pitches the ENGINE ladder off rX (the lope-swung rpm) and
        // the EXHAUST off plain r:
        //     eS[i].playbackRate = clamp(rX / CENTERS[i],    0.5, 2.1)
        //     xS[i].playbackRate = clamp(r  / EXCENTERS[i],  0.5, 2.1)
        // I had put the swing on both. Swinging the exhaust too doubles the
        // wobble, because engine and exhaust then move together instead of the
        // exhaust holding steady under a lumping engine.
        // note the per-rung wobble is INVERTED on the exhaust ladder, so an
        // exhaust rung never drifts in lockstep with the engine rung above it
        iC[i] = pinc(rpm, Pack.coast[i], revPitch) * pitchWob * (2.0f - rWob[i]);
    }
    for (int i = 0; i < Pack.nPull; i++) { gL[i] = 0.0f; iL[i] = 1.0f; }   // folded in

    // MEASURED on the bench: pk 1.17-1.27 sustained through the entire jake
    // section and 1.76 with the horn over it - hard-clamping continuously.
    //
    // jake.wav peaks near full scale and sums onto the exhaust bus raw, next to
    // the exhaust ladder, the engine and the action bus. The browser's 2.7x on
    // this voice is a NODE gain feeding a compressor; copying the number here
    // (or, as I did, adding the 0.68 duck that is meant to balance against it
    // while leaving the voice alone) just moves the overrun around.
    //
    // 0.55 is what fits: the jake still barks clearly over a ducked engine, and
    // the sum stays inside the clipper instead of sitting on it.
    // THE JAKE SAMPLE AT THE BROWSER'S LEVEL.
    //
    // Browser:  jakeGain[0] = 2.7 * jkF * lvl,  jkF = clamp((r-680)/250,0,1)
    // Here it was: jakeAmt * the same jkF * 0.55
    //
    // 0.55 against 2.7 is 13.8 dB quieter. That is why the jake never sounded
    // like anything and why an hour of inventing whistle terms, tone shifts and
    // gap-fillers did not fix it - all of that was trying to replace a sample
    // that was simply turned down.
    //
    // 2.7 is a NODE gain in the browser, feeding a compressor, so it is not a
    // raw multiplier here - the same trap as the 4.2 whistle gain. 1.6 puts the
    // sample clearly in front of a 0.68-ducked engine without pushing the bus
    // past the clipper.
    float jakeLvl = jakeOn ? Engine.jakeAmt() * clampf((rpm - 680.0f) / 250.0f, 0, 1) * 1.6f : 0;
    // JAKE PITCH - DELIBERATELY WAY BELOW THE SAMPLE'S NATIVE RATE.
    //
    // jake.wav was recorded at some particular engine speed and the old term
    // played it at 0.91-1.05 across the rpm range the jake actually lives in
    // (1280 at the downshift point, ~1780 after the blip) - i.e. essentially
    // native pitch, which is why it read as thin rather than as a big engine
    // holding a load back.
    //
    // JK_BASE shifts the whole thing down while keeping the shape: still
    // tracks rpm, still no octave sweep. 0.75 is about -5 semitones across
    // the working range (0.68 at 1280, 0.79 at 1780).
    //
    // Why not lower: resampling down slows the sample by the same factor, so
    // at 0.75 the blatt lasts 1.33x longer. Past roughly 0.60 the individual
    // exhaust pulses get far enough apart to hear as separate events and it
    // stops sounding like a deeper engine and starts sounding like tape drag.
    // 0.75 is the bottom of the band that still reads as pitch, not speed.
    #define JK_BASE 0.75f
    // The FLOOR had to come down with it. It was 0.72, but the term now wants
    // 0.75*0.91 = 0.682 at the downshift point - the old floor would have
    // clamped it and flattened the rpm drift exactly where the jake is
    // loudest and most audible.
    float jkPitch = clampf(JK_BASE * (1.0f + (rpm / 1600.0f - 1.0f) * 0.45f),
                           0.60f, 1.18f);
    for (int i = 0; i < Pack.nJake; i++) {
        gJ[i] = ladderW(Pack.jake, Pack.nJake, rpm, i) * jakeLvl;
        iJ[i] = (Pack.jake[i].rate / DC_OUT_RATE) * jkPitch;
    }

    boost = Engine.turboSpool();
    float boostPsi = Engine.boostPsi();
    float turboInc   = clampf((0.8f + boostPsi / 30.0f) * (Pack.turbo.rate / DC_OUT_RATE), 0.5f, 4.2f);
    // SAMPLED turbo: gated on how fast boost is MOVING, not on its level.
    //
    // The browser does:
    //     _bLag += (boost - _bLag) * 0.35
    //     _bChg  = |boost - _bLag|
    //     twhG   = clamp(turbo*(0.4+0.6*thr),0,1) * 1.1 * clamp((_bChg-0.22)*6,0,1)
    //
    // so turbo.wav only sounds WHILE boost is changing - spooling up or bleeding
    // down - and goes silent at steady boost, where the synthesised noise
    // whistle carries it instead. This played it continuously off boost LEVEL,
    // and turbo.wav is a 0.52 s clip: the shortest loop in the pack, going round
    // twice a second under a steady cruise. That is the turbo 'sounding loopy'.
    // TIME CONSTANT, corrected. The browser's 0.35 coefficient runs once per
    // animation frame (~60 Hz), so its lag settles in roughly 50 ms. I converted
    // it as 0.08 s per BLOCK - but a block is 5.8 ms here, which made the filter
    // far faster than the browser's, so bLag tracked boost almost exactly, bChg
    // stayed near zero and the gate hardly opened at all. Cruise sounded right
    // (silent is correct there) and everything else went missing.
    //
    // 0.05 s gives the same lag as the browser regardless of block size, and the
    // floor is dropped to 0.10 psi: the firmware's boost is smoother than the
    // browser's, so 0.22 was too high a bar for a real spool to clear.
    bLag += (boostPsi - bLag) * clampf((float)frames * invSR / 0.05f, 0, 1);
    float bChg = fabsf(boostPsi - bLag);
    // Blend, not a hard gate: the change term RAISES the whistle while boost is
    // moving instead of being the only thing that lets it through, so a steady
    // cruise is quiet but never silent and a spool still swells over the top.
    float bMove = clampf((bChg - 0.10f) * 6.0f, 0, 1);

    // SMOOTH THE GATE, AND NARROW ITS SWING.
    //
    // Two things made the turbo sit APART from the rest of the mix instead of
    // blending into it:
    //
    // 1. The swing was 0.25 -> 1.00, a 4x (12 dB) level jump on the sample,
    //    driven by bChg which comes off a 0.05 s lag. So the sample lurched in
    //    the instant boost moved and dropped straight back out, while the
    //    whistle beside it was moving with the shaft's inertia (0.60 s down).
    //    Two voices of the same turbo changing level at completely different
    //    rates is exactly what reads as 'not blended'.
    //
    // 2. bMove itself was applied RAW, so it inherited every wiggle in the
    //    boost signal.
    //
    // Fix: narrow the swing to 0.55 -> 1.00 (5 dB, a swell rather than a
    // gate) and slew the gate so it cannot step. 0.14 s up keeps a real spool
    // feeling immediate; 0.45 s down lets it settle into the whistle instead
    // of dropping out from under it.
    {
        float mk = (bMove > bMoveS) ? 0.14f : 0.45f;
        bMoveS += (bMove - bMoveS) * clampf((float)frames * invSR / mk, 0, 1);
    }
    float turboSampG = clampf(boost * (0.4f + 0.6f * workLvl), 0, 1) * 1.1f * turboLevel
                     * (0.55f + 0.45f * bMoveS);
    // whistle pitch tracks the SHAFT (keeps singing on lift as it coasts), not boost
    float shaftHz = Engine.shaftHz();
    // Rescaled for the corrected shaftHz (was calibrated against a value 9.55x
    // too small). Real shaft Hz: ~160 at the idle floor, ~1700 at stock full
    // boost, ~3500 on a compound. Spanning 160..2800 keeps the whistle sweeping
    // across its whole range on a stock turbo AND leaves headroom so an
    // upgraded turbo actually sounds faster instead of pinning at 1.0.
    // EXACTLY the browser: _turbo = clamp((out.shaftHz - 18) / 225, 0, 1).
    // Now that shaftHz is wT/60 again, this is the browser's own scale. The
    // 160/2640 and 160/3340 variants were both compensating for the 9.55x
    // shaftHz error and neither put the curve back in the right place.
    float turboN  = clampf((shaftHz - 18.0f) / 225.0f, 0, 1);
    // compressor SURGE flutter: 3 descending pitch pulses gating the whistle on lift
    float wf = Engine.wgFlutter(), bend = 1.0f, pop = 1.0f;
    if (wf > 0.02f) {
        // 4.5/s. The browser advances _flP by 0.135 per updateMix call
        // and updateMix runs on a 30 ms setInterval (33.3 Hz), so
        // 0.135*33.3 = 4.5. 8.1 assumed a 60 Hz animation frame - 1.8x
        // too fast, which turned three descending pops into a buzz.
        flP += 4.5f * ((float)frames * invSR);
        int pi = (int)flP;
        if (pi >= 3) { pop = 0.0f; bend = 0.30f; }
        else {
            float fp = flP - pi, pStep = powf(0.64f, (float)pi);
            bend = (1.0f - fp * 0.22f) * 0.42f * pStep;
            pop  = (fp < 0.45f) ? 1.0f : 0.14f;
        }
    } else flP = 0;
    // JAKE TONE IS LOWER. Under power the whistle corner sweeps 900 Hz to 7200
    // (the browser's 900 + 6300*_turbo). An engine brake is a deeper, blattier
    // noise than a spooling turbo - it is the exhaust valve hammering, not the
    // compressor singing - so pull the corner down while the jake is on.
    float fcA = clampf((900.0f + 6300.0f * turboN) * bend, 200.0f, 7600.0f);
    float fcB = fminf(fcA * 2, DC_OUT_RATE * 0.45f);
    float rA = 0.988f;
    float c1A = 2 * rA * cosf(2 * (float)M_PI * fcA * invSR);
    float c1B = 2 * rA * cosf(2 * (float)M_PI * fcB * invSR);
    float c2 = -rA * rA, gAB = 1 - rA;
    // Whistle level. The formula matches the browser - ((boost-3)/26)^2 - but
    // the firmware's turbo physics is twitchier than the browser's, and held at
    // full throttle in NEUTRAL the boost measurably sags (1.1 -> 0.8 psi on the
    // bench) because the governor holds the limiter by cutting fuel, which
    // starves the turbine. That part is physically right; a free-revving diesel
    // on the limiter cannot make boost.
    //
    // What is NOT right is the whistle SWINGING with it. A 90 mm compressor
    // wheel has real rotational inertia - the shaft does not change speed as
    // fast as manifold pressure does, so the whistle should not either. That
    // mismatch is the surging. Smoothing the LEVEL with a ~250 ms time constant
    // gives the whistle the shaft's inertia without touching the physics or the
    // pitch (which already tracks shaftHz, correctly).
    // WHISTLE LEVEL: SHAFT-LED, not boost-led.
    //
    // MEASURED on a 13-gear pull: boost dumps to 4-12 psi on every single gear
    // change and then rebuilds. The browser's ((boost-3)/26)^2 is SQUARED, so
    // that dump collapses the whistle to ~0.001 and the rebuild swells it back
    // 8x. Twelve gears, twelve swells - that is the 'breathing' / 'goes up and
    // down'. It is not a filter artefact, it is the level faithfully tracking a
    // manifold pressure that really does collapse twelve times.
    //
    // A whistle is made by the WHEEL, and the wheel has rotational inertia. The
    // manifold empties the instant a valve shuts; a 90 mm compressor at 100k rpm
    // does not. So lead with the shaft (turboN, which cannot change fast) and
    // let boost only MODULATE it - un-squared, and floored, so no boost dump can
    // take the whistle away.
    float whnSh = turboN * turboN;                       // shaft: the carrier
    float whnB  = clampf((boostPsi - 3.0f) / 26.0f, 0, 1);   // load: linear
    float whnT  = whnSh * (0.55f + 0.45f * whnB);        // never below 55%

    // NO JAKE TERM ON THE WHISTLE. The browser's whistle is _whn2^2 and
    // nothing else - it does not touch the turbo during an engine brake at
    // all. Its ENTIRE jake is: duck the engine to 0.68 and play jake.wav at
    // 2.7. I spent an hour adding a jake whistle, a lower tone and a
    // gap-filler, and all three were compensating for the real bug - the jake
    // SAMPLE was playing 13.8 dB below where the browser plays it.

    // SHARP ATTACK, slow release. A turbo coming on song is a fast event and a
    // coast-down is a slow one; one symmetric smoother cannot do both and the
    // 0.25 s I had before rounded off the attack, which is the part that should
    // be sharp. 25 ms up is almost instant at block rate; 0.8 s down is the
    // wheel coasting, and it is what spans a 1.4 s shift so the whistle holds
    // through the gear change instead of dropping out and sweeping back up.
    // Fast up, slow down - a turbo spools quickly and coasts slowly. With the
    // jake term gone there is no handover to rush, so this is back to the
    // compressor's own inertia.
    //
    // RELEASE MATCHED TO THE SAMPLE. The sample's gate now falls over 0.45 s
    // (see bMoveS above). At 0.60 s the whistle outlived it on every let-off,
    // so you heard the sample go first and the whistle hang on behind it -
    // one turbo coming apart into two voices leaving at different times.
    // Both at 0.45 s means the whole turbo recedes as one thing.
    float wk = (whnT > whnS) ? 0.025f : 0.45f;
    whnS += (whnT - whnS) * clampf((float)frames * invSR / wk, 0, 1);
    float whn = whnS;
    // Same raw-amplitude caveat as the whoosh above: the browser's 4.2 is a node
    // gain, this is a direct multiplier on a resonator near full scale. 2.1 keeps
    // the whistle clearly present (it was 1.05 = inaudible) without putting a
    // 4x amplitude on the bus. The A:B ratio stays the browser's 4.2:2.2.
    float whineAmp = whn * 2.1f * turboLevel * pop;
    // Browser: (0.30*_turbo*(0.3+0.7*_load))*2.7 + _wf*4.0*_pop, with NO ceiling.
    // 1.2/1.6 were 7-8 dB low. _load is the 0.18-smoothed throttle, so workLvl
    // is the right variable here, not the instantaneous loadT.
    // MEASURED: on the let-off block this hit drive 4.76 / pk 1.54 - a pop.
    //
    // In the browser `_wf*4.0` is a GAIN ON A FILTER NODE whose own output is
    // well below unity, so 4.0 is a sane number there. Here it multiplies a
    // resonator that is already near full scale, so the same 4.0 is a raw
    // amplitude of 4 landing on the master bus at exactly the moment the surge
    // fires - i.e. every time you lift off boost. That is the pop.
    //
    // Scaled to the same perceptual place without the spike: the steady term
    // keeps the browser's 2.7 shape, the surge term is brought to 1.2 and the
    // sum is ceilinged. The ceiling is not in the browser but the browser's
    // node gains cannot stack the way these raw amplitudes can.
    // The jake also drives the WHOOSH - the broadband rush of air through the
    // housing, which is most of what reads as 'loud' on a real engine brake.

    float whooshAmp = fminf((0.30f * turboN * (0.3f + 0.7f * workLvl)) * 2.7f
                            + wf * 1.2f * pop, 1.6f)
                    * turboLevel * whooshLevel;

    // Knock resonator Q must match the sim's bandpass (Q=4). At r=0.95 the
    // bandwidth is ~700Hz - broad enough that the noise driving it comes through
    // as hiss instead of a tonal clatter. r=0.973 gives ~375Hz = Q=4 at 1500Hz.
    float knFc = 1500.0f, knR = 0.973f;
    float knC1 = 2 * knR * cosf(2 * (float)M_PI * knFc * invSR), knC2 = -knR * knR, knGn = 1 - knR;
    // Browser: knockLFO.frequency = clamp(r/20, 18, 130). Clamp was missing.
    float knHz = clampf(rpm / 20.0f, 18.0f, 130.0f);
    float knockAmp = clampf((rpm - 650.0f) / 1450.0f, 0, 1);
    knockAmp *= knockAmp * 0.6f * knockLevel;   // browser 0.6, was 0.5
    float hotF = clampf((Engine.pyroF() - 300.0f) / 700.0f, 0, 1);
    float fireHz = rpm * 0.05f;
    const float fuelK = Engine.fuelOnK();      // 0 with the injectors cut - no firing pulse on the jake
    float grunt = (1.0f + 1.8f * workLvl * 0.55f) * (1.0f + 0.22f * hotF);

    fanG += (((Engine.fanOn() && run) ? 0.16f * fanLevel : 0.0f) - fanG) * 0.002f;
    if (fanG < 0) fanG = 0;
    // HYDRAULIC PUMP = the real Cat 323 pump recording (HydSamples.h), looped at
    // its NATIVE pitch with the volume swelling as the sticks are pushed - the
    // way Mat's HydraulicController plays it. Two earlier ideas were rejected
    // on that machine and are NOT used here: a synthesised triangle whine
    // ("firetruck / siren"), and pitching the pump to engine rpm (screeched).
    // Engaged-but-idle is a faint hum; the relief valve is the flow recording.
    hydG += (((hydOn && run) ? (0.025f + 0.30f * hydLoad) * hydLevel : 0.0f) - hydG) * 0.004f;
    float relTgt = (hydOn && run) ? clampf(hydPress, 0, 1) * 0.30f : 0.0f;
    relG += (relTgt * hydLevel - relG) * 0.01f;

    // LOW AIR ALARM - only while the engine is actually running.
    //
    // The warn bit itself is correct and stays as it is: air only builds when
    // the engine turns (the compressor term is scaled by rpm^2), start() brings
    // the tank up to AIR_START = 35 psi, and LOW_AIR is 55 - so a real truck
    // DOES buzz at you for the first few seconds after a cold start until the
    // governor pumps it up, and the dash lamp should keep showing that.
    //
    // What was wrong is that the BUZZER had no running gate, so sitting on the
    // bench with USB power and the engine off it sounded forever, with no way to
    // build air and shut it up. The reverse beeper immediately below already
    // gates on loopsLevel for exactly this reason; this now matches it.
    //
    // Deliberately gated on the ENGINE, not on USB: there is no reliable way to
    // tell bus power from battery power on this board (no VBUS sense wired to a
    // GPIO), and gating on 'engine off' is both detectable and the behaviour you
    // actually want - a parked truck with the key off does not buzz either.
    bool lowAir = ((Engine.warnBits() & 0x01) != 0) && run;

    // KEEP SILENT RUNGS MOVING - once per block, not per sample.
    //
    // A rung that is faded out used to have its position frozen, so returning to
    // that rpm resumed from exactly where it stopped: at a steady cruise one
    // ~1.2 s loop repeated from a fixed phase with the two crossfading rungs
    // locked at a constant offset. That periodicity is what reads as 'looping'.
    //
    // Advancing them inside the per-sample loop fixed the sound and took the
    // render to 106% of real time - the task watchdog killed core 0 and the
    // board rebooted mid-drive. A whole block of catch-up applied ONCE is
    // numerically the same place (the rate is constant across a block) for
    // 1/256th of the cost.
    // HYSTERESIS on the catch-up, and this matters: it is where the occasional
    // POP came from.
    //
    // The gains are computed once per block. A rung sitting right at the 0.02
    // boundary alternates between 'silent' (jumped a whole block forward) and
    // 'audible' (read normally) from one block to the next - so it gets read at
    // an audible gain from a position 256 samples displaced, mid-waveform. That
    // is a step discontinuity, and at a steady cruise a rung can hover on that
    // boundary for a long time, popping every few blocks.
    //
    // Only catch up a rung that is well BELOW the read threshold (0.002, ~54 dB
    // down). Between 0.002 and 0.02 it is neither read nor jumped - its position
    // simply pauses, which is inaudible at that level and cannot click.
    // ---- THE PULSE LOPE + LIMITER POPS: per-block gain envelope ----
    // The loudest engine rung is the MASTER: its own firing grid (found at
    // load) says where each real pulse peaks. Each pulse gets a gain (the
    // lope's random walk, or the limiter's cut/pop), applied through a smooth
    // window centred on the peak, so a whole combustion hit is lifted or
    // dropped and the seams fall in the quiet between pulses. Built in its own
    // tight loop (the main loop spills registers - measured), then applied to
    // both buses. Pitch: a CHUG kicks the ladder rates (pulseKick).
    // ---- SHAKE HANDS WITH DANGER: the battery protection's clip ----
    // Decoded + resampled 16 k -> 44.1 k (linear) for the whole block here, then
    // summed onto the action bus below - so the limiter and the engine chops
    // never touch it. Level is set at bake time (RMS-matched to the C15).
    static float dangerBuf[DC_AUDIO_FRAMES];
    if (dangerStopReq) { dangerStopReq = false; dangerReq = false; dangerOn = false; }
    if (dangerReq && !dangerLen()) dangerReq = false;          // no clip in this build or on the card
    if (dangerReq) { dangerReq = false; dangerOn = true; dangerPos = 0; dangerPred = 0; dangerIdx = 0;
                     dangerPh = 0; dangerA = 0; dangerB = dangerNext() * (1.0f / 32768.0f); }
    const bool dangerBlk = dangerOn;
    if (dangerOn) {
        const float inc = dangerRate() / DC_OUT_RATE;
        const size_t nf = frames < DC_AUDIO_FRAMES ? frames : DC_AUDIO_FRAMES;
        for (size_t i = 0; i < nf; i++) {
            dangerBuf[i] = (dangerA + (dangerB - dangerA) * dangerPh) * DANGER_GAIN;
            dangerPh += inc;
            while (dangerPh >= 1.0f) { dangerPh -= 1.0f; dangerA = dangerB; dangerB = dangerNext() * (1.0f / 32768.0f); }
        }
        if (dangerPos >= dangerLen()) dangerOn = false;   // played once
    }
    static float pulseG[DC_AUDIO_FRAMES], pulseB[DC_AUDIO_FRAMES];
    const float lopeA = Engine.pulseLopeAmt() * Engine.fuelOnK();
    // LIMITER POPS ARE A BIG-PLUNGER THING (Mat: "the limiter should only pop
    // with the big plungers, all others should rest"). Every other injector
    // just rests on the governor: no pops, no chop.
    const bool  bigPlunger = Engine.parts.fuel >= 2;
    const float limA  = bigPlunger ? limEnv : 0.0f;
    int mi = -1; float mg = 0.0f;
    for (int i = 0; i < Pack.nEng; i++) if (Pack.eng[i].pulsePer > 1 && gE[i] > mg) { mg = gE[i]; mi = i; }
    bool mCoast = false;
    if (mi < 0) for (int i = 0; i < Pack.nCoast; i++) if (Pack.coast[i].pulsePer > 1 && gC[i] > mg) { mg = gC[i]; mi = i; mCoast = true; }
    const bool pulseBlk = mi >= 0 && (lopeA > 0.01f || limA > 0.01f);
    float eMul = 1.0f, xMul = 1.0f;
    pulseKick *= expf(-blockDt * 1000.0f / fmaxf(Lope.kickMs, 20.0f));
    if (pulseKick > 0.5f) {
        float km = 1.0f + pulseKick / fmaxf(rpm, 300.0f);
        for (int i = 0; i < Pack.nEng; i++)   iE[i] *= km;
        for (int i = 0; i < Pack.nCoast; i++) iC[i] *= km;
    }
    if (pulseBlk) {
        if (!pulseRng) pulseRng = esp_random() | 1u;
        const Layer &M = mCoast ? Pack.coast[mi] : Pack.eng[mi];
        const Pos &MP  = mCoast ? pCoast[mi] : pEng[mi];
        const float inc = mCoast ? iC[mi] : iE[mi];
        const float invPer = 1.0f / M.pulsePer, invHw = 2.0f / fmaxf(Lope.width, 0.1f), len = (float)M.len;
        float pos = (float)MP.i + MP.f;
        const int code = mCoast ? 100 + mi : mi;
        if (code != pulseMaster) {            // handover: carry the count, re-aim the grid
            pulseMaster = code; pulseLastK = (int)floorf((pos - M.pulseOff) * invPer + 0.5f);
        }
        const size_t nf = frames < DC_AUDIO_FRAMES ? frames : DC_AUDIO_FRAMES;
        const float bUp = 1.0f - expf(-1000.0f / (8.0f * DC_OUT_RATE));                      // ~8 ms swell
        const float bDn = 1.0f - expf(-1000.0f / (fmaxf(Lope.bodyMs, 10.0f) * DC_OUT_RATE)); // settle
        for (size_t i = 0; i < nf; i++) {
            float q = (pos - M.pulseOff) * invPer;
            int k = (int)(q + 1024.5f) - 1024; float d = q - (float)k;   // floor without floorf
            int dk = k - pulseLastK; if (dk < -1) dk += M.pulsePpl;
            if (dk > 0) { pulseLastK = k; pulseCnt += (uint32_t)dk; pulseNewPulse(lopeA, limA); }
            else if (dk < 0) pulseLastK = k;
            float u = d * invHw, w = 0.0f;
            if (u > -1.0f && u < 1.0f) { w = 1.0f - u * u; w *= w; }
            pulseBody += (pulseBodyTgt - pulseBody) * (pulseBodyTgt > pulseBody ? bUp : bDn);
            pulseG[i] = (1.0f + (pulseCurG - 1.0f) * w) * pulseBody;
            pulseB[i] = pulseCurB * w;
            pos += inc; if (pos >= len) pos -= len;
        }
        eMul = 1.0f + (Lope.eng - 1.0f) * lopeA;
        xMul = 1.0f + (Lope.exh - 1.0f) * lopeA;
    } else pulseMaster = -1;

    {
        float blk = (float)frames;
        for (int i = 0; i < Pack.nEng; i++)
            if (gE[i] <= 0.01f) advance(pEng[i], Pack.eng[i].len, iE[i] * blk);
        for (int i = 0; i < Pack.nCoast; i++)
            if (gC[i] <= 0.01f) advance(pCoast[i], Pack.coast[i].len, iC[i] * blk);
    }

    pc1 = ESP.getCycleCount();
    // ---- LADDER READS + PULSE LOPE, in their OWN loop into block buffers ----
    // Split out of the main per-sample loop (2026-10-30): that loop is huge,
    // and every bit of state added to it spilled registers - the pulse lope
    // cost ~16% CPU in there (measured 82% -> 98-109%, dropouts).
    bool packMono = true;
    for (int i = 0; i < Pack.nEng; i++)   if (Pack.eng[i].dataR)   packMono = false;
    for (int i = 0; i < Pack.nCoast; i++) if (Pack.coast[i].dataR) packMono = false;
    static float busEL[DC_AUDIO_FRAMES], busER[DC_AUDIO_FRAMES], busXL[DC_AUDIO_FRAMES], busXR[DC_AUDIO_FRAMES];
    for (size_t n = 0; n < frames && n < DC_AUDIO_FRAMES; n++) {
        float eL = 0, eR = 0;      // engine bus
        float xL = 0, xR = 0;      // exhaust bus

        // ADVANCE EVERY RUNG, read only the audible ones.
        //
        // The skip used to cover the advance too, so a rung that faded out had
        // its playback position FROZEN, and coming back to that rpm resumed from
        // exactly where it stopped. Cruising at a steady rpm therefore played one
        // ~1.2 s loop from a fixed phase, over and over, with the two crossfading
        // rungs locked at a constant phase offset. That is the 'sounds too
        // looping while cruising' - the repeat was real and perfectly periodic.
        //
        // Advancing silent rungs keeps every loop running at its own rate as if
        // it had never stopped, so the phase relationship between rungs keeps
        // evolving and no cycle repeats against the same neighbour twice. The
        // advance itself is a few adds - it is READ_LR, with its PSRAM taps, that
        // costs, and that is still gated.
        // Read gate at 0.002 (-54 dB), not 0.02 (-34 dB). A rung switching on at
        // -34 dB appears instantly, which is a step of ~0.01 against a 0.5 bus -
        // small but a genuine discontinuity, and the per-rung wander now moves
        // gains across that boundary constantly. -54 dB is inaudible, so the
        // switch-on is too. Costs one more rung's taps in the handover region.
        for (int i = 0; i < Pack.nEng; i++) if (gE[i] > 0.01f) {
            float l = 0, r = 0;
            READ_LR(Pack.eng[i], pEng[i], l, r);
            eL += l * gE[i]; eR += r * gE[i];
            advance(pEng[i], Pack.eng[i].len, iE[i]);
        }
        if (idleG > 0.02f) {
            float l = 0, r = 0;
            READ_LR(Pack.idle, pIdle, l, r);
            eL += l * idleG; eR += r * idleG;
            advance(pIdle, Pack.idle.len, pIdleInc);
        }
        for (int i = 0; i < Pack.nCoast; i++) if (gC[i] > 0.01f) {
            float l = 0, r = 0;
            READ_LR(Pack.coast[i], pCoast[i], l, r);
            xL += l * gC[i]; xR += r * gC[i];
            advance(pCoast[i], Pack.coast[i].len, iC[i]);
        }
        for (int i = 0; i < Pack.nPull; i++) if (gL[i] > 0.02f) {
            float l = 0, r = 0;
            READ_LR(Pack.pull[i], pLoad[i], l, r);
            xL += l * gL[i] * duck; xR += r * gL[i] * duck;
            advance(pLoad[i], Pack.pull[i].len, iL[i]);
        }
        // oL/oR is the MASTER bus. It starts as the engine ladder only; the
        // exhaust bus (xL/xR) is summed in further down, AFTER its own shelf,
        // chop and limiter - exactly as the browser wires exBus.
        if (pulseBlk) {
            float g = pulseG[n], bw = pulseB[n];
            plE += (eL - plE) * PULSE_LP_K; plX += (xL - plX) * PULSE_LP_K;   // ~150 Hz, the chest of each hit
            eL = pulseSoft((eL * g + plE * bw) * eMul);
            xL = pulseSoft((xL * g + plX * bw) * xMul);
            if (packMono) { eR = eL; xR = xL; }                  // a mono pack: R is L (READ_LR duplicates it)
            else { eR = pulseSoft((eR * g + plE * bw) * eMul); xR = pulseSoft((xR * g + plX * bw) * xMul); }
            // the page's soft limit x/(1+0.3|x|) on the lifted hits (x5 + bass is a
            // lot of peak) - as a polynomial: four float DIVIDES a sample took the
            // render past real time and the watchdog rebooted it (measured)
        }
        busEL[n] = eL; busER[n] = eR; busXL[n] = xL; busXR[n] = xR;
    }

    pc2 = ESP.getCycleCount();
    for (size_t n = 0; n < frames; n++) {
        float wl = frand(), wr = frand();       // decorrelated noise for stereo turbo
        // TWO BUSES, matching the browser's graph:
        //   exhaust (coast+pull)  -> low shelf -> limiter gate -> master
        //   engine  (eng+idle)    -------------------------------> master
        // and turbo/knock/fan also go straight to master. The browser wires
        // exLS and limGate on exBus ALONE; this renderer had one bus, so the
        // chop, the shelf and the rev limiter were being applied to the turbo
        // whistle and the knock as well. A turbo does not stutter on the
        // limiter and a knock is not bass, so both were wrong.
        float eL = busEL[n], eR = busER[n];      // engine bus  (read above)
        float xL = busXL[n], xR = busXR[n];      // exhaust bus (read above)
        float oL = eL, oR = eR;
        // Keep the ENGINE LADDER on its own, so the misfire chop further down
        // can be applied to it WITHOUT also chopping the turbo, whoosh, knock
        // and fan that get summed onto oL between here and there. Those are not
        // combustion and a weak cylinder does not interrupt them.
        float engChopL = eL, engChopR = eR;

        // Per-firing amplitude shape. It decays across the firing period and used
        // to RESET INSTANTANEOUSLY, with fireAmp re-randomised on the same sample
        // - a step in gain applied to continuous audio, ~2.5dB at idle, 36 times
        // a second at 724rpm. A click per fire, summing to a buzz with harmonics
        // all the way up. Ramping the attack over ~0.45ms keeps the punch (under
        // 2% of the firing period) and loses the edge.
        firePhase += fireHz * invSR;
        if (firePhase >= 1) { firePhase -= 1; fireAmp = 1 + 0.10f * frand(); }
        // no firing pulse with the injectors cut (jake / overrun): nothing is burning
        float popD = 0.55f * (0.45f + 0.55f * workLvl) * fuelK, dd = 1 - firePhase;
        float popMul = fireAmp * ((1 - popD) + popD * dd * dd);
        popG += (popMul - popG) * 0.05f;
        // The firing shape is the engine firing, so it belongs on BOTH buses.
        // Applying it only to the master bus here left the exhaust - the loudest
        // part of the sound - with no per-event shape at all.
        oL *= popG; oR *= popG;
        xL *= popG; xR *= popG;

        if (whineAmp > 0.001f) {
            float yA = c1A * yA1 + c2 * yA2 + wl * gAB; yA2 = yA1; yA1 = yA;
            float yB = c1B * yB1 + c2 * yB2 + wl * gAB; yB2 = yB1; yB1 = yB;
            oL += (yA + (2.2f/4.2f) * yB) * whineAmp * loopsLevel;   // browser A:B = 4.2:2.2
            float yAr = c1A * yA1R + c2 * yA2R + wr * gAB; yA2R = yA1R; yA1R = yAr;
            float yBr = c1B * yB1R + c2 * yB2R + wr * gAB; yB2R = yB1R; yB1R = yBr;
            oR += (yAr + (2.2f/4.2f) * yBr) * whineAmp * loopsLevel;
        }
        if (whooshAmp > 0.001f) {
            whooshLP += (wl - whooshLP) * 0.05f; whooshLPR += (wr - whooshLPR) * 0.05f;
            oL += whooshLP * whooshAmp * loopsLevel;
            oR += whooshLPR * whooshAmp * loopsLevel;
        }
        // NOTE the *loopsLevel: every other synth layer is gated by it so the
        // whole engine fades out together on shutdown. This one was not, so the
        // turbo sample carried on whining after the motor had died.
        if (turboSampG > 0.002f && Pack.turbo.ok()) {
            float l = 0, r = 0;
            READ_LR(Pack.turbo, pTurbo, l, r);
            oL += l * turboSampG * loopsLevel; oR += r * turboSampG * loopsLevel;
            advance(pTurbo, Pack.turbo.len, turboInc);
        }
        if (knockAmp > 0.002f) {
            knPh += knHz * invSR; if (knPh >= 1) knPh -= 1;
            // SAWTOOTH, not a square. The browser drives the knock gain with
            //     knockLFO.type = 'sawtooth'   at clamp(rpm/20, 18, 130) Hz
            // and a saw gives every knock a decaying tail - a THUD. The square
            // this had instead held each burst flat for half a cycle and then
            // stepped to 0.18, which reads as a BUZZ, and is a large part of why
            // the board did not sound like the browser. Same rate, same depth,
            // different envelope shape.
            float knGate = 1.0f - knPh;          // 1 -> 0 across each firing event
            knGate = 0.18f + 0.82f * knGate * knGate;   // squared = sharper attack
            float kn = knC1 * knA1 + knC2 * knA2 + wl * knGn; knA2 = knA1; knA1 = kn;
            float knr = knC1 * knA1R + knC2 * knA2R + wr * knGn; knA2R = knA1R; knA1R = knr;
            oL += kn * knockAmp * knGate * loopsLevel;
            oR += knr * knockAmp * knGate * loopsLevel;
        }
        if (fanG > 0.002f) {
            fanLP += (wl - fanLP) * 0.045f; fanLPR += (wr - fanLPR) * 0.045f;
            oL += fanLP * fanG * loopsLevel; oR += fanLPR * fanG * loopsLevel;
        }
        if (hydG > 0.002f || relG > 0.002f) {
            // linear-interpolated loops into the 44.1 kHz mix: the SD pack's
            // hydpump.wav / hydflow.wav if it has them, else the baked Cat 323
            // loops if this build has them, else nothing.
            const Layer &LP = Pack.hydPump, &LF = Pack.hydFlow;
#if HAVE_BAKED_HYD
            const uint32_t NP = LP.ok() ? LP.len : sizeof(HYD_PUMP), NF = LF.ok() ? LF.len : sizeof(HYD_FLOW);
            const float RP = LP.ok() ? LP.rate : HYD_PUMP_RATE, RF = LF.ok() ? LF.rate : HYD_FLOW_RATE;
#else
            const uint32_t NP = LP.ok() ? LP.len : 0, NF = LF.ok() ? LF.len : 0;
            const float RP = LP.rate, RF = LF.rate;
#endif
            auto smpP = [&](uint32_t i) -> float {
                if (LP.ok()) return LP.data[i] * (1.0f / 32768.0f);
#if HAVE_BAKED_HYD
                return HYD_PUMP[i] * (1.0f / 128.0f);
#else
                return 0.0f;
#endif
            };
            auto smpF = [&](uint32_t i) -> float {
                if (LF.ok()) return LF.data[i] * (1.0f / 32768.0f);
#if HAVE_BAKED_HYD
                return HYD_FLOW[i] * (1.0f / 128.0f);
#else
                return 0.0f;
#endif
            };
            if (NP) {
                if (hydPos >= (float)NP) hydPos = 0;
                uint32_t i0 = (uint32_t)hydPos, i1 = (i0 + 1) % NP;
                float fr = hydPos - (float)i0;
                float a0 = smpP(i0), a1 = smpP(i1);
                float pw = (a0 + (a1 - a0) * fr) * hydG;
                hydPos += RP * invSR; if (hydPos >= (float)NP) hydPos -= (float)NP;
                oL += pw; oR += pw * 0.92f;
            }
            if (NF && relG > 0.002f) {
                if (relPos >= (float)NF) relPos = 0;
                uint32_t j0 = (uint32_t)relPos, j1 = (j0 + 1) % NF;
                float fj = relPos - (float)j0;
                float b0 = smpF(j0), b1 = smpF(j1);
                float rw = (b0 + (b1 - b0) * fj) * relG;
                relPos += RF * invSR; if (relPos >= (float)NF) relPos -= (float)NF;
                oL += rw * 0.95f; oR += rw;
            }
        }
        if (jakeLvl > 0.001f) for (int i = 0; i < Pack.nJake; i++) if (gJ[i] > 0.02f) {
            float l = 0, r = 0;
            READ_LR(Pack.jake[i], pJake[i], l, r);
            // jake is wired to exBus in the browser, so it gets the shelf and
            // the limiter gate like the exhaust it comes out of.
            xL += l * gJ[i]; xR += r * gJ[i];
            advance(pJake[i], Pack.jake[i].len, iJ[i]);
        }

        // Both of these modulate gain, and BOTH used to do it in a single sample.
        // A one-sample gain jump is a step discontinuity - a click - no matter how
        // clean the samples feeding it are. The chop snapped back to 1.0 the
        // instant chopDur crossed zero, twice per event, 3-6 times a second; the
        // limiter duty gate jumped straight between 1.0 and 0.10, a 20dB edge at
        // 22Hz. Ramping both over ~0.65ms still reads as a hard chop and a ragged
        // stutter, but the edge is no longer a click.
        // THE LOPE FOLLOWS THE SIM'S EVENTS (EngineSim::chopMiss). One per
        // engine cycle on the weak cylinder: a skipped or partial shot DUCKS the
        // engine (up to -12 dB on a dead hole), a fat one BARKS (+~2.5 dB), and
        // the governor hunt swells it all +/-~1.5 dB, louder on the fuel-on
        // half. The engine runs at full level between events - the old renderer
        // held it at a -11 dB floor and let it up only for the barks.
        float chopTgt = 1.0f;
        if (chopD > 0.02f) {
            if (evSeq != lastEvSeq) { lastEvSeq = evSeq; chopDur = evDur; chopAmp = evAmp; chopBarkNow = evBark; }
            chopDur -= invSR;
            if (chopDur > 0)
                chopTgt = chopBarkNow ? (1.0f + 0.35f * chopD * chopAmp)
                                      : fmaxf(0.25f, 1.0f - 0.75f * chopD * chopAmp);
            chopTgt *= 1.0f + 0.18f * evHunt * fminf(chopD, 1.0f);
            chopTgt = 1.0f + (chopTgt - 1.0f) * (1.0f - lopeA);   // the pulse lope replaces it
        }
        if (chopG < chopMinDbg) chopMinDbg = chopG;
        if (chopG > chopMaxDbg) chopMaxDbg = chopG;
        if (chopDur > 0 && !chopInEvDbg) { chopInEvDbg = true; chopHitsDbg++; }
        if (chopDur <= 0) chopInEvDbg = false;

        // The fast direction is now UP - a bark is a transient attack, and the
        // decay back to the floor is what gives it the crack. Both are quick
        // because the measured bursts are only ~25 ms wide; a slow release
        // would smear consecutive barks into the continuous rumble I kept
        // getting before.
        float chopK = (chopTgt > chopG) ? 0.45f : 0.060f;
        chopG += (chopTgt - chopG) * chopK;
        // ---- EXHAUST BUS CHAIN (browser: exBus -> exLS -> limGate -> m) ----
        // Everything in this block applies to the EXHAUST ONLY, which is how the
        // browser wires it. Previously the shelf, the misfire chop and the rev
        // limiter were applied to the summed output, so the turbo whistle and
        // the knock got chopped and limited too. A turbo does not stutter
        // because the engine is on the limiter.

        // 1. low shelf, 130 Hz +5 dB. This is the chest of the sound, and the
        //    renderer had nothing like it - only a high-pass taking bottom away.
        //    One-pole: split at 130 Hz, add back 10^(5/20)-1 = 0.778x the low
        //    band.
        lsL += (xL - lsL) * LS_K;
        lsR += (xR - lsR) * LS_K;
        xL += lsL * LS_BOOST;
        xR += lsR * LS_BOOST;

        // 2. misfire / lope chop
        //
        // THE CHOP HITS BOTH BUSES. It used to multiply only the exhaust, so on
        // an idling engine - where the engine ladder carries most of what you
        // hear and the exhaust layer is relatively quiet - the gallop was
        // almost inaudible. Turning the depth up did not help, because the
        // layer being chopped was not the loud one.
        //
        // Physically the chop belongs on both: a cylinder that does not fuel
        // properly makes less torque, so the combustion thump in the ENGINE
        // note drops at the same instant the exhaust pulse does. Chopping only
        // the pipe is like muting the tailpipe of a misfiring engine and
        // expecting the block to sound healthy.
        //
        // NOT equally, though. The exhaust pulse is the fuel burning, so it
        // nearly vanishes. The engine note also carries mechanical noise -
        // valvetrain, injector knock, piston slap, blower whine - which a weak
        // cylinder does not silence at all. So the engine bus gets a shallower
        // cut: at chopG 0.08 the exhaust is nearly gone while the engine side
        // only drops to about 0.45, which is what keeps it sounding like an
        // engine lumping rather than the audio cutting out.
        // Applied to the ENGINE LADDER ONLY, captured before the turbo, whoosh,
        // knock and fan were summed onto oL. Chopping oL here would stutter all
        // of those too - and a turbo does not hiccup because one cylinder is
        // lazy, which is the same mistake this file already documents for the
        // rev limiter. engChop was taken at the merge point for exactly this.
        float chopE = 1.0f - (1.0f - chopG) * 0.60f;
        oL += engChopL * (chopE - 1.0f);
        oR += engChopR * (chopE - 1.0f);
        xL *= chopG; xR *= chopG;

        // 3. rev limiter gate, 22 Hz square (browser limOsc)
        limEnv += ((lim ? 1.0f : 0.0f) - limEnv) * 0.02f;
        float limTgt = 1.0f;
        if (limEnv > 0.01f) {
            limPh += 22 * invSR;
            if (limPh >= 1) { limPh -= 1; limSeed = limSeed * 1664525u + 1013904223u; }
            float dutyC = 0.40f + ((limSeed >> 12) & 0xff) / 255.0f * 0.18f;
            limTgt = (1 - limEnv) + limEnv * ((limPh < dutyC) ? 1.0f : 0.10f);
            if (pulseBlk || !bigPlunger) limTgt = 1.0f;   // pulses pop instead / other injectors just rest
        }
        limG += (limTgt - limG) * 0.035f;
        xL *= limG; xR *= limG;
        if (limEnv > 0.01f && limPh < 0.05f && !pulseBlk && bigPlunger) { xL += frand() * 0.16f * limEnv; xR += frand() * 0.16f * limEnv; }

        // 4. exhaust joins the master bus
        oL += xL; oR += xR;

        // ---- action sounds -------------------------------------------------
        // These sum onto their OWN bus, not onto oL/oR, for two reasons.
        //
        // 1. They must not be chopped by the rev limiter or the misfire chop -
        //    a horn does not stutter because the engine is on the limiter.
        // 2. They must not be multiplied by `grunt`. grunt is a load-and-EGT
        //    term, so riding it made the horn 4.3 dB louder pulling a hill than
        //    at idle, and made the LOW AIR ALARM - a safety warning - scale with
        //    engine load. horn.wav also peaks at full scale, so on top of the
        //    engine it drove the tanh to 3.9 and clipped 38% of samples.
        float aL = 0.0f, aR = 0.0f;
        // SAMPLED HORN - matched to the poor-mans-beier firmware, which is the
        // reference for how this should behave.
        //
        // beier (DrivingBrain.cpp:1070, AudioEngine.cpp:224/294):
        //   press   -> playOneShot() from sample 0, envCurrent = 1.0 INSTANTLY,
        //              then loop = true so it sounds as long as you hold it
        //   release -> fadeOutAndStop(voice, cfHornOut) = 600 ms LINEAR ramp,
        //              and the sample KEEPS PLAYING AND LOOPING through it
        //
        // The previous version here got two of those three wrong: it latched at
        // the end of the clip instead of looping, and it FROZE the playback
        // position on release instead of letting the horn keep sounding as it
        // faded. A real air horn blows for as long as the valve is open and then
        // bleeds down - it does not stop dead partway through, and it cannot
        // run out while you are still holding the button.
        {
            if (hornOn) {
                hornGate = 1.0f;                  // instant attack, held at full
            } else if (hornGate > 0.0f) {
                hornGate -= HORN_FADE_STEP;       // 600 ms linear bleed-down
                if (hornGate < 0.0f) hornGate = 0.0f;
            }
            if (Pack.horn.ok() && hornGate > 0.0f) {
                float l = readBuf(Pack.horn.data, Pack.horn.len, hornPos);
                float r = Pack.horn.dataR ? readBuf(Pack.horn.dataR, Pack.horn.len, hornPos) : l;
                float g = 0.9f * hornLevel * hornGate;
                aL += l * g; aR += r * g;
                // advance and WRAP: loops for as long as it is sounding,
                // release fade included.
                advance(hornPos, Pack.horn.len, Pack.horn.rate * invSR);
            }
        }
        // Synth horn - used when the pack carries no horn role. Two detuned
        // reeds a fifth apart, which is what a real air horn is.
        //
        // This sits POST-limiter, so neither chopG nor limG can contain it. The
        // four partials sum coherently to 1.20, so at the old 0.30 it reached
        // 0.36 peak and, on top of a 0.60 engine, parked the tanh at full scale
        // for the whole blast - a held near-square top, which is precisely the
        // class-D crackle this renderer works so hard to avoid elsewhere.
        // 0.12 keeps the sum inside the headroom the limiter left behind.
        if (!Pack.horn.ok()) {
            // hornGate is already advanced above with the beier envelope. Do NOT
            // ramp it again here: that applied the filter twice per sample and
            // gave the synth horn a different attack and release from the
            // sampled one for no reason.
            if (hornGate > 0.0f) {
                synHornPh += 311.0f * invSR;
                if (synHornPh >= 1.0f) synHornPh -= 1.0f;
                float a = synHornPh * 2.0f * (float)M_PI;
                float h = sinf(a) * 0.52f + sinf(a * 1.4983f) * 0.40f
                        + sinf(a * 2.0f) * 0.18f + sinf(a * 2.9966f) * 0.10f;
                float g = 0.12f * hornLevel * hornGate;
                aL += h * g; aR += h * g;
            }
        }
        if (aoActive && aoBuf) {
            float l = readBuf(aoBuf->data, aoBuf->len, aoPos);
            float r = aoBuf->dataR ? readBuf(aoBuf->dataR, aoBuf->len, aoPos) : l;
            aL += l * aoGain; aR += r * aoGain;
            advance(aoPos, 0, aoBuf->rate * invSR);     // len 0 = do not wrap
            if (aoPos.i >= aoBuf->len) aoActive = false;
        }

        // ---- action one-shots (PSRAM resident) ----
        for (int v = 0; v < DC_SHOT_VOICES; v++) {
            ShotVoice &S = _sv[v];
            if (!S.active()) continue;
            float l, r;
            // Every clip in the pack is 44100, so fr is always 0 and the cubic
            // kernel evaluates to exactly y0. Reading the sample directly is
            // bit-identical and drops 4 PSRAM taps per voice to 1 - which
            // matters, because three tanhf calls already took this render to
            // 96% of real time once.
            if (S.buf->rate == DC_OUT_RATE) {
                // A looping voice can be one block past the end before the wrap
                // below runs, and this path indexes data[] with no bound.
                uint32_t i = (S.buf->len && S.pos.i >= S.buf->len)
                             ? (S.pos.i % S.buf->len) : S.pos.i;
                l = (float)S.buf->data[i] * (1.0f / 32768.0f);
                r = S.buf->dataR ? (float)S.buf->dataR[i] * (1.0f / 32768.0f) : l;
            } else {
                l = readBuf(S.buf->data, S.buf->len, S.pos);
                r = S.buf->dataR ? readBuf(S.buf->dataR, S.buf->len, S.pos) : l;
            }
            float g = S.gain * S.env;
            aL += l * g; aR += r * g;
            if (fabsf(l * g) > shotPeakDbg) shotPeakDbg = fabsf(l * g);
            advance(S.pos, 0, S.buf->rate * invSR);      // len 0 = never wrap

            // Release ramp, ~3 ms. Not optional: these clips are hard-cut
            // mid-waveform with no fade - pbrakeon ends at +23490, which is
            // 0.72 of full scale snapping to zero. That is the same step
            // discontinuity the chop, limiter and horn gates were each ramped
            // to avoid.
            if (S.loop) {
                // Looping voice: wrap and never run the end-of-clip release.
                // Only an explicit stopSlot() (which clears loop) ends it.
                if (S.buf->len && S.pos.i >= S.buf->len) S.pos.i %= S.buf->len;
                if (S.rel) S.env -= 1.0f / 132.0f;
                if (S.env <= 0.0f) { S.buf = nullptr; S.env = 0.0f; }
            } else {
                uint32_t tail = (S.buf->len > 160) ? S.buf->len - 160 : 0;
                if (S.rel || S.pos.i >= tail) S.env -= 1.0f / 132.0f;
                if (S.env <= 0.0f || S.pos.i >= S.buf->len) { S.buf = nullptr; S.env = 0.0f; }
            }
        }

        // ---- LOW AIR ALARM ----
        // The sim has flagged this since day one (warn bit 0x01, below 55psi) and
        // nothing ever made a sound. Pulsing buzzer, ~660Hz, on until the tank
        // recovers - it is a warning, so it is deliberately annoying. The gate is
        // ramped rather than switched: a hard on/off on a square wave is a click
        // at every pulse, the same mistake the chop and the limiter both made.
        if (lowAir) {
            laPh    += 660.0f * invSR; if (laPh    >= 1) laPh    -= 1;
            laPulse += 2.2f   * invSR; if (laPulse >= 1) laPulse -= 1;
            float want = (laPulse < 0.55f) ? 1.0f : 0.0f;
            laGate += (want - laGate) * 0.02f;
        } else laGate += (0.0f - laGate) * 0.02f;
        if (laGate > 0.002f) {
            float sq = (laPh < 0.5f ? 1.0f : -1.0f) * lowAirLevel * laGate;
            aL += sq; aR += sq;
        }

        if (backupOn && loopsLevel > 0.05f) {       // ~1kHz, 350ms on/off
            beepCtr++;                               // gated, or switching off in
            uint32_t per = (uint32_t)(0.70f * DC_OUT_RATE);  // reverse left it beeping
            if ((beepCtr % per) < (uint32_t)(0.35f * DC_OUT_RATE)) {
                beepPhase += 1000.0f * invSR; if (beepPhase >= 1) beepPhase -= 1;
                float sq = ((beepPhase < 0.5f) ? 0.22f : -0.22f) * beepLevel;
                aL += sq; aR += sq;
            }
        } else beepCtr = 0;

        // ---- speaker-protection high-pass (2-pole Butterworth) ----
        // Before the limiter on purpose: sub-bass the speaker cannot reproduce was
        // eating limiter headroom as well as making the cone crackle, so removing
        // it here means the audible band gets the headroom instead.
        if (hpHz > 20.0f) {
            float sm = oL, smR = oR;
            oL = hb0 * sm + hb1 * hpX1 + hb2 * hpX2 - ha1 * hpY1 - ha2 * hpY2;
            hpX2 = hpX1; hpX1 = sm;  hpY2 = hpY1; hpY1 = oL;
            oR = hb0 * smR + hb1 * hpX1R + hb2 * hpX2R - ha1 * hpY1R - ha2 * hpY2R;
            hpX2R = hpX1R; hpX1R = smR; hpY2R = hpY1R; hpY1R = oR;
        }

        // masterVol is NOT part of the drive. On the Teensy it was dacVolume, a
        // trim applied AFTER the limiter in the analog domain. Folding it into
        // dm (as this did) means turning the volume down also reduces
        // compression, so output falls faster than the gain asked for and the
        // control stops behaving like a volume - and it made volMaster and
        // audioDrive the same control multiplied together.
        // Engine drive rides grunt. The action bus gets a FIXED drive so its
        // level does not move with engine load, and is summed after the tanh so
        // a full-scale horn cannot push the engine's limiter into flattening.
        // A NaN anywhere upstream - e.g. /set?key=fuel0&val=nan reaching the
        // sim - poisons workLvl, which is persistent member state with no reset,
        // so the output locks to full-scale buzz or silence and does NOT recover
        // even after the NaN leaves. Scrub here rather than hunt every source.
        if (!isfinite(oL)) oL = 0.0f;
        if (!isfinite(oR)) oR = 0.0f;
        if (!isfinite(workLvl)) workLvl = 0.0f;
        // MEASURED on the bench pulling to 65 mph: drive ran 1.9-2.5 with clip
        // peaking at 13% - one sample in eight hard-clipping at high throttle.
        // That is the crackle, and the arithmetic says why: makeup is 1.4 and
        // grunt reaches ~2.4 with load and coolant temp, so dm hits 3.4 and
        // tanhf(0.65 * 3.4) sits deep in saturation. The header note above
        // already warned that dm is 'a trim on top of an automatic drive' - the
        // trim and the automatic part simply were never added up.
        //
        // Cap it. 1.8 keeps the load-dependent grunt audible (it still moves
        // 1.4 -> 1.8, which is the point of grunt) while keeping the tanh out of
        // hard saturation, and the compressor downstream now handles the peaks
        // that used to reach the clipper. Loudness lost here is loudness the
        // GAIN_SLOT rework gets back properly, in the amp, without distortion.
        float dm = makeup * grunt;
        if (!isfinite(dm)) dm = 1.0f;
        if (dm > DM_MAX) dm = DM_MAX;
        // Soft-clip the action bus WITHOUT tanhf. Three tanhf calls per sample
        // (two here plus the engine's) pushed the render to 96% of real time
        // with the horn held - one slow block from a dropout. This rational
        // approximation x/(1+|x|) has the same shape where it matters, costs a
        // divide instead of a transcendental, and never exceeds +/-1.
        float axL = aL * makeup, axR = aR * makeup;
        float aOutL = axL / (1.0f + fabsf(axL));
        float aOutR = axR / (1.0f + fabsf(axR));
        // Peak over the ENGINE bus AND the action bus. This measured oL alone,
        // so every one-shot - horn, air brake, start, stop, park brake - was
        // invisible to the telemetry: `pk` read 0.00 with the engine off no
        // matter what was sounding, which made the start/stop and horn problems
        // impossible to diagnose from the heartbeat and sent me chasing voice
        // stealing that may never have been happening.
        { float a = fabsf(oL) + fabsf(aOutL); if (a > outPeak) outPeak = a; }
        // drive/clip are measured AFTER the compressor, below - measuring here
        // reported the pre-compression level and made `clip` meaningless: it
        // read 0% while `drive` peaked at 1.08, which cannot both be true of
        // the same signal.
        // ---- COMPRESSOR (browser: createDynamicsCompressor) ----------------
        //   threshold -6 dB, ratio 3:1, attack 4 ms, release 150 ms
        // This is NOT the same as the soft clip below and does not replace it.
        // A clipper reshapes the waveform (adds harmonics); a compressor rides
        // the level envelope and leaves the shape alone. The browser has both -
        // comp on the master, tanh only inside the synth path - and the glue
        // that makes the browser sound cohesive is the compressor. The renderer
        // had only the clipper, which is why individual layers stayed audibly
        // separate instead of sounding like one engine.
        //
        // Peak detector on max(|L|,|R|) so stereo stays coherent - compressing
        // the channels independently would pump the image.
        // The first version of this POPPED at high level, and the cause was my
        // own: the detector tracked individual waveform PEAKS (attack 0.017 per
        // sample reaches 95% in ~4 ms, which at audio rate follows the signal,
        // not its envelope), so compEnv oscillated across the threshold on every
        // peak. Each crossing switched the if/else and stepped the gain, giving
        // amplitude modulation at the signal frequency - heard as grit that gets
        // worse the louder it is. Two fixes, both needed:
        //
        //  1. the detector follows the ENVELOPE, not the waveform. Attack is
        //     slow enough (~20 ms) that one cycle of a 40 Hz firing pulse cannot
        //     drag it across the threshold.
        //  2. the APPLIED gain is ramped separately. Even a perfect detector
        //     needs this: the branch boundary is still a discontinuity, and this
        //     renderer already learned that lesson on the chop and the limiter.
        {
            float pk = fmaxf(fabsf(oL), fabsf(oR));
            float k = (pk > compEnv) ? COMP_ATK : COMP_REL;
            compEnv += (pk - compEnv) * k;

            float gTgt = 1.0f;
            if (compEnv > COMP_THRESH) {
                // 3:1 above threshold: out = thr + (in-thr)/3
                float over = compEnv - COMP_THRESH;
                float want = COMP_THRESH + over * (1.0f / COMP_RATIO);
                gTgt = want / compEnv;
                if (gTgt < 0.05f) gTgt = 0.05f;      // never collapse the signal
            }
            // ramp the gain itself, ~1.5 ms, same trick as chopG and limG
            compG += (gTgt - compG) * 0.015f;
            oL *= compG; oR *= compG;
        }

        // `drive` is the tanh INPUT - how hard the saturator is being pushed.
        // Useful, but it is NOT a clip count: tanh output is bounded by
        // construction. `clip` must therefore be measured on the FINAL sample,
        // after the mix and the soft clip, where the only real clipping on this
        // board happens - the hard clamp two lines below.
        { float d = oL * dm; if (d < 0) d = -d; if (d > drivePeak) drivePeak = d; }

        // 0.92, not 0.80. The old value threw away 1.2 dB on top of everything
        // else, and with the compressor now riding the peaks there is no need
        // for that much static headroom.
        // HEADROOM GUARD. Everything above sums raw amplitudes onto two buses,
        // and several of them (jake, horn, the resonators) peak near full scale
        // on their own. When they coincide the sum rides the hard clamp for long
        // stretches - which is exactly the popping that kept coming back from a
        // different source each time. One shared gain riding the actual peak
        // catches all of them at once, including combinations I have not tested.
        //
        // It only engages above 0.95, so in normal operation it is inert and
        // costs nothing audible. Release is slow so it cannot pump.
        // The divide only happens on the rare samples that actually exceed the
        // ceiling; everything else is two fabsf and a compare.
        {   float pk = fmaxf(fabsf(oL), fabsf(oR)) * dm;
            float want = (pk > 0.95f) ? (0.95f / pk) : 1.0f;
            hdG += (want - hdG) * ((want < hdG) ? 0.08f : 0.0008f);
            oL *= hdG; oR *= hdG;
        }

        // NO tanhf here. Two calls per sample is 88,200 transcendentals a second,
        // and this renderer has crashed the task watchdog twice today at 103%.
        // The browser also has NO saturation on its master - its loudness comes
        // from the compressor alone - so removing this moves toward the reference
        // and buys the headroom at the same time.
        //
        // x/(1+|x|) is the same cheap soft clip already used on the action bus
        // just above, for exactly this reason. With the headroom guard now
        // riding the peak, this stage is a backstop rather than the main gain
        // control it used to be.
        float dL = oL * dm, dR = oR * dm;
        float mL = dL / (1.0f + fabsf(dL)) * 0.92f + aOutL * 0.45f;
        float mR = dR / (1.0f + fabsf(dR)) * 0.92f + aOutR * 0.45f;
        // SHAKE HANDS WITH DANGER goes in LAST, past the action bus's 0.45 ceiling
        // (Mat: "it has to be loud"). The engine is dead when it plays, so it
        // gets the whole speaker; the soft limiter below still rounds its peaks.
        if (dangerBlk) { mL += dangerBuf[n]; mR += dangerBuf[n]; }
        mL = mL / (1.0f + fabsf(mL) * 0.25f);
        mR = mR / (1.0f + fabsf(mR) * 0.25f);
        { float a = fabsf(mL); if (a >= 0.999f) _clipHits++; _clipTotal++; }
        if (mL >  1.0f) mL =  1.0f; else if (mL < -1.0f) mL = -1.0f;
        if (mR >  1.0f) mR =  1.0f; else if (mR < -1.0f) mR = -1.0f;
        int vL = (int)(mL * masterVol * 32767.0f);
        int vR = (int)(mR * masterVol * 32767.0f);
        out[2 * n + 0] = (int16_t)(vL > 32767 ? 32767 : vL < -32768 ? -32768 : vL);
        out[2 * n + 1] = (int16_t)(vR > 32767 ? 32767 : vR < -32768 ? -32768 : vR);
    }
    if (_clipTotal) {
        float f = (float)_clipHits / (float)_clipTotal;
        clipFrac += (f - clipFrac) * 0.05f;      // ~115 ms, matches _cpu's EMA
    }
    const uint32_t pc3 = ESP.getCycleCount();
    s_pc[0] += pc1 - pc0; s_pc[1] += pc2 - pc1; s_pc[2] += pc3 - pc2; s_pn++;
}
