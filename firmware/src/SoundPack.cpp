// =============================================================================
//  SoundPack.cpp - read a pack into PSRAM, index the roles, close the card
// =============================================================================
//  TWO paths, tried in this order:
//
//  1. manifest.txt + the individual WAVs. This is the FULL pack: seven engine
//     rungs and five separate exhaust rungs, which is the dual-ladder structure
//     the renderer was tuned against - a mechanical engine bank sounding under
//     the exhaust. Needs ~1.5 MB of PSRAM.
//
//  2. dieselcore.pak. This is the Teensy's fallback: four rungs, all labelled
//     "eng ", baked down to fit 488 KB of RAM. It works, but it is one collapsed
//     ladder and the engine bank is gone. Only used if the manifest is missing.
//
//  The Teensy build had to use (2) and stream it. We have 8 MB of octal PSRAM
//  and a 1.5 MB pack, so (1) loads resident and the card goes idle afterwards.
//  SdLoop, the two-voice handover and the underrun counters are all deleted.
//
//  The flash-baked C15 pack is also gone: c15_ats.h is 3.7 MB of const arrays
//  against a 3 MB app partition, so it could never have fitted here anyway.
//  No card means no samples, and AudioOut falls back to its synthesised engine.
// =============================================================================
#include <algorithm>
#include "SoundPack.h"
#include "Config.h"
#include <SD_MMC.h>

SoundPack Pack;

// ---------------------------------------------------------------------------
void SoundPack::release() {
    auto fl = [](Layer &l) { l.data = nullptr; l.dataR = nullptr; l.len = 0; };
    fl(idle); fl(horn); fl(air); fl(turbo); fl(lope);
    for (int i = 0; i < DC_MAX_LAYERS; i++) { fl(eng[i]); fl(coast[i]); fl(pull[i]); fl(jake[i]); }
    fl(shStart); fl(shStop); fl(shStarter); fl(shCrank); fl(shFireup); fl(shBov); fl(shAirDry); fl(shPbOn); fl(shPbOff); fl(hydPump); fl(hydFlow); fl(shDanger);
    for (int i = 0; i < 3; i++) fl(shTick[i]);
    nTick = 0;
    nEng = nCoast = nPull = nJake = 0;
    for (int i = 0; i < _nAlloc; i++) heap_caps_free(_allocs[i]);
    _nAlloc = 0;
    _bytes = 0;
    _loaded = false;
}

// Hand a failed allocation back so an aborted load does not burn an _allocs[]
// slot. The table is finite and psAlloc() returns null once it is full, which
// shows up as rungs silently missing from the ladder - silence at that rpm band,
// and very hard to trace back to here.
void SoundPack::psFree(int16_t *p) {
    if (!p) return;
    for (int i = 0; i < _nAlloc; i++) {
        if (_allocs[i] == p) {
            heap_caps_free(p);
            _allocs[i] = _allocs[--_nAlloc];
            return;
        }
    }
}

int16_t *SoundPack::psAlloc(uint32_t bytes) {
    if (_nAlloc >= (int)(sizeof(_allocs) / sizeof(_allocs[0]))) return nullptr;
    int16_t *p = (int16_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!p) {
        // Almost always the wrong memory_type for an N16R8: the generic S3 board
        // files assume quad PSRAM, and then this returns null with no other
        // symptom at all. See platformio.ini.
        Serial.printf("[pack] PSRAM alloc of %lu KB FAILED (%u KB free)\n",
                      (unsigned long)(bytes / 1024), (unsigned)(ESP.getFreePsram() / 1024));
        return nullptr;
    }
    _allocs[_nAlloc++] = p;
    return p;
}

// ---------------------------------------------------------------------------
//  WAV reader. 16-bit PCM only, mono or stereo; stereo is de-interleaved so the
//  render can read L and R independently at the same fractional position.
// ---------------------------------------------------------------------------
// THE CRANK LOOP, MADE SEAMLESS (Mat: "close the dead spot when it's cranking
// between the loops"). Measured through the board 2026-10-30: cranking.wav is
// 0.66 s and its last sample sits 0.14 away from its first (the clip peaks at
// 0.18), so every wrap was a step plus a hiccup in the churn, and two thin
// patches inside the clip dipped ~6 dB in the same place every loop.
//  1. fold the last 25 ms into the head with a linear crossfade and loop the
//     rest - the wrap now lands on the material the head was faded from;
//  2. lift anything below the clip's median 60 ms level toward it (never
//     cuts, at most +6 dB), so the churn has no thin spot.
static void sealCrankLoop(Layer &L) {
    const uint32_t F = (uint32_t)(0.025f * L.rate);
    if (!L.ok() || L.len < 8 * F) return;
    const uint32_t N = L.len - F;
    for (int c = 0; c < 2; c++) {
        int16_t *d = c ? L.dataR : L.data; if (!d) continue;
        for (uint32_t k = 0; k < F; k++) {
            float w = (float)k / (float)F;
            d[k] = (int16_t)lrintf(d[k] * w + d[N + k] * (1.0f - w));
        }
    }
    L.len = N;
    const uint32_t B = (uint32_t)(0.005f * L.rate);           // 5 ms blocks
    const int nb = (int)(N / B); if (nb < 16 || nb > 1024) return;
    float *e = (float *)malloc(sizeof(float) * nb * 3); if (!e) return;
    float *rms = e + nb, *srt = e + 2 * nb;
    for (int b = 0; b < nb; b++) { double a = 0;
        for (uint32_t i = b * B; i < (b + 1) * B; i++) { float v = L.data[i] * (1.0f / 32768.0f); a += v * v; }
        e[b] = (float)(a / B); }
    const int W = 6;                                           // +/-6 blocks = 60 ms, circular
    for (int b = 0; b < nb; b++) { double a = 0;
        for (int j = -W; j <= W; j++) a += e[(b + j + nb) % nb];
        rms[b] = sqrtf((float)(a / (2 * W + 1))); srt[b] = rms[b]; }
    std::nth_element(srt, srt + nb / 2, srt + nb);
    const float med = srt[nb / 2];
    for (int b = 0; b < nb; b++) e[b] = fminf(2.0f, fmaxf(1.0f, powf(med / fmaxf(rms[b], 1e-6f), 0.7f)));
    for (uint32_t i = 0; i < N; i++) {
        float fb = (float)i / B - 0.5f; int b0 = (int)floorf(fb); float t = fb - b0;
        float g = e[(b0 + nb) % nb] * (1 - t) + e[(b0 + 1) % nb] * t;
        for (int c = 0; c < 2; c++) { int16_t *d = c ? L.dataR : L.data; if (!d) continue;
            d[i] = (int16_t)constrain(lrintf(d[i] * g), -32767L, 32767L); }
    }
    free(e);
}

// WHERE THE FIRING PULSES ARE. A baked loop is a whole number of firing
// cycles at its rung rpm, so a pulse comes every (120/rpm)/6 s - six cylinders,
// four-stroke. Fold the loop's envelope onto one firing interval and take the
// loudest bin as the pulse peak. The pulse lope + limiter pops (EngineAudio)
// lift and drop these real combustion hits one at a time.
static void findPulses(Layer &L) {
    if (!L.ok() || L.rpm < 100) return;
    const float per = L.rate * 120.0f / L.rpm / 6.0f;
    if (per < 8 || per * 4 > L.len) return;
    const int BINS = 96; float acc[BINS] = {0}; uint32_t cnt[BINS] = {0};
    float e = 0, k = 1.0f - expf(-1.0f / (0.0015f * L.rate));
    for (uint32_t i = 0; i < L.len; i++) {
        e += (fabsf(L.data[i] * (1.0f / 32768.0f)) - e) * k;
        float ph = (float)i / per; ph -= (int)ph;
        int b = (int)(ph * BINS); if (b >= BINS) b = BINS - 1;
        acc[b] += e; cnt[b]++;
    }
    int best = 0;
    for (int b = 1; b < BINS; b++) if (cnt[b] && acc[b] / cnt[b] > acc[best] / fmaxf(cnt[best], 1)) best = b;
    L.pulsePer = per; L.pulseOff = (best + 0.5f) / BINS * per; L.pulsePpl = (int)lrintf(L.len / per);
}

bool SoundPack::loadWav(const char *dir, const char *file, Layer &L, float rpm) {
    char path[160];
    snprintf(path, sizeof(path), "%s/%s", dir, file);
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) { Serial.printf("[pack]   missing %s\n", path); return false; }

    uint8_t hdr[12];
    if (f.read(hdr, 12) != 12 || memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) {
        Serial.printf("[pack]   %s is not a WAV\n", file);
        f.close();
        return false;
    }

    uint16_t chans = 0, bits = 0;
    uint32_t rate = 0, dataLen = 0;
    bool haveFmt = false;

    // Walk the chunk list rather than assuming fmt-then-data: files written by
    // ffmpeg often carry a LIST/INFO chunk between the two.
    while (f.available() >= 8) {
        uint8_t ch[8];
        if (f.read(ch, 8) != 8) break;
        uint32_t sz = (uint32_t)ch[4] | ((uint32_t)ch[5] << 8) | ((uint32_t)ch[6] << 16) | ((uint32_t)ch[7] << 24);
        if (!memcmp(ch, "fmt ", 4)) {
            uint8_t fmt[16] = {0};   // a short fmt chunk left bits/rate as stack garbage
            uint32_t want = sz < 16 ? sz : 16;
            if (f.read(fmt, want) != (int)want) break;
            uint16_t tag = (uint16_t)fmt[0] | ((uint16_t)fmt[1] << 8);
            chans = (uint16_t)fmt[2] | ((uint16_t)fmt[3] << 8);
            rate  = (uint32_t)fmt[4] | ((uint32_t)fmt[5] << 8) | ((uint32_t)fmt[6] << 16) | ((uint32_t)fmt[7] << 24);
            bits  = (uint16_t)fmt[14] | ((uint16_t)fmt[15] << 8);
            if (tag != 1 || bits != 16 || chans < 1 || chans > 2) {
                Serial.printf("[pack]   %s: need 16-bit PCM mono/stereo, got tag %u %u-bit %uch\n",
                              file, tag, bits, chans);
                f.close();
                return false;
            }
            haveFmt = true;
            if (sz > want) f.seek(f.position() + (sz - want));
        } else if (!memcmp(ch, "data", 4)) {
            // Trust the file, not the header: a dataLen larger than the file
            // allocated the full amount and then failed the read, leaking it.
            uint32_t avail = (uint32_t)(f.size() - f.position());
            dataLen = (sz > avail) ? avail : sz;
            break;                       // position is now at the PCM
        } else {
            if (!f.seek(f.position() + sz + (sz & 1))) break;   // chunks are word-aligned
        }
    }

    if (!haveFmt || dataLen == 0) {
        Serial.printf("[pack]   %s: no fmt/data chunk\n", file);
        f.close();
        return false;
    }

    uint32_t frames = dataLen / (2u * chans);
    if (frames < 16) { Serial.printf("[pack]   %s too short\n", file); f.close(); return false; }

    int16_t *left = psAlloc(frames * 2);
    if (!left) { f.close(); return false; }
    int16_t *right = nullptr;
    if (chans == 2) {
        right = psAlloc(frames * 2);
        if (!right) { psFree(left); f.close(); return false; }
    }

    if (chans == 1) {
        // Read straight in, in chunks - one 100 KB read into PSRAM is fine but
        // SD_MMC is happier with a bounded buffer.
        uint32_t done = 0;
        while (done < frames) {
            uint32_t n = frames - done; if (n > 4096) n = 4096;
            int got = f.read((uint8_t *)(left + done), n * 2);
            if (got != (int)(n * 2)) { Serial.printf("[pack]   %s short read\n", file); f.close(); return false; }
            done += n;
        }
    } else {
        static int16_t tmp[2048 * 2];
        uint32_t done = 0;
        while (done < frames) {
            uint32_t n = frames - done; if (n > 2048) n = 2048;
            int got = f.read((uint8_t *)tmp, n * 4);
            if (got != (int)(n * 4)) { Serial.printf("[pack]   %s short read\n", file); f.close(); return false; }
            for (uint32_t k = 0; k < n; k++) {
                left[done + k]  = tmp[2 * k + 0];
                right[done + k] = tmp[2 * k + 1];
            }
            done += n;
        }
    }
    f.close();

    // Strip DC. Several clips in this pack carry a real offset - exh1000 is
    // +8% of full scale, airdry -0.91%, pbrakeon -0.53% - and it does damage in
    // three places: the RMS normaliser counted it as signal (so exh1000 is
    // ~1 dB quieter than its ladder neighbours), pickRung steps the output DC
    // when it crosses that rung, and a class-D amp passes DC straight to the
    // voice coil. The 170 Hz high-pass cannot save us either, because the
    // action bus bypasses it entirely. Cheapest to fix once, here, at boot.
    {
        int64_t sum = 0;
        for (uint32_t i = 0; i < frames; i++) sum += left[i];
        int32_t dc = (int32_t)(sum / (int64_t)frames);
        if (dc > 2 || dc < -2) {
            for (uint32_t i = 0; i < frames; i++) {
                int32_t v = (int32_t)left[i] - dc;
                left[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
            }
            Serial.printf("[pack]   %s: removed DC %+ld (%.2f%%)\n",
                          file, (long)dc, dc * 100.0f / 32768.0f);
        }
        if (right) {
            sum = 0;
            for (uint32_t i = 0; i < frames; i++) sum += right[i];
            dc = (int32_t)(sum / (int64_t)frames);
            if (dc > 2 || dc < -2)
                for (uint32_t i = 0; i < frames; i++) {
                    int32_t v = (int32_t)right[i] - dc;
                    right[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
                }
        }
    }

    L.data  = left;
    L.dataR = right;
    L.len   = frames;
    L.rate  = (float)rate;       // the header is authoritative, not the manifest
    L.rpm   = rpm;
    _bytes += frames * 2u * chans;
    return true;
}

// ---------------------------------------------------------------------------
static void sortLadder(Layer *a, int n) {
    // The rung picker and the crossfade both assume ascending rpm and will hand
    // over to the wrong clip otherwise.
    for (int i = 1; i < n; i++)
        for (int j = i; j > 0 && a[j - 1].rpm > a[j].rpm; j--) {
            Layer t = a[j - 1]; a[j - 1] = a[j]; a[j] = t;
        }
}

// ---------------------------------------------------------------------------
//  manifest.txt. One directive per line:
//      name   <text>
//      rate   <n>              (informational; each WAV header wins)
//      eng    <rpm> <file>
//      coast  <rpm> <file>
//      load   <rpm> <file>
//      jake   <rpm> <file>
//      turbo  <file>           (no rpm - it is pitched by boost, not rpm)
//      idle|lope|horn|air <file>
//  rpm figures are MEASURED off each loop, not taken from the filename: the
//  originals were wrong by up to 7%, and that number drives both the playback
//  pitch and where two rungs hand over.
// ---------------------------------------------------------------------------
bool SoundPack::loadManifest(const char *dir) {
    char path[160];
    snprintf(path, sizeof(path), "%s/manifest.txt", dir);
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) return false;

    Serial.printf("[pack] reading %s\n", path);
    char line[160];
    while (f.available()) {
        size_t n = f.readBytesUntil('\n', line, sizeof(line) - 1);
        line[n] = 0;
        char *s = line;
        while (*s == ' ' || *s == '\t' || *s == '\r') s++;
        if (*s == 0 || *s == '#') continue;

        char role[16] = {0}, a[96] = {0}, b[96] = {0};
        int got = sscanf(s, "%15s %95s %95s", role, a, b);
        if (got < 2) continue;

        if (!strcmp(role, "name")) {
            // take the rest of the line verbatim
            char *p = s + 4;
            while (*p == ' ' || *p == '\t') p++;
            char *e = p + strlen(p);
            while (e > p && (e[-1] == '\r' || e[-1] == '\n' || e[-1] == ' ')) *--e = 0;
            strncpy(_name, p, sizeof(_name) - 1);
            continue;
        }
        if (!strcmp(role, "rate")) continue;

        // roles with an rpm then a file
        if (got >= 3) {
            float rpm = atof(a);
            if (!strcmp(role, "eng")   && nEng   < DC_MAX_LAYERS) { if (loadWav(dir, b, eng[nEng], rpm))     nEng++;   continue; }
            if (!strcmp(role, "coast") && nCoast < DC_MAX_LAYERS) { if (loadWav(dir, b, coast[nCoast], rpm)) nCoast++; continue; }
            if (!strcmp(role, "load")  && nPull  < DC_MAX_LAYERS) { if (loadWav(dir, b, pull[nPull], rpm))   nPull++;  continue; }
            if (!strcmp(role, "jake")  && nJake  < DC_MAX_LAYERS) { if (loadWav(dir, b, jake[nJake], rpm))   nJake++;  continue; }
        }
        // roles that are just a file
        if (!strcmp(role, "turbo")) { loadWav(dir, a, turbo, 0); continue; }
        if (!strcmp(role, "lope"))  { loadWav(dir, a, lope,  0); continue; }
        if (!strcmp(role, "horn"))  { loadWav(dir, a, horn,  0); continue; }
        if (!strcmp(role, "air"))   { loadWav(dir, a, air,   0); continue; }
    }
    f.close();

    return (nEng + nCoast) > 0;
}

// ---------------------------------------------------------------------------
//  rclc.pak (or the older dieselcore.pak) - the Teensy's RAM-limited fallback. Header then an entry
//  table of {float rpm; uint32 ofs; uint32 frames; char role[4]}, then raw PCM.
// ---------------------------------------------------------------------------
bool SoundPack::loadPak(const char *dir) {
    struct PakEnt { float rpm; uint32_t ofs, frames; char role[4]; };

    char path[160];
    // rclc.pak first; dieselcore.pak (the pre-rename name) still loads
    snprintf(path, sizeof(path), "%s/rclc.pak", dir);
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) { snprintf(path, sizeof(path), "%s/dieselcore.pak", dir); f = SD_MMC.open(path, FILE_READ); }
    if (!f) return false;

    char magic[4];
    uint16_t ver = 0, cnt = 0;
    if (f.read((uint8_t *)magic, 4) != 4 || strncmp(magic, "DCPK", 4)) { f.close(); return false; }
    f.read((uint8_t *)&ver, 2);
    f.read((uint8_t *)&cnt, 2);
    if (ver != 1 || cnt == 0 || cnt > 64) { f.close(); return false; }

    Serial.printf("[pack] no manifest - falling back to %s (%u entries)\n", path, cnt);

    PakEnt ent[64];
    for (uint16_t k = 0; k < cnt; k++)
        if (f.read((uint8_t *)&ent[k], sizeof(PakEnt)) != (int)sizeof(PakEnt)) { f.close(); return false; }

    for (uint16_t k = 0; k < cnt; k++) {
        Layer *dst = nullptr;
        if      (!strncmp(ent[k].role, "eng ", 4) && nEng   < DC_MAX_LAYERS) dst = &eng[nEng];
        else if (!strncmp(ent[k].role, "exh ", 4) && nCoast < DC_MAX_LAYERS) dst = &coast[nCoast];
        else if (!strncmp(ent[k].role, "jake", 4) && nJake  < DC_MAX_LAYERS) dst = &jake[nJake];
        else if (!strncmp(ent[k].role, "turb", 4))                           dst = &turbo;
        else if (!strncmp(ent[k].role, "lope", 4))                           dst = &lope;
        // horn and air were missing here, so on a .pak pack Pack.horn and
        // Pack.air stayed permanently empty - the horn always fell through to
        // the synth, and triggerAir() silently did nothing at all.
        else if (!strncmp(ent[k].role, "horn", 4))                           dst = &horn;
        else if (!strncmp(ent[k].role, "air ", 4))                           dst = &air;
        if (!dst) continue;

        int16_t *p = psAlloc(ent[k].frames * 2);
        if (!p) { f.close(); return false; }
        if (!f.seek(ent[k].ofs)) { f.close(); return false; }
        if (f.read((uint8_t *)p, ent[k].frames * 2) != (int)(ent[k].frames * 2)) { f.close(); return false; }

        dst->data  = p;
        dst->dataR = nullptr;
        dst->len   = ent[k].frames;
        dst->rate  = 44100.0f;
        dst->rpm   = ent[k].rpm;
        _bytes += ent[k].frames * 2;

        if      (dst == &eng[nEng])     nEng++;
        else if (dst == &coast[nCoast]) nCoast++;
        else if (dst == &jake[nJake])   nJake++;
    }
    f.close();
    return (nEng + nCoast) > 0;
}

// ---------------------------------------------------------------------------
bool SoundPack::loadFromSD(const char *packName) {
    release();
    // Default pack folder is /packs/rclc; a card still laid out with the
    // pre-rename /packs/dieselcore keeps working.
    const char *nm = (packName && *packName) ? packName
                   : (SD_MMC.exists("/packs/rclc") || !SD_MMC.exists("/packs/dieselcore")) ? "rclc" : "dieselcore";
    strncpy(_name, nm, sizeof(_name) - 1);

    char dir[128];
    snprintf(dir, sizeof(dir), "/packs/%s", nm);

    bool ok = loadManifest(dir);
    if (!ok) { release(); strncpy(_name, nm, sizeof(_name) - 1); ok = loadPak(dir); }
    if (!ok) {
        Serial.printf("[pack] nothing loadable in %s\n", dir);
        release();
        return false;
    }

    // Action one-shots are not in the manifest and were historically not in the
    // .pak role table either, so pick them up by convention AFTER whichever
    // loader ran. Doing this only in the manifest path meant a .pak pack never
    // got a horn and triggerAir() did nothing.
    if (!horn.ok()) loadWav(dir, "horn.wav", horn, 0);
    if (!air.ok())  loadWav(dir, "airbrake1.wav", air, 0);
    // Action one-shots. Filenames are fixed across every pack and none of them
    // appear in a manifest, so they load by convention here - which also means
    // a .pak pack gets them, unlike the old manifest-only pickup.
    loadWav(dir, "start.wav",     shStart,  0);
    // Optional three-part start. If all three are present the firmware runs the
    // browser sequence (starter -> looped crank -> random catch -> fireup);
    // otherwise it falls back to the single start.wav.
    loadWav(dir, "starter.wav",   shStarter, 0);
    loadWav(dir, "cranking.wav",  shCrank,   0);
    sealCrankLoop(shCrank);
    for (int i = 0; i < nEng; i++)   findPulses(eng[i]);
    for (int i = 0; i < nCoast; i++) findPulses(coast[i]);
    loadWav(dir, "fireup.wav",    shFireup,  0);
    loadWav(dir, "stop.wav",      shStop,   0);
    loadWav(dir, "bov.wav",       shBov,    0);
    loadWav(dir, "airdry.wav",    shAirDry, 0);
    loadWav(dir, "pbrakeon.wav",  shPbOn,   0);
    loadWav(dir, "pbrakeoff.wav", shPbOff,  0);
    loadWav(dir, "hydpump.wav",   hydPump,  0);   // optional: WORK-mode pump loop
    loadWav(dir, "hydflow.wav",   hydFlow,  0);   // optional: relief / flow loop
    loadWav(dir, "danger.wav",    shDanger, 0);   // optional: battery cut-off clip
    { const char *tk[3] = { "tick1.wav", "tick2.wav", "tick3.wav" };
      for (int i = 0; i < 3; i++) if (loadWav(dir, tk[i], shTick[nTick], 0)) nTick++; }

    sortLadder(eng, nEng);
    sortLadder(coast, nCoast);
    sortLadder(pull, nPull);
    sortLadder(jake, nJake);

    // A pack with only ONE ladder must put it in coast[]. eng[] gets faded out
    // approaching idle on the assumption a separate bank carries idle - with a
    // single ladder that leaves the truck at 0.04 of level at 750 rpm, which is
    // very nearly silent. coast[] is the path that sounds at every rpm.
    if (nCoast == 0 && nEng > 0) {
        for (int i = 0; i < nEng; i++) coast[i] = eng[i];
        nCoast = nEng;
        nEng = 0;
    }
    // idle ALIASES the bottom rung on purpose. It is never played as its own
    // stream - the renderer forces idleG to 0 - because the identical recording
    // sounding twice at unrelated phases is a flanger sitting on top of idle.
    if (nEng > 0)        idle = eng[0];
    else if (nCoast > 0) idle = coast[0];

    _loaded = (nCoast >= 1);
    if (!_loaded) { Serial.println("[pack] no usable ladder"); release(); return false; }

    Serial.printf("[pack] \"%s\"\n", _name);
    if (nEng) {
        Serial.printf("[pack]   engine  %d rung%s:", nEng, nEng == 1 ? "" : "s");
        for (int i = 0; i < nEng; i++) Serial.printf(" %.0f", eng[i].rpm);
        Serial.println(" rpm");
    }
    Serial.printf("[pack]   exhaust %d rung%s:", nCoast, nCoast == 1 ? "" : "s");
    for (int i = 0; i < nCoast; i++) Serial.printf(" %.0f", coast[i].rpm);
    Serial.println(" rpm");
    if (nPull) {
        Serial.printf("[pack]   loaded  %d rung%s:", nPull, nPull == 1 ? "" : "s");
        for (int i = 0; i < nPull; i++) Serial.printf(" %.0f", pull[i].rpm);
        Serial.println(" rpm");
    }
    Serial.printf("[pack]   jake %s, turbo %s, horn %s\n",
                  nJake ? "yes" : "no", turbo.ok() ? "yes" : "no", horn.ok() ? "yes" : "no");
    Serial.printf("[pack]   %lu KB in PSRAM (%u KB still free) - card idle from here\n",
                  (unsigned long)(_bytes / 1024), (unsigned)(ESP.getFreePsram() / 1024));
    return true;
}
