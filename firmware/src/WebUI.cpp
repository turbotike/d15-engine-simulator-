// =============================================================================
//  WebUI.cpp - AP + dash + tuning, served by the S3
// =============================================================================
#include "WebUI.h"
#include "Config.h"
#include "Settings.h"
#include "EngineSim.h"
#include "EngineAudio.h"
#include "AudioOut.h"
#include "Lights.h"
#include "Outputs.h"
#include "SoundPack.h"
#include "RcInput.h"
#include "Trailer.h"
#include "Tilt.h"
#include "Suspension.h"
#include "Hydraulics.h"
#include "ServoOut.h"
#include "Battery.h"
#include "ChanMap.h"
#include "DcEeprom.h"
#include "WebAssets.h"   // generated from WebPage.h by tools/pack_web.py (gzipped page + /img files)

#include <WiFi.h>
#include <WebServer.h>

WebUI Web;
static WebServer http(80);
WebControls WebCtl;

// ---------------------------------------------------------------------------
//  The tunables the phone can reach.
//
//  The page builds its tuning sheet from whatever /settings returns, so adding
//  an entry here makes a slider appear by itself - no HTML to edit. Each row
//  binds a name to a live float in the firmware, so a write takes effect on the
//  next audio block rather than needing a reflash.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  The tunables the phone can reach.
//
//  KEY NAMES AND RANGES ARE THE TEENSY'S, taken from WebLink.cpp's SET dispatch
//  table - NOT invented here. The dash page looks settings up by name, so a
//  rename silently breaks a slider: `volMaster` is not `masterVol`, the drive
//  control is `audioDrive`, presence is `audioPresence`, and the light trims
//  are `lightBrightness*`. Anything renamed here stops binding on the phone.
//
//  `scale` handles the one unit mismatch the Teensy shipped with: the UI sends
//  throttleExpo as 0-100 but the firmware stores and reports 0-1.
// ---------------------------------------------------------------------------
// Shadow for the master-volume knob. KNOBS[] can only bind a float*, so the
// knob writes here and applyWebVolume() pushes it through AudioOut::setMasterVol,
// which updates the sampled renderer AND the synth fallback together.
static float g_webMasterVol = DC_MASTER_VOL;
static void applyWebVolume() {
    static float last = -1.0f;
    if (g_webMasterVol != last) { last = g_webMasterVol; Audio.setMasterVol(g_webMasterVol); }
}

struct Knob { const char *key; float *ptr; float lo, hi; float scale; };

static Knob KNOBS[] = {
    // ---- audio ----
    // Bound to AudioOut's master, not EngAudio's, so it reaches BOTH renderers.
    // Pointing it straight at EngAudio.masterVol meant that on a board with no
    // SD card - the fallback case the synth engine exists for - moving the
    // phone's volume slider changed nothing audible at all.
    { "volMaster",     &g_webMasterVol,         0.0f,   1.0f,   1.0f },
    // audioDrive (Loudness Drive) and audioPresence (Presence / Cut) are NOT
    // exposed any more. Both now have one measured-correct value:
    //   makeup 1.4   -> drive 1.22-1.49 at full load, clip 0%. Past ~2 it is
    //                   deep limiting, which flattens the engine's dynamics and
    //                   adds its own distortion while sounding "louder".
    //   engLevel 1.0 -> the ladder at unity. It sat at 0.50 as the orphaned
    //                   half of a power sum, a flat -6 dB for nothing.
    // If it is still not loud enough that is the AMP (GAIN_SLOT, see Config.h),
    // and no slider here can fix it.
    { "audioWhoosh",   &EngAudio.whooshLevel,   0.0f,   1.5f,   1.0f },
    { "audioKnock",    &EngAudio.knockLevel,    0.0f,   1.5f,   1.0f },
    { "revPitch",      &EngAudio.revPitch,      0.4f,   1.5f,   1.0f },
    { "turboLevel",    &EngAudio.turboLevel,    0.0f,   2.0f,   1.0f },
    // hpHz (speaker high-pass) and lowAirLevel are deliberately NOT here. They
    // were added believing the page built its controls from /settings - it does
    // not, it uses a hardcoded sections[] table, so they never appeared as
    // sliders and were only reachable by hand-crafting a /set URL. Both now
    // have one correct fixed value in EngineAudio.h, chosen by measurement.

    // ---- engine / drivetrain ----
    // CLUTCH RELEASE TIME, in seconds - how long the driver takes to let the
    // pedal up on a launch. This replaced a "slipper clutch" slider that cut
    // the clutch's torque CAPACITY, which was the wrong mechanism: it slipped
    // for the whole run instead of just the launch, and the torque it refused
    // to pass was torque the truck never got, so the run fell flat.
    //
    // Time is the knob a driver actually has. 0 dumps it (which is what the
    // bench tests were doing by accident, hence the bogging); ~1.2 s feeds it
    // in and keeps the engine up in its band while the truck gets moving.
    // SCALE IS 1.0, NOT 0.1. The last column multiplies the UI value on the
    // way in (hSet: *ptr = f * scale) and divides it on the way out, so 0.1
    // here would have stored a tenth of whatever the slider said - 1.2 s
    // on the page, 0.12 s in the clutch. It never bit only because the page
    // had no slider for it until now.
    { "clutchRelease",  &Engine.g_clutchReleaseUI, 0.0f, 3.0f,  1.0f },
    { "raceEscTop",     &Engine.g_raceEscTopMph,  65.0f, 160.0f, 1.0f },
    { "tankGal",        &Engine.tankGal,        20.0f, 300.0f,  1.0f },
    { "fuelScale",      &Engine.fuelScale,       0.1f,  60.0f,  1.0f },
    { "turboSpoolUp",   &Engine.turboSpoolUp,    0.1f,   4.0f,  1.0f },
    { "turboSpoolDown", &Engine.turboSpoolDown,  0.1f,   4.0f,  1.0f },
    { "gradeMax",       &Engine.gradeMax,        0.0f,   0.08f, 1.0f },
    { "boostMax",       &Engine.boostMax,       15.0f, 120.0f,  1.0f },
    { "throttleExpo",   &Cfg.throttleExpo,       0.0f, 100.0f,  0.01f },
    { "trailerLoad",    &Cfg.trailerLoad,        0.0f,   3.0f,  1.0f },
    { "dragBrake",      &Cfg.dragBrake,          0.0f,   1.0f,  1.0f },

    // ---- RC stick thresholds ----
    { "pwmGearPark",          &Cfg.pwmGearPark,        900.0f, 1500.0f, 1.0f },
    { "pwmGearForward",       &Cfg.pwmGearForward,    1500.0f, 2100.0f, 1.0f },
    { "pwmGearHysteresis",    &Cfg.pwmGearHysteresis,    0.0f,  300.0f, 1.0f },
    { "gearDebounceMs",       &Cfg.gearDebounceMs,       0.0f,  600.0f, 1.0f },
    { "pwmHornLow",           &Cfg.pwmHornLow,        1000.0f, 2000.0f, 1.0f },
    { "pwmHornHigh",          &Cfg.pwmHornHigh,       1000.0f, 2000.0f, 1.0f },
    { "pwmLightsOn",          &Cfg.pwmLightsOn,       1000.0f, 2000.0f, 1.0f },
    { "steerSignalEnabled",   &Cfg.steerSignalEnabled,   0.0f,    1.0f, 1.0f },
    { "steerSignalThreshold", &Cfg.steerSignalThreshold,20.0f,  600.0f, 1.0f },
    { "steerSignalHoldMs",    &Cfg.steerSignalHoldMs,    0.0f, 5000.0f, 1.0f },

    // ---- light brightness ----
    // The page sliders run 0-255; the lights take 0-1. These had scale 1.0 with
    // a 0-1 clamp, so any slider position past 1 was full brightness and the
    // slider jumped back to the far left on reload.
    { "lightBrightnessHead",   &Lite.brHead,   0.0f, 255.0f, 1.0f / 255.0f },
    { "lightBrightnessTail",   &Lite.brTail,   0.0f, 255.0f, 1.0f / 255.0f },
    { "lightBrightnessBrake",  &Lite.brBrake,  0.0f, 255.0f, 1.0f / 255.0f },
    { "lightBrightnessRev",    &Lite.brRev,    0.0f, 255.0f, 1.0f / 255.0f },
    { "lightBrightnessSig",    &Lite.brSig,    0.0f, 255.0f, 1.0f / 255.0f },
    { "lightBrightnessMarker", &Lite.brRun,    0.0f, 255.0f, 1.0f / 255.0f },
    { "lightBrightnessBeacon", &Lite.brBeacon, 0.0f, 255.0f, 1.0f / 255.0f },
    { "lightBrightnessCabin",  &Lite.brCabin,  0.0f, 255.0f, 1.0f / 255.0f },
    // This board has two spare channels where the Teensy had fog/roof/3rd.
    { "lightBrightnessFog",    &Lite.brAux1,   0.0f, 255.0f, 1.0f / 255.0f },
    { "lightBrightnessRoof",   &Lite.brAux2,   0.0f, 255.0f, 1.0f / 255.0f },
    // Horn volume: the Teensy scaled a horn SAMPLE with this. Here it scales
    // whichever horn is live - the pack's horn.wav, or the synthesised reeds
    // when the pack has no horn role.
    { "volHorn",               &EngAudio.hornLevel, 0.0f, 2.0f, 1.0f },
    { "volRevbeep",            &EngAudio.beepLevel, 0.0f, 2.0f, 1.0f },
    // These six were MISSING, and that is why their sliders painted hard-left:
    // the page renders any key absent from /settings at s.min, which is 0 for
    // every vol* control. The Teensy only got away with it because the C3 kept
    // a generic key store that echoed back anything the page had ever sent.
    { "volStartup",            &Cfg.volStartup,   0.0f, 2.0f, 1.0f },
    { "volShutdown",           &Cfg.volShutdown,  0.0f, 2.0f, 1.0f },
    { "volParkbrake",          &Cfg.volParkbrake, 0.0f, 2.0f, 1.0f },
    { "volAirbrake",           &Cfg.volAirbrake,  0.0f, 2.0f, 1.0f },
    { "volAirDry",             &Cfg.volAirDry,    0.0f, 2.0f, 1.0f },
    { "volBlinker",            &Cfg.volBlinker,   0.0f, 2.0f, 1.0f },
    { "volCoupling",           &Cfg.volCoupling,  0.0f, 2.0f, 1.0f },
    { "volBov",                &Cfg.volBov,       0.0f, 2.0f, 1.0f },
    { "volTick",               &Cfg.volTick,      0.0f, 2.0f, 1.0f },
    // APPENDED, never inserted: saved knobs load by index (see WEB_MAGIC).
    { "combinedRearLights",    &Cfg.combinedRearLights, 0.0f, 1.0f, 1.0f },
    { "battProtect",           &Cfg.battProtect,        0.0f, 1.0f, 1.0f },   // appended
    { "battTrim",              &Cfg.battTrim,           0.85f, 1.15f, 1.0f }, // appended
};
static const int NKNOB = sizeof(KNOBS) / sizeof(KNOBS[0]);

// SECOND KNOB BLOCK. The first block (720 + 2 + 50*4 = 922) cannot grow: the
// servo outputs are saved at 940. This one sits in the unused 300-499 range
// and its magic carries the row COUNT, so rows appended later load as their
// defaults instead of reading whatever bytes follow - no reset needed when the
// table grows. APPEND ONLY. Room for 49 rows.
static float g_retiredKnob[3] = {0, 0, 0};   // placeholders for removed settings
static Knob KNOBS2[] = {
    { "trailerSync",        &Cfg.trailerSync,  0.0f,    1.0f, 1.0f },
    { "trailerAir",         &Cfg.trailerAir,   0.0f,    1.0f, 1.0f },
    { "steerOut",           &Cfg.steerOut,     0.0f,    8.0f, 1.0f },
    { "steerMin",           &Cfg.steerMin,   800.0f, 1500.0f, 1.0f },
    { "steerCtr",           &Cfg.steerCtr,  1300.0f, 1700.0f, 1.0f },
    { "steerMax",           &Cfg.steerMax,  1500.0f, 2200.0f, 1.0f },
    { "steerInv",           &Cfg.steerInv,     0.0f,    1.0f, 1.0f },
    { "aux1Func",           &Cfg.aux1Func,     0.0f,    6.0f, 1.0f },
    { "aux2Func",           &Cfg.aux2Func,     0.0f,    6.0f, 1.0f },
    // three RETIRED rows (the INA226 load sensor, removed 2026-10-31). KNOBS2
    // loads by index, so these slots stay or every row below shifts.
    { "_retired0",          &g_retiredKnob[0], -1e6f,  1e6f,  1.0f },
    { "_retired1",          &g_retiredKnob[1], -1e6f,  1e6f,  1.0f },
    { "_retired2",          &g_retiredKnob[2], -1e6f,  1e6f,  1.0f },
    { "lightBrightness3rd", &Lite.br3rd,       0.0f,  255.0f, 1.0f / 255.0f },
    { "hydMode",            &Cfg.hydMode,      0.0f,    2.0f, 1.0f },
    { "hydSrv1",            &Cfg.hydSrv[0],    0.0f,    8.0f, 1.0f },
    { "hydSrv2",            &Cfg.hydSrv[1],    0.0f,    8.0f, 1.0f },
    { "hydSrv3",            &Cfg.hydSrv[2],    0.0f,    8.0f, 1.0f },
    { "hydSrv4",            &Cfg.hydSrv[3],    0.0f,    8.0f, 1.0f },
    { "hydIn1",             &Cfg.hydIn[0],     0.0f,   18.0f, 1.0f },
    { "hydIn2",             &Cfg.hydIn[1],     0.0f,   18.0f, 1.0f },
    { "hydIn3",             &Cfg.hydIn[2],     0.0f,   18.0f, 1.0f },
    { "hydIn4",             &Cfg.hydIn[3],     0.0f,   18.0f, 1.0f },
    { "hydInv1",            &Cfg.hydInv[0],    0.0f,    1.0f, 1.0f },
    { "hydInv2",            &Cfg.hydInv[1],    0.0f,    1.0f, 1.0f },
    { "hydInv3",            &Cfg.hydInv[2],    0.0f,    1.0f, 1.0f },
    { "hydInv4",            &Cfg.hydInv[3],    0.0f,    1.0f, 1.0f },
    { "hydSwitch",          &Cfg.hydSwitch,    0.0f,   17.0f, 1.0f },
    { "volHydPump",         &EngAudio.hydLevel, 0.0f,   2.0f, 1.0f },
    { "fullStopTime",       &Cfg.fullStopTime, 0.0f,    3.0f, 1.0f },
    { "fuelScale",          &Engine.fuelScale, 1.0f,   50.0f, 1.0f },   // appended 2026-10-31: fuel burn rate x real time
};
static const int NKNOB2 = sizeof(KNOBS2) / sizeof(KNOBS2[0]);
static const int WEB2_EE_ADDR = 300;
static const uint16_t WEB2_MAGIC = 0xB200;          // | row count
static_assert(WEB2_EE_ADDR + 2 + NKNOB2 * 4 <= 500, "WEB2 block runs into the parts store at 500");

static Knob* findKnob(const String& k) {
    for (int i = 0; i < NKNOB; i++)  if (k == KNOBS[i].key)  return &KNOBS[i];
    for (int i = 0; i < NKNOB2; i++) if (k == KNOBS2[i].key) return &KNOBS2[i];
    return nullptr;
}

// RC channel assignment - "which stick does what". Same key names as the Teensy
// so the page's channel pickers bind. These are ints into ChanMap, not floats,
// so they are handled separately from the knobs above.
static const struct { const char *k; uint8_t fn; } CHKEYS[] = {
    { "chSteer",  CH_STEERING }, { "chThrottle", CH_THROTTLE }, { "chGear",   CH_GEAR  },
    { "chBrake",  CH_BRAKE    }, { "chEngine",   CH_ENGINE   }, { "chLights", CH_LIGHTS },
    { "chHorn",   CH_HORN     }, { "chJake",     CH_JAKE     }, { "chAux",    CH_AUX   },
};
static const int NCHKEY = sizeof(CHKEYS) / sizeof(CHKEYS[0]);

// The dyno is fire-and-poll: the page posts CMD:DYNO then watches /status for a
// `dyno` field. Same shape the Teensy printed over serial, so the page's parser
// is unchanged:  "<hp>hp@<rpm>;egt=..;boost=..;drt=..;srg=..;pcp=..;bln=..;hlth=.."
static String g_dyno = "";

// Where the knob values live across a power cycle. Sits well clear of the
// inherited address map (parts @500-510, servo map @512-675, chan map @690).
static const int WEB_EE_ADDR = 720;
// 0xD53B: raceEscTop was inserted mid-table without bumping this, so every
// saved knob after clutchRelease loaded one slot over. Bumping resets the
// web knobs to defaults once instead of leaving them scrambled.
static const uint16_t WEB_MAGIC = 0xD53B;

static void knobsLoad() {
    uint16_t magic = 0;
    EEPROM.get(WEB_EE_ADDR, magic);
    if (magic != WEB_MAGIC) return;              // never saved, keep the defaults
    for (int i = 0; i < NKNOB; i++) {
        float v = 0;
        EEPROM.get(WEB_EE_ADDR + 2 + i * 4, v);
        // Clamp on LOAD too, not just on /set. Values are keyed by array index
        // under one global magic, so inserting or removing a KNOBS[] row shifts
        // every saved value onto the wrong knob - and an unclamped tankGal
        // landing in masterVol is a deafening full-scale square wave. Bump
        // WEB_MAGIC whenever the table changes.
        if (isfinite(v)) {
            if (v < KNOBS[i].lo) v = KNOBS[i].lo;
            if (v > KNOBS[i].hi) v = KNOBS[i].hi;
            *KNOBS[i].ptr = v;
        }
    }
}

static void knobs2Load() {
    uint16_t magic = 0;
    EEPROM.get(WEB2_EE_ADDR, magic);
    if ((magic & 0xFF00) != WEB2_MAGIC) return;
    int n = magic & 0xFF; if (n > NKNOB2) n = NKNOB2;
    for (int i = 0; i < n; i++) {
        float v = 0;
        EEPROM.get(WEB2_EE_ADDR + 2 + i * 4, v);
        float lo = KNOBS2[i].lo * KNOBS2[i].scale, hi = KNOBS2[i].hi * KNOBS2[i].scale;
        if (isfinite(v)) *KNOBS2[i].ptr = v < lo ? lo : (v > hi ? hi : v);
    }
}
static void knobs2Save() {
    EEPROM.put(WEB2_EE_ADDR, (uint16_t)(WEB2_MAGIC | NKNOB2));
    for (int i = 0; i < NKNOB2; i++) EEPROM.put(WEB2_EE_ADDR + 2 + i * 4, *KNOBS2[i].ptr);
}

static void knobsSave() {
    EEPROM.put(WEB_EE_ADDR, WEB_MAGIC);
    for (int i = 0; i < NKNOB; i++) EEPROM.put(WEB_EE_ADDR + 2 + i * 4, *KNOBS[i].ptr);
}

// ---------------------------------------------------------------------------
// The page goes out gzipped (33 KB instead of 515 KB) and its pictures as
// separate /img files the phone caches. The old single 515 KB write blocked the
// loop for seconds and, at range, never finished: "the UI will not load".
static void hRoot() {
    http.sendHeader("Content-Encoding", "gzip");
    http.sendHeader("Cache-Control", "no-cache");
    http.send_P(200, "text/html", (PGM_P)WEB_PAGE_GZ, sizeof(WEB_PAGE_GZ));
}

static void hNotFound() {
    String u = http.uri();
    for (int i = 0; i < WEB_NIMG; i++) {
        if (u == WEB_IMGS[i].path) {
            http.sendHeader("Cache-Control", "max-age=604800");
            http.send_P(200, "image/png", (PGM_P)WEB_IMGS[i].data, WEB_IMGS[i].len);
            return;
        }
    }
    http.send(404, "text/plain", "not found");
}

// Telemetry. Same field names the page already consumes - on the Teensy these
// came out of a D[] array filled from the serial link; here they are read
// straight off the sim, so there is no staleness and no "link" to lose.
static void hStatus() {
    uint8_t warn  = Engine.warnBits();
    bool    run   = Engine.running();
    int     engine = run ? 1 : 0;

    // was hardcoded head=false/high=false, so the LIGHTS pill always said OFF
    int lightMode = WebCtl.lightModeNow;

    static const char *protos[5] = { "--", "ibus", "sbus", "sbus", "pwm" };
    int rm = (int)RC.mode(); if (rm < 0 || rm > 4) rm = 0;

    // One snprintf into a static buffer instead of ~30 String appends. Each
    // String(float) allocated, and at 7 polls a second that was steady heap
    // churn on a long session. Same keys, same formats as before.
    static char js[2000];
    snprintf(js, sizeof(js),
        "{\"sim\":1,\"engine\":%d,\"gear\":2,\"simgear\":%d,\"throttle\":%d,\"motor\":%d,"
        "\"lightMode\":%d,\"uptimeMs\":%lu,\"rpm\":%.0f,\"mph\":%.1f,\"psi\":%.1f,"
        "\"air\":%.0f,\"airSec\":%.0f,\"cool\":%.0f,\"oil\":%.0f,\"oilPsi\":%.0f,\"pyro\":%.0f,"
        "\"volts\":%.1f,\"trans\":%.0f,\"fuelPct\":%.1f,\"brk\":%.0f,\"warn\":%d,\"jake\":%d,"
        "\"proto\":\"%s\",\"link\":1,\"cpu\":%.0f,\"xrun\":%lu,\"drive\":%.2f,\"clip\":%.0f,"
        "\"pack\":\"%s\",\"hlth\":%.0f,\"melt\":%.3f,"
        "\"et60\":%.2f,\"etq\":%.2f,\"trap\":%.1f,\"drag\":%d,\"dt\":%.1f,\"dd\":%.0f,"
        "\"pack\":%.2f,\"bec\":%.2f,\"cells\":%u,\"soc\":%.0f,\"batt\":\"%s\",\"battOn\":%d,"
        "\"tr\":%d,\"lope\":%d,\"beacon\":%d,"
        "\"hydOn\":%d,\"hydPsi\":%.0f,\"hydTemp\":%.0f,\"hydPos\":[%.0f,%.0f,%.0f,%.0f],"
        "\"rx\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],"
        "\"dyno\":\"%s\",\"dyon\":%d,\"dyp\":%.3f,\"dyhp\":%.0f,\"gok\":%d,\"gp\":%.1f,\"gr\":%.1f,\"gt\":%.1f,\"gov\":%d,\"gax\":%d,\"ghl\":%.2f,\"gl\":%.2f,\"gf\":%.2f,\"son\":%d,\"sr\":%.2f,\"sp\":%.2f,\"sul\":%d,\"sur\":%d,\"sct\":%d,\"scr\":%d,\"stv\":%.0f}",
        engine, (int)Engine.gear(), 1470 + (int)(Engine.load() * 500), (int)Out.escMicros(),
        lightMode, (unsigned long)millis(), Engine.rpm(), Engine.speedMph(), Engine.boostPsi(),
        Engine.airPsi(), Engine.airSecPsi(), Engine.coolantF(), Engine.oilF(), Engine.oilPsi(), Engine.pyroF(),
        Engine.volts(), Engine.transF(), Engine.fuelFrac() * 100.0f, Engine.brakePsi(), (int)warn,
        (int)WebCtl.jakeNow, protos[rm], Audio.cpuFrac() * 100.0f,
        (unsigned long)Audio.underruns(), EngAudio.peekDrive(), EngAudio.clipping() * 100.0f,
        Pack.loaded() ? Pack.name() : "none", Engine.health(), Engine.pistonMelt(),
        // the time slip, so the race panel can show it instead of serial only
        Engine.et60(), Engine.etQtr(), Engine.trapMph(), Engine.dragRunning() ? 1 : 0,
        Engine.dragTime(), Engine.dragDistFt(),
        Batt.volts(), Batt.bec(), (unsigned)Batt.cells(), Batt.soc() * 100.0f, Batt.stateName(), Batt.active() ? 1 : 0,
        Trailer.state(), Engine.lopeForce ? 1 : 0, WebCtl.beacon ? 1 : 0,
        Hyd.on() ? 1 : 0, Hyd.pressPsi(), Hyd.tempF(),
        Hyd.pos(0) * 100.0f, Hyd.pos(1) * 100.0f, Hyd.pos(2) * 100.0f, Hyd.pos(3) * 100.0f,
        RC.ch(0), RC.ch(1), RC.ch(2), RC.ch(3), RC.ch(4), RC.ch(5), RC.ch(6), RC.ch(7),
        RC.ch(8), RC.ch(9), RC.ch(10), RC.ch(11), RC.ch(12), RC.ch(13), RC.ch(14), RC.ch(15),
        Engine.dynoResult()[0] ? Engine.dynoResult() : g_dyno.c_str(),
        Engine.dynoLive() ? 1 : 0, Engine.dynoProgress(), Engine.dynoHpNow(),
        Tilt.present() ? 1 : 0, Tilt.pitchDeg(), Tilt.rollDeg(), Tilt.tiltDeg(), Tilt.rolledOver() ? 1 : 0,
        (int)Tilt.axis, Tilt.hill, Tilt.accLat(), Tilt.accFwd(),
        Susp.P.on ? 1 : 0, Susp.rollNow(), Susp.pitchNow(), Susp.usL(), Susp.usR(), Susp.P.ctrL, Susp.P.ctrR, Susp.P.travel);
    http.send(200, "application/json", js);
}

static void hSettings() {
    String j; j.reserve(2048);
    j = "{";
    for (int i = 0; i < NKNOB; i++) {
        if (i) j += ',';
        j += '"'; j += KNOBS[i].key; j += "\":";
        // report in the UI's units - throttleExpo is stored 0-1 but shown 0-100
        j += String(*KNOBS[i].ptr / KNOBS[i].scale, 3);
    }
    for (int i = 0; i < NKNOB2; i++) {
        j += ",\""; j += KNOBS2[i].key; j += "\":";
        j += String(*KNOBS2[i].ptr / KNOBS2[i].scale, 3);
    }
    for (int i = 0; i < NCHKEY; i++) {
        j += ",\""; j += CHKEYS[i].k; j += "\":"; j += String((int)Chan.of(CHKEYS[i].fn));
    }
    // the fuel and timing maps - eight sites each, the heart of the TUNE tab
    for (int i = 0; i < 8; i++) j += ",\"fuel" + String(i) + "\":" + String(Engine.fuelMap[i], 2);
    for (int i = 0; i < 8; i++) j += ",\"tim"  + String(i) + "\":" + String(Engine.timMap[i], 1);
    // current build and what has been earned
    j += ",\"race\":"      + String(Engine.raceMode() ? 1 : 0);
    j += ",\"raceLaunch\":" + String(Engine.raceLaunchGear());
    j += ",\"raceSkip\":"  + String(Engine.raceSkipGear());
    j += ",\"raceSkip2\":" + String(Engine.raceSkip2Gear());
    j += ",\"racePower\":" + String(Engine.racePower());
    j += ",\"raceTop\":"   + String(Engine.raceTopGear());
    EngineSim::Parts p = Engine.parts, u = Engine.unlocks;
    j += ",\"parts\":\"" + String(p.rods) + "," + String(p.pistons) + "," + String(p.turbo)
       + "," + String(p.ic) + "," + String(p.fuel) + "\"";
    j += ",\"unl\":\"" + String(u.rods) + "," + String(u.pistons) + "," + String(u.turbo)
       + "," + String(u.ic) + "," + String(u.fuel) + "\"";
    j += '}';
    http.send(200, "application/json", j);
}

static void hSet() {
    String k = http.arg("key"), v = http.arg("val");
    bool found = false;
    if (k.length()) {
        if (Knob* kb = findKnob(k)) {
            float f = v.toFloat();
            if (isfinite(f)) {
                if (f < kb->lo) f = kb->lo;
                if (f > kb->hi) f = kb->hi;
                *kb->ptr = f * kb->scale;             // UI units -> firmware
                found = true;
            }
        }
        // fuel0..fuel7 and tim0..tim7 are indexed, not named - same ranges the
        // Teensy clamped to (fuel 0.7-2.0 injection, timing -5..+10 degrees).
        if (!found && k.startsWith("fuel") && k.length() == 5) {
            int i = k[4] - '0';
            float fv = v.toFloat();   // toFloat() parses "nan" - constrain passes NaN straight through
            if (i >= 0 && i < 8 && isfinite(fv)) { Engine.fuelMap[i] = constrain(fv, 0.7f, 2.0f); found = true; }
        }
        if (!found && k.startsWith("tim") && k.length() == 4) {
            int i = k[3] - '0';
            float tv = v.toFloat();
            if (i >= 0 && i < 8 && isfinite(tv)) { Engine.timMap[i] = constrain(tv, -5.0f, 10.0f); found = true; }
        }
        if (!found) {
            for (int i = 0; i < NCHKEY; i++) {
                if (k == CHKEYS[i].k) {
                    Chan.set(CHKEYS[i].fn, (uint8_t)constrain(v.toInt(), 0, 15));
                    found = true;
                    break;
                }
            }
        }
    }
    http.send(200, "text/plain", found ? "OK" : "unknown key");
}

static void hSave() {
    // ONE commit for the whole save. Each of these used to commit internally,
    // and commit() rewrites the entire 1 KB NVS blob - so pressing Save fired
    // roughly fifty flash writes back to back, each one stalling core 0 with
    // the cache disabled. The DMA only holds 46 ms of audio.
    knobsSave();            // writes, no commit
    knobs2Save();
    Chan.save();            // writes, no commit
    Engine.saveParts();     // writes, no commit
    EEPROM.commit();        // one blob write
    http.send(200, "text/plain", "saved");
}

static void hReset() {
    EEPROM.put(WEB_EE_ADDR, (uint16_t)0);
    EEPROM.put(WEB2_EE_ADDR, (uint16_t)0);
    EEPROM.commit();
    // The page does .then(r => r.json()) on this. Returning text made it throw,
    // so a SUCCESSFUL reset reported "Reset failed" while having already wiped
    // the stored values. Return the settings object it expects.
    hSettings();
}

static void hCmd() {
    String a = http.arg("action");
    if      (a == "ENGINE_ON")  Engine.start();
    else if (a == "ENGINE_OFF") Engine.stop();
    // The page's main ENGINE button sends TOGGLE, which fell through to a bare
    // 200 OK - so it looked like it worked and did nothing.
    else if (a == "ENGINE_TOGGLE") { if (Engine.running()) Engine.stop(); else Engine.start(); }
    else if (a == "REFUEL")     Engine.refuel();
    else if (a == "HORN_ON")    Audio.setHorn(true);
    else if (a == "HORN_OFF")   Audio.setHorn(false);
    else if (a == "AIR")        EngAudio.triggerAir();
    // ---- Remote Buttons (these used to fall through to a bare "OK") ----
    else if (a == "LIGHT_NEXT") WebCtl.lightNext = true;
    else if (a == "JAKE_CYCLE") WebCtl.jakeCycle = true;
    else if (a == "BEACON_ON")  WebCtl.beacon = true;
    else if (a == "BEACON_OFF") WebCtl.beacon = false;
    else if (a == "WORK_TOGGLE") WebCtl.workToggle = true;
    // ---- wireless trailer (DIYGuy protocol, see Trailer.h) ----
    else if (a == "TRAILER_LIGHTS")     Trailer.lightTest();
    else if (a == "TRAILER_LEGS_UP")    Trailer.hold(TR_LEGS_UP);
    else if (a == "TRAILER_LEGS_DOWN")  Trailer.hold(TR_LEGS_DOWN);
    else if (a == "TRAILER_RAMPS_UP")   Trailer.hold(TR_RAMPS_UP);
    else if (a == "TRAILER_RAMPS_DOWN") Trailer.hold(TR_RAMPS_DOWN);
    else if (a == "TRAILER_STOP")       Trailer.release();
    else if (a == "DYNO") {
        // starts the LIVE pull; the result arrives in /status "dyno" when it ends
        g_dyno = "";
        Engine.dynoClearResult();
        http.send(200, "text/plain", Engine.dynoStart() ? "DYNO:1" : "DYNO:0");
        return;
    }
    else if (a == "DYNO_ABORT") { Engine.dynoAbort(); }
    else if (a == "DYNO_CURVE") { http.send(200, "application/json", Engine.dynoCurveJson()); return; }
    else if (a.startsWith("REBUILD")) {
        Engine.rebuild();
        // sloppy wrenching leaves a weaker engine - REBUILD:<pct>
        int c = a.indexOf(':');
        if (c > 0) {
            float q = a.substring(c + 1).toFloat();
            if (q >= 50 && q < 100) Engine.applyDamage(100.0f - q);
        }
        g_dyno = "";
    }
    else if (a.startsWith("INSTALL:")) {
        int idx = a.substring(8).toInt();
        int c2 = a.indexOf(':', 8);
        int tier = (c2 > 0) ? a.substring(c2 + 1).toInt() : 0;
        bool ok = Engine.installPart((uint8_t)idx, (uint8_t)tier);
        EEPROM.commit();   // saveParts() only buffers: pulling the battery used to lose the change
        char buf[48];
        snprintf(buf, sizeof(buf), "PART:%d:%d:%d", idx, tier, ok ? 1 : 0);
        http.send(200, "text/plain", buf);
        return;
    }
    // ---- RACE MODE ---------------------------------------------------------
    else if (a.startsWith("RACE:")) {
        // RACE:on  |  RACE:on:launch:top  |  RACE:on:launch:skip:top  |
        // RACE:on:launch:skip:skip2:top  (the four-gear ladder)
        int v[6] = {0, 0, 0, 0, 0, 0}, n = 0, p = 5;
        while (n < 6) {
            v[n++] = a.substring(p).toInt();
            int c = a.indexOf(':', p); if (c < 0) break; p = c + 1;
        }
        int on = v[0];
        if (n == 3)      Engine.setRaceGears(v[1], 0,    v[2]);
        else if (n == 4) Engine.setRaceGears(v[1], v[2], v[3]);
        else if (n >= 5) Engine.setRaceGears(v[1], v[2], v[4], v[3]);
        Engine.setRaceMode(on != 0);
        char buf[64];
        snprintf(buf, sizeof(buf), "RACE:%d:%d:%d:%d:%d",
                 Engine.raceMode() ? 1 : 0,
                 Engine.raceLaunchGear(), Engine.raceSkipGear(),
                 Engine.raceSkip2Gear(), Engine.raceTopGear());
        http.send(200, "text/plain", buf);
        return;
    }
    // ---- PERFORMANCE INJECTORS, for the TUNE tab --------------------------
    // INJ:0 stock / INJ:1 big / INJ:2 dual-fuelers. Uses grantPart() rather
    // than installPart() on purpose: installPart refuses any tier not already
    // in `unlocks`, which is correct for the earn-by-breaking career path but
    // would make a deliberate UI button silently do nothing.
    //
    // WHY THIS MATTERS FOR TUNING, and it is the whole point of the button:
    // fuelMap is clamped by fuelCap in EngineCore (`effTrim = min(fuelTrim,
    // fuelCap)`), and stock fuelCap is 1.05. So on stock injectors a fuel
    // tune above ~1.05 does NOTHING - MEASURED on the bench: a 1.60 fuel map
    // made no extra power until the cap moved. Bigger injectors are what let
    // the fuel side of the map actually take effect.
    // RACE CLASS - 0 street / 1 ~1500hp / 2 ~3000hp. Sets the torque
    // multiplier AND the mass; see setRacePower in EngineSim.h for why both
    // are needed (650 hp at 32 t is 25x worse power-to-weight than a real
    // drag truck, so gears alone never made it quick).
    else if (a.startsWith("POWER:")) {
        Engine.setRacePower((uint8_t)a.substring(6).toInt());
        char buf[40];
        snprintf(buf, sizeof(buf), "POWER:%u", Engine.racePower());
        http.send(200, "text/plain", buf);
        return;
    }
    else if (a.startsWith("INJ:")) {
        int tier = a.substring(4).toInt();
        bool ok = Engine.grantPart(4, (uint8_t)constrain(tier, 0, 2));
        EEPROM.commit();   // as above
        char buf[48];
        snprintf(buf, sizeof(buf), "INJ:%d:%d", Engine.parts.fuel, ok ? 1 : 0);
        http.send(200, "text/plain", buf);
        return;
    }
    // ---- GY-521 (accessory header) ----
    // ---- active front suspension ----
    else if (a == "SUSP") { http.send(200, "application/json", Susp.json()); return; }
    else if (a.startsWith("SUSP_TEST:")) { Susp.test(a.substring(10).toInt()); }
    else if (a.startsWith("SUSP_SET:")) {
        int c = a.indexOf(':', 9); String k = a.substring(9, c); float v = a.substring(c + 1).toFloat();
        SuspParams &P = Susp.P;
        if      (k == "on")     P.on = v > 0.5f;
        else if (k == "chL")    P.chL = (int8_t)constrain((int)v, 0, 7);
        else if (k == "chR")    P.chR = (int8_t)constrain((int)v, 0, 7);
        else if (k == "ctrL")   P.ctrL = (int16_t)constrain((int)v, 1100, 1900);
        else if (k == "ctrR")   P.ctrR = (int16_t)constrain((int)v, 1100, 1900);
        else if (k == "invL")   P.invL = v > 0.5f;
        else if (k == "invR")   P.invR = v > 0.5f;
        else if (k == "travel") P.travel = constrain(v, 20.0f, 500.0f);
        else if (k == "roll")   P.roll = constrain(v, 0.0f, 2000.0f);
        else if (k == "pitch")  P.pitch = constrain(v, 0.0f, 2000.0f);
        else if (k == "twist")  P.twist = constrain(v, -400.0f, 400.0f);
        else if (k == "freq")   P.freq = constrain(v, 0.5f, 4.0f);
        else if (k == "damp")   P.damp = constrain(v, 0.1f, 1.2f);
        Susp.save();
        http.send(200, "application/json", Susp.json()); return;
    }
    else if (a == "TILT_LEVEL")         { Tilt.learnLevel(); }
    else if (a.startsWith("TILT_AXIS:")) { Tilt.axis = (uint8_t)constrain(a.substring(10).toInt(), 0, 3); Tilt.save(); }
    else if (a.startsWith("TILT_HILL:")) { Tilt.hill = constrain(a.substring(10).toFloat(), 0.0f, 1.0f); Tilt.save(); }
    else if (a == "PARTS") {
        EngineSim::Parts p = Engine.parts, u = Engine.unlocks;
        char buf[80];
        snprintf(buf, sizeof(buf), "PARTS:%d,%d,%d,%d,%d;UNL:%d,%d,%d,%d,%d",
                 p.rods, p.pistons, p.turbo, p.ic, p.fuel,
                 u.rods, u.pistons, u.turbo, u.ic, u.fuel);
        http.send(200, "text/plain", buf);
        return;
    }
    http.send(200, "text/plain", "OK");
}

// Light test passthrough - drives the PCA9685 directly, overriding the engine
// until cleared. Handy for checking a wiring loom without a transmitter.
// /trailer                 -> {"on":1,"mac":"FE:49:01:00:00:01","state":2,...}
// /trailer?on=0|1          -> enable / disable the link
// /trailer?mac=FE:49:...   -> the trailer board's MAC (set on ITS web page)
static void hTrailer() {
    String err = "";
    if (http.hasArg("on"))  Trailer.setEnabled(http.arg("on").toInt() != 0);
    if (http.hasArg("mac") && !Trailer.setMac(http.arg("mac").c_str())) err = "bad MAC - use FE:49:01:00:00:01 style";
    String j = Trailer.json();
    j.remove(j.length() - 1);
    j += ",\"err\":\""; j += err; j += "\"}";
    http.send(200, "application/json", j);
}

static void hTestLight() {
    int ch = http.arg("ch").toInt(), val = http.arg("val").toInt();
    Lite.testLight((uint8_t)ch, (uint8_t)constrain(val, 0, 255));
    http.send(200, "text/plain", "OK");
}
static void hTestAll() {
    int val = http.arg("val").toInt();
    Lite.testAll((uint8_t)constrain(val, 0, 255));
    http.send(200, "text/plain", "OK");
}
// /servo            -> state of all 8 headers
// /servo?ch=2&on=1  -> make CH2 a servo output (refused if the truck reads it)
// /servo?ch=2&us=1600
static void hServo() {
    String err = "";
    if (http.hasArg("ch")) {
        int i = http.arg("ch").toInt() - 1;   // UI says CH1..CH8
        if (http.hasArg("on")) err = Servos.setOn(i, http.arg("on").toInt() != 0);
        if (http.hasArg("us")) Servos.setUs(i, http.arg("us").toInt());
    }
    String j = Servos.json();
    j.remove(j.length() - 1);
    j += ",\"err\":\""; j += err; j += "\"}";
    http.send(200, "application/json", j);
}

static void hLightTest() {
    if (http.arg("on").toInt() == 0) Lite.testClear();
    http.send(200, "text/plain", "OK");
}

// ---------------------------------------------------------------------------
bool WebUI::begin(const char *ssid, const char *pass) {
    knobsLoad();                              // saved tuning before the radio
    knobs2Load();

    // AP + STA: the STA side never connects, it is what ESP-NOW sends on for
    // the wireless trailer (Trailer.h). Same mode DIYGuy's controller uses.
    WiFi.mode(WIFI_AP_STA);
    if (!WiFi.softAP(ssid, pass, 1)) { _up = false; return false; }
    IPAddress ip = WiFi.softAPIP();
    snprintf(_ip, sizeof(_ip), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

    http.on("/",          hRoot);
    http.on("/status",    hStatus);
    http.on("/settings",  hSettings);
    http.on("/set",       hSet);
    http.on("/save",      hSave);
    http.on("/reset",     hReset);
    http.on("/cmd",       hCmd);
    http.on("/testlight", hTestLight);
    http.on("/testall",   hTestAll);
    http.on("/lighttest", hLightTest);
    http.on("/servo",     hServo);
    http.on("/trailer",   hTrailer);
    http.onNotFound(hNotFound);              // /img/*.png
    http.begin();

    _up = true;
    return true;
}

void WebUI::stop() {
    if (!_up) return;
    http.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    _up = false;
}

void WebUI::poll() { if (_up) { http.handleClient(); applyWebVolume(); } }

uint32_t WebUI::clients() const { return _up ? WiFi.softAPgetStationNum() : 0; }
