// =============================================================================
//  EngineSim.cpp - drivetrain physics (ported from the shipped ESP32 truck).
// =============================================================================
#include "EngineSim.h"
#include "EngineAudio.h"   // ChugParams (the lope phrase)
#include "EngineCore.h"
#include "DcEeprom.h"     // ESP32: NVS-backed, Teensy-compatible API

static EngineCore g_core;
static CoreOut    g_lastCore = {};

EngineSim Engine;

// ---- upgradeable-part tier tables (index by tier 0..2) ----
float PT_RODBAR[3]   = {270.0f, 290.0f, 420.0f};       // rod break pressure (bar), rated stock ~600 hp / mid 1500 / top 3000 (dyno 2026-10-30)
float PT_MELTF[3]    = {1600.0f, 1680.0f, 2000.0f};    // piston melt EGT (degF); mid pistons melt at the 3000 hp build (1700F)
// TURBO TIERS. The STOCK tier was a full step below the browser, on BOTH
// parameters at once:
//                  was (stock)   browser default
//     turbMult        2.6             6.5
//     turbCeil      12500           22000
// i.e. this firmware's 'big turbo' tier is what the browser calls stock. With
// 2.5x less turbine power the shaft could not hold speed through a 1.4 s fuel
// cut, so boost collapsed on every shift and the whistle swept up from nothing
// in every gear - which is what 'the turbo sucks' and 'boost does not hold'
// were both describing. It was the wrong part fitted, not a tuning error, and I
// went looking at the shaft coast-down and the fuel governor first because I had
// not read coreTick() against the browser's as a whole.
//
// Stock now matches the browser. The upgrade tiers move up accordingly so the
// parts progression still means something.
float PT_TMULT[3]    = {6.5f, 10.0f, 14.0f};           // turbine strength
float PT_TCEIL[3]    = {22000.0f, 32000.0f, 42000.0f}; // shaft ceiling (rad/s)
float PT_ICEFF[3]    = {0.68f, 0.85f, 0.93f};          // intercooler efficiency
float PT_FCAP[3]     = {1.05f, 1.6f, 2.2f};            // injector fuel-trim cap
// THE PARTS DECIDE THE HORSEPOWER (Mat 2026-10-30: "make the parts selectable
// and that determines the actual hp, and it's always on" - the race CLASS
// multiplier is gone). Bigger injectors FLOW more at the same command, a
// bigger turbo runs more boost off the same wastegate setting; the tune and
// the other parts decide whether it burns clean, smokes, melts or throws a rod.
float PT_FFLOW[3]    = {1.0f, 2.5f, 5.0f};             // injector flow vs stock 57s
float PT_WGMUL[3]    = {1.0f, 2.2f, 4.0f};             // boost the turbo runs per psi of wastegate setting
static const int   PARTS_EE_ADDR  = 500;                            // EEPROM base (Teensy 4.0 has 1080B)
static const uint8_t PARTS_MAGIC  = 0xC5;

void EngineSim::applyParts() {
    auto t3 = [](uint8_t v){ return v > 2 ? 2 : v; };
    g_core.turbMult = PT_TMULT[t3(parts.turbo)];
    g_core.turbCeil = PT_TCEIL[t3(parts.turbo)];
    g_core.icEff    = PT_ICEFF[t3(parts.ic)];
    g_core.fuelCap  = PT_FCAP[t3(parts.fuel)];
    g_core.fuelFlow = PT_FFLOW[t3(parts.fuel)];
    g_rodBar        = PT_RODBAR[t3(parts.rods)];
    g_meltF         = PT_MELTF[t3(parts.pistons)];
}
void EngineSim::loadParts() {
    if (EEPROM.read(PARTS_EE_ADDR) == PARTS_MAGIC) {
        EEPROM.get(PARTS_EE_ADDR + 1, parts);
        EEPROM.get(PARTS_EE_ADDR + 1 + (int)sizeof(Parts), unlocks);
    }   // else: struct defaults = bone stock, unlocks stock
    applyParts();
}
void EngineSim::saveParts() {
    EEPROM.update(PARTS_EE_ADDR, PARTS_MAGIC);
    EEPROM.put(PARTS_EE_ADDR + 1, parts);
    EEPROM.put(PARTS_EE_ADDR + 1 + (int)sizeof(Parts), unlocks);
    applyParts();
}
bool EngineSim::installPart(uint8_t idx, uint8_t tier) {
    if (idx > 4 || tier > 2) return false;
    uint8_t* u = &unlocks.rods + idx;      // Parts is 5 packed uint8_t
    uint8_t* p = &parts.rods   + idx;
    // every part is selectable now (Mat 2026-10-30); fitting one also counts as earning it
    if (tier > *u) *u = tier;
    *p = tier; saveParts(); return true;
}

// Eaton Fuller RTLO 13-speed x 3.55 axle
// Even 10-speed, constant 37% step. Overall ratios scaled from the same 3.55
// diff so the torque at the wheel is in the same place as before.
const float EngineSim::gearRatio[N_GEARS] = {12.31f,8.99f,6.56f,4.79f,3.50f,2.55f,1.87f,1.36f,1.00f,0.73f};
// 526 rpm drop on every shift, 1st to 10th. 1st unchanged at 373 so the launch
// feels the same; top gear 22 = ~75 mph at the 1950 shift point.
const float EngineSim::RPM_PER_MPH_BASE[N_GEARS] = {373,272,199,145,106,77,57,41,30,22};
float       EngineSim::rpmPerMph[N_GEARS]        = {373,272,199,145,106,77,57,41,30,22};

const float EngineSim::RAD = 60.0f / (2.0f * 3.14159265f);
// CAT C15 6NZ 550hp published curve: 1850 lb-ft plateau 1200-1400, 550hp @ 2100.
// Points every 100 rpm from 600 to 2100.

EngineSim::DynoResult EngineSim::dynoRun() const {
    // REAL transient pull: a fresh core spools from low boost, the head breathes,
    // EGT integrates, and the tune decides whether she sings or grenades.
    DynoResult res = {};
    EngineCore dc; dc.reset(); dc.wgPsi = boostMax * PT_WGMUL[parts.turbo > 2 ? 2 : parts.turbo];
    auto t3 = [](uint8_t v){ return v > 2 ? 2 : v; };            // dyno runs the INSTALLED build
    dc.turbMult = PT_TMULT[t3(parts.turbo)]; dc.turbCeil = PT_TCEIL[t3(parts.turbo)];
    dc.icEff    = PT_ICEFF[t3(parts.ic)];    dc.fuelCap  = PT_FCAP[t3(parts.fuel)];
    dc.fuelFlow = PT_FFLOW[t3(parts.fuel)];
    float egtMax = 0, boostPk = 0, pcpMax = 0;
    bool surgeAny = false;
    const float dtp = 0.01f;
    for (float t = 0; t < 15.0f; t += dtp) {
        float r = 700.0f + 1400.0f * (t / 15.0f);
        float f01 = clampf((r - 600.0f) / 1500.0f, 0, 1) * 7.0f;
        int k = (int)f01; if (k > 6) k = 6; float fr = f01 - k;
        dc.fuelTrim = clampf(fuelMap[k] + (fuelMap[k+1] - fuelMap[k]) * fr, 0.7f, 2.0f);
        dc.advDeg   = timMap[k] + (timMap[k+1] - timMap[k]) * fr;
        // Ramp the rack in over the first 2 s instead of slamming full fuel
        // at 700 rpm with no boost. That start condition put AFR at 11 and
        // EGT past 1400 F before the turbo existed, so a BONE-STOCK tune
        // blew up on the dyno while a maxed-slider tune survived - the
        // tuning game ran exactly backwards. A real operator loads the
        // engine up before going to full fuel.
        float rack = clampf(t / 2.0f, 0.25f, 1.0f);
        CoreOut co = dc.tick(dtp, r, rack);
        if (co.egtF > egtMax)      egtMax = co.egtF;
        if (co.boostPsi > boostPk) boostPk = co.boostPsi;
        if (co.pcpBar > pcpMax)    pcpMax = co.pcpBar;
        if (co.surge && t > 1.0f)  surgeAny = true;
        if (co.egtF > g_meltF) res.blown = true;              // melt point - piston lets go
        float hp = co.torqueNm * r / 9549.0f / 0.7457f;       // full power, no derate
        if (hp > res.hp) { res.hp = hp; res.rpm = r; }
    }
    res.egtPeak = egtMax; res.boostPsi = boostPk;
    res.pcpOver = pcpMax > g_rodBar;                        // cylinder pressure past the limit
    if (res.pcpOver) res.blown = true;                    // ...threw a rod on the pull
    res.derated = (egtMax > 1150.0f && !res.blown);       // "running hot" advisory
    res.surged = surgeAny;
    return res;
}

float EngineSim::soundBoostPsi() const { return g_boostPsi / PT_WGMUL[parts.turbo > 2 ? 2 : parts.turbo]; }
float EngineSim::soundShaftHz()  const { return g_shaftHz * (PT_TCEIL[0] / PT_TCEIL[parts.turbo > 2 ? 2 : parts.turbo]); }
float EngineSim::turboTone()     const { static const float T[3] = {1.0f, 0.85f, 0.72f}; return T[parts.turbo > 2 ? 2 : parts.turbo]; }

float EngineSim::engineTorqueNm() const {
    if (g_state != ES_RUN || g_throttle <= 0.05f) return 0.0f;
    return fmaxf(g_lastCore.torqueNm, 0.0f) * (PEAK_TORQUE / 2508.0f) * g_derate;
}

// ---- THE LIVE DYNO PULL (see EngineSim.h) ----
bool EngineSim::dynoStart() {
    if (g_state != ES_RUN || g_dynoOn) return false;
    g_dynoOn = true; g_dynoT = 0; g_dynoRpm = fmaxf(g_rpm, 900.0f); g_dynoThr = 0.25f; g_dynoHpNow = 0;
    for (int i = 0; i < DYNO_PTS; i++) { g_dynoHp[i] = 0; g_dynoTq[i] = 0; }
    g_dynoN = 0; g_dynoPkHp = g_dynoPkRpm = g_dynoPkTq = g_dynoEgt = g_dynoBoost = 0;
    g_dynoSurge = g_dynoPcp = g_dynoLowFuel = false; g_dynoRes[0] = 0;
    return true;
}
void EngineSim::dynoTick(float dt) {
    g_dynoT += dt;
    const float HOLD = 2.5f, SWEEP = 15.0f;
    if (g_dynoT < HOLD) {                                   // load up at 1000, build boost
        g_dynoRpm += (1000.0f - g_dynoRpm) * fminf(1.0f, dt * 3.0f);
        g_dynoThr = fminf(1.0f, 0.25f + g_dynoT / 2.0f);    // ease the rack in, as an operator would
    } else {
        g_dynoThr = 1.0f;
        g_dynoRpm = 1000.0f + 1100.0f * fminf(1.0f, (g_dynoT - HOLD) / SWEEP);
    }
    // measure off the engine core: the same numbers the instant dyno reports
    float tq = fmaxf(g_lastCore.torqueNm, 0.0f);
    float hp = tq * g_dynoRpm / 9549.0f / 0.7457f;
    g_dynoHpNow += (hp - g_dynoHpNow) * fminf(1.0f, dt * 8.0f);
    if (g_dynoT >= HOLD) {
        int k = (int)((g_dynoRpm - 1000.0f) / 50.0f + 0.5f);
        if (k >= 0 && k < DYNO_PTS) { g_dynoHp[k] = g_dynoHpNow; g_dynoTq[k] = tq * 0.7376f; if (k + 1 > g_dynoN) g_dynoN = k + 1; }
        if (g_dynoHpNow > g_dynoPkHp) { g_dynoPkHp = g_dynoHpNow; g_dynoPkRpm = g_dynoRpm; }
        if (tq * 0.7376f > g_dynoPkTq) g_dynoPkTq = tq * 0.7376f;
    }
    if (g_lastCore.egtF > g_dynoEgt) g_dynoEgt = g_lastCore.egtF;
    if (g_boostPsi > g_dynoBoost) g_dynoBoost = g_boostPsi;
    if (g_lastCore.surge && g_dynoT > 1.0f) g_dynoSurge = true;
    if (g_lastCore.pcpBar > g_rodBar) g_dynoPcp = true;
    if (g_fuelGal < 2.0f) g_dynoLowFuel = true;              // sucking air: the starve zone
    if (g_dynoT >= HOLD + SWEEP) dynoFinish(false);
}
void EngineSim::dynoFinish(bool aborted) {
    g_dynoOn = false; g_dynoThr = 0;
    bool blown = (g_state != ES_RUN) && g_health <= 8.0f;    // it died on the rollers
    snprintf(g_dynoRes, sizeof(g_dynoRes),
             "%.0fhp@%.0frpm;egt=%.0f;boost=%.0f;drt=%d;srg=%d;pcp=%d;bln=%d;hlth=%.0f;tq=%.0f;abt=%d;dry=%d;bat=%d;lowf=%d",
             g_dynoPkHp, g_dynoPkRpm, g_dynoEgt, g_dynoBoost,
             (g_dynoEgt > 1150.0f && !blown) ? 1 : 0, g_dynoSurge ? 1 : 0,
             (g_dynoPcp && blown) ? 1 : 0, blown ? 1 : 0, g_health, g_dynoPkTq, aborted ? 1 : 0,
             (!blown && g_fuelGal <= 0.05f) ? 1 : 0,          // ran the tanks dry mid-pull
             (!blown && g_starve >= 1.0f) ? 1 : 0,            // battery cut-off mid-pull
             g_dynoLowFuel ? 1 : 0);                          // starved for fuel through it
    Serial.printf("[dyno] live pull: %s\n", g_dynoRes);
}
String EngineSim::dynoCurveJson() const {
    String j; j.reserve(700);
    j = "{\"rpm\":[";
    for (int i = 0; i < g_dynoN; i++) { if (i) j += ','; j += String(1000 + i * 50); }
    j += "],\"hp\":[";
    for (int i = 0; i < g_dynoN; i++) { if (i) j += ','; j += String((int)g_dynoHp[i]); }
    j += "],\"tq\":[";
    for (int i = 0; i < g_dynoN; i++) { if (i) j += ','; j += String((int)g_dynoTq[i]); }
    j += "]}";
    return j;
}

float EngineSim::timAt(float rpm) const {
    float f = clampf((rpm - 600.0f) / 1500.0f, 0, 1) * 7.0f;
    int k = (int)f; if (k > 6) k = 6; float fr = f - k;
    return timMap[k] + (timMap[k+1] - timMap[k]) * fr;
}
float EngineSim::barkFactor(float rpm) const { return powf(clampf(timAt(rpm) / 8.0f, 0, 1), 1.6f); }

float EngineSim::fuelAt(float rpm) const {
    float f = clampf((rpm - 600.0f) / 1500.0f, 0, 1) * 7.0f;
    int k = (int)f; if (k > 6) k = 6; float fr = f - k;
    return fuelMap[k] + (fuelMap[k+1] - fuelMap[k]) * fr;
}

// Sim chopMiss(): an advanced cam on big fuel misfires lumpy at idle and barks near
// redline. In the sim this is a TORQUE perturbation fed straight into the rev equation,
// so the crank physically shakes - the board only ever did it in the audio, which is why
// the needle sat glassy where the browser's dances.
// ---------------------------------------------------------------------------
//  BIG INJECTORS GIVE A LUMPY IDLE. Researched, not invented.
// ---------------------------------------------------------------------------
//  The named mechanism is the injector's BALLISTIC REGION (dead band): to
//  pass idle's tiny fuel quantity a big nozzle needs such a short pulse that
//  the needle never fully opens, and down there delivered mass per shot is a
//  steep, poorly-repeatable function of energisation time. SAE work on short-
//  pulse injection calls the injector-to-injector spread 'unacceptable', and
//  patents on minimum controllable injection quantity describe needle bounce
//  making delivery 'unpredictable shot to shot as well as injector to
//  injector'. Tuner rule of thumb: 30-50% oversize stays driveable, past that
//  idle quality is the price.
//
//  It is NOT that the duty cycle itself goes unstable - it is that dM/dt is
//  enormous down there, so pre-existing differences between the six injectors
//  (invisible at rated power) get magnified into real fuelling differences.
//  Idle is only ~7-15% of full-load delivery per shot.
//
//  THE C15 IS MEUI, NOT HEUI - cam-actuated, no injection control pressure.
//  That matters here and makes it worse, not better: injection pressure is set
//  by plunger velocity, i.e. by engine speed, AND nozzle flow area is in the
//  pressure equation. So at 600 rpm the pressure is already a small fraction
//  of rated, and fitting bigger nozzles LOWERS IT FURTHER. Big nozzles hurt a
//  MEUI idle twice - they wreck the metering and the atomisation. A common
//  rail does not have that double penalty, which is exactly why this engine
//  family has the reputation.
//
//  Modelled as the research describes it, in two layers:
//    A - a FIXED per-cylinder gain bias, period = 2 crank revs, so the same
//        cylinder is consistently lazy. At 600 rpm that is a ~5 Hz beat
//        against a 30 Hz firing rate - the 'galloping'. +/-1-3% stock,
//        +/-8-15% on big injectors, with one clear outlier.
//    B - random shot-to-shot jitter from needle bounce. +/-1% stock,
//        +/-3-8% big. This is what keeps it from sounding like a loop.
//  Both scale ~1.5-2.5x cold and must fall to ~0 by 1000-1100 rpm or under
//  load - that threshold is the strongest validation target in the research,
//  confirmed by several accounts of big-injector misfires that vanish above
//  1000 rpm. Three reinforcing reasons: the pulse leaves the ballistic region,
//  plunger velocity raises injection pressure, and 50 Hz firing blurs
//  individual events into a tone.
//
//  Deliberately NOT modelled: large rpm swings. A 15 L flywheel MASKS
//  per-cylinder torque deficits, so the acoustic unevenness is large while the
//  rpm wander stays small (research: +/-15-30 rpm warm). The torque
//  perturbation below is what carries the sound.
float EngineSim::injectorIdleChop(float r, float thr) const {
    // 0 stock / 1 big / 2 dual-fuelers, matching PT_FCAP 1.05 / 1.6 / 2.2.
    // Stock is 0.03, not 0.10. MEASURED through the chain: 0.10 put total chop
    // at 0.245 on a bone-stock map, which reads as "noticeable" when a stock
    // C15 should idle near-smooth - clattery, yes, but even. The research puts
    // stock injector-to-injector spread at 1-3% against 8-15% for oversized,
    // and that ratio is what these three numbers encode. 0.03 lands stock at
    // 0.19 total (the tuned-lope term alone is already 0.15 on a stock map).
    static const float OVER[3] = { 0.03f, 0.55f, 1.00f };
    float over = OVER[parts.fuel > 2 ? 2 : parts.fuel];

    // Dies out by ~1100 rpm, and under load. Both are the same mechanism:
    // more fuel per shot means a longer pulse, out of the dead band.
    float rpmFade  = clampf((1100.0f - r) / 400.0f, 0, 1);
    float loadFade = clampf((0.35f - thr) / 0.30f, 0, 1);

    // Cold is worse: longer ignition delay turns the same fuelling spread into
    // a bigger torque spread, and cold fuel atomises badly.
    float cold = 1.0f + 1.2f * coldFactor();

    return over * rpmFade * loadFade * cold;
}

float EngineSim::chopMiss(float dt, float r, float thr) {
    float advI = timAt(680.0f), trmI = fuelAt(680.0f);
    float base = fminf(1.5f, clampf((advI - 4.0f)/6.0f, 0, 1)*0.8f
                           + clampf((trmI - 0.85f)/0.6f, 0, 1)*0.6f);
    // Injector size adds to the TUNED lumpiness rather than replacing it.
    // They are different causes: the tune term above is a lope you dialled in
    // with timing and fuel, this one is the hardware being too big to meter
    // idle cleanly. A bone-stock map with dual-fuelers should still be lumpy.
    base = fminf(1.8f, base + injectorIdleChop(r, thr) * 0.95f);
    // LOPE IDLE button on the web page: a cammed-up idle on demand, without
    // touching the tune. Same scale as an aggressive timing map (~1.1).
    // THE LOPE CHARACTER FOLLOWS THE WHOLE REV RANGE.
    //
    // This faded to nothing by 1500 rpm - (1500-r)/350 - so the engine lost
    // its character the instant it started pulling, which is exactly the
    // 'needs to follow the whole rpm' complaint. A 3000 hp build does not
    // suddenly idle like a stock truck at 1600 rpm; the injectors are still
    // oversized and the timing is still advanced, so the roughness carries up
    // the range - it just gets proportionally smaller as fuelling leaves the
    // ballistic region.
    //
    // Now it keeps a FLOOR all the way to redline instead of vanishing: full
    // depth below 1200, tapering to 35% by 2100 and holding there. The taper
    // is real - the research is clear that the chop largely clears off idle -
    // but it should thin out, not switch off.
    // 0.80 FLOOR, not 0.35. MEASURED through the whole chain: at 0.35 the
    // depth fell to 0.41 up top, which gives an audio floor of 0.64 - only
    // 3.9 dB of swing. The chop was technically still running and completely
    // inaudible, which is the 'should be choppy the whole way' complaint.
    // At 0.80 the depth holds near 1.0 and the floor drops to ~0.28, i.e.
    // 11 dB - audible at redline without being as violent as idle.
    // ======================================================================
    //  THE LOPE, REBUILT FROM THE 2026-10-30 RESEARCH (Mat: "the lope sound is
    //  bad, it's not right"). Two layered mechanisms on a big-injector C15:
    //
    //  (A) ONE LUMP PER ENGINE CYCLE, ON THE SAME CYLINDER. Oversized injectors
    //      asked for ~20 mm3 at idle sit at their minimum controllable quantity,
    //      so a shot is skipped, given the minimum, or "some quantity
    //      in-between" (Cat patent US6837212) - and an over-big one now and then
    //      comes in fat. The dominant idle fluctuation is the 1/2 order: once
    //      per 720 deg, i.e. rpm/120 Hz (5.8 Hz at 700 rpm), at one slot.
    //  (B) THE GOVERNOR HUNTING - "fuel on, fuel off" - a slow quasi-periodic
    //      surge around 0.5-2 Hz, period randomised, +/-25-60 rpm on a built
    //      engine, louder on the fuel-on half.
    //
    //  It fades out by ~1.5x idle and as soon as there is real throttle or
    //  load ("bump the throttle and it smooths right out"). Above that a
    //  built engine keeps a milder big-injector roughness (Mat once asked for
    //  character through the whole range), at a third of the idle depth.
    //  Nothing here runs with the injectors cut (g_fuelOnK, jake / overrun).
    //
    //  The old version barked on a ducked floor (the engine sat at -11 dB
    //  BETWEEN events) at a fixed ~4.2 Hz walking a 6-entry table - closer to
    //  a sound mod than a truck. Here the engine runs at full level and the
    //  weak cylinder drops OUT.
    // ======================================================================
    float idleR  = idleRpmNow();
    float lopeBand = clampf((1.5f * idleR - r) / (0.5f * idleR), 0, 1);        // 1 at idle, 0 by 1.5x idle
    float thrGate  = clampf((0.30f - thr) / 0.25f, 0, 1);                       // real throttle smooths it
    float rough    = 0.33f;                                                     // built-engine roughness above idle
    // HOW WELL THE IDLE IS TUNED (Mat 2026-10-30, after hearing the random
    // lope: "let's say we messed the tuning up, this is how it should idle").
    // A built engine with a sane idle cell lopes in a RHYTHM - "chug chug
    // chugh chu-chu chug chu-chu" with deep barks. Idle timing pushed well
    // past stock (sim stock at 680 rpm = 2.2) or idle fuel off by a lot turns
    // that into the random, uneven stumble.
    float mess = fmaxf(fmaxf((advI - 2.2f - 2.0f) / 3.0f, (trmI - 1.15f) / 0.30f), (0.90f - trmI) / 0.10f);
    mess = clampf(mess, 0, 1); g_lopeMess = mess;
    float dIdle = base * (rough + (1.0f - rough) * lopeBand * thrGate)
                       * clampf((r - 545.0f)/120.0f, 0, 1);
    float dRed  = base * clampf((r - 1990.0f)/110.0f, 0, 1)
                       * clampf((thr - 0.5f)/0.3f, 0, 1);
    float d = fmaxf(dIdle, dRed);
    g_chopD = d;
    if (d <= 0.02f) { g_hunt *= 0.98f; return 0; }
    bool hi = dRed > dIdle; g_chopHi = hi;
    auto r01 = [this]{ return frand()*0.5f + 0.5f; };          // frand() is -1..1

    // ---- (B) governor hunt: a slow sine whose period is re-rolled each half
    {
        g_huntPh += dt * g_huntHz;
        if (g_huntPh >= 0.5f) { g_huntPh -= 0.5f; g_huntSign = -g_huntSign;
                                g_huntHz = 0.6f + r01() * 1.0f; }               // 0.6-1.6 Hz, new each half
        float sw = sinf(g_huntPh * 6.2832f) * g_huntSign;                       // -1..1
        // only the idle band hunts; it is a governor-at-light-fuel thing
        g_hunt = sw * lopeBand * thrGate;
    }

    // ---- (A) one event per engine cycle (720 deg), the same slot every time
    float cycHz = hi ? 6.5f : fmaxf(r, 300.0f) / 120.0f;
    g_chopP += dt * cycHz;
    if (g_chopP >= 1.0f) {
        g_chopP -= 1.0f;
        // what the weak injector did this cycle
        float u = r01();
        g_chopRhythm = false; g_chopDeep = 0.0f;   // the rhythm lives in EngineAudio now (THE PULSE LOPE)
        if (false) {
        } else if (hi) {                            // near-redline bark under power
            g_chopBark = true;  g_chopAmp = 0.7f + r01() * 0.3f;
        } else if (u < 0.10f * d) {                 // an over-big shot comes in fat: bark + smoke
            g_chopBark = true;  g_chopAmp = 0.6f + r01() * 0.4f; g_chopDeep = 0.6f;
        } else if (u < 0.10f * d + 0.35f) {         // skipped - a dead hole, only air
            g_chopBark = false; g_chopAmp = 0.85f + r01() * 0.15f;
        } else {                                    // minimum / in-between - a partial miss
            g_chopBark = false; g_chopAmp = 0.25f + r01() * 0.45f;
        }
        // one firing interval of a six-cylinder four-stroke (120 deg)
        g_chopDur = hi ? (0.05f + r01() * 0.04f) : (20.0f / fmaxf(r, 300.0f)) * (0.85f + r01() * 0.3f);

        g_chopSeq++;
    }
    g_chopDur -= dt;
    // torque: a skipped/partial shot loses its work (~ +/-10-25 rpm per event at
    // idle with J = 4); a fat one adds; the hunt is a slow +/-100 Nm swing
    // (~ +/-40 rpm at 1 Hz) - the sim's crank really moves, so the pitch does too.
    float tq = 0.0f;
    // MEASURED 2026-10-30: hunt*d*100 put a big-plunger idle (d = 1.8) at
    // +/-90 rpm and still swung a stock one. The hunt now has its own curve:
    // nothing below d 0.25 (a stock tune), ~100 Nm = ~+/-40 rpm at the LOPE
    // button (1.1), capped 150 Nm = ~+/-60 rpm built (the research's ceiling).
    // The per-event kick is capped at d = 1 (~+/-20 rpm per event, research).
    float dEv = fminf(d, 1.0f);
    // big plunger: EngineAudio's pulse lope is the lope - no random events here
    if (g_chopDur > 0 && pulseLopeAmt() < 0.5f) tq = g_chopBark ? (g_chopAmp * dEv * 220.0f) : (-g_chopAmp * dEv * 300.0f);
    tq += g_hunt * 100.0f * clampf((d - 0.25f) / 0.85f, 0, 1.5f) * (0.5f + 0.5f * g_lopeMess);
    return tq;
}
float EngineSim::lopeAmt() const { return clampf((timMap[0] - 4.0f) / 6.0f, 0, 1); }

void EngineSim::begin() { g_state = ES_OFF; g_rpm = 0; g_speed = 0; g_gear = 1; loadParts(); }

// ---- air tank persistence -------------------------------------------------
// EEPROM map: parts at 500, channel map at 690 (+12 bytes, so through 702).
// 710 is clear of both.
static const int AIR_EE_ADDR = 710;
static const uint16_t AIR_MAGIC = 0xA17B;

void EngineSim::saveAir() {
    EEPROM.put(AIR_EE_ADDR, AIR_MAGIC);
    EEPROM.put(AIR_EE_ADDR + 2, g_airPsi);
    EEPROM.put(AIR_EE_ADDR + 2 + (int)sizeof(float), g_airSec);
    EEPROM.commit();          // put() no longer commits; this is a shutdown path
    s_airSaved = g_airPsi;
}

// Opportunistic save, for the case the shutdown path cannot cover: pulling the
// pack without switching the engine off first. Only writes when the pressure has
// moved more than AIR_SAVE_DELTA since the last write, because commit() rewrites
// the whole NVS blob - and a flash write disables the cache, which stalls the
// render task on core 0. That is an audible dropout, so this must stay rare.
// 8 psi means a full 0-125 charge costs about 15 writes.
void EngineSim::maybeSaveAir() {
    if (fabsf(g_airPsi - s_airSaved) < AIR_SAVE_DELTA) return;
    saveAir();
}

void EngineSim::restoreAir(float elapsedMin) {
    uint16_t magic = 0;
    EEPROM.get(AIR_EE_ADDR, magic);
    if (magic != AIR_MAGIC) return;              // never saved: leave the defaults
    float p = 0, s = 0;
    EEPROM.get(AIR_EE_ADDR + 2, p);
    EEPROM.get(AIR_EE_ADDR + 2 + (int)sizeof(float), s);
    if (!isfinite(p) || !isfinite(s)) return;    // corrupt blob, don't poison the sim
    // Parked leak-down. A healthy tractor loses a couple of psi an hour through
    // the glad hands and valve seats; 2.5 psi/hr means a truck left a week will
    // genuinely be flat, which is correct, and one parked overnight will not.
    if (elapsedMin > 0) {
        float bleed = 2.5f * (elapsedMin / 60.0f);
        p -= bleed; s -= bleed;
    }
    g_airPsi = clampf(p, 0, AIR_MAX);
    g_airSec = clampf(s, 0, AIR_MAX);
    // Re-derive the dependent flags so boot does not report a stale warning for
    // one tick before the first update() runs.
    g_lowAir   = g_airPsi < LOW_AIR;
    g_springOn = (g_airPsi < SPRING_REL) || g_parkBrake;
    g_compLoaded = (g_airPsi <= AIR_CUTIN);
    // Seed the write tracker, or the first maybeSaveAir() would spend a write
    // immediately on a tank that has not changed at all.
    s_airSaved = g_airPsi;
}
void EngineSim::restore(float savedFuelGal, float savedCoolF, float elapsedMin) {
    if (savedFuelGal >= 0) g_fuelGal = clampf(savedFuelGal, 0, tankGal);
    if (savedCoolF > AMBIENT_F && elapsedMin >= 0) {
        // Newton cooling while parked: big iron block, tau ~ 38 min
        float t = expf(-elapsedMin / 38.0f);
        g_coolantF = AMBIENT_F + (savedCoolF - AMBIENT_F) * t;
        g_oilF     = AMBIENT_F + (savedCoolF + 12 - AMBIENT_F) * t;
        g_transF   = AMBIENT_F + (savedCoolF - AMBIENT_F) * t * 0.8f;
    }
}

void EngineSim::start() {
    if (g_health <= 8.0f) return;                                 // melted/seized: rebuild first
    if (g_starve >= 1.0f) return;                                 // battery cut off: fresh pack first
    if (g_state == ES_OFF) { g_state = ES_CRANK; crankT = 0; g_trigStart = true;
        g_shutMute = 1.0f;                     // undo any previous shutdown mute
        if (g_fuelGal <= 0.05f) { g_state = ES_OFF; return; }     // dry tanks: no start
        crankDur = 1.0f + coldFactor()*3.0f;                      // warm 1s .. dead cold 4s
        if (g_airPsi < AIR_START) g_airPsi = AIR_START;
        g_brakeApplied = 0; s_peakApplied = 0; g_compLoaded = true; }
}
void EngineSim::stop() { if (g_state == ES_CRANK || g_state == ES_RUN) { g_state = ES_STOP; g_trigStop = true; } }

void EngineSim::meltdown() {                          // piston crown let go - catastrophic
    g_meltEvent = true;                               // violent death (audio)
    g_trigStop  = true;
    g_health    = 0;                                  // needs a full rebuild to run again
    g_pistonMelt = 1.0f;
    g_state     = ES_OFF;                             // dies instantly; truck coasts (ES_OFF)
    g_shiftCut  = 1;
}

float EngineSim::roadDrag(float v_ms, float mass, float brake01) {
    // At a standstill the browser returns the BRAKE force, not zero:
    //     var b = brake*(0.40*MASS*GRAV);  if (v<=0.01) return b;
    // Returning 0 here meant a braked or park-braked truck had nothing holding
    // it and could creep - which also defeated the parkBrake -> brake01=1.0
    // override a few lines below.
    float b = brake01 * (0.40f * mass * GRAV);
    // FULL STOP TIME: full brakes stop the truck in g_stopTime seconds from the
    // speed it was doing when the brakes went on (g_stopFrom); partial brake
    // scales it. A force, not a speed override, so the coupled driveline, the
    // gears and the ESC all follow it. It only ever ADDS braking - the air-brake
    // figure still wins if it is the stronger of the two.
    // Slider 0 = INSTANT: a stop time of 0.05 s, i.e. the truck is stopped
    // within a couple of frames of full brakes (and proportionally quickly on
    // partial brake). Max on the slider is 3 s.
    // Partial brake adds up to 2 s on top of the slider (full = the slider value,
    // half = +1 s, a touch = +~1.8 s), so a light brake is never a slam even
    // with the slider at 0.
    if (g_stopEnabled && brake01 > 0.05f) {
        float T = fmaxf(g_stopTime + (1.0f - brake01) * 2.0f, 0.05f);
        float vRef = fmaxf(g_stopFrom, 5.0f) * MPH_MS;
        float bStop = mass * (vRef / T);
        if (bStop > b) b = bStop;
    }
    if (v_ms <= 0.01f) return b;
    return CRR*mass*GRAV + 0.5f*AIR_RHO*CDA*v_ms*v_ms + DAMP*v_ms + b;
}

// What the drive tyres can put down. Street: ~half the mass on the drive
// axle(s), mu 0.65 - a heavy rig squats and cannot wheelie. Race: a bobtail
// on slicks with the weight transferred back, ~60% on the axle at mu 0.95.
// That is 0.32 g street / 0.57 g race, which is the real ceiling on any
// launch - a 3000 hp truck leaves on its tyres, not its dyno sheet.
float EngineSim::tractionN(float mass) const {
    // Race: 0.75 of the weight on the drive axle (a bobtail launching hard
    // transfers nearly everything rearward) on slicks at mu 1.2 - a prepped
    // strip is stickier than mu 1. That is 0.9 g, where a drag truck's 60 ft
    // actually lives; 0.57 g gave a 2.8 s 60 ft and a 15 s run.
    float axle = fminf(mass * (g_raceMode ? 0.75f : 0.50f), 15000.0f);
    float mu   = g_raceMode ? 1.20f : 0.65f;
    return mu * axle * 9.81f;
}

void EngineSim::stepCoupled(float dt, float throttle, float jakeAmt, float brake01, float mass) {
    if (g_clutchPedal > 0.5f) jakeAmt = 0.0f;            // clutch switch kills the jake, like the real truck
    float ratioT = gearRatio[g_gear - 1] * DIFF * g_fdScale;   // g_fdScale: race rear end
    if (g_parkBrake) brake01 = max(brake01, 1.0f);       // spring brakes hold everything
    float we = g_rpm;
    float wt = g_speed * rpmPerMph[g_gear - 1];
    float v_ms = g_speed * MPH_MS;
    float Fdrag = roadDrag(v_ms, mass, brake01) + mass * GRAV * g_grade;   // hills load or push

    // real C15 6NZ curve (scaled into the sim's world); Jake 600E retard scales with rpm
    const float C15_SCALE = PEAK_TORQUE / 2508.0f;        // keep sim mass/drag balance
    // RACE POWER multiplies the engine's output. PEAK_TORQUE 2900 Nm is about
    // 650 hp at 2100 rpm - a mildly built 6NZ, not a competition engine - so
    // without this a "race" run is a street truck with a different gear
    // ladder, which is exactly why it bogged. 1.75x lands near 1140 hp and
    // 3.50x near 2300 hp; the jake retard is deliberately NOT scaled, since a
    // bigger fuel system does not make the exhaust brake stronger.
    const float raceTq = raceTqMult();
    float Teng = (throttle > 0.05f)
               ? fmaxf(g_lastCore.torqueNm, 0.0f) * C15_SCALE * raceTq * g_derate * DRIVE_EFF
               // Jake retard RAMPS WITH RPM, exactly as the sim does:
               //   jbk = 1400 + 1400*clamp((rpm-1100)/1000, 0, 1)
               // The old flat jakeAmt*2600 hit nearly twice as hard as the sim at 1100rpm
               // and slightly softer at redline - the opposite shape to a real Jake brake,
               // which builds as the engine spins faster.
               : -(ENG_BRAKE + jakeAmt * (1400.0f + 1400.0f * clampf((we - 1100.0f) / 1000.0f, 0, 1)))
                 * (we / MAX_RPM);
    Teng -= g_ptoLoad * 260.0f * (we / MAX_RPM);         // hydraulic pump drag off the PTO
    float grab = (wt >= IDLE_RPM - 60.0f) ? 1.0f : clampf(throttle * 1.5f, 0, 1);
    // COMING TO A STOP ON THE JAKE: the driver declutches before he lugs it.
    //
    // With the box now HOLDING 4th down to a stop (JAKE_FLOOR_GEAR), wheel-rpm
    // in 4th falls under idle below ~4.7 mph. Left coupled, the engine gets
    // dragged down to LUG_MIN_RPM and sits there sounding strangled - and with
    // s_clutchLocked still set it was the other half of the 330 rpm / 2.3 mph
    // deadlock measured on the bench.
    //
    // Nobody rides a gear into the ground. Pull the clutch as synchronous
    // speed drops under idle so the engine falls back to a clean idle while
    // the service brakes finish the stop.
    // ...AND NOT ONLY ON THE JAKE. MEASURED in race mode, which has no
    // downshifts: braking from 116 mph the box stayed in 10th, the engine
    // was dragged to the 330 floor, and because the locked branch derives
    // speed FROM rpm the truck sat pinned at 18.3 mph with the brakes hard
    // on - 10th had a minimum road speed. Street only escaped this by
    // rowing down. No driver lets an engine be dragged under idle in any
    // gear; the clutch goes in. So: off the throttle and wheel speed under
    // idle, in ANY gear, declutch. At a stop the engine is then idling with
    // the clutch out, which is what a stopped truck does.
    // HARD STOP: declutch while the engine is still ABOVE idle. Waiting for
    // wt to fall under idle meant every hard stop dragged the engine to
    // ~660 first and you heard it sag before it recovered.
    if (throttle < 0.1f && (wt < IDLE_RPM - 20.0f || (brake01 > 0.5f && wt < IDLE_RPM + 220.0f))) {
        grab = 0.0f;
        s_clutchLocked = false;
    }
    // THE PEDAL APPLIES WHENEVER IT IS OUT - not only in manual mode.
    //
    // This was `if (g_manual)`, so in an automatic race launch the whole
    // release-time feed-out ran the pedal from 1.00 to 0.00 and changed
    // NOTHING about the clutch: grab stayed at 1.0 (full throttle, wt under
    // idle) and the only effect was the lock-up gate. MEASURED: the [pedal]
    // trace counted down 1.00 -> 0.09 while the engine sat at exactly 2100
    // the entire time. The slider had been dead since it was added.
    if (g_manual || g_clutchPedal > 0.0f) {      // proportional pedal, or a launch feed-out
        grab = min(grab, 1.0f - g_clutchPedal);
        if (g_clutchPedal > 0.5f) s_clutchLocked = false;
    }

    float slip = we - wt;
    if (s_clutchLocked && g_speed < CLUTCH_RELEASE_MPH && throttle < 0.06f) s_clutchLocked = false;
    // A SLIPPING CLUTCH MUST NOT LOCK. Without this the capacity cut above
    // would be bypassed the moment slip fell under the lock threshold, the
    // clutch would weld solid, and the engine would get dragged down again.
    //
    // Now gated on the PEDAL instead of a capacity cut: while the pedal is
    // still coming out on a launch the clutch must not weld, and once it is
    // home the clutch locks normally with its full rating. That is the whole
    // difference between a release time and a slipper.
    if (!s_clutchLocked && grab > 0.98f && fabsf(slip) < CLUTCH_LOCK_SLIP
        && g_clutchPedal < 0.10f) s_clutchLocked = true;
    if (g_clutchPedal >= 0.10f) s_clutchLocked = false;

    if (s_clutchLocked) {
        float Tload = Fdrag * WHEEL_R / ratioT, k = WHEEL_R / ratioT;
        float Jtot = J_ENGINE + mass * k * k;
        float rpm = we + (Teng - Tload) / Jtot * dt * RAD;
        // Clamp at the TOP only. Flooring rpm at IDLE_RPM here and then
        // deriving speed from it gave every gear a minimum road speed the
        // physics could not go below - 1.8 mph in 1st, 19.4 mph in 10th -
        // no matter how hard the brakes were applied. With the park brake
        // set, or on spring brakes after an air loss, the truck still crept
        // forward. Speed is the real state when the clutch is locked.
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        float spd = rpm / rpmPerMph[g_gear - 1];
        if (spd < 0) spd = 0;
        // TYRES CAP A LOCKED DRIVELINE TOO. This branch had none: 11,400 Nm
        // through 6th at 8000 kg came out at 2.5 g, so every in-gear pull
        // at race power lasted ~0.2 s and the whole 17.7 s run was shift
        // time. Only applied while accelerating - braking is left alone.
        {
            float vNow = g_speed * MPH_MS;
            float vMax = vNow + ((tractionN(mass) - Fdrag) / mass) * dt;
            if (vMax > vNow && spd * MPH_MS > vMax) {
                spd = vMax / MPH_MS;
                rpm = spd * rpmPerMph[g_gear - 1];
            }
        }
        g_speed = spd;
        g_rpm   = spd * rpmPerMph[g_gear - 1];
        // Let go before the engine stalls with the truck. Unlike the test
        // above this is NOT gated on throttle, so holding a little pedal
        // cannot defeat the service or park brakes.
        if (g_speed < CLUTCH_RELEASE_MPH * 0.5f) s_clutchLocked = false;
        if (g_rpm < LUG_MIN_RPM) g_rpm = LUG_MIN_RPM;
    } else if (grab < CLUTCH_OUT_GRAB) {                        // clutch OUT: free engine to idle
        float idleT = idleRpmNow() + g_ptoLoad * 90.0f;   // PTO working idle bump
        float target = (throttle > 0.05f) ? idleT + throttle*(MAX_RPM-idleT)*0.92f : idleT;
        // linear slew (matches neutral free-rev feel)
        if (target > we) g_rpm = we + fminf(target - we, (1150.0f - 0.30f*(we-IDLE_RPM))*dt);
        else g_rpm = we - fminf(we - target, (240.0f + 0.28f*(we-IDLE_RPM))*dt);
        v_ms -= (Fdrag / mass) * dt; if (v_ms < 0) v_ms = 0; g_speed = v_ms / MPH_MS;
    } else {                                                    // SLIPPING/LAUNCH: lug from idle, grunt
        // Clutch capacity is DYNAMIC in the sim, not a fixed number: it tracks a slowly
        // decaying peak of engine torque, so a built motor gets a clutch that can hold it
        // and a stock one doesn't.  CAP = clamp(tqRef*1.5, 3000, 8000) * heat fade.
        // The old fixed 4600 meant a big build slipped far too early and a stock engine
        // held on far too well.
        g_tqRef = fmaxf(fabsf(Teng), g_tqRef * 0.9985f);
        // THE SLIPPER. Lowering the clutch's torque capacity is what keeps the
        // engine up: whatever the clutch cannot transmit, it slips instead, so
        // the crank is no longer welded to road speed. At full slip the
        // capacity drops to 30% and the engine pulls away from the driveline
        // the way a drag truck's does.
        // 0.35 of cut, not 0.70 - I overcorrected and the clutch stopped
        // driving. At 0.70 a slider setting of 0.6 cut capacity to 58%, so
        // the truck fell flat and all that happened was slip and heat:
        // whatever the clutch does not transmit the truck does not get.
        //
        // At 0.35 a 0.6 setting leaves 79% capacity - enough to keep the
        // engine off the driveline's leash without giving away the drive.
        // The point of a slipper is to hold revs UP while still pulling, not
        // to disconnect the engine.
        // No capacity cut any more - see setClutchRelease in EngineSim.h. The
        // clutch has its full rating; what varies is how fast the PEDAL comes
        // up, which is what a driver actually controls.
        const float slipCap = 1.0f;
        // CAPACITY JUST UNDER PEAK TORQUE - that is what makes a launch PULL.
        //
        // Was tqRef*1.5 clamped to 8000. Two failure modes, both measured:
        //  * 3000 hp class makes ~11,400 Nm. 8000 < that, so the engine
        //    out-muscled the clutch, sat pinned on the 2100 limiter and the
        //    truck left on pure slip. Screaming, not pulling.
        //  * 1500 hp class makes ~5,700 Nm. Cap was 1.5x = 8000 > that, so
        //    the clutch yanked the engine straight down to the 330 floor.
        //    The original 'bogged harddddd'.
        // Neither has a stable operating point. With capacity a hair UNDER
        // peak torque there is one: the engine is dragged down off the
        // limiter until its torque rise meets the clutch - about 1500-1600
        // rpm on this curve - and sits there, loaded and growling, slipping
        // the truck up to speed until wheel speed catches it and it locks.
        // That is exactly how a drag slipper clutch is set, and it is the
        // sound of an engine working. Street is unchanged: 650 hp tqRef is
        // ~2460, under the 3000 floor, so a stock truck still dumps and
        // stalls like a stock truck.
        // RACE: size the clutch to the BUILD, not to tqRef's recent memory.
        // tqRef decays ~14%/s, and ten seconds of staging on the limiter
        // with the fuel chopped can leave it under the engine's real peak -
        // then 0.95x of it cannot pull the engine down and it slips on the
        // limiter forever. A drag clutch is bought for the engine's peak.
        float tqSized = g_tqRef;
        if (g_raceMode)
            tqSized = fmaxf(tqSized, 2508.0f * C15_SCALE * raceTq * g_derate * DRIVE_EFF);
        float capEff = clampf(tqSized * 0.95f, 3000.0f, 24000.0f) * slipCap
                     * clampf(1.0f - g_clutchHeat*0.6f, 0.30f, 1.0f);   // fades when hot
        // A race clutch (multi-disc ceramic) takes slip a street organic
        // plate cannot. At the street rate a 3 s launch slip faded the cap
        // under the engine's limiter torque, so it climbed straight back to
        // 2100 before wheel speed ever caught it.
        const float heatGain = g_raceMode ? 0.4e-4f : 1.1e-4f;
        g_clutchHeat = clampf(g_clutchHeat + fabsf(slip)*grab*heatGain*dt, 0, 1.4f);  // slip cooks it
        float stiff = CLUTCH_STIFF * (1 + grab*grab*CLUTCH_ENGAGE_FIRM);
        // A slipping clutch transmits its FRICTION CAPACITY, not whatever a
        // spring term computes from raw slip. grab*stiff*slip reached 19,000
        // Nm at 0.6 throttle from a standstill - twenty times the engine's
        // output - so capEff (3000-8000) was the only thing bounding it and
        // the engine lost every time. Scale the holding capacity by pedal
        // engagement so a partly-engaged clutch slips instead of stalling.
        // GRAB WAS APPLIED TWICE AND KILLED EVERY LAUNCH.
        //
        // MEASURED off the line, 3000 hp class, 2 s release: the engine sat
        // pinned at EXACTLY 2100 rpm from 0.1 mph to 5.6 mph while the pedal
        // fed 1.00 -> 0.19, and boost read 29.9 the whole way. The revs never
        // moved, because the clutch was transmitting almost nothing - the
        // truck only reached 5.6 mph because that is where 1st gear's
        // gearing finally matches 2100 rpm. That is the "it will not pull
        // through a launch" complaint, and it is this line.
        //
        // grab (= 1 - clutchPedal during a launch) was scaling BOTH the
        // capacity and the spring term, so at pedal 0.9 the clutch could pass
        // a tenth of a tenth of its rating. The engine was isolated from the
        // load, so it just leaned on the limiter.
        //
        // A slipping clutch transmits its FRICTION CAPACITY. Capacity rises
        // with how far the plates are engaged - that is one grab, and it is
        // the physical one. The spring term only decides the SIGN and lets
        // the torque taper as slip vanishes near lock-up; it must not be
        // scaled by engagement a second time.
        float Tcap = capEff * grab;
        float Tc = clampf(stiff*slip, -Tcap, Tcap);
        // The idle governor has to actually DEFEND idle. At 2 Nm per rpm of
        // droop it produced 700 Nm at the 330 floor, against a clutch spring
        // making 2800 Nm at a third throttle - so any launch dragged the
        // engine to LUG_MIN_RPM and parked it there, at 0 mph, until the
        // throttle was nearly wide open. A real engine fights back hard as
        // it is pulled under idle, and stalls only when it runs out of
        // torque - not because the spring maths outruns it. The rate rises
        // steeply with droop so it holds idle firmly but still lets a
        // genuinely overloaded launch pull it down and stall.
        float droop = idleRpmNow() - we;
        float Tgov = (droop > 0.0f)
                   ? IDLE_GOV_GAIN * droop * (1.0f + droop * 0.02f)
                   : 0.0f;
        float we_new = clampf(we + (Teng + Tgov - Tc)/J_ENGINE*dt*RAD, LUG_MIN_RPM, MAX_RPM);
            // Browser, slipping branch: `if(throttle<0.05) eng.rpm += (wheelRpm-eng.rpm)*clamp(dt*5,0,1)`
            // Off the throttle the DRIVELINE drags the engine to wheel speed -
            // that is what makes a coasting truck's revs track road speed
            // instead of falling to idle.
            //
            // GATED ON ACTUALLY MOVING, which the browser gets for free and this
            // did not. At a standstill g_speed is 0, so wheelRpm is 0, and this
            // term drags the engine toward a dead stop. MEASURED on the bench:
            // a mid-throttle launch in 1st sat at 514-577 rpm for 18 seconds at
            // 0.0 mph and then stalled - the fuel governor was fighting a term
            // pulling the crank to zero. Below ~1 mph the clutch is slipping and
            // the driveline cannot drag the engine anywhere.
            if (g_throttle < 0.05f && g_speed > 1.0f) {
                float wheelRpm = g_speed * rpmPerMph[g_gear-1];
                we_new += (wheelRpm - we_new) * min(1.0f, dt*5.0f);
            }
        g_rpm = we_new;
        float Fwheel = Tc * ratioT / WHEEL_R;
        float Ftrac = tractionN(mass);
        if (Fwheel >  Ftrac) Fwheel =  Ftrac;
        if (Fwheel < -Ftrac) Fwheel = -Ftrac;
        v_ms += (Fwheel - Fdrag) / mass * dt; if (v_ms < 0) v_ms = 0; g_speed = v_ms / MPH_MS;
    }
}

void EngineSim::update(float dt, float throttle01, bool inDrive, bool inReverse,
                       float jakeAmt, float loadAccel, float brake01) {
    if (dt > 0.1f) dt = 0.1f; if (dt < 0) dt = 0;
    if (loadAccel < 0.1f) loadAccel = 0.1f;
    brake01 = clampf(brake01, 0, 1); jakeAmt = clampf(jakeAmt, 0, 1);
    // LIVE DYNO: the cell owns the controls - full rack, neutral (the wheels and
    // ESC stay dead, it is strapped down), no jake. The brake aborts the pull.
    if (g_dynoOn) {
        if (brake01 > 0.5f || g_state != ES_RUN) dynoFinish(brake01 > 0.5f);
        else { dynoTick(dt); throttle01 = g_dynoThr; inDrive = false; inReverse = false; jakeAmt = 0; brake01 = 0; }
    }
    g_throttle = throttle01; g_drive = inDrive; g_jakeAmt = jakeAmt; g_reverse = inReverse;
    if (g_rpm > 1050.0f) g_jakeRpmOk = true; else if (g_rpm < 950.0f) g_jakeRpmOk = false;   // Jacobs low-rpm cut-out
    // (No auto jake on descents - Mat 2026-10-31: "no auto jake". The jake is
    // the driver's switch only; downhill the box just holds its gear.)
    // full-stop-time reference: the speed when the driver came onto the brakes
    if (brake01 > 0.05f && !g_stopLatched) { g_stopFrom = fmaxf(g_speed, g_revSpeed); g_stopLatched = true; }
    if (brake01 < 0.02f) g_stopLatched = false;
    // ---- FEED THE CLUTCH PEDAL OUT ON A LAUNCH ----------------------------
    // g_launchT is armed when Drive is selected from a standstill. While it
    // runs, the pedal is held proportionally out and then eased home, so the
    // clutch takes up over clutchRelease seconds instead of slamming.
    //
    // MEASURED why this matters: grab is `(wt >= IDLE_RPM-60) ? 1 : throttle*1.5`,
    // and at a standstill wt is 0 - so at full throttle grab went straight to
    // 1.0 and the clutch clamped SOLID at zero road speed. Every launch test
    // so far was a clutch dump, which is why it bogged however much power or
    // gearing was thrown at it.
    // ---- RE-STAGE IN DRIVE: a transbrake --------------------------------------
    // After a run the box is still in the top gear, and the launch only
    // armed on the N->D edge - so a second run from a standstill in D would
    // have slip-launched in 10th. Now: stopped in D, foot on the brake (or
    // park brake), throttle up = STAGED. The pedal is held in, the gear goes
    // back to the launch gear, the engine free-revs and builds boost against
    // nothing (no clutch heat, no creep through the brakes). Let the brake
    // off with the throttle still up and the launch arms exactly as it does
    // off the N->D edge: pedal fed out over the release time, clocks reset.
    {
        static bool s_staged = false;
        bool holding = (brake01 > 0.5f) || g_parkBrake;
        bool stillD  = g_raceMode && inDrive && !inReverse && g_speed < 0.5f && !shifting;
        // No `g_launchT <= 0` gate: going into Drive arms a feed-out timer, and
        // that gate meant you could not stage until it ran out - by which time the
        // clutch had come out against the brakes, the truck crept, and the launch
        // never used the release time (Mat: 'clutch let out time no longer works').
        if (stillD && holding && g_throttle > 0.5f) {
            if (g_gear != g_raceLaunch) { g_gear = g_raceLaunch; g_manReq = g_gear; }
            g_launchT = 0.0f;
            g_clutchPedal = 1.0f;
            s_staged = true;
        } else if (s_staged && stillD && !holding && g_throttle > 0.5f) {
            g_launchT = clampf(g_clutchReleaseUI, 0.0f, 3.0f);
            g_clutchPedal = (g_launchT > 0.01f) ? 1.0f : 0.0f;
            g_raceSkipDone = false;
            g_dragDist = 0.0f; g_dragTime = 0.0f;
            g_et60 = 0.0f; g_etQtr = 0.0f; g_trapMph = 0.0f;
            g_dragRun = true;
            s_staged = false;
        } else if (!stillD || g_throttle < 0.2f) {
            s_staged = false;
        }
    }
    // ---- RACE REAR END, swapped only at a standstill ------------------------
    // The locked branch derives SPEED from rpm through this table, so
    // changing it while rolling would step the road speed 18% in one tick.
    {
        float want = g_raceMode ? RACE_FD_SCALE : 1.0f;
        if (want != g_fdScale && g_speed < 1.0f) {
            g_fdScale = want;
            for (int i = 0; i < N_GEARS; i++)
                rpmPerMph[i] = RPM_PER_MPH_BASE[i] * g_fdScale;
        }
    }
    // ---- DRAG CLOCKS ------------------------------------------------------
    if (g_dragRun) {
        g_dragTime += dt;
        g_dragDist += g_speed * MPH_MS * dt;
        // 60 ft = 18.288 m
        if (g_et60 <= 0.0f && g_dragDist >= 18.288f) g_et60 = g_dragTime;
        // 1320 ft = 402.336 m - the finish line
        if (g_dragDist >= 402.336f) {
            g_etQtr = g_dragTime; g_trapMph = g_speed; g_dragRun = false;
        }
        // Abandon a run that is clearly over rather than timing forever.
        if (g_dragTime > 60.0f) g_dragRun = false;
    }

    if (g_launchT > 0.0f) {
        g_launchT = fmaxf(0.0f, g_launchT - dt);
        float rel = clampf(g_clutchReleaseUI, 0.0f, 3.0f);
        g_clutchPedal = (rel > 0.01f) ? clampf(g_launchT / rel, 0.0f, 1.0f) : 0.0f;
    } else if (brake01 > 0.50f && g_speed < 10.0f && g_drive) {   // 10 mph / 0.50: in before the dip
        // CLUTCH IN WHEN STOPPING. A driver pushes the pedal as the truck
        // comes to rest - nobody lets an engine be dragged to a stall by the
        // driveline. Without this, braking hard in gear pulled the crank down
        // with the wheels and killed it, which is the stalling-on-the-brakes
        // bug.
        //
        // Gated on a real brake application AND low speed AND being in gear,
        // so it cannot disengage the drive at road speed - braking at 50 mph
        // still engine-brakes normally, which is what you want.
        g_clutchPedal = fminf(1.0f, g_clutchPedal + dt * 12.0f);    // ~80 ms, a stamp not a press
    } else if (g_clutchPedal > 0.0f && !g_manual) {
        // not launching and nobody is holding a pedal - let it home
        g_clutchPedal = fmaxf(0.0f, g_clutchPedal - dt * 4.0f);
    }
    bool running = (g_state == ES_RUN);
    // chopMiss() used to be called ONLY from the neutral free-rev branch, so
    // g_chopD held whatever value it had when you last sat in neutral and kept
    // driving the audio chop forever while driving - and the near-redline bark
    // could only ever fire in neutral, where you would rarely hold 2000 rpm.
    // Evaluate it every tick so both the audio and the torque perturbation see
    // the real current state.
    g_chopTq = running ? chopMiss(dt, g_rpm, throttle01) : 0.0f;
    if (!running) { g_chopD = 0.0f; g_chopHi = false; }
    // INJECTORS OFF = NO LOPE. On the jake, and on any closed-throttle overrun
    // above idle (the governor cuts fuel), nothing is burning, so there is no
    // misfire to chop and no lumpy cylinder - the engine is an air pump. The
    // chop used to keep running through the jake, which was the shudder. Fades
    // over ~0.1 s both ways so the lope comes back as the fuel does.
    {
        // The jake only cuts the fuel where a Jacobs actually works - above ~1000
        // rpm (it cuts out below that). Before, the jake SWITCH being on at idle
        // counted as injectors-off, which silenced the big-plunger lope until the
        // jake was turned off (Mat: "the lope didn't come on till I restarted").
        bool fuelCut = throttle01 < 0.05f && ((jakeEngaged() && g_rpm > 1000.0f) || g_rpm > idleRpmNow() + 150.0f);
        g_fuelOnK += ((fuelCut ? 0.0f : 1.0f) - g_fuelOnK) * fminf(1.0f, dt * 10.0f);
        g_chopTq *= g_fuelOnK; g_chopD *= g_fuelOnK;
    }
    // RACE LAUNCH. Dropping into Drive in race mode engages the gear you
    // nominated instead of 1st, so a clutch dump at 2000 rpm puts the engine
    // straight onto the ratio you want. Edge-triggered: only on the SELECTION,
    // so the box is free to row up normally afterwards.
    {
        static bool s_prevDrive = false;
        if (g_raceMode && inDrive && !s_prevDrive && g_speed < 1.0f) {
            g_gear   = g_raceLaunch;
            g_raceSkipDone = false;        // a fresh launch re-arms the skip
            // Start feeding the pedal out. Floored at the instant of
            // selection, home clutchRelease seconds later.
            // Feed the clutch out from now - UNLESS the driver is on the brakes,
            // in which case hold it in and let the brake-off (staged launch above)
            // start the feed-out. Otherwise the timer ran out while braked.
            bool held = (brake01 > 0.5f) || g_parkBrake;
            g_launchT = held ? 0.0f : clampf(g_clutchReleaseUI, 0.0f, 3.0f);
            // Start the clocks. Distance and time both run from the instant
            // the gear goes in, which is where a real tree starts them.
            g_dragDist = 0.0f; g_dragTime = 0.0f;
            g_et60 = 0.0f; g_etQtr = 0.0f; g_trapMph = 0.0f;
            g_dragRun = true;
            g_clutchPedal = (g_launchT > 0.01f || held) ? 1.0f : 0.0f;
            g_manReq = g_raceLaunch;
            shifting = false; shiftTo = 0;
        }
        s_prevDrive = inDrive;
    }

    bool parkGear = (!inDrive && !inReverse);

    // ---- fuel command: the GOVERNOR, ported from the sim ----
    // This was a raw throttle passthrough, which is why the board never dug in when you
    // lugged it and never cut fuel on a shift. The sim runs three cases:
    //   shifting    -> 0 (fuel cut through the change)
    //   clutch out  -> idle governor: rpm-error term + idle-hold term scaled by trim
    //   driving     -> throttle*0.9 PLUS a term that adds fuel as the engine is pulled
    //                  below the rpm the pedal is asking for, with a 0.08 idle floor
    { float fuelTgt;
      float idleNow = idleRpmNow();
      float target  = idleNow + g_throttle * (MAX_RPM - idleNow);
      bool  clutchOut = (g_clutchPedal > 0.85f) || parkGear;
      if (!running)      fuelTgt = 0;
      else if (shifting) fuelTgt = 0;                                  // cut through the shift
      else if (g_reverse) fuelTgt = g_throttle * 0.5f;                  // sim: gentle crawl fuelling
      else if (clutchOut) {
          // STAGING ON THE LIMITER. In neutral the idle governor owns the
          // fuel - an rpm-error term capped around 0.9 total - so a free rev
          // never asked for full fuel and boost topped out near 5 psi. That
          // is right for blipping the throttle, and wrong for staging a drag
          // run, where the point is to sit on the limiter and make boost
          // before the clutch drops.
          //
          // A real truck does exactly this: hold it wide open against the
          // governor in neutral and the turbo spools, because there IS load -
          // the limiter is fighting the fuel. So when the throttle has been
          // HELD past 0.9 for a moment, hand fuelling over to the throttle
          // and let the rev limiter be the thing that stops it.
          //
          // The hold requirement matters. Without it a throttle stab in
          // neutral would dump full fuel and the free rev would turn into a
          // limiter bounce every time you blipped it. g_stageHold builds over
          // ~0.45 s of wide-open and decays fast, so a blip never triggers it
          // but staging for a launch does.
          if (g_throttle > 0.90f) g_stageHold = fminf(1.0f, g_stageHold + dt / 0.45f);
          else                    g_stageHold = fmaxf(0.0f, g_stageHold - dt / 0.12f);

          float tI = fmaxf(1.0f, fuelMap[0]);                          // idle-band trim
          // CATCH THE FALL. The ECM sees the revs dropping onto idle and adds
          // fuel on the way down, so a released free rev lands softly instead of
          // undershooting (MEASURED: 785 -> 686 at a 720 idle, then a bounce).
          // Rate term only within 220 rpm of idle and only while falling.
          static float s_prevR = 0.0f;
          float fall = (dt > 0) ? (s_prevR - g_rpm) / dt : 0.0f;  s_prevR = g_rpm;
          float nearIdle = clampf((idleNow + 220.0f - g_rpm) / 220.0f, 0, 1);
          // RATE DAMPING, both ways. The governor was proportional only, through
          // the fuel lag, so it overshot both sides: MEASURED +/-95-120 rpm at a
          // steady 1 Hz at a stock idle. Real governors also act on how fast the
          // revs are moving - add fuel while they fall, take it while they rise.
          float gov = clampf((target - g_rpm) * 0.005f, 0, 0.30f)
                    + clampf((idleNow + 40.0f - g_rpm) * 0.006f / tI, 0, 0.6f / tI)
                    + clampf(fall * 0.0004f, -0.20f, 0.30f) * nearIdle;
          if (gov < 0.0f) gov = 0.0f;
          // Blend, not a switch - a step in fuel command is a lurch.
          fuelTgt = gov + (clampf(g_throttle * 0.95f, 0, 1.0f) - gov) * g_stageHold;
      } else {
          g_stageHold = 0.0f;
          float ge = (target - g_rpm) / MAX_RPM;                       // how far it's been pulled down
          fuelTgt = clampf(g_throttle * 0.9f + fmaxf(0.0f, ge) * 1.6f, 0.08f, 1.0f);
      }
      // SHIFT FUEL CUT IS DIRECT, not lagged - and this is why boost fell on
      // every gear change.
      //
      // The browser's governor is:
      //     if (shiftT > 0) fuelCmd = 0;          <- direct, bypasses _fdN
      //     else if (...)   { _fdN += ...; fuelCmd = _fdN; }
      //
      // The shift case NEVER TOUCHES _fdN, so _fdN keeps its pre-shift value and
      // fuel is restored INSTANTLY when the shift ends. This code instead ramped
      // g_fuelDeliv down to zero (FUEL_FALL 7 = ~140 ms) and then had to climb
      // all the way back at FUEL_RISE 2.4 (~420 ms) afterwards. The turbine was
      // therefore starved for the shift PLUS half a second of re-fuelling, which
      // is what dumped 5-6 psi per shift and left the whistle sweeping up from
      // nothing in every gear.
      //
      // Hold the lag state frozen through the shift and feed the core a hard
      // zero, exactly as the browser does.
      if (shifting && running) {
          g_fuelDeliv = 0.0f;              // what the core sees this tick
      } else {
          if (g_fuelHold > 0.0f) {         // first tick after a shift: restore
              g_fuelDeliv = g_fuelHold;
              g_fuelHold  = 0.0f;
          }
          // FUEL STARVATION (battery cutoff): drop the fuel in random 60-160 ms
          // gaps, more and longer as starve rises - a diesel sucking air as it
          // runs dry, not a clean switch-off.
          // the tank running dry sucks air the same way: from ~2 gal down
          float starveNow = fmaxf(g_starve, clampf((2.0f - g_fuelGal) / 2.0f, 0, 1));
          if (starveNow > 0.0f && running) {
              static uint32_t s_gapEnd = 0, s_nextGap = 0;
              uint32_t ms = millis();
              if (ms >= s_nextGap) {
                  bool cut = (random(1000) / 1000.0f) < (0.25f + 0.6f * starveNow);
                  s_gapEnd  = cut ? ms + 60 + random(100) : ms;
                  s_nextGap = ms + 120 + random(160);
              }
              if (ms < s_gapEnd) fuelTgt = 0.0f;
              else fuelTgt *= (1.0f - 0.5f * starveNow);
          }
          float r = (fuelTgt > g_fuelDeliv) ? FUEL_RISE : FUEL_FALL;
          g_fuelDeliv += (fuelTgt - g_fuelDeliv) * min(1.0f, dt * r);
      } }
    // ---- physical engine core, EVERY TICK ----
    // This used to live inside stepCoupled(), which only runs in the coupled-drive
    // branch - so in neutral, reverse and mid-shift the combustion model wasn't running
    // at all and g_lastCore went stale. The sim calls coreTick() every frame regardless
    // of what the driveline is doing, and the free-rev branch reads its torque.
    { float f01 = clampf((g_rpm - 600.0f) / 1500.0f, 0, 1) * 7.0f;
      int kk = (int)f01; if (kk > 6) kk = 6; float frc = f01 - kk;
      g_core.fuelTrim = clampf(fuelMap[kk] + (fuelMap[kk+1] - fuelMap[kk]) * frc, 0.7f, 2.0f);
      g_core.advDeg   = timMap[kk] + (timMap[kk+1] - timMap[kk]) * frc;
      g_core.wgPsi    = boostMax * PT_WGMUL[parts.turbo > 2 ? 2 : parts.turbo];
      bool run_ = (g_state == ES_RUN);
      // Hand the core the jake amount. It needs it: an engine brake keeps mass
      // flowing through the turbine (cool, compressed, unburned), so the turbo
      // holds speed instead of spooling down as it would on a bare fuel cut.
      g_lastCore = g_core.tick(dt, run_ ? g_rpm : 0.0f, run_ ? g_fuelDeliv : 0.0f,
                               (run_ && jakeEngaged()) ? g_jakeAmt : 0.0f); }
    g_clutchHeat = fmaxf(0.0f, g_clutchHeat - dt * 0.018f);   // clutch cools slow

    // ---- air-brake system ----
    if (running) {
        if (g_airPsi >= AIR_CUTOUT) { if (g_compLoaded) { g_compLoaded = false; g_govUnload = true;
            g_airPsi -= 1.5f; } }                        // air-dryer purge blows off a little
        else if (g_airPsi <= AIR_CUTIN) g_compLoaded = true;
        if (g_compLoaded) { float rn = clampf((g_rpm-IDLE_RPM)/(MAX_RPM-IDLE_RPM),0,1);
            g_airPsi += (COMP_IDLE + rn*rn*(COMP_REV-COMP_IDLE)) * dt; }   // rn^2: gotta hold revs
    }
    g_airPsi -= 0.010f * dt;                             // real trucks always leak a little
    g_airPsi = clampf(g_airPsi, 0, AIR_MAX);
    // secondary tank charges off the primary through a check valve (never back)
    if (g_airPsi > g_airSec) g_airSec += min(g_airPsi - g_airSec, 9.0f * dt);
    g_airSec -= 0.008f * dt; g_airSec = clampf(g_airSec, 0, AIR_MAX);
    // ---- THE DRIVER GETS ON THE PEDAL WHEN THE JAKE RUNS OUT ---------------
    //
    // Once the box stops rowing down at JAKE_FLOOR_GEAR the jake alone cannot
    // bring the truck to a stop - and it should not. Below JAKE_BRAKE_MPH a
    // driver on a grade is already covering the pedal, so feather the service
    // brakes in underneath the jake.
    //
    // This is applied BEFORE the air relay below on purpose: it then goes
    // through the real system - pressure demand clamped to tank pressure, the
    // 9/s apply lag, air consumed out of the tank, and the release hiss when
    // it comes off. It is a brake application, not a fudge on the drag term,
    // so it sounds like one.
    // Not while PARKED: the spring brakes hold it, and a service application
    // here used to sit on until you pulled away - then hiss straight after the
    // park-brake release sound (Mat: 'brake release sound after the park brake').
    if (g_jakeAmt > 0.3f && g_throttle < 0.1f && g_state == ES_RUN && !g_parkBrake) {
        float need = clampf((JAKE_BRAKE_MPH - g_speed) / JAKE_BRAKE_MPH, 0, 1);
        float want = need * JAKE_BRAKE_MAX * g_jakeAmt;
        // AND BRING IT ALL THE WAY TO A STOP.
        //
        // MEASURED on the bench: the truck parked itself at 330 rpm / 2.3 mph
        // and sat there indefinitely. The deadlock is a three-way standoff -
        // CLUTCH_RELEASE_MPH is 2.2, so at 2.3 mph the clutch never unlocks;
        // held coupled, the engine is dragged to LUG_MIN_RPM (330); and at
        // 330 rpm jake torque is near zero because it scales with we/MAX_RPM.
        // Nothing left could slow it the final 0.1 mph.
        //
        // The brake has to push past that threshold rather than taper into an
        // equilibrium above it, so it firms up for the last few mph.
        //
        // Blended over 3 mph rather than switched at 3 mph: a hard
        // `if (speed < 3) want = 0.75` stepped the pedal 0.36 -> 0.75 in one
        // frame, and while the air relay's 9/s apply lag would smear that, a
        // step into a filter is still a step - it is the driver's foot that
        // should move smoothly, not the hydraulics covering for it.
        float firm = clampf((3.0f - g_speed) / 3.0f, 0, 1);
        want = fmaxf(want, firm * 0.80f);
        // Ramp rather than stamp - the pedal goes down over ~0.4 s. Without
        // this the whole thing arrives in one frame and the relay's own lag is
        // the only thing softening a step input.
        g_jakePedal += (want - g_jakePedal) * min(1.0f, dt / 0.40f);
        if (g_jakePedal > brake01) brake01 = g_jakePedal;
    } else {
        g_jakePedal -= dt / 0.55f;                   // off the pedal, slower
        if (g_jakePedal < 0) g_jakePedal = 0;
        if (g_jakePedal > brake01) brake01 = g_jakePedal;
    }

    float demand = clampf(brake01 * BRAKE_MAX_PSI, 0, g_airPsi);
    float prevApp = g_brakeApplied;
    float relay = (demand > g_brakeApplied) ? 9.0f : 14.0f;
    g_brakeApplied += (demand - g_brakeApplied) * min(1.0f, dt * relay);
    float dApp = g_brakeApplied - prevApp;
    if (dApp > 0) g_airPsi -= dApp * BRAKE_AIR_COST;
    // Peak only while the pedal is actually ON. Tracking it unconditionally made
    // ONE release fire the hiss ~20 times: the hiss reset the peak to 0, but the
    // pressure was still bleeding down, so the next tick's leftover counted as a
    // new peak and fired again (logged: 0.80, 0.75, 0.72 ... 0.12 with brk 0.00).
    // That machine-gun of pssh was the "air brakes doing weird things".
    // Releasing the PARK brake has its own sound (pbrakeoff); a service
    // application still pending at that moment must not hiss on top of it.
    { static bool s_prevPark = false; if (s_prevPark && !g_parkBrake) s_peakApplied = 0; s_prevPark = g_parkBrake; }
    if (brake01 >= 0.05f && g_brakeApplied > s_peakApplied) s_peakApplied = g_brakeApplied;
    if (brake01 < 0.05f && s_peakApplied > 4) { g_hissEvent = clampf(s_peakApplied/BRAKE_MAX_PSI,0.12f,1); s_peakApplied = 0; }
    g_lowAir = g_airPsi < LOW_AIR;
    // parkGear is NEUTRAL, not the park brake. Using it meant selecting
    // neutral asserted full spring brakes and lit the PARK lamp with 120 psi
    // in the tanks, while actually setting the park brake in gear did
    // nothing at all. g_parkBrake is the real control.
    g_springOn = (g_airPsi < SPRING_REL) || g_parkBrake;
    float effBrake = g_springOn ? 1.0f : (g_brakeApplied / BRAKE_MAX_PSI);
    if (effBrake > 1) effBrake = 1;

    // ---- fuel system: the tank drains by what the injectors SPRAY ----
    // It used to burn by horsepower (BSFC), so bigger injectors only drank more
    // when they made more power - at idle a big-plunger engine sipped like stock.
    // Now it is the fuel actually injected (EngineCore fuelKgS, 3.18 kg/gal):
    // 59s and big plungers pour 2.5x / 5x the fuel on the same throttle, rich
    // or not (Mat: "with the big injectors the fuel goes a lot faster").
    if (running) {
        float gph = max(0.6f, g_lastCore.fuelKgS * 3600.0f / 3.18f);   // small floor: never a free idle
        // Oversized injectors can't meter a small shot (that's the lope) - they
        // waste fuel at idle and light load even when the governor trims them:
        // +0.7 gal/h on 59s, +1.8 on big plungers.
        gph += 0.45f * (PT_FFLOW[parts.fuel > 2 ? 2 : parts.fuel] - 1.0f);
        g_gph = gph;
        g_fuelGal -= gph * dt / 3600.0f;                 // REAL TIME, straight from the sim - no burn-rate multiplier (Mat)
        if (g_fuelGal <= 0) { g_fuelGal = 0; stop(); }   // ran the tanks dry: REFUEL to restart
        // The BATTERY cut-off no longer empties the tank (the fuel gauge is the
        // real tank now - Mat: "I want a real fuel gauge and you have to refuel
        // it"), so a finished starve stops the engine itself.
        if (g_starve >= 1.0f) stop();
    }

    // ---- thermal + damage (thermostat @160F, fan clutch, block cooldown) ----
    if (running) {
        float rn = clampf((g_rpm-IDLE_RPM)/(MAX_RPM-IDLE_RPM),0,1);
        float heatIn = 0.45f + rn*0.55f + g_load*2.0f;
        float airflow = 0.55f + (g_speed/42.0f) + (g_fanOn?1.3f:0);
        if (g_coolantF < 160.0f) airflow *= 0.35f;       // thermostat closed: fast warm-up
        g_coolantF += (heatIn - (g_coolantF-AMBIENT_F)*0.0078f*airflow) * dt;
        if (g_coolantF >= FAN_ON) g_fanOn = true; else if (g_coolantF <= FAN_OFF) g_fanOn = false;
        g_oilF += ((g_coolantF+12) - g_oilF) * min(1.0f, dt*0.022f);
        if (g_coolantF > DAMAGE_F) g_health -= (g_coolantF-DAMAGE_F)*0.04f*dt;  // overheat warps
        g_health -= dt * (100.0f / (1000.0f * 3600.0f));         // wear: ~1000 h to a rebuild
        // (EGT no longer drains health gradually - piston melt above is catastrophic instead)
        if (g_health <= 8.0f && g_state == ES_RUN) stop();       // she's done - spun a bearing
        g_health = clampf(g_health, 0, 100);
    } else { g_coolantF += (AMBIENT_F-g_coolantF)*min(1.0f,dt*0.02f); g_fanOn=false; }

    // EGT / pyro: rides fueling hard, jake sucks it right back down
    { float fm = clampf(fuelMap[(int)clampf((g_rpm-600.0f)/1500.0f*7.0f,0,7)],0.7f,1.3f);
      float egtTgt = running ? g_lastCore.egtF : AMBIENT_F;
      // The hardcoded 240 F override is GONE. It existed because nothing told
      // the core about the jake, so egtF would have read whatever a dead fuel
      // cut produced and the gauge had to be faked. Now the core models the
      // jake properly - pumped mass flow at COMPRESSION temperature, ~700 K
      // (800 F) - so egtF is already correct during a brake and forcing 240
      // would fight it by 560 F.
      //
      // 800 F is also the right answer physically: a jake cools a pyro from a
      // pulling 1200-1400 F, which is why drivers use it on a grade, but it is
      // not 240 F - the engine is still compressing air 17:1 twelve hundred
      // times a minute.
      float tau = (egtTgt > g_pyroF) ? 0.55f : 0.16f;    // heats fast, cools slower
      g_pyroF += (egtTgt - g_pyroF) * min(1.0f, dt*tau); }

    // transmission temp: load + clutch abuse heat it, road airflow cools it
    { float slipHeat = (!s_clutchLocked && g_speed > 0.5f) ? 0.9f : 0;
      float heat = running ? (0.14f + g_load*0.6f + slipHeat) : 0;
      float cool = (g_transF - AMBIENT_F) * (0.0035f + g_speed*0.00012f);
      g_transF += (heat - cool) * dt; g_transF = clampf(g_transF, AMBIENT_F, 320); }

    // electrics: 14V charging, sagging on crank, key-off battery
    { float vTgt = g_haveVolts ? g_extVolts
                 : ((g_state==ES_CRANK) ? 10.2f : (running ? 14.1f - g_load*0.5f : 12.5f));
      g_volts += (vTgt - g_volts) * min(1.0f, dt*4.0f); }

    // oil pressure: rpm-driven, thick cold oil pushes it high
    { float rn = clampf((g_rpm-500.0f)/(MAX_RPM-500.0f),0,1);
      float cold = 1.0f + clampf((180.0f-g_oilF), 0, 140) * 0.0035f;
      float pTgt = running ? (18.0f + rn*44.0f) * cold * (0.30f + 0.70f * g_health / 100.0f)
                          : ((g_state==ES_CRANK)?12.0f:0.0f);   // hurt bearings bleed oil pressure
      g_oilPsi += (min(pTgt, 92.0f) - g_oilPsi) * min(1.0f, dt*3.0f); }

    g_derate = 1;                                        // NO derate - full power to the edge
    // Aluminum piston crowns melt past ~1325F EGT. No nanny pulling power back -
    // you get every horse right up to the line, then she LETS GO: molten crown,
    // engine dead instantly, truck coasts on momentum until you park it.
    if (g_state == ES_RUN) {
        if (g_pyroF > g_meltF) g_pistonMelt += (g_pyroF - g_meltF) * 0.0007f * dt;   // built pistons: survive a hard pull
        else                   g_pistonMelt -= 0.03f * dt;
        g_pistonMelt = clampf(g_pistonMelt, 0, 1);
        if (g_pistonMelt >= 1.0f) { g_deathCause = 0; if (unlocks.pistons < 2) { unlocks.pistons++; saveParts(); } meltdown(); }   // thermal - melted crown
        // over-advanced timing on a full cylinder spikes pressure -> cracks/throws a rod
        float pcp = g_lastCore.pcpBar;
        if (pcp > g_rodBar) g_mechDamage += (pcp - g_rodBar) * 0.006f * dt;
        else              g_mechDamage -= 0.05f * dt;
        g_mechDamage = clampf(g_mechDamage, 0, 1);
        if (g_mechDamage >= 1.0f) { g_deathCause = 1; if (unlocks.rods < 2) { unlocks.rods++; saveParts(); } meltdown(); }   // mechanical - threw a rod
    }
    g_warn = 0;
    if (g_lowAir) g_warn|=1; if (g_springOn) g_warn|=2; if (g_coolantF>OVERHEAT) g_warn|=4;
    if (g_fanOn) g_warn|=8;
    if (g_pistonMelt > 0.02f || g_pyroF > 1250.0f) g_warn|=16;   // MELT DANGER lamp
    if (g_health<60) g_warn|=32;
    if (fuelFrac() < 0.125f) g_warn|=64;                 // low fuel lamp

    float rpm = g_rpm;
    switch (g_state) {
        case ES_OFF:  rpm += (0-rpm)*min(1.0f,dt*3.0f); g_load=0;
                      if (g_speed > 0.1f) {                  // killed while rolling: COAST down
                          // Use the SAME mass as the driving branch. This used to coast on
                          // BASE_MASS (37t) while driving ran at 8t bobtail, so killing the
                          // engine while rolling made the truck suddenly feel 4x heavier.
                          float cm = g_loadMass;
                          float v = g_speed*MPH_MS - (roadDrag(g_speed*MPH_MS, cm, effBrake)/cm)*dt;
                          g_speed = (v > 0 ? v : 0) / MPH_MS;
                          if (g_speed < 0.5f) { g_speed = 0; g_gear = 1; }
                      } break;
        // ES_CRANK. The engine used to ramp straight toward IDLE_RPM from the
        // first frame, so the ladders faded up and you heard the engine running
        // while start.wav was still cranking. A real engine turns at starter
        // speed (~150-250 rpm, below the ladder's bottom rung) and only catches
        // at the END of the crank. Hold it at cranking speed for most of
        // crankDur, then let it pull up over the last stretch - which is where
        // the catch is in the recording.
        case ES_CRANK: {
            crankT += dt; g_load = 0;
            float cf = (crankDur > 0.01f) ? (crankT / crankDur) : 1.0f;
            if (cf < CRANK_CATCH) {
                // spinning on the starter: well under the lowest rung, so the
                // sampled ladders stay essentially silent and start.wav is all
                // you hear.
                rpm += (CRANK_RPM - rpm) * min(1.0f, dt * 6.0f);
            } else {
                rpm += (IDLE_RPM - rpm) * min(1.0f, dt * 2.2f);
            }
        }
                       if (crankT >= crankDur) { g_state = ES_RUN; rpm = idleRpmNow(); } break;
        case ES_RUN: {
            if (!g_reverse && g_revSpeed > 0) g_revSpeed -= g_revSpeed*min(1.0f,dt*2.5f);
            // A shift can only be in progress while actually IN DRIVE. `shifting` is
            // cleared in exactly one place - the float-shift branch below - and that
            // branch only runs when g_drive is set and g_reverse is not. So selecting
            // R or N mid-shift used to leave `shifting` true FOREVER, and the fuel
            // governor cuts fuel to zero whenever it is set:
            //     else if (shifting) fuelTgt = 0;   // cut through the shift
            // The result is an engine that will not rev and a truck that will not move,
            // which is what "reverse doesn't work" was. Manual mode hits it far more
            // often because every flick of the shift lever starts a shift.
            if (g_reverse || !g_drive) { shifting = false; shiftTo = 0; }
            // RACE MASS OVERRIDES THE TRAILER LOAD. A drag truck has no
            // trailer and a stripped chassis; carrying 32 t into a race run
            // is why it bogged whatever gear it launched in.
            float rm = raceMassKg();
            float mass = (rm > 0.0f) ? rm : (g_loadMass / loadAccel);
            if (g_reverse) {                                     // gentle crawl
                g_speed -= g_speed*min(1.0f,dt*2.5f); if (g_speed<1) g_gear=1;
                float tgt = g_throttle*REV_TOP_MPH;
                float rate = (tgt>g_revSpeed)?REV_BUILD:2.2f;
                g_revSpeed += (tgt-g_revSpeed)*min(1.0f,dt*rate);
                g_revSpeed -= effBrake*g_revSpeed*min(1.0f,dt*3.0f);
                g_revSpeed = clampf(g_revSpeed,0,REV_TOP_MPH);
                float sr = IDLE_RPM + (g_revSpeed/REV_TOP_MPH)*(MAX_RPM-IDLE_RPM)*0.55f;
                float lr = IDLE_RPM + g_throttle*(MAX_RPM-IDLE_RPM)*0.45f;
                rpm += (max(sr,lr)-rpm)*min(1.0f,dt*2.5f); g_load = g_throttle*0.5f;
            } else if (!g_drive) {
                // FREE-REV ON REAL TORQUE, as the sim does:
                //   rpm += (tqRaw + chopMiss - (90 + 0.075*rpm)) / 3.5 * 9.549 * dt
                // The old linear slew ramped at a fixed rate, so blipping it in neutral
                // felt identical no matter how you'd tuned it. Now fuel and timing
                // actually change how it picks up, and the misfire chop shakes the crank.
                float tqRaw = g_lastCore.torqueNm;
                // REV FEEL, from the 2026-10-30 research (Mat: "it's not heavy").
                // MEASURED before: 700->2000 in 0.61 s, about twice a real C15.
                //  * BOOST FUEL LIMIT: with no boost in neutral Cat's FRC limit
                //    holds the fuel back, so the engine makes ~45% of what the
                //    fuel could, rising to full as boost comes in. That is the
                //    wind-up of a heavy diesel instead of a petrol snap.
                //  * FRICTION grows with rpm^2 (Volvo 16 L friction-power scaled to
                //    15.2 L): 125 Nm at idle, 234 at 1500, 342 at 2100 - steep at
                //    the top, gentle onto idle, no undershoot.
                //  * J = 4.0 kg*m^2 (crank + 35.8 kg flywheel + 15.5 in clutch).
                //  * JAKE works in neutral too (Jacobs 340 sheet, SAE J1621:
                //    ~1360 Nm at 2100, tapering out below 1000 rpm).
                float frc = 0.45f + 0.55f * clampf(g_boostPsi / 18.0f, 0, 1);
                if (tqRaw > 0) tqRaw *= frc;
                float acc = 77.0f + 0.050f*rpm + 3.63e-5f*rpm*rpm;
                float tJake = 0.0f;
                if (jakeEngaged() && g_throttle < 0.05f && rpm > 1000.0f) {
                    // hp from the Jacobs table -> Nm, linear in rpm between 1100 and 2100
                    float hp = 117.0f + (rpm - 1100.0f) * (400.0f - 117.0f) / 1000.0f;
                    tJake = fmaxf(0.0f, hp) * 7121.0f / rpm * clampf((rpm - 1000.0f) / 100.0f, 0, 1) * g_jakeAmt;
                }
                rpm += (tqRaw + g_chopTq - acc - tJake) / J_ENGINE * 9.549f * dt;
                // The old 5*(rpm-2130) restoring rate was 350 rpm/s at the
                // clamp against 5700-12000 rpm/s of free-rev acceleration, so
                // it did nothing and every blip pinned the bare 2200 clamp.
                // 60x is a real governor: it holds 2100-2140 and flutters.
                if (rpm > 2100.0f) rpm -= (rpm - 2100.0f) * 60.0f * dt;
                rpm = clampf(rpm, 120.0f, 2200.0f);               // sim bounds, not a hard 2100
                g_speed -= g_speed*min(1.0f,dt*0.25f); if (g_speed<1) g_gear=1; g_load=g_throttle;
            } else if (shifting) {
                // FLOAT SHIFT, matched to the sim: stay in the OLD gear, let the engine
                // drift toward the NEW gear's synchronous speed, and only drop it in when
                // the revs actually line up (or we run out of patience). The old code
                // snapped the gear across instantly and then slurred the revs after it,
                // which is why it never sounded like someone shifting a 13-speed.
                float v = g_speed*MPH_MS - (roadDrag(g_speed*MPH_MS,mass,effBrake)/mass)*dt;
                if (v<0) v=0; g_speed = v/MPH_MS;
                int tgt = shiftTo ? shiftTo : g_gear;
                // EXACTLY the browser: matchRpm = speed*RPM_MPH[tgt-1], UNCLAMPED.
                // Flooring this at IDLE_RPM meant that below ~1.8 mph the shift
                // completed against a fake target - |rpm-match|<40 was satisfied
                // by the clamp itself, not by the revs actually lining up.
                float matchRpm = g_speed*rpmPerMph[tgt-1];
                // ...but the REVS never aim below idle. Out of gear, the engine
                // idles. MEASURED braking to a stop: the box was mid-downshift
                // at 1.6 mph, the rev target was speed x ratio = ~200 rpm, and
                // the slew dragged the crank to 227 then 0 - the 'dies when you
                // come to a stop'. Sync still tests the true matchRpm above;
                // below idle it never syncs and shiftTimer completes the change.
                float revTgt = fmaxf(matchRpm, idleRpmNow());
                // THE REV SLEW IS DIRECTIONAL.
                //
                // dt*3.4 is a ~0.29 s smooth glide, and that is right for an
                // UPSHIFT: you are off the fuel, coasting down toward a lower
                // synchronous speed, and the engine eases across.
                //
                // On a DOWNSHIFT it is wrong, and it is what kept making the
                // brake sound like an automatic however the shift points were
                // tuned. A driver does not ease the revs up - he drops the
                // clutch and the DRIVELINE SLAMS the engine up to the new
                // gear's speed. Measured in the log: 1341 -> 1535 took a whole
                // second, when it should be near-instant once the gear is in.
                //
                // So: glide down, SNAP up. dt*14 is ~70 ms, which reads as the
                // engine being thrown up rather than drifting there - and it is
                // what puts the revs up where the Jake bites and the turbo can
                // spool, instead of arriving there a second late.
                bool down = (tgt < g_gear);

                // THE REV SLEW IS DIRECTIONAL, AND A DOWNSHIFT HAS A NEUTRAL PHASE.
                //
                // UPSHIFT: off the fuel, the engine coasts DOWN to a lower
                // synchronous speed on its own. dt*3.4 (~0.29 s) is that glide.
                //
                // DOWNSHIFT: the driver is out of gear blipping the throttle to
                // drive the revs UP. That is not a driveline snap - the clutch is
                // out and the engine is accelerating its own flywheel, which a big
                // diesel does at roughly BLIP_RPM_PER_S. A fixed dt*14 was wrong
                // twice over: too fast for a real blip, and fast enough that the
                // match test below fired on the first frame.
                if (down) {
                    // Rate-limited, not exponential: a blip is the driver holding
                    // the pedal until the needle gets there, so the revs climb at
                    // a steady rpm/s instead of easing in asymptotically.
                    float step = BLIP_RPM_PER_S * dt;
                    float gap  = revTgt - rpm;
                    rpm += (gap >  step) ?  step : ((gap < -step) ? -step : gap);
                } else {
                    // Race: a drag shift is slammed and the driveline pulls
                    // the engine down to the next gear; street waits for the
                    // revs to fall on their own.
                    rpm += (revTgt-rpm)*min(1.0f, dt*(g_raceMode ? 9.0f : 3.4f));
                }
                g_load=0;
                shiftTimer  -= dt;
                if (neutralTimer > 0.0f) neutralTimer -= dt;

                // ENGAGE ONLY WHEN THE DRIVER WOULD ACTUALLY HAVE THE LEVER IN.
                //
                // The rpm-match test ALONE was the bug. With the revs converging
                // in ~70 ms it was true on the first or second frame, so the gear
                // dropped straight in and the 0.76-1.36 s shiftTimer was a ceiling
                // that never got reached. One 70 ms rev ramp with no gap is
                // precisely what an automatic sounds like.
                //
                // Now the neutral phase must EXPIRE too: the revs have to line up
                // AND the driver has to have spent a believable time out of gear.
                // shiftTimer stays as the patience backstop for a shift that never
                // syncs (a missed gear, revs chased by a changing road speed).
                bool synced  = fabsf(rpm-matchRpm) < SHIFT_MATCH_RPM;
                bool settled = (neutralTimer <= 0.0f);
                if ((synced && settled) || shiftTimer<=0) {
g_gear = tgt; g_manReq = g_gear; shiftTo = 0; shifting = false;
                    // Block the frame-scope auto-shift test for this frame. It
                    // runs AFTER this line, so with `shifting` already cleared
                    // and g_gear already advanced it would re-evaluate against
                    // the brand new gear before the truck has gained any speed -
                    // and fire a second shift immediately. MEASURED: g4 engaged
                    // at 13.0 mph then shifted again at 14.2, where g4's real
                    // upshift point is 14.6. That is the 'random in-between
                    // shift'.
                    s_justEngaged = true;
                }
            } else {                                             // engine-first coupled drive
                // SUB-STEPPED. The clutch spring is integrated with explicit
                // Euler and its loop gain is 1 - grab*stiff/J_ENGINE*dt*RAD,
                // which crosses -1 at dt = 18.8 ms. Any frame longer than that
                // - an SD read, a WiFi serve, an NVS write - makes it ring, and
                // past ~50 ms it becomes a hard limit cycle slamming between the
                // 330 rpm floor and 1400+ every other frame. That is the launch
                // warble. Capping the internal step at 5 ms keeps the loop gain
                // near +0.5 regardless of how long the caller's frame was, and
                // changes nothing about the model itself.
                {
                    const float MAXSTEP = 0.005f;
                    int nsub = (int)(dt / MAXSTEP) + 1;
                    if (nsub > 32) nsub = 32;          // bounds the worst case
                    float sdt = dt / (float)nsub;
                    for (int s = 0; s < nsub; s++)
                        stepCoupled(sdt, g_throttle, g_jakeAmt, effBrake, mass);
                }
                rpm = g_rpm;
                float wheelRpm = g_speed*rpmPerMph[g_gear-1];
                // Start a shift by naming the target gear - do NOT change g_gear here.
                // The float-shift branch above engages it once the revs match.
                if (g_manual) {
                    if (g_manReq != g_gear) {                       // driver-commanded shift
                        shiftTo = g_manReq; shifting = true; shiftTimer = SHIFT_TIME; g_fuelHold = g_fuelDeliv;
                        // Manual request gets the same neutral phase, by direction.
                        neutralTimer = LEVER_TIME + ((g_manReq < g_gear)
                                       ? NEUTRAL_MIN : NEUTRAL_MIN * 0.55f);
                    }
                }
                // The auto-shift test used to live HERE, inside the coupled-drive
                // branch only. The browser runs it at FRAME scope, after all the
                // drive branches:
                //     if (shiftT<=0 && trans.mode==='D') { if (wr>1950...) }
                // Inside the branch it could not fire while the clutch was
                // slipping or during the settle after a shift, so a gear change
                // was delayed until the coupled branch happened to run with the
                // revs already past the point - which is why gears appeared with
                // no float shift and then held for seconds. Moved below.
                g_load = g_throttle;
            }
            if (shifting) g_shiftCut = 0.55f; else g_shiftCut += (1-g_shiftCut)*min(1.0f,dt*8.0f);
            break;
        }
        // ES_STOP. The rpm ramp here USED to be audible: the ladders kept
        // playing all the way down as rpm fell, so you heard the sim taper off
        // UNDERNEATH stop.wav - two different wind-downs at once, and the
        // sampled one is the real recording of this engine dying. Now the rpm
        // still falls (gauges, the ESC and the tick scheduler all read it) but
        // g_shutMute ramps the LADDERS out fast, so stop.wav plays into silence.
        // 0.22 s is quick enough not to read as a fade and slow enough not to
        // be a step.
        case ES_STOP: rpm += (0-rpm)*min(1.0f,dt*1.6f); g_load=0; g_shiftCut=1;
                      g_shutMute -= dt / 0.22f; if (g_shutMute < 0) g_shutMute = 0;
                      if (rpm<100){ g_state=ES_OFF; rpm=0; g_speed=0; g_gear=1; shifting=false; shiftTo=0; } break;
    }
    // ---- AUTO SHIFT, at frame scope (browser stepEngine: `if(shiftT<=0 &&
    // trans.mode==='D')` sits after every drive branch, not inside one) ----
    // s_justEngaged: a shift that completed earlier in THIS frame clears
    // `shifting` on its way out, so without this the test below would see a
    // settled box one line after the gear changed and shift straight again.
    if (s_justEngaged) s_justEngaged = false;
    else if (g_state == ES_RUN && g_drive && !g_reverse && !shifting && !g_manual) {
        // RACE MODE SHIFTS OFF REAL ENGINE RPM.
        // wr is wheel-referenced - speed x the gear's rpm-per-mph. On the
        // street that is the right reference: the clutch is locked, so wr IS
        // engine rpm, and using it means the box ignores a momentary flare.
        //
        // On a drag launch the clutch is deliberately slipping (see the
        // release-time feed-out), so the two diverge hard. MEASURED: the skip
        // fired at 5.1 mph where wr had just crossed the point but the engine
        // was at 1503, and every later upshift landed at ~1530 instead of the
        // 2080 redline point - because wr lagged the engine the whole run.
        // A drag shift is made on the TACHO, so use rpm.
        float wr = g_raceMode ? rpm : (g_speed * rpmPerMph[g_gear-1]);
        // NO RACE SHIFT WHILE THE CLUTCH IS STILL SLIPPING. Engine rpm only
        // means something once wheel speed has caught it. MEASURED: the skip
        // fired at "2100 rpm" in 6th at 10.3 mph - the engine was on the
        // limiter on slip, wheel speed was 793 - so 8th landed at 519 rpm
        // with zero boost and spent four seconds climbing out. Wait until
        // the two are within 120 rpm (lock-up is at 55).
        bool raceLocked = !g_raceMode
                       || fabsf(rpm - g_speed * rpmPerMph[g_gear-1]) < 120.0f;

        // DOWNSHIFT POINT DEPENDS ON WHETHER THE DRIVER IS BRAKING.
        //
        // Coasting or pulling, DOWNSHIFT (1100) is right: let the revs come down
        // and change when the engine runs out of pull.
        //
        // On the JAKE it is wrong, and it is what made the brake sound like an
        // automatic. MEASURED on a jake run: every gear was held until the revs
        // died to ~1100, then dropped, and the revs came back to only ~1400. An
        // automatic waits for the engine to fall out of its range; a driver does
        // the opposite - he grabs the next one down EARLY, while he still has
        // revs, because an engine brake only works up in the rev range. Drop at
        // 1550 and the revs land near 2000 after the change, which is where a
        // Jake actually holds a truck back.
        bool jakeDown = (g_jakeAmt > 0.3f && g_throttle < 0.1f);
        float dnPoint = jakeDown ? JAKE_DOWNSHIFT : DOWNSHIFT;
        dnPoint += g_dnJitter;      // this driver's offset for THIS change

        // ON A CLIMB A DRIVER HOLDS THE GEAR LOWER: he lets it rev out further
        // before going up and grabs the next one down sooner, instead of the
        // box hunting on the hill. Up to +100 / +250 rpm by a ~13% grade.
        // Stays inside the hunt rules above: (1100+35+250)*1.37 = 1897 < the
        // 1930 upshift floor, and 2020+100 is clamped under the limiter chop.
        float upPoint = g_upPoint;
        bool  holdUp  = false;     // downhill off the throttle: never upshift
        if (g_realGradeOn && !g_raceMode) {
            float climb = clampf((g_grade - 0.03f) / 0.10f, 0, 1);
            upPoint = fminf(upPoint + 100.0f * climb, 2050.0f);
            if (!jakeDown) dnPoint += 250.0f * climb;
            holdUp = (g_grade < -0.03f && g_throttle < 0.1f);
        }

        // THE JAKE STOPS ROWING AT THE FLOOR GEAR. See EngineSim.h: below
        // 4th the gears span 1-3 mph each while the jake brakes hardest, so
        // continuing to shift just produces a burst of changes a second
        // apart. A driver holds the gear and uses the pedal instead, which is
        // what jakeAssist() below does.
        int dnFloor = jakeDown ? JAKE_FLOOR_GEAR : 1;

        // Straight sequential - the ratios are even, so there is no splitter
        // half to skip.
        // Each change gets its OWN point and duration - see EngineSim.h. The
        // re-roll happens after the shift is committed so the next one differs.
        // RACE MODE HOLDS THE TOP GEAR YOU PICKED. Without this the box runs
        // to 10th and the run ends in overdrive, off the power, which is not
        // what you tuned for.
        int topGear = g_raceMode ? g_raceTop : N_GEARS;
        // THE SKIP. On the first upshift after a race launch, jump straight to
        // the nominated gear instead of taking the next one. One skip per
        // launch - g_raceSkipDone - because skipping on every upshift would
        // just be a shorter gearbox, not a drag strategy.
        // THE SKIP NEEDS A SPEED GATE. This fired the moment the LAUNCH gear
        // reached its upshift point, which on 2nd is about 7 mph - and 6th at
        // 7 mph wants 554 rpm, below the 680 idle. The engine physically
        // cannot pull it, so it bogged and the downshift logic rescued it by
        // rowing back down. The skip was being COMMANDED correctly and was
        // useless as a strategy, which is why it felt like it had no power.
        //
        // So wait until the truck is actually fast enough for the target gear
        // to sit in the power band. Require the skip gear to land at or above
        // 1300 rpm - comfortably over idle and into where a C15 makes torque.
        // Until then the box upshifts normally, and the skip happens later.
        float skipRpm = g_speed * rpmPerMph[g_raceSkip ? g_raceSkip - 1 : 0];
        if (g_raceMode && g_raceSkip && !g_raceSkipDone && !shifting
        // 1100, not 1300. At 1300 the skip could not fire until 5th (18 mph),
        // so the box walked 1-2-3-4-5 first and the skip saved nothing - which
        // is the 'rowed through all the gears' complaint. A C15 pulls happily
        // from 1100, so the skip can come much earlier: out of 3rd at ~14 mph
        // it lands 6th at 1100 and goes straight to work.
        // 850, AND ONLY FROM THE LAUNCH GEAR.
        //
        // MEASURED on a 6000 kg race run: the box upshifted at 5.1, 7.1, 9.7,
        // 13.2 and 17.4 mph, and 6th only cleared an 1100 rpm gate at 14.3
        // mph. By then the gearbox had ALREADY walked to 5th on its own, so
        // the skip never fired - it was racing the normal upshift logic and
        // losing. A light truck accelerates through the gears faster than a
        // conservative gate can open.
        //
        // So the skip must fire EARLY, out of the launch gear, which means
        // accepting a lower landing rpm. 850 is below the 1100 I had but a
        // 2300 hp engine pulls from there without complaint - that is the
        // whole point of the power. From 1st at 5.2 mph that reaches 3rd
        // (1040 rpm) comfortably and 4th (758) marginally.
        //
        // Restricted to g_gear == g_raceLaunch so it is genuinely one jump off
        // the line, not an opportunistic skip half way up the box.
        // GATE REMOVED. The 850 rpm landing requirement was a protection: it
        // held the skip back until the target gear would sit in the power
        // band. Per the user, nothing holds a shift back any more - if the
        // skip is selected it takes it, and a bog is the tune's problem, not
        // the gearbox's. skipRpm is still computed for the trace.
        // The one condition that stays is the CLUTCH, and it is not a bog
        // protection - it is what makes the reference meaningful. The launch
        // stages on the limiter with the pedal floored, so rpm is ALREADY
        // past the shift point on the first frame in Drive; without this the
        // skip would fire at 0 mph with the clutch still in. Wait until the
        // pedal is essentially home and the engine is driving the truck.
            && g_gear == g_raceLaunch && g_clutchPedal < 0.15f && raceLocked
            && wr > g_upPoint) {
            shiftTo = g_raceSkip; shifting = true;
            // A skip spans several ratios, so the revs have further to fall
            // and the lever has further to travel. Give it the long end of
            // the researched range rather than a normal single-gear change.
            shiftTimer = SHIFT_TIME_MAX;
            neutralTimer = NEUTRAL_MAX + LEVER_TIME;
            // SCALE QUARTER MILE. This is a 1/14 truck: the real strip is
            // ~94 ft and the run is 8-10 s of driving, so the whole virtual
            // quarter has to fit in that. A 1.25 s neutral phase per change
            // was most of a 15 s run. Drag shift: ~0.4 s out of gear.
            if (g_raceMode) { neutralTimer = 0.40f; shiftTimer = 0.80f; }
            g_fuelHold = g_fuelDeliv;
            g_raceSkipDone = true;
            // Trace AT THE DECISION. The 1 Hz heartbeat in main.cpp samples
            // rpm up to a second after the change was commanded, by which
            // time the revs have already dropped - every 'it shifts at 1122'
            // reading was that lag, not the shift point.
            Serial.printf("[shift] SKIP g%d->%d at %.0f rpm (pt %.0f) %.1f mph\n",
                          (int)g_gear, (int)shiftTo, wr, g_upPoint, g_speed);
            rollDriver();
            return;
        }
        // Same clutch condition on the normal upshift - see the skip above.
        if      (wr > upPoint && !holdUp && g_gear < topGear
                 && (!g_raceMode || g_clutchPedal < 0.15f) && raceLocked) {
            // RACE MODE TAKES THE LADDER THE UI SET - launch, skip, top - and
            // nothing in between. Asked for three times; the first attempt was
            // pulled because 3rd -> 10th at 10 mph is 230 rpm and it fell on
            // its face. That was the LADDER being wrong, not the idea: the UI
            // now shows the landing rpm of every rung so a dead one is visible
            // before the run, and the defaults (6 > 8 > 10) land each gear at
            // ~1120 rpm. Street stays a normal sequential box.
            {
                int next = g_gear + 1;
                if (g_raceMode) {
                    // the next rung above the current gear: skip, skip2, then top
                    next = g_raceTop;
                    if (g_raceSkip2 && g_gear < g_raceSkip2) next = g_raceSkip2;
                    if (g_raceSkip  && g_gear < g_raceSkip)  next = g_raceSkip;
                    if (next <= g_gear) next = g_gear + 1;    // never backwards
                }
                shiftTo = next;
            }
            shifting = true;
            shiftTimer = SHIFT_TIME_MIN + frand01() * (SHIFT_TIME_MAX - SHIFT_TIME_MIN);
            // An upshift waits for the revs to FALL, which gravity and friction
            // do on their own - no blip to drive, so it is the quicker change.
            // The research puts a downshift at 1.3-1.5x an upshift, and a
            // downshift here comes out at ~1.11 s, so this wants to be ~0.79 s:
            // 1.17 * NEUTRAL_MIN plus the lever. Sized from the RATIO rather
            // than guessed, because the first pass left this at 0.55 and made
            // the upshift 2.09x quicker than the downshift instead of 1.4x.
            neutralTimer = NEUTRAL_MIN * 1.17f + LEVER_TIME;
            // A multi-gear jump has further for both the revs and the lever to
            // travel, so scale the neutral phase by how many ratios it spans -
            // same reasoning as the skip branch, which hardcodes NEUTRAL_MAX.
            // Without this a 3->10 change would get a single-gear duration and
            // snap through instantly.
            {
                int span = shiftTo - g_gear;
                if (span > 1) {
                    neutralTimer = fminf(NEUTRAL_MAX, neutralTimer * (1.0f + 0.25f * (span - 1)))
                                 + LEVER_TIME;
                    shiftTimer   = SHIFT_TIME_MAX;
                }
            }
            if (g_raceMode) { neutralTimer = 0.40f; shiftTimer = 0.80f; }   // slammed, see the skip
            Serial.printf("[shift] UP   g%d->%d at %.0f rpm (pt %.0f) %.1f mph\n",
                          (int)g_gear, (int)shiftTo, wr, g_upPoint, g_speed);
            g_fuelHold = g_fuelDeliv;
            rollDriver();
        }
        // NO DOWNSHIFTS IN RACE MODE. This branch is the last of the three
        // protections and the one that actually ruined the runs: after the
        // skip dropped the engine to 998 rpm it 'rescued' the truck with a
        // 3<2 change, and from then on the box was walking back up through
        // gears it had already taken. On a drag strip you go UP the box and
        // only up. It stays live outside race mode, where coasting and the
        // jake both need it.
        else if (!g_raceMode && wr < dnPoint && g_gear > dnFloor) {
            shiftTo = g_gear-1; shifting = true;
            // A downshift on the brake is a deliberate, brisker move than an
            // upshift under load - a driver is already off the fuel and just
            // needs the lever across.
            shiftTimer = (SHIFT_TIME_MIN * 0.80f)
                       + frand01() * (SHIFT_TIME_MAX - SHIFT_TIME_MIN) * 0.75f;
            // THE NEUTRAL PHASE, SIZED BY THE BLIP.
            // How far must the revs climb to meet the gear below? A full gear is
            // ~500 rpm, a splitter ~250. Time it from the REAL gap at
            // BLIP_RPM_PER_S and add the lever movement on top, so a big drop
            // down the box audibly takes longer than a small one.
            {
                int   gi    = (g_gear - 2 < 0) ? 0 : g_gear - 2;   // target gear index
                float mNext = g_speed * rpmPerMph[gi];
                float blip  = fabsf(mNext - rpm);
                neutralTimer = NEUTRAL_MIN + NEUTRAL_PER_K * (blip / 1000.0f);
                if (neutralTimer > NEUTRAL_MAX) neutralTimer = NEUTRAL_MAX;
                neutralTimer += LEVER_TIME;   // the hands, either side of the blip
            }
            g_fuelHold = g_fuelDeliv;
            rollDriver();
        }
    }

    // ---- NaN guards, ported from the sim (isFinite checks it has and we never had).
    // Integrating rpm from raw torque can produce a NaN, and clampf() will NOT catch it
    // because every comparison against NaN is false. A NaN rpm becomes a NaN playback
    // position and readBuf() then indexes out of bounds -> hard fault -> frozen Teensy.
    if (!isfinite(rpm))          rpm = 650.0f;
    if (!isfinite(g_speed))      g_speed = 0.0f;
    if (!isfinite(g_fuelDeliv))  g_fuelDeliv = 0.12f;
    if (!isfinite(g_chopD))      g_chopD = 0.0f;
    if (!isfinite(g_tqRef))      g_tqRef = 2400.0f;
    // the dyno brake holds the engine on its sweep, and it is working - full load
    if (g_dynoOn && g_state == ES_RUN) { rpm = g_dynoRpm; g_load = 1.0f; }
    g_rpm = rpm;
    // rev limiter (audio chops off it)
    g_limiter = (g_state==ES_RUN && rpm >= MAX_RPM-40 && g_throttle > 0.55f);

    // ---- the grade: the truck's REAL pitch (GY-521) when fitted ----
    if (g_realGradeOn) {
        g_grade += (g_realGrade - g_grade) * min(1.0f, dt * 4.0f);
    }
    // ---- rolling terrain: grade wanders slowly as you drive ----
    else if (g_state == ES_RUN && g_speed > 2.0f) {
        g_gradePhase += g_speed * dt * 0.006f;             // hills pass with distance
        float tgt = (sinf(g_gradePhase) * 0.6f + sinf(g_gradePhase*2.7f + 1.3f) * 0.4f) * gradeMax;
        g_grade += (tgt - g_grade) * min(1.0f, dt * 0.5f);
    } else g_grade += (0 - g_grade) * min(1.0f, dt * 0.3f);

    // ---- turbo: driven by the physical core (exhaust energy spins it for real) ----
    g_tShaft   = g_core.shaftRpm();
    g_boostPsi = g_lastCore.boostPsi;
    g_shaftHz  = g_lastCore.shaftHz;
    // slow-decaying peak, so the gate reference tracks what this turbo can do
    if (g_boostPsi > g_boostPeakSeen) g_boostPeakSeen = g_boostPsi;
    else g_boostPeakSeen -= (g_boostPeakSeen - g_boostPsi) * 0.02f * dt;
    // ---- blow-off: charge while pinned on the wastegate, fire on the lift (hardcore boost-scaled) ----
    // Gate on what the turbo can ACTUALLY reach, not on the slider. Raising
    // boostMax with a stock turbo used to push these thresholds above the
    // achievable ceiling, silently killing the blow-off AND the surge
    // flutter - turn the boost up, lose all the turbo drama.
    float gateRef = fminf(boostMax, g_boostPeakSeen > 8.0f ? g_boostPeakSeen : boostMax);
    bool onGate = (g_state==ES_RUN) && g_boostPsi >= gateRef * 0.82f;
    if (onGate && g_throttle > 0.5f) { g_bHold = fminf(1.4f, g_bHold + dt); g_bPeak = fmaxf(g_bPeak, g_boostPsi); }
    else if (g_throttle < 0.5f)      g_bHold = fmaxf(0.0f, g_bHold - dt * 0.7f);
    if (g_prevThr > 0.4f && g_throttle < 0.12f && g_bHold > 0.65f) {
        float am = clampf((g_bPeak - 18.0f) / 60.0f, 0, 1); am *= am;
        g_bovAmp = 0.25f + am * 1.5f; g_bHold = 0; g_bPeak = 0;
    }
    // golden compressor SURGE flutter: only after being pinned on the wastegate 1s+ then
    // snapping shut hard -> the 90mm billet shudders (3 descending whistle pulses, 0.8s)
    bool atGate = (g_state==ES_RUN) && g_boostPsi >= gateRef * 0.88f;
    g_surgeT = fmaxf(0.0f, g_surgeT - dt);
    g_wgFlutter = (g_surgeT > 0) ? g_surgeAmt : 0.0f;
    if (g_prevThr > 0.4f && g_throttle < 0.12f && g_gateT > 1.0f) {
        g_surgeAmt = clampf((g_boostPsi - 20.0f) / 45.0f, 0, 1);
        g_surgeT   = 0.8f;
        g_core.stallShaft(0.7f);                              // surge knocks the shaft back
    }
    g_gateT = (atGate && g_throttle > 0.5f) ? (g_gateT + dt) : 0.0f;
    g_prevThr = g_throttle;
}
