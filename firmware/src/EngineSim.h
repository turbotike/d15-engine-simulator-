// =============================================================================
//  EngineSim.h - the drivetrain physics (ported from the shipped ESP32 truck).
//  Two-mass clutch, real flywheel + fuel-lag, Eaton auto-box, lug-from-idle
//  launch, air brakes, thermal, rev limiter. Publishes state; audio reads it.
// =============================================================================
#pragma once
#include <Arduino.h>

enum EngState : uint8_t { ES_OFF = 0, ES_CRANK, ES_RUN, ES_STOP };

class EngineSim {
public:
    void begin();
    // Call every control tick. throttle01/brake01 0..1, inDrive = forward gear,
    // inReverse = reverse gear, jakeAmt 0/0.55/1.0, loadAccel scales tractive accel.
    void update(float dt, float throttle01, bool inDrive, bool inReverse,
                float jakeAmt, float loadAccel, float brake01);
    void start();   // OFF -> crank -> run
    void stop();    // run -> shutdown

    // ---- published state (audio + outputs read these) ----
    float    rpm()        const { return g_rpm; }
    float    speedMph()   const { return g_speed; }
    // The ESC follows THIS. Street: flat out at 65 sim-mph. Race: flat out
    // at g_raceEscTopMph (a TUNE knob), so the model keeps accelerating
    // through the whole run instead of being pinned from the 8->10 shift
    // onward. Scale-correct for a 1/14 truck is the trap speed (~136): then
    // the model's 94 ft pass and the virtual quarter end together.
    float    speedFrac()  const {
        return clampf(g_speed / (g_raceMode ? g_raceEscTopMph : SIM_TOP_MPH), 0, 1);
    }
    float    reverseFrac()const { return clampf(g_revSpeed / REV_TOP_MPH, 0, 1); }
    uint8_t  gear()       const { return (uint8_t)g_gear; }
    float    load()       const { return g_load; }
    float    boostPsi()   const { return g_boostPsi; }
    float    shaftHz()    const { return g_shaftHz; }            // whistle pitch tracks the shaft
    float    wgFlutter()  const { return g_wgFlutter; }          // 0..1 compressor-surge shudder
    float    airPsi()     const { return g_airPsi; }
    float    airSecPsi()  const { return g_airSec; }
    float    brakePsi()   const { return g_brakeApplied; }
    float    coolantF()   const { return g_coolantF; }
    float    oilF()       const { return g_oilF; }
    float    oilPsi()     const { return g_oilPsi; }
    float    pyroF()      const { return g_pyroF; }
    float    transF()     const { return g_transF; }
    float    volts()      const { return g_volts; }
    float    fuelGal()    const { return g_raceMode ? g_raceFuelGal : g_fuelGal; }   // the tank feeding the engine
    float    fuelGph()    const { return g_gph; }
    float    fuelFrac()   const { return g_raceMode ? g_raceFuelGal / RACE_CELL_GAL : g_fuelGal / tankGal; }
    float    health()     const { return g_health; }

    // ---- TUNING: fuel & timing maps (8 rpm bands, 600-2100) ----
    // fuelMap 0.7..1.3 = injection quantity trim (power vs EGT/burn),
    // timMap  -5..+10 deg = timing advance (sweet spot per band -> power bonus,
    // overadvanced = slow engine damage).  Stock = 1.0 / 0.0.
    // PORTED FROM THE SIM (DieselCoreSim `fuel[]` / `tim[]`). The board used to boot
    // completely untuned - stock fuel, ZERO timing advance - which is why it sounded
    // tame next to the browser: lopeAmt() is (timMap[0]-4)/6, so 0 deg meant no lope
    // at all, and the loaded/bark layer scales off advance over stock too.
    // Bands are 600..1930 rpm, same cellAt() interpolation as the sim.
    // Ships STOCK - the board is a stock Cat out of the box, you tune it yourself.
    // These are the sim's STOCK_TIM values, not its tuned tim[]. Note the old default
    // was all-zero, which isn't stock either: bark scales off advance-over-stock, so
    // zero meant the engine could never sound untuned-but-alive the way the sim does.
    float fuelMap[8] = {1,1,1,1,1,1,1,1};                                   // 0.7..2.0 injection (needs AIR to burn!)
    float timMap[8]  = {2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 4.5f, 5.0f, 5.5f};    // sim STOCK_TIM, -5..+10 deg
    float boostMax   = 28.0f;                       // wastegate target 28..60 psi (turbo tune) - stock
    // the balance: EGT ~ fuel/air ratio; boost feeds air but raises cylinder
    // pressure with timing; overboost surges/overspeeds the S410G.  A balanced
    // tune runs ~1000 hp cool; a maxed-slider tune melts down mid-pull.
    struct DynoResult { float hp, rpm, egtPeak, boostPsi; bool derated, surged, pcpOver, blown; };
    DynoResult dynoRun() const;                    // full pull with tuning diagnostics (instant, bench)
    // ---- THE LIVE DYNO PULL (Mat: "an actual long pull like a real truck dyno") ----
    // Strapped down in neutral (wheels/ESC dead), full throttle: 2.5 s held at
    // 1000 rpm to load up and build boost, then the dyno brake lets it climb to
    // 2100 over 15 s. The real engine runs the whole time - sound, gauges, heat -
    // and a bad tune can melt or throw a rod mid-pull. Brake aborts.
    static const int DYNO_PTS = 23;                // 1000..2100 every 50 rpm
    bool  dynoStart();                             // false = engine not running
    void  dynoAbort()               { if (g_dynoOn) dynoFinish(true); }
    bool  dynoLive()  const         { return g_dynoOn; }
    float dynoProgress() const      { return g_dynoOn ? clampf(g_dynoT / 17.5f, 0, 1) : 0.0f; }
    float dynoHpNow() const         { return g_dynoHpNow; }
    const char* dynoResult() const  { return g_dynoRes; }   // "" until a pull finishes
    void  dynoClearResult()         { g_dynoRes[0] = 0; }
    String dynoCurveJson() const;
    void applyDamage(float d) { g_health = clampf(g_health - d, 0, 100); }
    void rebuild()            { g_health = 100; g_pistonMelt = 0; g_mechDamage = 0; }
    void setHealth(float h)   { g_health = clampf(h, 0, 100); if (h > 8) { g_pistonMelt = 0; g_mechDamage = 0; } }
    float pistonMelt() const  { return g_pistonMelt; }               // 0..1 crown melt
    int   deathCause() const  { return g_deathCause; }               // 0 melt / 1 over-pressure
    bool  takeMeltEvent()     { bool m = g_meltEvent; g_meltEvent = false; return m; }

    // ---- upgradeable parts (persisted to EEPROM; boots bone-stock, earned by breaking things) ----
    struct Parts { uint8_t rods = 0, pistons = 0, turbo = 0, ic = 0, fuel = 0; };
    Parts parts;                                 // installed hardware
    Parts unlocks;                               // highest tier earned per part
    void  loadParts();                           // read EEPROM (or stock defaults) + apply
    void  saveParts();                           // write EEPROM + re-apply
    void  applyParts();                          // push part specs into the core + damage limits
    bool  installPart(uint8_t idx, uint8_t tier);// idx 0=rods 1=pistons 2=turbo 3=ic 4=fuel
    float rodBar()  const { return g_rodBar; }
    float meltF()   const { return g_meltF;  }
    // the REAL hill from the GY-521 (Tilt): replaces the rolling-terrain generator while on
    void setRealGrade(float g, bool on) { g_realGrade = clampf(g, -0.35f, 0.35f); g_realGradeOn = on; }
    void setRealVolts(float v)    { g_extVolts = v; g_haveVolts = true; }

    // ---- manual transmission mode (PS4 full-manual) ----
    void  setManual(bool on)          { g_manual = on; }
    void  setTrailer(bool on)         { g_trailer = on; g_loadMass = on ? 42000.0f : 8000.0f; }
    // UI's Trailer Load is 0-3, not a flag: empty / light / heavy / lowboy
    void  setLoadLevel(float lvl) {
        static const float M[4] = { 8000.0f, 20000.0f, 32000.0f, 42000.0f };
        int i = (int)(lvl + 0.5f); if (i < 0) i = 0; if (i > 3) i = 3;
        g_loadMass = M[i]; g_trailer = (i > 0);
    }
    float clutchHeat() const          { return g_clutchHeat; }
    void  setClutchPedal(float c01)   { g_clutchPedal = clampf(c01, 0, 1); }   // 0=engaged 1=floored
    // FULL STOP TIME (web: fullStopTime). Seconds for FULL brakes to stop the truck
    // from the speed it was doing when the brakes went on; partial brake = slower.
    // 0 = an instant stop. Max 3 s. See roadDrag().
    void  setStopTime(float s)        { g_stopTime = s < 0 ? 0 : s; g_stopEnabled = true; }
    void  setPtoLoad(float l01)       { g_ptoLoad = clampf(l01, 0, 1); }     // hydraulic pump drag
    void  setParkBrake(bool on)       { if (g_parkBrake && !on) g_hissEvent = 0.9f;  g_parkBrake = on; }
    bool  parkBrake() const           { return g_parkBrake; }
    void  setManualGear(int g)        { g_manReq = (int8_t)constrain(g, 1, N_GEARS); }

    // ---- RACE MODE --------------------------------------------------------
    // A drag-strip mode: sit in NEUTRAL and free-rev to build boost, then drop
    // it into a chosen launch gear and let it row up to a chosen top gear.
    //
    // Why it needs its own mode rather than just picking gears manually:
    //   * The box normally starts in 1st. A drag launch wants a gear YOU pick
    //     (a loaded 10-speed launches in 1st; a light one on a prepped surface
    //     might take 3rd or 4th and carry the revs).
    //   * It must not upshift past the gear you nominated, so a run ends on
    //     the ratio you tuned for instead of shifting into overdrive and
    //     falling off the power.
    //   * Free-revving in neutral has to actually spin the turbo, so there is
    //     boost on the clutch dump - that is the whole point of staging.
    // RACE MODE IS AN ENGINE AND A CHASSIS, NOT JUST A GEAR LADDER.
    //
    // This used to only pick gears, and that was why a drag run bogged no
    // matter which gear it launched in. The numbers, worked out on the bench:
    //
    //   as configured   PEAK_TORQUE 2900 Nm = ~650 hp at 2100 rpm
    //                   g_loadMass  32000 kg (loaded tractor-trailer)
    //                   -> 0.020 hp/kg
    //   a real 3000 hp  ~10000 Nm, ~6000 kg, no trailer
    //                   -> 0.500 hp/kg, i.e. TWENTY-FIVE TIMES better
    //
    // Stock C15 6NZ is 550 hp / 1850 lb-ft, so 2900 Nm was already a mildly
    // built engine - nowhere near a competition build. 650 hp dragging 32
    // tonnes bogs in 2nd because it genuinely would; no launch gear fixes
    // that, and chasing the gearing was treating the symptom.
    //
    // So arming race mode now also:
    //   * drops the mass to a bare drag truck (no trailer, stripped)
    //   * multiplies the torque to the chosen power level
    // and disarming puts both back, so the road truck is untouched.
    //
    // racePower: 0 = street (as-is), 1 = 1500 hp, 2 = 3000 hp.
    void  setRacePower(uint8_t lvl) { g_racePower = (lvl > 2) ? 2 : lvl; }

    // ---- CLUTCH RELEASE TIME ----------------------------------------------
    // How long the driver takes to let the pedal up on a launch.
    //
    // I built the wrong thing first: a permanent cut to the clutch's torque
    // CAPACITY. That is a slipper clutch, and it was wrong twice over - it
    // slipped forever instead of only during the launch, and whatever torque
    // it refused to pass was torque the truck never got, so the run just fell
    // flat. Capacity is not the knob; TIME is.
    //
    // This instead feeds the existing g_clutchPedal out from 1 (floored) to 0
    // (home) over clutchRelease seconds. grab already honours the pedal via
    //     grab = min(grab, 1 - g_clutchPedal)
    // so a gradual release gives a gradual take-up for free, and once the
    // pedal is home the clutch is a NORMAL clutch again with full capacity.
    //
    // 0.0 = dump it (what every test so far was doing by accident)
    // 2.0 = feed it in over two seconds, which is what keeps the engine up in
    //       its band while the truck gets moving.
    float g_clutchReleaseUI = 1.2f;      // seconds; public for the WebUI knob
    // Sim speed at which the real truck is flat out in race mode. See
    // speedFrac(). Public for the WebUI knob. Dial-in on the driveway: if
    // the model crosses 94 ft before the sound finishes, raise it; if it is
    // still short when the sound finishes, lower it.
    float g_raceEscTopMph = 130.0f;
    void  setClutchRelease(float s) { g_clutchReleaseUI = clampf(s, 0.0f, 3.0f); }
    float clutchRelease() const     { return g_clutchReleaseUI; }
    // Live pedal position during a launch, so the UI and audio can see it.
    float clutchPedalNow() const    { return g_clutchPedal; }
    // Drag telemetry. et60/etQtr stay at their tripped values after the run
    // so the numbers can be read at leisure.
    float dragDistFt() const { return g_dragDist * 3.28084f; }
    float dragTime()   const { return g_dragTime; }
    float et60()       const { return g_et60; }
    float etQtr()      const { return g_etQtr; }
    float trapMph()    const { return g_trapMph; }
    bool  dragRunning()const { return g_dragRun; }
    uint8_t racePower() const       { return g_racePower; }
    // Torque multiplier the physics applies. 1.0 off, or scaled so peak
    // torque lands near the figure a build of that power actually makes.
    float raceTqMult() const {
        // RACE CLASS RETIRED (2026-10-30): the parts decide the horsepower, always.
        return 1.0f;
        if (!g_raceMode || g_racePower == 0) return 1.0f;
        // MADE THE LABELS HONEST. These were 1.75 / 3.50, which on a 2900 Nm
        // / 650 hp base is ~1140 hp and ~2300 hp - so the button marked
        // 3000HP was delivering about three quarters of it, which is the
        // 'thats not 3000hp' complaint, and it was right.
        //   1500 hp needs 1500/650 = 2.31
        //   3000 hp needs 3000/650 = 4.62
        return (g_racePower == 1) ? 2.31f : 4.62f;   // ~1500 hp / ~3000 hp
    }
    // NO RACE MASS OVERRIDE. The truck weighs what a truck weighs.
    //
    // I had this forcing 6000 kg in the 3000 hp class, which is LIGHTER than
    // a real bare tractor (7500-9000 kg) - a fantasy number doing the work
    // the power should be doing. And it was unnecessary: setLoadLevel already
    // offers 8000 / 20000 / 32000 / 42000 kg, and level 0 at 8000 IS the
    // bobtail tractor. A real 3000 hp drag truck is essentially a bobtail,
    // so the honest setup is level 0 and let the trailer load be real.
    //
    // At 2300 hp the honest numbers come out at 0.29 hp/kg bobtail, 0.12 with
    // an empty trailer and 0.07 loaded - which is a meaningful spread the
    // driver can feel, instead of one hidden constant.
    float raceMassKg() const { return 0.0f; }   // 0 = use the real mass
    // RE-ROLL THE SHIFT POINT THE MOMENT THE MODE CHANGES.
    // This was the whole bug behind 'it needs to pull to redline'. Setting
    // the flag alone left g_upPoint holding whatever the last STREET shift
    // rolled (~1840-2020, or lower), and rollDriver() only runs when a change
    // commits - so the first shifts of a run, the ones you actually listen
    // to, used the stale street point. MEASURED with the point already moved
    // to 2080: the box still changed at 1121/1595/1606/1642. The redline
    // number was correct and simply never reached g_upPoint in time.
    void  setRaceMode(bool on)        { g_raceMode = on; rollDriver(); }
    bool  raceMode() const            { return g_raceMode; }
    // THREE GEARS, because a drag run SKIPS. Measured on the bench: a 4th
    // gear launch at 32 t bogged the engine to 6 rpm and the box had to row
    // itself down 4-3-2-1 to recover, losing about four seconds. 4.79:1
    // against that mass from a standstill is simply too tall, staged boost or
    // not - the moment the clutch takes up the engine is pulled below its
    // torque peak and the boost collapses (29.8 -> 2.6 -> 0 psi in two
    // readings).
    //
    // So launch LOW and skip: 1st or 2nd to get it rolling, then jump
    // straight to a gear that is actually in the power band instead of
    // walking every ratio. That is what a real drag truck does, and it is why
    // two gears were not enough.
    //
    //   launch - the gear the clutch drops into
    //   skip   - the gear it jumps to once rolling (0 = no skip)
    //   top    - the highest gear the box will take
    // FOUR-GEAR LADDER: launch > skip > skip2 > top (Mat: "the real guys use 3").
    // skip2 = 0 keeps the old three-gear ladder.
    void  setRaceGears(int launch, int skip, int top, int skip2 = 0) {
        g_raceLaunch = (int8_t)constrain(launch, 1, N_GEARS);
        g_raceTop    = (int8_t)constrain(top,    1, N_GEARS);
        g_raceSkip   = (int8_t)constrain(skip,   0, N_GEARS);
        g_raceSkip2  = (int8_t)constrain(skip2,  0, N_GEARS);
        if (g_raceTop < g_raceLaunch) g_raceTop = g_raceLaunch;
        // A skip below the launch gear or above the top makes no sense; 0
        // disables it rather than silently clamping to something surprising.
        if (g_raceSkip && (g_raceSkip <= g_raceLaunch || g_raceSkip > g_raceTop))
            g_raceSkip = 0;
        int lo = g_raceSkip ? g_raceSkip : g_raceLaunch;
        if (g_raceSkip2 && (g_raceSkip2 <= lo || g_raceSkip2 >= g_raceTop)) g_raceSkip2 = 0;
        g_raceSkipDone = false;
    }
    int   raceLaunchGear() const      { return g_raceLaunch; }
    int   raceTopGear()    const      { return g_raceTop; }
    int   raceSkipGear()   const      { return g_raceSkip; }
    int   raceSkip2Gear()  const      { return g_raceSkip2; }

    // Grant an upgrade tier outright. installPart() refuses anything not
    // already in `unlocks` (parts are earned by breaking things), which is
    // right for the career path but wrong for a deliberate UI button - the
    // button would silently do nothing. This unlocks AND fits in one call.
    bool  grantPart(uint8_t idx, uint8_t tier) {
        if (idx > 4 || tier > 2) return false;
        uint8_t* u = &unlocks.rods + idx;
        if (tier > *u) *u = tier;
        return installPart(idx, tier);
    }
    bool  manual() const              { return g_manual; }

    // realism tunables (web-adjustable)
    float tankGal   = 120.0f;   // twin 60 gal tanks
    float fuelScale = 1.0f;     // UNUSED: the tank burns in real time from the sim (Mat: no burn-rate slider). Kept: a saved settings slot
    float turboSpoolUp   = 1.0f;   // 1.0 = stock S410G 4s spool
    float turboSpoolDown = 1.0f;   // 1.0 = stock 1.2s coast-down
    float gradeMax       = 0.0f;   // flat world (rolling-hills feature retired with its slider)

    float grade()  const { return g_grade; }               // current road grade
    bool  fanOn()  const { return g_fanOn; }               // fan clutch engaged

    // BorgWarner S410G turbo telemetry
    float turboRpm()    const { return g_tShaft; }              // 0..145,000
    // g_tShaft is REAL shaft rpm (_wTurbo * 9.5493). The old divisor of 120000
    // was written for a rad/s-scaled value, and the stock turbo only reaches
    // ~10,700 rad/s = 102,000 rpm in normal running - so this returned a peak of
    // about 0.09 instead of ~1.0. Two things read it and both were starved:
    // the baked turbo SAMPLE played at a tenth of its level (so the big-turbo
    // spool whoosh was essentially missing and only the synth whistle carried
    // the turbo), and EngineAudio's loadT lost its boost term entirely, so
    // lugging a grade at part throttle rendered as LIGHT load.
    // 102000 is the real stock operating ceiling, so this now spans 0..1.
    float turboSpool()  const { return clampf(g_tShaft / 102000.0f, 0, 1); }
    float wastegate01() const { return g_tWg; }

    void  refuel()            { g_fuelGal = tankGal; g_raceFuelGal = RACE_CELL_GAL; }   // fills the road tanks AND the race cell
    void  setFuelGal(float g)  { g_fuelGal = clampf(g, 0, tankGal); }
    // RACE FUEL CELL (Mat: "in race mode doing pulls at 3000 hp it should be drinking
    // fuel"). A pull burns ~0.5 gal for real - invisible on 120 gal of road tanks,
    // so race mode runs off a small cell like a real puller: ~5% a pass.
    static constexpr float RACE_CELL_GAL = 10.0f;
    float roadFuelGal() const   { return g_fuelGal; }
    float raceFuelGal() const   { return g_raceFuelGal; }
    void  setRaceFuelGal(float g) { g_raceFuelGal = clampf(g, 0, RACE_CELL_GAL); }
    // Battery = fuel (Battery.cpp). The real pack's state of charge drives the tank,
    // and starve (0..1) cuts fuel in random bursts - the engine sputters, then dies.
    void  setFuelFrac(float f) { g_fuelGal = clampf(f, 0, 1) * tankGal; }
    void  setStarve(float s)   { g_starve = clampf(s, 0, 1); }
    float g_starve = 0;
    // cold-start restore: how long the truck sat + coolant temp when it was shut down
    void  restore(float savedFuelGal, float savedCoolF, float elapsedMin);

    // ---- AIR TANKS ACROSS A POWER CYCLE ------------------------------------
    // A parked truck holds its air. The tanks leak down slowly over hours, but
    // they do NOT start empty, and the governor only pumps while the engine
    // turns - so booting at 0 psi meant the low-air alarm sounded on every
    // power-up with nothing you could do about it until you started up and held
    // revs. Saving the tank pressure on shutdown and bleeding it by elapsed time
    // on boot is how the real thing behaves and removes the problem at its root.
    void  saveAir();                     // call on shutdown / before power-off
    void  restoreAir(float elapsedMin);  // call at boot, after loadParts()
    void  maybeSaveAir();                // periodic; writes only on a real change
    uint16_t warnBits()   const { return g_warn; }
    bool     limiter()    const { return g_limiter; }
    EngState state()      const { return (EngState)g_state; }
    bool     running()    const { return g_state != ES_OFF; }
    // TRUE only while the starter is turning it and it has not caught yet.
    // running() is `state != ES_OFF`, which includes ES_CRANK - so the audio
    // could not tell cranking from running and faded the ladders up under the
    // starter sample. The engine is not firing yet; it should make no engine
    // noise at all.
    bool     cranking()   const { return g_state == ES_CRANK; }
    float    throttle()   const { return g_throttle; }
    float    jakeAmt()    const { return g_jakeAmt; }
    float    chopDepth()  const { return g_chopD; }   // shared with the audio (sim window._chD)
    // PUBLISHED so the audio does not keep its own copy of the pattern. It
    // did, twice: first the chop RATE went stale when the sim moved to the
    // firing rate and the sound kept galloping at a flat 4 Hz, then the
    // per-cylinder table had to be edited in two files. The amplitude is
    // physics - one weak cylinder - so the sim owns it and the audio reads it.
    float    chopAmpNow() const { return g_chopAmp; }
    float    fuelOnK()    const { return g_fuelOnK; }   // 0 = injectors cut (jake / overrun), 1 = firing
    uint8_t  chopCylNow() const { return g_chopCyl; }
    // lope events for the audio (one per engine cycle): seq bumps on each
    uint32_t chopSeq()   const { return g_chopSeq; }
    bool     chopBark()  const { return g_chopBark; }
    float    chopDurNow()const { return g_chopDur; }
    float    huntNow()   const { return g_hunt; }           // -1..1 governor hunt
    float    chopDeep()  const { return g_chopDeep; }       // 0..1 how deep this bark is (rhythm CHUG = 1)
    float    lopeMess()  const { return g_lopeMess; }       // 0 tuned rhythm .. 1 messed-up random
    bool     chopRhythm() const { return g_chopRhythm; }    // always false now (kept for EngineAudio)
    // THE PULSE LOPE (EngineAudio): big-plunger injectors only, the whole rev
    // range, unless the idle tune is messed up (then the random stumble).
    // the engine's own output torque right now (Nm, 0 off throttle) - in or out
    // of gear, so a blip in neutral rocks the chassis too (Suspension torque twist)
    float    engineTorqueNm() const;
    // THE TURBO SOUND, SCALED TO THE TURBO FITTED. The bigger turbos run up to 4x
    // the boost off the same wastegate (and spin to a higher shaft ceiling); fed
    // raw to the audio they pinned the whistle and screeched (Mat: "the compound
    // on high boost sounds bad"). These map each turbo back onto the range the
    // stock one was tuned in. turboTone(): 1 stock, deeper for the bigger ones.
    float    soundBoostPsi() const;
    float    soundShaftHz()  const;
    float    turboTone()     const;
    float    pulseLopeAmt() const { return parts.fuel >= 2 ? 1.0f - g_lopeMess : 0.0f; }
    bool     chopHi()     const { return g_chopHi; }  // top-end bark vs idle lope

    // ---- SHIFT STATE, for the audio ----------------------------------------
    // The float-shift model below was ported from the sim complete, but none of
    // it was observable: g_shiftCut was computed every frame and read by
    // nothing, so the exhaust never knew a shift was happening. The box did the
    // right thing mechanically and sounded like a slushbox. These are what the
    // sim's audio path actually reads.
    bool     inShift()    const { return shifting; }           // sim shiftT>0
    int      shiftTarget()const { return shiftTo; }             // sim shiftTo
    float    shiftCut()   const { return g_shiftCut; }          // 0.55 mid-shift, ramps to 1
    // 1 normally; ramps to 0 through shutdown so the sampled stop owns the
    // wind-down instead of competing with the sim's.
    float    shutMute()   const { return g_shutMute; }
    // Override the temperature-derived crank time, so the sim can be matched to
    // the length of whatever start.wav the pack ships. Call right after start().
    void  setCrankDur(float s) { if (s > 0.2f && s < 12.0f) crankDur = s; }
    // The engine LIT. Called by the three-part start sequence when its dice
    // roll succeeds, so the sim leaves ES_CRANK on the same frame the fireup
    // sample starts - the audio and the physics catch together.
    void  catchNow() { if (g_state == ES_CRANK) crankT = crankDur; }
    // TRUE while floating DOWN a gear. The sim answers this with
    //     shiftT>0 && shiftTo < trans.gear
    // and uses it to feed the exhaust 0.85 throttle with your foot off - the
    // driver blipping the revs up to meet the lower gear. It is the most
    // recognisable sound of someone rowing a 13-speed, and we had none of it.
    bool     blipping()   const { return shifting && shiftTo != 0 && shiftTo < g_gear; }
    // --- browser-parity audio drivers ---
    float    barkFactor(float rpm) const;   // exhaust volume from timing (0 stock .. 1 = +10 loud)
    float    lopeAmt()    const;            // lope-idle amount from idle-cell timing
    bool     lopeForce = false;             // RETIRED (the LOPE IDLE button is gone - big plungers lope); kept for the status field
    float    takeBovAmp() { float a=g_bovAmp; g_bovAmp=0; return a; }  // blow-off pssh on let-off
    // jake is actually barking: armed + in gear + off throttle + rolling + above ~1100
    // Matches the SIM's audio gate exactly: jakeStage && armed && throttle<0.05 && rpm>660.
    // NOT gated on being in gear or moving - the sim barks the jake in neutral too. The
    // TORQUE stays gated on drive (stepCoupled only runs coupled), same as the sim, so it
    // makes the noise without retarding anything that isn't turning.
    // CUT-OUT BELOW ~1000 rpm like a real Jacobs (it was 660, so it barked at idle -
    // Mat: "the jake doesn't shut off at idle"). g_jakeRpmOk carries the hysteresis:
    // on above 1050, off below 950, so it can't flicker at the line.
    bool     jakeEngaged()const { return g_jakeAmt > 0.01f && g_state == ES_RUN &&
                                          g_clutchPedal < 0.5f &&                                     // clutch switch interlock
                                          g_throttle < 0.05f && g_jakeRpmOk; }
    // jakeEngaged() above is the AUDIO gate and matches the sim - it barks in neutral.
    // This is the "actually retarding the truck" gate: in gear and rolling. Brake lights,
    // the ESC drag brake and anything physical must use THIS one, or the jake sounding
    // at idle sticks the brake lights on.
    bool     jakeRetarding() const { return jakeEngaged() && g_drive && g_speed > 8.0f; }
    // clamp the sim road speed to the ESC's braked output (keeps sound == wheels)

    static inline float clampf(float x, float a, float b){ return x < a ? a : (x > b ? b : x); }

private:
    // ---- tunables (mirror the shipped truck) ----
    static constexpr float IDLE_RPM = 680.0f, MAX_RPM = 2100.0f;
    static constexpr float DIFF = 3.55f, J_ENGINE = 4.0f;   // kg*m^2: crank + 35.8 kg flywheel + 15.5 in clutch (2026-10-30 research)
    static constexpr float FUEL_RISE = 2.4f, FUEL_FALL = 7.0f;
    static constexpr float CLUTCH_STIFF = 2.0f, CLUTCH_CAP = 4600.0f, CLUTCH_LOCK_SLIP = 55.0f;
    static constexpr float CLUTCH_ENGAGE_FIRM = 18.0f, CLUTCH_RELEASE_MPH = 2.2f, CLUTCH_OUT_GRAB = 0.15f;
    static constexpr float LUG_MIN_RPM = 330.0f, IDLE_GOV_GAIN = 2.0f;
    static constexpr float ENG_IDLE_RATE = 3.2f, ENG_FREEREV_RATE = 2.8f;
    static constexpr float WHEEL_R = 0.50f, DRIVE_EFF = 0.85f, BASE_MASS = 37000.0f;
    static constexpr float PEAK_TORQUE = 2900.0f, ENG_BRAKE = 340.0f, JAKE_TORQUE = 2600.0f;
    // DAMP was 240 - double the sim's 120, so the board dragged itself down and never
    // pulled like the browser. Every other drag term already matched.
    static constexpr float CRR = 0.008f, CDA = 8.0f, AIR_RHO = 1.20f, DAMP = 120.0f, GRAV = 9.81f;
    // Match the sim: it revs to 1950 before grabbing the next gear and lugs to 1100
    // before dropping one, and a float shift takes 1.4s in both directions.
    static constexpr float MPH_MS = 0.44704f, UPSHIFT = 1950.0f, DOWNSHIFT = 1100.0f;
    // Downshift point while the engine brake is applied. A driver rowing down
    // on the Jake changes EARLY to keep the revs up where the brake works,
    // rather than waiting for the engine to die at 1100 like an auto box.
    // 1350, and the ceiling is arithmetic not taste: a shift changes the ratio
    // by 37%, so the revs jump 1.37x. For a downshift not to immediately trip
    // the 1950 upshift it must satisfy  dnPoint * 1.37 < 1950, i.e. < 1423.
    //
    // History, because I walked this number up and down twice: 1550 made the
    // box HUNT (g7>8 g8<7 forever - 1550 * 1.37 = 2124, above the upshift
    // point), then 1350, then 1280.
    //
    // 1000 is where it belongs, and it matches what drivers actually do - the
    // research puts the real range at 1000-1300 before grabbing the next one
    // down. The effect on spacing is not uniform, which is worth knowing:
    //
    //            span at 1280        span at 1000
    //   g10        2.16 s              4.71 s     <- top gears get LONGER
    //   g9         3.10 s              2.42 s
    //   g6         1.17 s              0.91 s
    //   g5         0.91 s              0.71 s     <- low gears get SHORTER
    //
    // A lower point means a lower road speed per gear, and the jake is also
    // weaker down there (torque scales with we/MAX_RPM: 0.48 at 1000 vs 0.61
    // at 1280), so the bottom gears both cover less ground AND brake less.
    // That is why JAKE_FLOOR_GEAR went to 5 at the same time - the gears whose
    // spacing this hurts are exactly the ones that no longer run.
    //
    // Hunting is still clear with room to spare: dn_max 1035 against an
    // upshift floor of 1840.
    static constexpr float JAKE_DOWNSHIFT = 1000.0f;

    // ---- THE JAKE DOES NOT ROW YOU DOWN TO A STOP -----------------------
    // The last four gears felt 'rapid as hell' and no shift-timing number
    // could fix it, because the gears genuinely run out. MEASURED off the
    // ratio tables:
    //
    //   gear   speed span before it downshifts again
    //   g10    10.8 mph        g5    4.5 mph
    //   g9     15.5 mph        g4    3.2 mph
    //   g8     11.4 mph        g3    2.4 mph
    //   g7      8.8 mph        g2    1.7 mph
    //   g6      5.8 mph        g1    1.3 mph
    //
    // g1-g4 together span under 9 mph. WORSE, the deceleration is RISING at
    // the same time: jake torque is multiplied by the gear ratio, so g1
    // brakes 16.9x harder at the wheel than g10 does. Shorter gears and
    // harder braking compound - the last few changes cannot help but be
    // instant.
    //
    // A real driver does not do this. The jake is a highway and grade tool;
    // you come down a few gears on it and the SERVICE BRAKES finish the job.
    // So: stop auto-downshifting on the jake below this gear, and let the
    // brakes take over.
    //
    // 5, raised from 4 when JAKE_DOWNSHIFT came down to 1000. The lower shift
    // point shortens exactly the gears that were already too short (g5 drops
    // from 0.91 s to 0.71 s), so the floor moved up to cut them out. Nine
    // shifts on the jake became six, then five.
    static constexpr int   JAKE_FLOOR_GEAR = 5;      // ~9.4 mph at the dn point

    // Below that, the driver gets on the brake pedal. This is a REAL brake
    // application - it goes through the same air relay as the RC brake input,
    // so it consumes air, builds pressure through the relay lag and triggers
    // the release hiss, exactly like a driver pressing the pedal.
    // 8.5, not 11: with the floor at 5th the last downshift now happens at
    // 9.4 mph, and at 11 the brake was already coming on BEFORE that shift -
    // so the truck was slowing on the pedal while still rowing a gear, which
    // is not what a driver does. Start under the last shift, not over it.
    static constexpr float JAKE_BRAKE_MPH  = 8.5f;   // start feathering in here
    static constexpr float JAKE_BRAKE_MAX  = 0.55f;  // how hard he presses
    static constexpr float SHIFT_TIME = 1.4f, SHIFT_MATCH_RPM = 40.0f;

    // ---- THE DRIVER IS NOT A MACHINE --------------------------------------
    // Instrumented, nine shifts in a row: every one fired at wr 1950-1954 and
    // every one took exactly 1.4 s. That consistency is WHY it sounds like an
    // automatic rather than someone rowing it - a human shifts at 1900 one
    // time and 2050 the next, is quick on an easy change and slow when the
    // lever snags, and does not hit the same rpm twice in a row.
    //
    // So each shift gets its own randomised point and duration. The ranges are
    // bounded by the same arithmetic as everything else: the upshift point must
    // stay clear of the 1950-ish redline chop, and the downshift point must
    // satisfy dnPoint * 1.37 < 1950 (i.e. < 1423) or the box hunts.
    // RANGES SOLVED FROM THE CONSTRAINTS, not picked by feel:
    //   (1) dn_max * 1.37 < up_min   or the box hunts (a shift moves the revs
    //       by 1/0.73 = 1.37x, so too high a downshift trips the upshift)
    //   (2) up_max < 2050            or the upshift lands in the limiter chop
    //
    // First attempt was +/-110 on a 1950 centre with the jake dn at 1350+/-70.
    // Checked it before flashing: dn_max 1420 * 1.37 = 1945, above the lowest
    // upshift 1840 - it WOULD have hunted, same failure as the 1550 attempt.
    //
    // 1930 +/- 90 gives upshifts anywhere in 1840..2020 (a 180 rpm window, more
    // than enough to stop it sounding metronomic) and leaves the jake downshift
    // ceiling at 1840/1.37 = 1343.
    static constexpr float UPSHIFT_CENTRE   = 1930.0f;
    static constexpr float UPSHIFT_SPREAD   =   90.0f;
    // 35, not 50. The constraint is dn_max * ratio_step < up_min, or the gear
    // lands above its own upshift point and the box shifts straight back up.
    // g8->g7 is the tightest step in the 10-speed at 1.390: with spread 50,
    // dn_max 1330 * 1.390 = 1849 against an up_min of 1840 - over by 9 rpm, so
    // that one change could hunt on an unlucky roll. 35 gives 1315 * 1.390 =
    // 1828 and every step clears. Checked on paper this time rather than
    // after flashing it.
    static constexpr float DOWNSHIFT_SPREAD =   35.0f;
    // RAISED to clear the neutral phase. These are the PATIENCE BACKSTOP - the
    // point at which the driver gives up matching and just puts it in. The old
    // 0.95-1.75 s was fine when the rpm-match test ended every shift early, but
    // a downshift now needs NEUTRAL_MAX + LEVER_TIME = 1.25 s before it can
    // engage, so a 0.76 s backstop would fire FIRST and bypass the whole thing.
    static constexpr float SHIFT_TIME_MIN   = 1.60f;   // a quick, clean change
    static constexpr float SHIFT_TIME_MAX   = 2.40f;   // fumbled the gate
    // Re-rolled on every shift so no two are alike.
    float g_upPoint = UPSHIFT_CENTRE, g_dnJitter = 0.0f;

    // How long the throttle has been HELD wide open in neutral. Gates the
    // staging fuel so a blip free-revs normally but holding it makes boost.
    float  g_stageHold  = 0.0f;
    bool   g_raceMode   = false;
    uint8_t g_racePower = 0;        // 0 street / 1 ~1500hp / 2 ~3000hp
    // Counts down while the pedal is being fed out on a launch. Negative/zero
    // means no launch in progress and the pedal is left alone.
    float   g_launchT = 0.0f;
    // ---- DRAG TIMING ------------------------------------------------------
    // A real race has a finish line, so race mode gets one: NHRA quarter mile
    // is 1320 ft / 402.3 m, and the 60-foot time is the number that actually
    // tells you whether the launch was any good. Big-rig quarter-mile ETs run
    // roughly 16-20 s with 90-100 mph traps, which is the ballpark to judge
    // against.
    float g_dragDist = 0.0f;      // metres travelled since the launch
    float g_dragTime = 0.0f;      // seconds since the launch
    float g_et60 = 0.0f;          // 60 ft time, 0 until tripped
    float g_etQtr = 0.0f;         // quarter-mile ET
    float g_trapMph = 0.0f;       // speed at the finish line
    bool  g_dragRun = false;      // a run is in progress
    int8_t g_raceLaunch = 1;        // gear the launch drops into
    int8_t g_raceTop    = 10;       // highest gear the box will take
    int8_t g_raceSkip   = 0;        // jump straight to this once rolling (0 = off)
    int8_t g_raceSkip2  = 0;        // the 4th rung of the ladder (0 = off)
    bool   g_raceSkipDone = false;  // one skip per launch, not every upshift

    // ---- A DOWNSHIFT IS NOT INSTANT -------------------------------------
    // RESEARCHED against driver reports: a real downshift in a
    // non-synchronised Eaton is 1.0-1.5 s end to end, and 0.6-0.9 s of that
    // is spent IN NEUTRAL while the driver blips the revs up to meet the
    // lower gear. That neutral phase is the audible event - it is the gap
    // you hear between gears, and the firmware had none of it.
    //
    // The old code ran one dt*14 rev ramp (~70 ms) and then engaged as soon
    // as the revs came within SHIFT_MATCH_RPM. So the whole change finished
    // in 70-150 ms, roughly 10x too fast, and no amount of moving the shift
    // POINT could fix it - the shift had no DURATION. The research also says
    // the shift points were right all along: drivers drop to ~1000-1300 and
    // blip up to ~1400-1600, which brackets JAKE_DOWNSHIFT at 1280.
    //
    // Drivers describe the neutral time as proportional to the blip - the
    // higher you run the revs up, the longer you wait before dropping it in.
    // So NEUTRAL_MIN is the floor for a small splitter-sized blip and the
    // rest scales with how far the revs must actually climb: a full gear is
    // ~500 rpm, a splitter ~250.
    static constexpr float NEUTRAL_MIN   = 0.42f;   // settle time, small blip
    static constexpr float NEUTRAL_PER_K = 0.78f;   // extra seconds per 1000 rpm of blip
    static constexpr float NEUTRAL_MAX   = 0.95f;   // a two-gear grab on a grade
    // ...plus the lever itself. The research budgets ~0.15 s to pull it out of
    // gear and ~0.15 s to push it into the next one, either side of the blip.
    // Without this the modelled change came to 0.81 s - the neutral phase was
    // the right length on its own but nothing accounted for the hands, so the
    // total sat below the 1.0-1.5 s a real downshift takes.
    static constexpr float LEVER_TIME    = 0.30f;   // out of gear + into gear
    // How fast the engine actually comes up when blipped in neutral. A big-bore
    // truck diesel with a heavy flywheel is ~800-1500 rpm/s; 1100 sits mid-band
    // and makes a 500 rpm full-gear blip take ~0.45 s, which lands the total
    // downshift inside the researched 1.0-1.5 s.
    static constexpr float BLIP_RPM_PER_S = 1100.0f;
    // Cranking. CRANK_RPM is real starter speed and sits BELOW the pack's
    // lowest rung (601 rpm), so the sampled ladders are near silent while
    // start.wav plays. CRANK_CATCH is the fraction of the crank spent spinning
    // before the engine catches and pulls up to idle.
    static constexpr float CRANK_RPM = 220.0f, CRANK_CATCH = 0.72f;
    static constexpr float SIM_TOP_MPH = 65.0f, REVERSE_RATIO = 13.5f, REV_TOP_MPH = 2.4f, REV_BUILD = 0.9f;
    static constexpr float AIR_START = 35, AIR_CUTIN = 95, AIR_CUTOUT = 120, AIR_MAX = 125;
    // How far the tank must move before maybeSaveAir() spends a flash write.
    static constexpr float AIR_SAVE_DELTA = 8.0f;
    static constexpr float SPRING_REL = 60, LOW_AIR = 55, BRAKE_MAX_PSI = 62, BRAKE_AIR_COST = 0.28f;
    static constexpr float COMP_IDLE = 0.8f, COMP_REV = 6;   // idle barely builds - rev for air
    static constexpr float AMBIENT_F = 72, FAN_ON = 212, FAN_OFF = 200, OVERHEAT = 235, DAMAGE_F = 250;
    // TEN SPEED, even ratios.
    //
    // Was a real Eaton RTLO-16913A - 5 main gears with a SPLITTER on the top
    // four, so 6..13 were the L/H halves of main gears 6..9. Shifting all
    // thirteen sequentially gave:
    //     gears 1-5   drop 520-590 rpm per shift
    //     gears 6-13  drop only 260-330
    // Eight of twelve shifts were half the size of the first five, so up top it
    // sounded like someone flicking a splitter rather than making shifts - which
    // is exactly what it was doing.
    //
    // A straight 10-speed with a constant 37% step drops 526 rpm on EVERY shift,
    // first to top. 1st stays at 373 rpm/mph so the launch feel is unchanged and
    // top gear stays at 22, i.e. ~75 mph at the 1950 rpm shift point.
    static constexpr int   N_GEARS = 10;
    static const float gearRatio[N_GEARS];
    // rpmPerMph is MUTABLE now: race mode fits a taller rear end (see
    // RACE_FD_SCALE) and this table is the one place the final drive lives.
    // RPM_PER_MPH_BASE is the 3.55 street truck it is scaled from.
    static const float RPM_PER_MPH_BASE[N_GEARS];
    static float       rpmPerMph[N_GEARS];
    // THE RACE REAR END. 10th at 2100 rpm on the 3.55 is 95.5 mph, so the
    // truck reached the limiter BEFORE the stripe and leaned on it. A
    // racer gears for the strip: x0.82 is a 2.91, which puts 10th at ~116
    // mph and lets it pull to the line. Gear STEPS are unchanged, so every
    // landing rpm in the UI ladder readout still holds.
    // 0.75, not 0.82: at 0.82 10th still met the limiter ~1 s before the
    // stripe (2024 rpm at 112 mph). 0.75 is a 2.66, 10th tops at 127.
    // 0.60 (a 2.13), per the user: "race gear needs to be way taller". At
    // 0.75 10th met the limiter right at the stripe and then sat on it.
    // Now 10th tops at 159 mph and the truck is POWER-limited around 145 on
    // 3000 hp (35 kN at the wheel vs ~30 kN of drag), so it never reaches the
    // limiter in top - still pulling at the line, which is the sound wanted.
    static constexpr float RACE_FD_SCALE = 0.60f;
    float g_fdScale = 1.0f;

static const float RAD;

    float roadDrag(float v_ms, float mass, float brake01);
    float tractionN(float mass) const;   // what the drive tyres can put down
    void  stepCoupled(float dt, float throttle, float jakeAmt, float brake01, float mass);

    // ---- state ----
    float g_rpm=0, g_throttle=0, g_load=0, g_speed=0, g_revSpeed=0, g_shiftCut=1;
    // Ladder mute for shutdown: 1 normally, ramped to 0 across ES_STOP so the
    // sim's own wind-down is not heard under stop.wav.
    float g_shutMute = 1.0f;
    // Fuel delivery frozen at the moment a shift starts, restored when it ends -
    // the browser's shift case bypasses its fuel lag entirely, so fuel comes
    // straight back instead of ramping up over ~420 ms and starving the turbine.
    float g_fuelHold = 0.0f;
    // True for the remainder of the frame in which a float shift ENGAGED, so
    // the frame-scope auto-shift test cannot fire a second shift immediately.
    bool  s_justEngaged = false;
    float g_jakeAmt=0, g_fuelDeliv=0, g_boostPsi=0, g_shaftHz=0, g_derate=1;
    int   g_state=ES_OFF, g_gear=1;
    bool  g_drive=false, g_reverse=false;
    float crankT=0; bool shifting=false; float shiftTimer=0; float idleWobble=0;
    // Time still owed OUT OF GEAR before the lever can go in. Counts down
    // alongside shiftTimer; engagement needs both this expired AND the revs
    // synced, so a shift can no longer complete the instant they line up.
    float neutralTimer=0;
    int   shiftTo=0;                        // gear being floated INTO (0 = none), like the sim's shiftTo
    // misfire chop (sim chopMiss): a lumpy cam physically shakes the crank, it isn't
    // just an audio effect. g_chopD is the shared depth the audio also renders from.
    float g_chopD=0, g_chopP=0, g_chopDur=0, g_chopAmp=1, g_fuelOnK=1;
    // Which cylinder of the six the chop pattern is on. A fixed table walked
    // in order is what makes it a cam-like gallop rather than a random
    // stumble - see the note at g_chopAmp in chopMiss().
    uint8_t g_chopCyl=0;
    uint32_t g_chopSeq=0; bool g_chopBark=false;
    float g_hunt=0, g_huntPh=0, g_huntHz=1.0f, g_huntSign=1.0f;
    bool  g_jakeRpmOk=false;               // jake low-rpm cut-out (hysteresis), see jakeEngaged()
    float g_chopDeep=0, g_lopeMess=0, g_lopeWant=0; uint32_t g_lopeStep=0; bool g_chopRhythm=false;
    // Two detuned random walks that make the lope APERIODIC - see the long
    // note in chopMiss(). A competition diesel hunts through its fuel pump;
    // it does not groove like a cammed gas engine.
    float g_lopeW1=0, g_lopeW2=0;
    float g_lopeOut=1.0f;        // smoothed output, the audible lope
    bool  g_chopHi=false;
    float chopMiss(float dt, float r, float thr);   // returns a TORQUE perturbation (Nm)
    // Lumpy idle from OVERSIZED INJECTORS - the ballistic-region effect, see
    // the long note on the implementation. Additive with the tuned lope: one
    // is a lope you dialled in, the other is hardware too big to meter idle.
    float injectorIdleChop(float r, float thr) const;
    float fuelAt(float rpm) const;                  // fuelMap interpolated (sim cellAt(fuel,r))
    bool  s_clutchLocked=false;
    // air
    float g_airPsi=0, g_brakeApplied=0, s_peakApplied=0; bool g_lowAir=false, g_springOn=true, g_compLoaded=true;
    // How far the driver has the service-brake pedal down while the jake is on
    // and the truck is below JAKE_BRAKE_MPH. Ramped, not stamped, so it reads
    // as a foot going down rather than a step input into the air relay.
    float g_jakePedal=0;
    float g_hissEvent=0; bool g_govUnload=false;
    float g_airSec=0;                       // secondary tank (fills off primary via check valve)
    float s_airSaved = -999.0f;             // last pressure written to NVS
    // fuel / temps / electrics (CAT C15 6NZ realism pass)
    float g_gph=0;                          // live fuel burn, gal/h
    float g_fuelGal=120.0f;                 // road tanks; starts full; persisted in NVS "fuel" (main.cpp)
    float g_raceFuelGal=10.0f;              // race fuel cell (race mode); persisted too
    float g_pyroF=AMBIENT_F;                // EGT pre-turbo
    float g_transF=AMBIENT_F;               // transmission temp
    float g_oilPsi=0, g_volts=12.5f;
    float g_boostPeakSeen=0;                // rolling max boost, for the BOV gate
    float g_chopTq=0;                       // chop torque, recomputed every tick
    float g_tShaft=0, g_tWg=0;              // S410G shaft rpm + wastegate
    float g_grade=0, g_gradePhase=0, g_realGrade=0; bool g_realGradeOn=false;        // rolling-terrain generator
    float crankDur=2.5f;                    // set from coolant temp at start()
    inline float coldFactor() const { return clampf((160.0f-g_coolantF)/120.0f, 0, 1); }
    inline float idleRpmNow() const { return IDLE_RPM + coldFactor()*110.0f; } // cold high idle
    // thermal
    float g_coolantF=AMBIENT_F, g_oilF=AMBIENT_F, g_health=100; bool g_fanOn=false;
    float g_extVolts=12.5f; bool g_haveVolts=false;   // real pack volts, if fed (setRealVolts)
    float g_pistonMelt=0, g_mechDamage=0; bool g_meltEvent=false;
    // live dyno
    bool  g_dynoOn=false; float g_dynoT=0, g_dynoRpm=1000, g_dynoThr=0, g_dynoHpNow=0;
    float g_dynoHp[DYNO_PTS]={0}, g_dynoTq[DYNO_PTS]={0}; int g_dynoN=0;
    float g_dynoPkHp=0, g_dynoPkRpm=0, g_dynoPkTq=0, g_dynoEgt=0, g_dynoBoost=0;
    bool  g_dynoSurge=false, g_dynoPcp=false;
    char  g_dynoRes[200]={0};
    void  dynoTick(float dt);
    void  dynoFinish(bool aborted);     // thermal / mechanical death
    int   g_deathCause=0;                                            // 0 = melt, 1 = over-pressure
    float g_rodBar=270.0f, g_meltF=1600.0f;                          // failure limits (set by parts)
    void  meltdown();                                                 // she lets go -> dead
    uint16_t g_warn=0; bool g_limiter=false;
    uint32_t rng=0x1234567;
    inline float frand(){ rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return (int32_t)rng*(1.0f/2147483648.0f); }
    inline float frand01(){ return frand()*0.5f + 0.5f; }   // 0..1
    // Re-roll this driver's shift points. Called the moment a change is
    // committed, so the NEXT one is never identical to the last.
    inline void rollDriver() {
        // RACE MODE WINDS EACH GEAR OUT TO THE REDLINE.
        //
        // The street point is 1930 +/- 90, so it changed gear at 1840-2020 -
        // well short of the 2100 limiter. Right for a loaded truck on the road
        // (shifting early saves fuel and there is nothing up there anyway), but
        // not how a drag run is driven: you pull each gear to the limiter and
        // THEN grab the next, because every rpm left on the table is speed you
        // do not carry into the following gear.
        //
        // 2080 is 20 under the limiter, so it uses the whole range without
        // sitting on the rev-limiter chop waiting for the shift. Checked what
        // it buys: the next gear now lands at ~1520 rpm instead of ~1420,
        // i.e. further up the band on every change. The jitter also shrinks -
        // a driver on a run is deliberate, where a street shift is casual.
        g_upPoint  = g_raceMode ? (2080.0f + frand() * 15.0f)
                                : (UPSHIFT_CENTRE + frand() * UPSHIFT_SPREAD);
        g_dnJitter = frand() * DOWNSHIFT_SPREAD;
    }

public:
    // one-shot events for the audio (reading clears)
    float takeBrakeHiss(){ float v=g_hissEvent; g_hissEvent=0; return v; }
    bool  takeGovUnload(){ bool b=g_govUnload; g_govUnload=false; return b; }
    bool  takeStartTrig(){ bool b=g_trigStart; g_trigStart=false; return b; }
    bool  takeStopTrig() { bool b=g_trigStop;  g_trigStop=false;  return b; }
private:
    bool   g_manual = false;
    int8_t g_manReq = 1;
    float  g_clutchPedal = 0;
    float  g_stopTime = 0, g_stopFrom = 0;   // full-stop-time brake (setStopTime)
    bool   g_stopLatched = false, g_stopEnabled = false;
    float  g_ptoLoad = 0;
    bool   g_parkBrake = false;
    bool g_trigStart=false, g_trigStop=false; bool g_trailer=false;
    float g_bHold=0, g_bPeak=0, g_prevThr=0, g_bovAmp=0, g_clutchHeat=0;
    // 32000 kg - a LOADED tractor-trailer, which is what this truck is.
    //
    // Was 8000, i.e. bobtail. That is why everything happened too fast:
    // MEASURED on the bench, a jake rundown from 62 mph took ELEVEN SECONDS
    // (5.5 mph/s), so each of ten gears got about one second and the whole
    // thing felt rushed however the shift points were tuned.
    //
    // The arithmetic: ~2440 Nm of jake retard at 1600 rpm through gear 9 is
    // 14.7 kN at the wheels.
    //     8000 kg -> 4.1 mph/s   (matches the measurement)
    //    32000 kg -> 1.0 mph/s   (a rundown takes 30+ s, real time per gear)
    // BASE_MASS = 37000 was already declared in this header and the physics
    // simply never used it.
    //
    // setLoadLevel() still offers 8000/20000/32000/42000 from the web UI, so
    // bobtail is one click away if it is ever wanted.
    float g_loadMass=32000.0f;              // kg, set by setLoadLevel()
    float g_tqRef=2400.0f;                  // decaying peak-torque memory -> dynamic clutch capacity (sim _tqRef)
    float g_wgFlutter=0, g_surgeAmt=0, g_surgeT=0, g_gateT=0;   // golden compressor-surge state
    float timAt(float rpm) const;
};

extern EngineSim Engine;
