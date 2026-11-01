// =============================================================================
//  RCLC D15 - main
// =============================================================================
//  Control loop ported from the Teensy build's main.cpp. The RC-to-sim mapping,
//  the gear hysteresis and the automatic turn signals are the same logic with
//  the same thresholds (now in Settings.h instead of the web blob).
//
//  What is deliberately absent compared to the Teensy firmware:
//    * the Teensy audio graph - audio is AudioOut, an I2S task (see AudioOut.h)
//    * Servos / ServoMap - this board has no servo bank, the receiver drives
//      them directly
//    * WebLink - no web UI yet; its tunables live in Settings.h
//  Hydraulics IS ported (work mode): the Teensy took its sticks from a PS4 pad,
//  which the S3 cannot pair, so here they are RC channels.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SD_MMC.h>

#include "Config.h"
#include "Settings.h"
#include "DcEeprom.h"
#include "EngineSim.h"
#include "ChanMap.h"
#include "RcInput.h"
#include "ServoOut.h"
#include "Battery.h"
#include <Preferences.h>
#include "Outputs.h"
#include "Lights.h"
#include "SoundPack.h"
#include "EngineAudio.h"
#include "AudioOut.h"
#include "WebUI.h"
#include "Trailer.h"
#include "Tilt.h"
#include "Suspension.h"
#include "Hydraulics.h"

Settings Cfg;

// ---- control state ---------------------------------------------------------
static float throttle01 = 0, brake01 = 0, jakeAmt = 0;
static float g_bestB = 0, g_bestF = 0, g_bestT = 0;   // 'D' dyno sweep result, applied by 'V'
static bool  inDrive = false, inReverse = false, engineOn = false;
static bool  bHead = false, bHigh = false, bHorn = false;
static bool  bBeacon = false, bHaz = false, bFog = false, bRoof = false, bCabin = false;
static int   bInd = 0;
static uint16_t g_steerPwm = 1500;     // raw steering stick, for the servo output
static bool  g_gestHaz = false;        // hazards latched by 3 taps to full right
static uint8_t g_webLight = 0;         // LIGHT_NEXT / 3 taps full left: 0..4
static uint8_t g_webJake  = 0;         // JAKE_CYCLE: 0 off, 1 LO, 2 HI
static bool  g_workBtn = false;        // WORK MODE button latch (cleared on leaving PARK)
static bool  g_hydDrive = false;       // work mode is using the throttle/steering sticks (last loop)
static float g_benchStick = 0.5f;      // bench: last 0-9 key, stands in for the Fn 1 stick in work mode
static uint32_t g_crankTraceUntil = 0;  // bench Q: start the engine and log the start sequence audio
static uint32_t g_rpmTraceUntil = 0;   // bench R: 20 Hz rpm trace (rev-feel tuning)
static bool  sdOk = false;
static uint32_t lastTickUs = 0;
static float tickHeat = 0.0f, tickTimer = 0.0f;   // cooling ticks after shutdown

// ---------------------------------------------------------------------------
static void banner() {
    Serial.println();
    Serial.println("=====================================================");
    Serial.println(" RCLC D15");
    Serial.printf (" PSRAM %u KB free, heap %u KB free\n",
                   (unsigned)(ESP.getFreePsram() / 1024),
                   (unsigned)(ESP.getFreeHeap() / 1024));
    if (ESP.getFreePsram() == 0) {
        Serial.println(" *** NO PSRAM. Check board_build.arduino.memory_type");
        Serial.println(" *** = qio_opi. This module is an N16R8 - OCTAL psram.");
    }
    Serial.println("=====================================================");
}

static void benchKeys() {
    if (!Serial.available()) return;
    char c = Serial.read();
    switch (c) {
        case 'e': engineOn = !engineOn; if (engineOn) Engine.start(); else { Engine.stop(); Engine.saveAir(); } break;
        case 'd': inDrive = true;  inReverse = false; break;
        case 'r': inReverse = true; inDrive = false;  break;
        case 'n': inDrive = false; inReverse = false; break;
        case 'j': jakeAmt = (jakeAmt > 0.5f) ? 0.0f : 1.0f; break;
        case ' ': throttle01 = 0; g_benchStick = 0.5f; break;
        case 'b': brake01 = (brake01 > 0.05f) ? 0.0f : 0.8f; break;
        case 'l': bHead = !bHead; break;
        case 'h': bHigh = !bHigh; break;
        case 'k': bBeacon = !bBeacon; break;
        case 'x': bHaz = !bHaz; break;
        case 'c': bCabin = !bCabin; break;
        case 'f': bFog = !bFog; break;
        case 'o': bRoof = !bRoof; break;
        case 'g': bHorn = !bHorn; Audio.setHorn(bHorn); break;
        case 'a': EngAudio.triggerAir(); break;   // air-brake dump one-shot
        case ',': bInd = (bInd == -1) ? 0 : -1; break;
        case '.': bInd = (bInd == 1)  ? 0 :  1; break;
        // ---- TUNE from the bench ------------------------------------------
        // The TUNE tab is the real interface for this, but the web UI only
        // serves on the board's own AP and this PC cannot join it while it is
        // on the house mesh. These let the tuning map be exercised over the
        // USB cable so the maths can be verified without the network.
        //   'M' = max-effort tune (all 8 cells fuel 1.6, timing 8 deg)
        //   'm' = back to stock
        //   'P' = print the map and what the core is doing with it
        // Race mode from the bench, so it can be tested without the AP.
        // 'G' arms it launching in 4th and holding 8th; 'g' disarms.
        // launch 2, skip to 5, hold 8. 4th bogged a 32 t launch to 6 rpm on
        // the bench, so start low and skip into the power band instead.
        // G = full competition: ~2300 hp at 6000 kg. The gear ladder alone
        // was never going to make it quick - 650 hp at 32 t is 0.020 hp/kg
        // against a real drag truck's 0.500.
        // Race keys drop the TRAILER rather than inventing a weight: load
        // level 0 is 8000 kg, a real bobtail tractor, which is what a drag
        // truck actually is. Hook a trailer back on from the UI and the run
        // gets honestly slower.
        case 'G': Engine.setRaceGears(5, 7, 10, 9); Engine.setRacePower(2);
                  Engine.setLoadLevel(0);
                  Engine.setRaceMode(true);
                  Serial.println("[race] ARMED 3000hp - launch 5 > 7 > 9 > 10"); break;
        // '(' / ')' trim the slipper clutch from the bench. NOT , / . - those
        // are already the turn indicators.
        // '(' / ')' set the CLUTCH RELEASE TIME in seconds - how long the
        // driver takes to let the pedal up on a launch. 0 is a dump (which is
        // what every earlier test did by accident), ~1.2 s feeds it in.
        case '(': Engine.setClutchRelease(Engine.clutchRelease() - 0.2f);
                  Serial.printf("[clutch] release %.1f s\n", Engine.clutchRelease());
                  break;
        case ')': Engine.setClutchRelease(Engine.clutchRelease() + 0.2f);
                  Serial.printf("[clutch] release %.1f s\n", Engine.clutchRelease());
                  break;
        // 'Q' - the time slip. 60 ft is the launch-quality number; the
        // quarter-mile ET and trap speed are the run.
        case 'Q': Serial.printf(
                    "[slip] 60ft %.2fs  1/4 %.2fs @ %.1f mph   dist %.0f ft  t %.1fs\n",
                    Engine.et60(), Engine.etQtr(), Engine.trapMph(),
                    Engine.dragDistFt(), Engine.dragTime());
                  break;
        case 'W': Engine.setRaceGears(5, 7, 10, 9); Engine.setRacePower(1);
                  Engine.setLoadLevel(0);
                  Engine.setRaceMode(true);
                  Serial.println("[race] ARMED ~1500hp class - launch 5 > 7 > 9 > 10"); break;
        case 'Y': Engine.setRacePower(0); Engine.setRaceMode(false);
                  Serial.println("[race] off"); break;
        case 'I': Engine.grantPart(4, 2);
                  Serial.printf("[inj] tier %u (dual)\n", Engine.parts.fuel); break;
        // 'S' = STOCK TRUCK: every part back to tier 0 and the STREET class.
        // Pair it with 'm' for the stock map. Needed because a "stock" run
        // with the dual injectors and the 3000hp multiplier still fitted is
        // not stock - it is a 3000hp truck with a lazy map, which is what the
        // first compare demo actually played.
        // 'H' = HEALTH. Prints the two things that silently refuse a start -
        // health <= 8 (spun a bearing / melted, start() returns) and dry
        // tanks (fuel <= 0.05 gal) - plus the melt and mechanical damage
        // behind them. Nothing on the bench showed these; a dead engine after
        // a max-tune session took four blind probes to explain.
        // 'N' = settings storage report: bytes held in the new "rclc" NVS area and
        // the pre-rename "dieselcore" one (proves the one-time migration happened)
        case 'N': {
            Preferences a, b; size_t na = 0, nb = 0;
            if (a.begin("rclc", true))       { na = a.getBytesLength("store"); a.end(); }
            if (b.begin("dieselcore", true)) { nb = b.getBytesLength("store"); b.end(); }
            Serial.printf("[nvs] rclc: %u bytes   dieselcore (old): %u bytes\n", (unsigned)na, (unsigned)nb);
            break;
        }
        // 'u' = battery report
        // the web page's Remote Buttons, from the bench
        case 'v': WebCtl.lightNext = true; break;
        case 'w': WebCtl.jakeCycle = true; break;
        case 'y': Trailer.hold(TR_LEGS_UP); break;      // one press = 0.7 s of legs-up
        case 'z': WebCtl.workToggle = true; break;      // WORK MODE button
        // '~' = a line from tools/lope_designer.html: ~key=val key=val ...
        case '~': {
            char line[160]; size_t n = Serial.readBytesUntil('\n', line, sizeof(line) - 1); line[n] = 0;
            bool ok = true;
            for (char *tok = strtok(line, " \r"); tok; tok = strtok(nullptr, " \r")) {
                char *eq = strchr(tok, '='); if (!eq) { ok = false; continue; }
                *eq = 0;
                // bench demo keys: injector tier, idle timing cells, the LOPE IDLE button
                if (!strcmp(tok, "inj"))       { Engine.grantPart(4, (uint8_t)constrain(atoi(eq + 1), 0, 2)); EEPROM.commit(); continue; }
                if (!strcmp(tok, "tim0"))      { Engine.timMap[0] = constrain((float)atof(eq + 1), -5.0f, 10.0f); continue; }
                if (!strcmp(tok, "tim1"))      { Engine.timMap[1] = constrain((float)atof(eq + 1), -5.0f, 10.0f); continue; }
                // part-table calibration: ff=a,b,c (flow) wgmul= tm= tc= ic= fc= rod= melt=  (NOT "wg" - that is the wastegate)
                {
                    extern float PT_FFLOW[3], PT_WGMUL[3], PT_TMULT[3], PT_TCEIL[3], PT_ICEFF[3], PT_FCAP[3], PT_RODBAR[3], PT_MELTF[3];
                    struct { const char *k; float *t; } T[] = {{"ff",PT_FFLOW},{"wgmul",PT_WGMUL},{"tm",PT_TMULT},{"tc",PT_TCEIL},
                                                               {"ic",PT_ICEFF},{"fc",PT_FCAP},{"rod",PT_RODBAR},{"melt",PT_MELTF}};
                    bool hit = false;
                    for (auto &e : T) if (!strcmp(tok, e.k)) { sscanf(eq + 1, "%f,%f,%f", &e.t[0], &e.t[1], &e.t[2]); Engine.applyParts(); hit = true; }
                    if (hit) continue;
                }
                // parts=r,p,t,i,f fits a build directly (bench calibration); dyno=1 pulls it
                if (!strcmp(tok, "parts")) {
                    int v[5] = {0,0,0,0,0}; sscanf(eq + 1, "%d,%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3], &v[4]);
                    uint8_t *pp = &Engine.parts.rods, *uu = &Engine.unlocks.rods;
                    for (int i = 0; i < 5; i++) { pp[i] = (uint8_t)constrain(v[i], 0, 2); if (uu[i] < pp[i]) uu[i] = pp[i]; }
                    Engine.saveParts(); EEPROM.commit(); continue;
                }
                if (!strcmp(tok, "dynopull")) { Serial.println(Engine.dynoStart() ? "[dyno] live pull started" : "[dyno] engine not running"); continue; }
                if (!strcmp(tok, "dyno")) {
                    EngineSim::DynoResult r = Engine.dynoRun();
                    Serial.printf("[dyno] parts %u%u%u%u%u  %.0f hp @ %.0f  boost %.0f psi  egt %.0fF  derate %d surge %d pcp %d blown %d\n",
                                  Engine.parts.rods, Engine.parts.pistons, Engine.parts.turbo, Engine.parts.ic, Engine.parts.fuel,
                                  r.hp, r.rpm, r.boostPsi, r.egtPeak, r.derated, r.surged, r.pcpOver, r.blown);
                    continue;
                }
                if (!strcmp(tok, "bench")) {   // cycles per op: float divide vs multiply vs x/(1+|x|)
                    volatile float a = 1.2345f, acc = 0.0f; float x = 0.37f;
                    uint32_t c0 = ESP.getCycleCount();
                    for (int i = 0; i < 1000; i++) { acc = acc + a / (x + (float)i); }
                    uint32_t c1 = ESP.getCycleCount();
                    for (int i = 0; i < 1000; i++) { acc = acc + a * (x + (float)i); }
                    uint32_t c2 = ESP.getCycleCount();
                    for (int i = 0; i < 1000; i++) { float y = a * (float)i * 0.001f; acc = acc + y / (1.0f + fabsf(y)); }
                    uint32_t c3 = ESP.getCycleCount();
                    for (int i = 0; i < 1000; i++) { acc = acc + expf(-a * (float)i * 0.001f); }
                    uint32_t c4 = ESP.getCycleCount();
                    Serial.printf("[bench] div %.1f  mul %.1f  softclip %.1f  expf %.1f cycles/op (%g)\n",
                                  (c1 - c0) / 1000.0f, (c2 - c1) / 1000.0f, (c3 - c2) / 1000.0f, (c4 - c3) / 1000.0f, (float)acc);
                    continue;
                }
                // GY-521: tilt=1 status, tiltlevel=1 (parked flat), tiltaxis=0..3 (+X -X +Y -Y = front), hill=0..1
                if (!strcmp(tok, "wg"))        { Engine.boostMax = constrain((float)atof(eq + 1), 15.0f, 120.0f); continue; }   // wastegate (bench)
                if (!strcmp(tok, "fuelgal"))   { Engine.setFuelGal((float)atof(eq + 1)); continue; }   // bench: set the tank
                if (!strcmp(tok, "danger"))    { if (atoi(eq + 1)) EngAudio.playDanger(); else EngAudio.stopDanger(); continue; }
                if (!strcmp(tok, "tilt"))      { Tilt.print(); continue; }
                if (!strcmp(tok, "tiltlevel")) { Tilt.learnLevel(); Tilt.print(); continue; }
                if (!strcmp(tok, "tiltaxis"))  { Tilt.axis = (uint8_t)constrain(atoi(eq + 1), 0, 3); Tilt.save(); Tilt.print(); continue; }
                if (!strcmp(tok, "hill"))      { Tilt.hill = constrain((float)atof(eq + 1), 0.0f, 1.0f); Tilt.save(); Tilt.print(); continue; }
                if (!strcmp(tok, "prof")) { extern void renderProfPrint(); renderProfPrint(); continue; }
                if (!strcmp(tok, "state")) {
                    Serial.printf("[eng] gph=%.1f wg=%.0f fuelOn=%.2f inj=%u tim0=%.2f tim1=%.2f lopeforce=%d mess=%.2f pulse=%.2f\n",
                                  Engine.fuelGph(), Engine.boostMax, Engine.fuelOnK(), Engine.parts.fuel, Engine.timMap[0], Engine.timMap[1], (int)Engine.lopeForce,
                                  Engine.lopeMess(), Engine.pulseLopeAmt());
                    continue;
                }
                if (!lopeSet(tok, eq + 1)) ok = false;
            }
            Serial.println(ok ? "[lope] ok" : "[lope] bad key");
            break;
        }
        case 'q': g_crankTraceUntil = millis() + 6000; if (!engineOn) { engineOn = true; Engine.start(); } break;
        case 'R': g_rpmTraceUntil = millis() + 7000; break;   // 20 Hz rpm trace for 7 s (rev-feel tuning)
        case 'X': Cfg.hydIn[0] = (Cfg.hydIn[0] == 1.0f) ? 4.0f : 1.0f; break;   // Fn 1: throttle stick <-> RX CH2
        case 'K': Cfg.hydMode = (float)(((int)Cfg.hydMode + 1) % 3); Serial.printf("[hyd] mode %d\n", (int)Cfg.hydMode); break;
        case 'L': Serial.printf("[trailer] %s  lights %u  jake %u  beacon %d  lope %d\n",
                                Trailer.json().c_str(), WebCtl.lightModeNow, WebCtl.jakeNow,
                                (int)WebCtl.beacon, (int)Engine.lopeForce);
                  Serial.printf("[hyd] mode %d work %d  psi %.0f  %.0fF  pos %.2f %.2f %.2f %.2f  pto %.2f  rpm %.0f\n",
                                (int)Cfg.hydMode, (int)Hyd.on(), Hyd.pressPsi(), Hyd.tempF(),
                                Hyd.pos(0), Hyd.pos(1), Hyd.pos(2), Hyd.pos(3), Hyd.ptoLoad(), Engine.rpm());
                  break;
        case 'u': Serial.printf("[batt] pack %.2f V (%.2f V/cell, %uS)  soc %.0f%%  BEC %.2f V  state %s  protection %s  trim %.3f\n",
                                Batt.volts(), Batt.cells() ? Batt.volts() / Batt.cells() : 0.0f, Batt.cells(),
                                Batt.soc() * 100.0f, Batt.bec(), Batt.stateName(),
                                Batt.active() ? "ACTIVE" : (Cfg.battProtect > 0.5f ? "waiting for ESC BEC" : "off"), Cfg.battTrim);
                  break;
        case 'H': Serial.printf("[health] %.1f%%  melt %.3f  fuel %.1f gal (%.0f%%)  coolant %.0fF  -> %s\n",
                                Engine.health(), Engine.pistonMelt(),
                                Engine.fuelGal(), Engine.fuelFrac() * 100.0f,
                                Engine.coolantF(),
                                Engine.health() <= 8.0f ? "WILL NOT START: rebuild (Z)"
                              : Engine.fuelGal() <= 0.05f ? "WILL NOT START: dry tanks (Z)"
                              : "ok to start");
                  break;
        // 'D' = DYNO SWEEP. The 'M' max-slider map (fuel 1.60, +8 everywhere)
        // MELTED the engine on the first pits-to-pits demo: health 0, melt
        // 1.000 after an 11.76 s pass. Max sliders is not max power - fuel
        // past the air is only heat. dynoRun() is const (no damage), so sweep
        // wastegate x fuel x timing, print every pull, and remember the best
        // one that LIVES (not blown, not thrown a rod). 'V' applies it.
        case 'D': {
            static const float BOOST[2] = { 95.0f, 120.0f };
            static const float FUEL[6]  = { 1.10f, 1.20f, 1.30f, 1.40f, 1.50f, 1.60f };
            static const float TIM[3]   = { 4.0f, 6.0f, 8.0f };
            float b0 = Engine.boostMax, f0[8], t0[8];
            for (int i = 0; i < 8; i++) { f0[i] = Engine.fuelMap[i]; t0[i] = Engine.timMap[i]; }
            float bestHp = 0; g_bestB = 0; g_bestF = 0; g_bestT = 0;
            Serial.println("[dyno]  wg    fuel  tim   ->    hp @ rpm    egtF  boost  flags (D hot, S surge, P rod, B BLOWN)");
            for (int bi = 0; bi < 2; bi++) for (int fi = 0; fi < 6; fi++) for (int ti = 0; ti < 3; ti++) {
                Engine.boostMax = BOOST[bi];
                for (int i = 0; i < 8; i++) { Engine.fuelMap[i] = FUEL[fi]; Engine.timMap[i] = TIM[ti]; }
                EngineSim::DynoResult r = Engine.dynoRun();
                bool lives = !r.blown && !r.pcpOver;
                Serial.printf("[dyno] %4.0f  %5.2f  %+3.0f   -> %5.0f @ %4.0f   %5.0f  %5.1f  %s%s%s%s%s\n",
                              BOOST[bi], FUEL[fi], TIM[ti], r.hp, r.rpm, r.egtPeak, r.boostPsi,
                              r.derated ? "D" : "", r.surged ? "S" : "", r.pcpOver ? "P" : "",
                              r.blown ? "B" : "", lives && r.hp > bestHp ? "  <- best so far" : "");
                if (lives && r.hp > bestHp) { bestHp = r.hp; g_bestB = BOOST[bi]; g_bestF = FUEL[fi]; g_bestT = TIM[ti]; }
            }
            Engine.boostMax = b0;
            for (int i = 0; i < 8; i++) { Engine.fuelMap[i] = f0[i]; Engine.timMap[i] = t0[i]; }
            Serial.printf("[dyno] BEST THAT LIVES: wg %.0f  fuel %.2f  tim %+.0f  = %.0f hp   (press V to apply)\n",
                          g_bestB, g_bestF, g_bestT, bestHp);
            break;
        }
        case 'V': if (g_bestF > 0) {
                      Engine.boostMax = g_bestB;
                      for (int i = 0; i < 8; i++) { Engine.fuelMap[i] = g_bestF; Engine.timMap[i] = g_bestT; }
                      Serial.printf("[tune] applied best-that-lives: wg %.0f fuel %.2f tim %+.0f\n", g_bestB, g_bestF, g_bestT);
                  } else Serial.println("[tune] run D first");
                  break;
        // 'Z' = SERVICE: full tanks + a fresh engine, same as the UI's REBUILD
        // plus a fill-up. Prints H afterwards so the state is visible.
        case 'Z': Engine.refuel(); Engine.rebuild();
                  Serial.printf("[service] refuelled + rebuilt: health %.0f%%  fuel %.0f%%\n",
                                Engine.health(), Engine.fuelFrac() * 100.0f);
                  break;
        case 'S': for (uint8_t i = 0; i < 5; i++) Engine.installPart(i, 0);
                  Engine.setRacePower(0);
                  EEPROM.commit();     // saveParts() only writes the buffer - it came back modded after a reboot
                  Serial.printf("[stock] rods %u pistons %u turbo %u ic %u inj %u  class street\n",
                                Engine.parts.rods, Engine.parts.pistons, Engine.parts.turbo,
                                Engine.parts.ic, Engine.parts.fuel);
                  break;
        case 'M': for (int i = 0; i < 8; i++) { Engine.fuelMap[i] = 1.60f;
                                                Engine.timMap[i]  = 8.0f; }
                  Serial.println("[tune] MAX: fuel 1.60, timing +8 all cells");
                  break;
        case 'm': { static const float ST[8] = {2.0f,2.5f,3.0f,3.5f,4.0f,4.5f,5.0f,5.5f};
                    for (int i = 0; i < 8; i++) { Engine.fuelMap[i] = 1.0f;
                                                  Engine.timMap[i]  = ST[i]; }
                    Serial.println("[tune] STOCK restored"); }
                  break;
        case 'P': Serial.print("[tune] fuel:");
                  for (int i = 0; i < 8; i++) Serial.printf(" %.2f", Engine.fuelMap[i]);
                  Serial.print("  tim:");
                  for (int i = 0; i < 8; i++) Serial.printf(" %.1f", Engine.timMap[i]);
                  Serial.println();
                  // PUBLIC accessors only. fuelAt()/timAt() are private, and
                  // torqueNm()/egtF()/fuelCapNow() do not exist - pyroF() is
                  // the EGT. Read the header rather than guessing the names.
                  Serial.printf("[tune] %.0f rpm  boost %.1f psi  egt %.0fF  "
                                "load %.2f\n",
                                Engine.rpm(), Engine.boostPsi(),
                                Engine.pyroF(), Engine.load());
                  break;
        // Master volume from the bench. The TUNE tab has a slider but the web
        // UI only serves on the board's own AP, so this is the only way to trim
        // it while listening over USB.
        case '[': Cfg.masterVol = fmaxf(0.0f, Cfg.masterVol - 0.05f);
                  Audio.setMasterVol(Cfg.masterVol);
                  Serial.printf("[vol] %.2f\n", Cfg.masterVol); break;
        case ']': Cfg.masterVol = fminf(1.0f, Cfg.masterVol + 0.05f);
                  Audio.setMasterVol(Cfg.masterVol);
                  Serial.printf("[vol] %.2f\n", Cfg.masterVol); break;
        // 'C' - what is the chop ACTUALLY doing? Reports the rendered
        // envelope swing in dB, strikes per second, and chopD. If swing is
        // near 0 dB the chop is not reaching the output at all; if hits/s is
        // near zero it is not firing; if chopD is 0 the physics never armed it.
        case 'C': {
            float mn, mx; uint32_t hits;
            static uint32_t lastC = 0;
            EngAudio.takeChopStats(mn, mx, hits);
            uint32_t now = millis();
            float secs = lastC ? (now - lastC) * 0.001f : 1.0f;
            lastC = now;
            Serial.printf("[chop] gain %.3f..%.3f = %.1f dB   %.1f hits/s   "
                          "chopD %.2f  amp %.2f  rpm %.0f\n",
                          mn, mx, 20.0f * log10f(fmaxf(mx, 1e-4f) / fmaxf(mn, 1e-4f)),
                          hits / fmaxf(secs, 0.1f), EngAudio.chopDepthDbg(),
                          Engine.chopAmpNow(), Engine.rpm());
            break;
        }
        case 'T': Lite.testAll(255); break;
        case 't': Lite.testClear(); break;
        default:
            if (c >= '0' && c <= '9') { throttle01 = (c - '0') / 9.0f; g_benchStick = throttle01; }
            break;
    }
}

static void readRc() {
    uint16_t thrPwm  = RC.ch(Chan.of(CH_THROTTLE));
    uint16_t gearPwm = RC.ch(Chan.of(CH_GEAR));

    throttle01 = (thrPwm > 1500) ? fminf((thrPwm - 1500) / 500.0f, 1.0f) : 0.0f;   // capped: some radios send >2000 us
    // Clamped: at a 988us endpoint this reached 1.27, so the last 27% of back
    // stick did nothing and escBrake left its documented 0..1 range.
    brake01    = (thrPwm < 1470) ? fminf((1470 - thrPwm) / 380.0f, 1.0f) : 0.0f;

    // P / R / D selector with hysteresis, so a stick sitting on a boundary
    // cannot chatter between states. RC manual shifting was left out on purpose:
    // you cannot hear the shift point from across a field, so the feedback the
    // feature depends on does not exist outdoors. The box shifts on physics.
    {
        // THREE POSITIONS, matching the beier firmware exactly:
        //     low  (below pwmGearPark + hys)      PARK
        //     mid  (the band between)             REVERSE
        //     high (above pwmGearForward - hys)   FORWARD
        //
        // This was inverted: low read as REVERSE and the middle band selected
        // nothing at all, so a 3-position switch gave park->reverse,
        // reverse->neutral, and no way to deliberately reach reverse.
        static int gearSel = 0;              // 0 park, 1 reverse, 2 forward
        static int gearPend = 0;             // position waiting to be confirmed
        static uint32_t gearPendMs = 0;      // when it was first seen
        float hy = Cfg.pwmGearHysteresis;

        int want;
        if      (gearPwm < Cfg.pwmGearPark + hy)    want = 0;   // PARK
        else if (gearPwm > Cfg.pwmGearForward - hy) want = 2;   // FORWARD
        else                                        want = 1;   // REVERSE

        // DEBOUNCE BY STABILITY, not by delay.
        //
        // The previous version gated on 'enough time since the LAST change',
        // which only postponed each transition instead of suppressing the
        // pass-through. Throwing park -> forward sweeps the stick through the
        // middle band, so it went:
        //   1. enters REVERSE band, debounce has long expired -> COMMITS to
        //      reverse, fires shPbOff, and setParkBrake(false) raises the air
        //      hiss event as well
        //   2. reaches FORWARD band 30 ms later, but the timer was just reset
        //      so it is blocked
        //   3. finally commits to forward
        // One throw, a reverse you never selected, and two or three sounds.
        // That is the 'too many sounds when the park brake is released'.
        //
        // Now a new position must be held CONTINUOUSLY for gearDebounceMs
        // before it is accepted. Sweeping through the middle band never
        // dwells there long enough, so it is simply never selected.
        // POWER-ON: take the switch where it IS. gearSel starts at 0 (PARK),
        // so with the switch in FORWARD the first frames walked PARK ->
        // FORWARD through the debounce and hissed. Seed it from the first
        // real reading instead - no transition, no sound.
        static bool gearSeeded = false;
        if (!gearSeeded) { gearSel = want; gearPend = want; gearSeeded = true; }
        if (want != gearSel) {
            if (want != gearPend) { gearPend = want; gearPendMs = millis(); }
            else if ((millis() - gearPendMs) > (uint32_t)Cfg.gearDebounceMs) {
                int prev = gearSel;
                gearSel = want;

                if (gearSel == 0 && prev != 0)
                    { /* sound moved to the Engine.parkBrake() edge - see loop() */ }
                // Leaving PARK does NOT hiss if the throttle is buried - this
                // is the gear-selector path, so it is the one that actually
                // fires on a park-to-drive launch. Same reasoning as the
                // Engine.parkBrake() edge below: a staged truck already has
                // the brakes off, and the hiss lands right on the launch.
                // Going INTO park always sounds.
                else if (gearSel != 0 && prev == 0 && throttle01 <= 0.85f)
                    { /* sound moved to the Engine.parkBrake() edge - see loop() */ }
            }
        } else gearPend = gearSel;      // settled; cancel any pending change

        inReverse = (gearSel == 1);
        inDrive   = (gearSel == 2);
        Engine.setParkBrake(gearSel == 0);
        Engine.setManual(false);
    }

    engineOn = (RC.ch(Chan.of(CH_ENGINE)) > 1600);
    jakeAmt  = (RC.ch(Chan.of(CH_JAKE))   > 1600) ? 1.0f : 0.0f;

    uint16_t lp = RC.ch(Chan.of(CH_LIGHTS));
    uint16_t st = RC.ch(Chan.of(CH_STEERING));
    uint16_t ap = RC.ch(Chan.of(CH_AUX));
    bHead   = lp > Cfg.pwmLightsOn;
    bHigh   = lp > 1800;
    bHorn   = RC.ch(Chan.of(CH_HORN)) > Cfg.pwmHornLow;
    // CH_BRAKE is NOT the park brake. The gear selector's PARK position is, the
    // same way the beier firmware does it - wiring both meant a CH_BRAKE switch
    // could release park while the gear lever still read PARK.
    bBeacon = ap > 1650;
    bHaz    = ap < 1350;

    g_steerPwm = st;

    // STEERING-HOLD TURN SIGNALS + GESTURES, as the page describes them:
    //   * hold the stick past the threshold for Hold Time -> that blinker
    //     latches; it cancels once you have been back near centre for 0.8 s
    //   * tap full right 3x (within ~1.2 s) -> hazards on/off
    //   * tap full left 3x -> cycle the lights (same as the LIGHTS button)
    // The old code blinked the instant you passed 150 us and latched for the
    // hold time AFTER - so every correction flashed an indicator, and the
    // gestures the page advertised did not exist.
    {
        uint32_t now = millis();
        static int lastInd = 0, side = 0;
        static uint32_t sideSince = 0, centreSince = 0;
        float thr = Cfg.steerSignalThreshold;
        int dir = (st > 1500 + thr) ? 1 : (st < 1500 - thr ? -1 : 0);
        if (Cfg.steerSignalEnabled > 0.5f && !g_hydDrive) {  // a crane on the drive sticks owns the steering stick
            if (dir != side) { side = dir; sideSince = now; }
            if (dir != 0 && now - sideSince >= (uint32_t)Cfg.steerSignalHoldMs) lastInd = dir;
            if (abs((int)st - 1500) < thr * 0.5f) { if (!centreSince) centreSince = now; }
            else centreSince = 0;
            if (lastInd && centreSince && now - centreSince > 800) lastInd = 0;

            // taps: in and out of FULL lock quickly, three in a row, same side
            static int  tapSide = 0, taps = 0;
            static bool atLock = false;
            static uint32_t lockIn = 0, lastTap = 0;
            int lock = (st > 1900) ? 1 : (st < 1100 ? -1 : 0);
            if (lock && !atLock) {
                atLock = true; lockIn = now;
                if (lock != tapSide) { tapSide = lock; taps = 0; }
            }
            if (!lock && atLock) {
                atLock = false;
                if (now - lockIn < 450) {                       // a tap, not a hold
                    if (now - lastTap > 1200) taps = 0;
                    taps++; lastTap = now;
                    if (taps >= 3) {
                        if (tapSide > 0) g_gestHaz = !g_gestHaz;
                        else             WebCtl.lightNext = true;
                        Serial.printf("[gesture] 3 taps %s -> %s\n", tapSide > 0 ? "right" : "left",
                                      tapSide > 0 ? (g_gestHaz ? "hazards ON" : "hazards OFF") : "lights next");
                        taps = 0;
                    }
                } else taps = 0;
            }
        } else {
            lastInd = 0;
        }
        bInd = lastInd;
    }
}

// CLUTCH PEDAL TRACE. I claimed the release ramp worked off an ET alone and
// never actually watched the pedal move, which is exactly the kind of
// unverified claim that wastes a bench session. This prints it while it is
// in motion so the ramp can be seen rather than assumed.
static void pedalTrace() {
    static uint32_t ms = 0;
    float cp = Engine.clutchPedalNow();
    if (cp > 0.001f && millis() - ms > 120) {
        ms = millis();
        Serial.printf("[pedal] %.2f  rpm %4.0f  %4.1fmph  g%d\n",
                      cp, Engine.rpm(), Engine.speedMph(), Engine.gear());
    }
}
static float g_escMin = 3000.0f;   // lowest ESC pulse since the last heartbeat
static void heartbeat() {
    static uint32_t last = 0;
    if (millis() - last < 1000) return;
    last = millis();
    // `drive` is the number to tune by: |signal * dm| going into the tanh.
    // Below 1.0 nothing is limiting. Around 1.0-1.3 the tanh is shaping peaks,
    // which is what it is for. Much past ~1.5 it flattens them and a class-D
    // turns that near-square top into crackle. `clip` is the percentage of
    // samples over 1.0 in the last block.
    // Shows the float shift in flight, so you can confirm from the bench that
    // the box is rowing gears and not just slurring between them:
    //     ">7"   floating UP into 7   (fuel cut, revs falling)
    //     "<5*"  floating DOWN into 5 (throttle blipped, revs rising)
    char shiftTag[6] = " ";
    if (Engine.inShift() && Engine.shiftTarget())
        snprintf(shiftTag, sizeof(shiftTag), "%c%d%s",
                 Engine.blipping() ? '<' : '>', Engine.shiftTarget(),
                 Engine.blipping() ? "*" : "");

    Serial.printf("[%s] rpm %4.0f  %4.1fmph  g%u%-5s load %.2f  boost %4.1f  "
                  "%5.1fF | esc %4.0f (min %4.0f)%s | pk %.2f  drive %.2f  clip %3.0f%%  cpu %3.0f%%  xrun %lu  %s\n",
                  RC.modeName(), Engine.rpm(), Engine.speedMph(), Engine.gear(),
                  shiftTag,
                  Engine.load(), Engine.boostPsi(), Engine.coolantF(),
                  Out.escMicros(), g_escMin, inReverse ? " R" : (brake01 > 0.05f ? " BRK" : ""),
                  Audio.peak(), EngAudio.takeDrive(), EngAudio.clipping() * 100.0f,
                  Audio.cpuFrac() * 100.0f, (unsigned long)Audio.underruns(),
                  Pack.loaded() ? "sampled" : "synth");
    g_escMin = 3000.0f;
}

// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(400);
    banner();

    EEPROM.begin();          // NVS-backed; must precede anything that reads it
    Chan.begin();

    Wire.begin(PIN_SDA, PIN_SCL, DC_I2C_HZ);
    analogReadResolution(12);
    Batt.begin();            // pack + BEC sense on IO4 / IO5
    Susp.begin();
    if (Tilt.begin())  Serial.printf("[tilt] GY-521 found @0x%02X - real hills on\n", Tilt.addr);

    if (Lite.begin(DC_PCA_ADDR, DC_PCA_HZ)) {
        Serial.printf("[lights] PCA9685 ok at 0x%02X, %d channels\n", DC_PCA_ADDR, LT_COUNT);
    } else {
        Serial.printf("[lights] PCA9685 NOT FOUND at 0x%02X\n", DC_PCA_ADDR);
        Serial.println("         If it answers at 0x41 you have a rev2 board.");
    }

    SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_DAT0);
    sdOk = SD_MMC.begin("/sdcard", true);         // true = 1-bit
    // The socket's card-detect contacts are not wired, so a failed mount is the
    // only way to know there is no card in. Not fatal - the placeholder engine
    // needs no samples.
    Serial.printf("[sd] %s\n", sdOk ? "mounted" : "no card / mount failed");

    // Read the whole pack into PSRAM, then leave the card alone. No streaming,
    // so from here on nothing in the audio path can be starved by a card access.
    if (sdOk) Pack.loadFromSD();
    lopeLoad();              // the pulse lope shaped on tools/lope_designer.html

    Engine.begin();
    {   Preferences p; p.begin("fuel", true);                    // the tank as you left it
        float g = p.getFloat("gal", -1.0f); p.end();
        if (g >= 0) Engine.setFuelGal(g); }
    Engine.loadParts();
    // Air tanks come back where they were parked. There is no RTC on this board,
    // so we cannot know how long it actually sat - pass 0 and let the tanks hold
    // what they held. That errs toward a truck that keeps its air, which is both
    // the common case (plugged in again the same session) and the harmless one:
    // worst case you start with more air than a week in the cold would leave you.
    Engine.restoreAir(0.0f);
    Engine.setLoadLevel(Cfg.trailerLoad);

    Out.begin();                                  // holds neutral 1.2s to arm

    if (Audio.begin()) {
        Audio.setMasterVol(Cfg.masterVol);
        Serial.printf("[audio] I2S up, %d Hz, render on core %d\n",
                      (int)DC_OUT_RATE, DC_AUDIO_CORE);
        if (Pack.loaded()) {
            Serial.printf("[audio] sampled engine, %s (%lu KB resident)\n",
                          Pack.name(), (unsigned long)(Pack.bytesResident() / 1024));
        } else {
            Serial.println("[audio] no pack - synthesised engine (put a pack on the card)");
        }
    } else {
        Serial.println("[audio] I2S FAILED to start");
    }

    RC.begin();
    Servos.begin();          // CONTROL tab outputs; must follow RC.begin()
    Hyd.begin();             // cylinders come back where they were left

    // WiFi last: the audio task is already running and pinned to core 0, so the
    // radio comes up on core 1 alongside this loop and should not disturb it.
    // Watch `xrun` in the heartbeat - if it starts climbing once a phone
    // connects, this is the first suspect.
    if (Web.begin()) {
        Serial.printf("[web] AP \"RCLC-D15\" up, open http://%s/\n", Web.ip());
        Serial.println("[web] password truckin123");
        Trailer.begin();     // ESP-NOW rides the AP's channel 1
    } else {
        Serial.println("[web] AP failed to start - truck runs fine without it");
    }

    Serial.println("bench keys: 0-9 throttle  space idle  d/r/n gear  e engine  b brake");
    Serial.println("            j jake  l head  h high  k beacon  x hazard  c cab  f fog");
    Serial.println("            o roof  g horn  a air  , / . indicators  T/t light test");
    lastTickUs = micros();
}

void loop() {
    RC.poll();

    uint32_t nowUs = micros();
    float dt = (nowUs - lastTickUs) * 1e-6f;
    if (dt < 0.002f) return;                  // ~500 Hz ceiling on the sim tick
    if (dt > 0.100f) dt = 0.100f;             // never integrate a huge step
    lastTickUs = nowUs;

    bool rc = RC.active();
    if (rc) {
        readRc();
    } else if (RC.everSaw()) {
        // SIGNAL LOST after we had one. Without this we fell through to
        // benchKeys(), which only changes anything if a serial byte arrives - so
        // throttle01 and inDrive KEPT their last values and Out.drive() carried
        // on commanding the ESC. Unplug the receiver at 70% throttle in Drive
        // and the truck drove away until the battery died.
        RC.failsafe();
        throttle01 = 0.0f; brake01 = 1.0f;     // coast down with the brakes on
        inDrive = false; inReverse = false;
        jakeAmt = 0.0f; bHorn = false;
    } else {
        benchKeys();                           // never had a receiver: bench use
    }

    // engine on/off edge
    {
        static bool wasOn = false;
        if (engineOn && !wasOn) Engine.start();
        if (!engineOn && wasOn) { Engine.stop(); Engine.saveAir(); }
        wasOn = engineOn;
    }

    // throttle expo: 0 = linear, 1 = full cubic (standard RC curve)
    {
        float e = Cfg.throttleExpo, t = throttle01;
        if (e > 0.001f) throttle01 = e * t * t * t + (1.0f - e) * t;
    }

    // Every pass, not just at boot. Setting this once in setup() meant the
    // Trailer Load selector changed the saved value and the label while the
    // truck kept hauling whatever it booted with until a power cycle.
    // ---- web page buttons (Remote Buttons card) ----
    if (WebCtl.lightNext) { WebCtl.lightNext = false; g_webLight = (g_webLight + 1) % 5; }
    {   // THE JAKE: ON/OFF, LAST CHANGE WINS. It was "whichever asks for more"
        // with a 3-step page button (off/LO/HI), so the page could never turn
        // off a jake the radio switch held on, and a second tap went to HI
        // instead of off (Mat: "the jake button does not turn the jake off").
        // Now flipping the radio switch sets it, tapping the page toggles it.
        static float rcPrev = -1.0f; static bool jakeOn = false;
        if (rcPrev < 0.0f) rcPrev = jakeAmt;                    // first pass: no edge
        if (jakeAmt != rcPrev) { jakeOn = jakeAmt > 0.5f; rcPrev = jakeAmt; }   // the radio switch moved
        if (WebCtl.jakeCycle) { WebCtl.jakeCycle = false; jakeOn = !jakeOn; }  // the page button
        jakeAmt = jakeOn ? 1.0f : 0.0f;
        g_webJake = jakeOn ? 2 : 0;
        WebCtl.jakeNow = jakeOn ? 2 : 0;
    }

    // ---- WORK MODE (hydraulics on the servo outputs) ----
    // Two ways to run, decided by which sticks the functions use:
    //  * a function on the THROTTLE or STEERING stick (a crane worked from the
    //    drive sticks): PARK only, the ESC stays at neutral, and an automatic
    //    PTO throttle lifts the revs with demand.
    //  * every function on its OWN channel (a dump box on the right stick):
    //    the truck still drives like a real one. Pump flow follows the real
    //    engine revs - rev it and the box comes up faster - and only when
    //    standing in neutral/park does the PTO lift the idle for you.
    float hydCmd[4] = { 0, 0, 0, 0 };
    bool  hydUsesDrive = false;
    {
        Hyd.setMode((int)Cfg.hydMode);
        for (int i = 0; i < Hyd.nFn(); i++) {
            int src = (int)Cfg.hydIn[i];
            if (src == 1 || src == 2) hydUsesDrive = true;
        }
        bool parked = !inDrive && !inReverse && Engine.speedMph() < 0.5f;
        if (WebCtl.workToggle) { WebCtl.workToggle = false; g_workBtn = !g_workBtn; }
        if (hydUsesDrive && !parked) g_workBtn = false;
        bool want;
        if (Cfg.hydSwitch >= 16.5f)      want = true;                    // always engaged
        else if (Cfg.hydSwitch >= 1.0f)  want = RC.active() && RC.ch((uint8_t)((int)Cfg.hydSwitch - 1)) > 1600;
        else                             want = g_workBtn;
        {   // PTO engage: the little air-shift puff of the PTO solenoid
            bool on = want && (parked || !hydUsesDrive) && Hyd.nFn() > 0;
            if (on && !Hyd.on() && engineOn) EngAudio.playShot(&Pack.air, Cfg.volAirbrake * 0.18f, DC_SHOT_AIR);
            Hyd.setOn(on);
        }

        if (Hyd.on()) {
            float benchThr = g_benchStick;                 // bench: 0-9 keys drive Fn 1 (0 down, 9 up)
            for (int i = 0; i < 4; i++) {
                int src = (int)Cfg.hydIn[i];
                uint16_t us = 1500;
                if (RC.active()) {
                    if (src == 1)      us = RC.ch(Chan.of(CH_THROTTLE));
                    else if (src == 2) us = RC.ch(Chan.of(CH_STEERING));
                    else if (src >= 3) us = RC.ch((uint8_t)(src - 3));
                }
                if (us < 800 || us > 2200) us = 1500;          // no signal on that channel
                if (i == 0 && !RC.everSaw()) us = (uint16_t)(1050 + benchThr * 900.0f);
                float c = ((int)us - 1500) / 450.0f;
                const float DB = 0.08f;                        // stick deadband
                c = (fabsf(c) < DB) ? 0.0f : (c - (c > 0 ? DB : -DB)) / (1.0f - DB);
                c = c < -1 ? -1 : (c > 1 ? 1 : c);
                if (Cfg.hydInv[i] > 0.5f) c = -c;
                hydCmd[i] = c;
            }
            if (hydUsesDrive) {
                throttle01 = engineOn ? 0.10f + 0.25f * Hyd.demand() : 0.0f;   // PTO throttle
                brake01 = 0.0f;
                bInd = 0;
            } else if (!inDrive && !inReverse && engineOn) {
                // standing still: PTO high idle while the box is working
                float pto = 0.08f + 0.22f * Hyd.demand();
                if (throttle01 < pto) throttle01 = pto;
            }
        }
    }

    // ---- GY-521 tilt (optional, accessory header): real hills + rollover cut ----
    Tilt.update();
    // ---- active front suspension (2 servos, SuperScale-style) ----
    {   static float lastMph = 0, accG = 0;
        float mph = Engine.speedMph();
        if (dt > 0.0005f) accG += ((mph - lastMph) * 0.44704f / 9.81f / dt - accG) * fminf(1.0f, dt / 0.08f);
        lastMph = mph;
        float steer = RC.active() ? constrain(((int)g_steerPwm - 1500) / 500.0f, -1.0f, 1.0f) : 0.0f;
        Susp.update(dt, steer, mph, inReverse ? -accG : accG, Engine.engineTorqueNm() / 2900.0f);   // / stock peak torque (EngineSim PEAK_TORQUE)
    }
    Engine.setRealGrade(Tilt.grade(), Tilt.present());
    if (Tilt.rolledOver()) { throttle01 = 0.0f; brake01 = 1.0f; }   // on its side: no drive until it's back up


    Engine.setLoadLevel(Cfg.trailerLoad);
    Engine.setStopTime(Cfg.fullStopTime);
    Engine.update(dt, throttle01, inDrive, inReverse, jakeAmt, 1.0f, brake01);
    Hyd.tick(dt, Engine.rpm(), Engine.running(), hydCmd);
    Engine.setPtoLoad(Hyd.ptoLoad());
    g_hydDrive = Hyd.on() && hydUsesDrive;
    // Cat 323 pump loop swells with stick demand; relief = the oil-flow recording
    EngAudio.setHydraulics(Hyd.on(), Hyd.demand(), Hyd.relief());
    // the cylinders: each function's servo follows its simulated ram position,
    // and HOLDS it when work mode is off (a parked boom stays up)
    for (int i = 0; i < 4; i++) {
        int h = (int)Cfg.hydSrv[i] - 1;
        if (i < Hyd.nFn() && h >= 0) Servos.claim(1 + i, (int8_t)h, 1000 + (int)(Hyd.pos(i) * 1000.0f));
        else                         Servos.claim(1 + i, -1, 1500);
    }

    // ---- engine one-shots + cooling ticks ----
    // EngineSim has been raising these every frame since the port and nothing
    // was listening. The gain trims are level correction, not arbitrary: these
    // WAVs span 54 dB (stop.wav is -33 dBFS RMS, pbrakeon is -7.4), and the
    // Teensy played every one at 1.0 - which is why its stop was inaudible.
    // ---- THREE-PART START, ported from the browser's startSequence() --------
    // The browser does NOT play one fixed start clip. It plays the starter,
    // LOOPS a cranking sample while the engine churns, rolls for a light-off
    // every 220 ms, and only on the catch stops the loop and plays fireup. So
    // the startup is a different length every time, and a cold or worn engine
    // audibly churns longer. One 2.0 s clip could never do that, which is why
    // the board's start sounded short and samey next to the browser.
    //
    //   p = (0.18 + t*0.35)                     odds climb as you crank
    //       * (1 - min(0.65, (melt+mech)*0.8))  a worn engine resists
    //       * (0.45 + 0.55*warm)                a cold one resists
    //   t >= 9 s forces it, so you can never be stuck cranking.
    static int   seqState = 0;        // 0 idle, 1 starter, 2 cranking
    static float seqT = 0, seqRoll = 0, seqWorn = 0, seqWarm = 0;

    bool haveSeq = Pack.shStarter.ok() && Pack.shCrank.ok() && Pack.shFireup.ok();

    if (Engine.takeStartTrig()) {
        if (haveSeq) {
            // Hold the sim in CRANK until the catch decides. crankDur is set
            // long here and shortened the moment it lights.
            Engine.setCrankDur(9.5f);
            seqState = 1; seqT = 0; seqRoll = 0;
            seqWorn  = EngineSim::clampf(Engine.pistonMelt() +
                                         (100.0f - Engine.health()) * 0.01f, 0, 1);
            seqWarm  = EngineSim::clampf(tickHeat, 0, 1);
            EngAudio.playShot(&Pack.shStarter, Cfg.volStartup * 2.50f, DC_SHOT_ENG);
        } else {
            // Single-file fallback: match the crank to the clip as before.
            if (Pack.shStart.ok() && Pack.shStart.rate > 1.0f)
                Engine.setCrankDur((float)Pack.shStart.len / Pack.shStart.rate);
            EngAudio.playShot(&Pack.shStart, Cfg.volStartup * 2.50f, DC_SHOT_ENG);
        }
        tickHeat = 0.0f;
    }

    if ((int32_t)(g_crankTraceUntil - millis()) > 0) {
        static uint32_t tq = 0;
        if (millis() - tq >= 20) { tq = millis(); char vb[120]; EngAudio.voiceDbg(vb, sizeof(vb));
            Serial.printf("[crank] %lu st %d t %.2f rpm %.0f shot %.3f drive %.3f |%s\n", (unsigned long)millis(), seqState, seqT,
                          Engine.rpm(), EngAudio.takeShotPeak(), EngAudio.takeDrive(), vb); }
    }
    if (seqState) {
        seqT += dt;
        if (seqState == 1) {
            // browser: beginCrank at starterBuf.duration * 0.35
            // 0.75, not 0.35. The browser's 0.35 is a fraction of ITS starter
            // clip; ours is 1.28 s, so 0.35 put the looping crank in at 0.45 s
            // while the starter was still winding up - two starter sounds
            // overlapping, which is most of the 'weird sound'. 0.75 brings the
            // crank in at 0.96 s, as the starter settles into its churn.
            float engageAt = 0.75f * (float)Pack.shStarter.len / Pack.shStarter.rate;
            if (seqT >= engageAt) {
                EngAudio.playShot(&Pack.shCrank, Cfg.volStartup * 2.20f,
                                  DC_SHOT_CRANK, true);   // LOOPED
                seqState = 2; seqT = 0; seqRoll = 0;
            }
        } else {
            seqRoll += dt;
            if (seqRoll >= 0.220f) {                      // browser rolls every 220 ms
                seqRoll = 0;
                float pr = (0.18f + seqT * 0.35f)
                         * (1.0f - fminf(0.65f, seqWorn * 0.8f))
                         * (0.45f + 0.55f * seqWarm);
                if (seqT >= 9.0f) pr = 1.0f;              // always lights eventually
                if ((float)random(10000) / 10000.0f < pr) {
                    // Release the crank loop AND the starter together - the
                    // starter motor disengages the moment it fires. Both go
                    // out on the 3 ms ramp rather than being cut.
                    EngAudio.stopSlot(DC_SHOT_CRANK);
                    EngAudio.stopSlot(DC_SHOT_ENG);
                    // Own slot: sharing DC_SHOT_ENG meant this released the
                    // starter mid-sample instead of letting it ramp out.
                    EngAudio.playShot(&Pack.shFireup, Cfg.volStartup * 2.00f, DC_SHOT_FIRE);
                    Engine.catchNow();                    // let the sim come up to idle
                    seqState = 0;
                }
            }
        }
    }
    // A piston crown letting go played the ordinary shutdown sample, because
    // takeMeltEvent() had no caller anywhere in the tree. Read it before the
    // stop trigger so a meltdown can be told apart from a normal key-off.
    bool melted = Engine.takeMeltEvent();
    if (Engine.takeStopTrig()) {
        EngAudio.playShot(&Pack.shStop,
                          Cfg.volShutdown * (melted ? 26.00f : 18.00f), DC_SHOT_ENG);
        tickHeat = 1.0f; tickTimer = 1.6f;   // first tick lands inside stop.wav
    }
    {   float ba = Engine.takeBovAmp();
        if (ba > 0.01f)
            EngAudio.playShot(&Pack.shBov,
                              EngineSim::clampf(ba, 0, 2.2f) * 1.15f * 0.75f * Cfg.volBov,
                              DC_SHOT_BOV);   // NOT _ENG: it was cutting start/stop
    }
    if (tickHeat > 0.05f && Pack.nTick > 0) {
        tickTimer -= dt;
        if (tickTimer <= 0.0f) {
            // The shipped ticks peak at only 1.5% of full scale, so they need a
            // big makeup gain - they want re-baking and this is the stopgap.
            //
            // x40 was too much: with the default volTick of 0.55 that is an
            // effective 22.0, and the cooling ticks came out louder than they
            // should sit against a dead engine. x22 puts them at 12.1, about
            // -5 dB, which is present without snapping.
            //
            // Scaled HERE and not in volTick because volTick is a saved
            // setting - changing its default would not move an EEPROM that
            // already holds the old value, and would silently overwrite a
            // level set from the web UI.
            EngAudio.playShot(&Pack.shTick[(int)(millis() % (uint32_t)Pack.nTick)],
                              Cfg.volTick * 22.0f, DC_SHOT_TICK);
            tickHeat *= 0.94f;
            tickTimer = (0.3f + (millis() % 700) * 0.001f) / fmaxf(tickHeat, 0.15f);
            if (tickTimer > 6.0f) tickTimer = 6.0f;
        }
    }

    // ---- ESC ----
    // The air brake inside EngineSim already bleeds road speed and the ESC
    // follows that speed, so sound and wheels agree without any clamp hack.
    // escBrake only lets the ESC ramp DOWN faster while braking / on the jake;
    // it never takes the pulse below neutral (that was reverse on a fwd/rev ESC
    // and spun the motor backwards). Cfg.dragBrake is no longer used.
    float escBrake = 0.0f;
    if (!inReverse) {
        escBrake = brake01;
        if (Engine.jakeRetarding() && Engine.jakeAmt() > escBrake) escBrake = Engine.jakeAmt();
    }
    Out.drive(Engine.speedFrac(), Engine.reverseFrac(), inReverse, escBrake);
    if (Out.escMicros() < g_escMin) g_escMin = Out.escMicros();   // lowest pulse this second (heartbeat)

    // ---- lights ----
    bool braking = brake01 > 0.05f || Engine.jakeRetarding();
    // Light mode is a rotary switch: 0 OFF, 1 MARK, 2 ROOF, 3 HEAD, 4 FULL.
    // The RC lights channel and the page's LIGHTS button each pick one; the
    // higher wins, so the button can add lights but never kill the radio's.
    uint8_t rcLight = bHead ? (bHigh ? 4 : 3) : 0;
    uint8_t lightMode = rcLight > g_webLight ? rcLight : g_webLight;
    WebCtl.lightModeNow = lightMode;
    Lite.aux1 = (AuxFunc)constrain((int)Cfg.aux1Func, 0, 6);
    Lite.aux2 = (AuxFunc)constrain((int)Cfg.aux2Func, 0, 6);

    LightState ls;
    ls.running  = engineOn;
    ls.park     = lightMode >= 1;
    ls.head     = lightMode >= 3;
    ls.highBeam = lightMode >= 4;
    ls.braking  = braking;
    ls.reverse  = inReverse;
    ls.indicate = bInd;
    ls.hazard   = bHaz || g_gestHaz || Batt.hazard();    // battery cutoff = hazards on
    ls.fog      = bFog;
    ls.roof     = bRoof || lightMode >= 2;
    ls.beacon   = bBeacon || WebCtl.beacon;
    ls.cabin    = bCabin;
    ls.combinedRear = Cfg.combinedRearLights > 0.5f;
    Lite.update(ls, millis());

    // ---- wireless trailer: mirror what the truck's lamps are showing ----
    {
        uint8_t tail = Lite.level8(LT_TAIL);       // tail + brake
        if (Lite.aux1 == AUX_BRAKE) tail = max(tail, Lite.level8(LT_AUX1));   // brake moved to an AUX
        if (Lite.aux2 == AUX_BRAKE) tail = max(tail, Lite.level8(LT_AUX2));
        // the FRONT indicator channels: a pure blink even in American combined
        // rear mode, so the trailer board decides how its own rear lamps combine
        Trailer.update(tail, Lite.level8(LT_RUN), Lite.level8(LT_REV),
                       Lite.level8(LT_TURNL), Lite.level8(LT_TURNR),
                       ls.beacon, Cfg.trailerSync > 0.5f);
        if (Trailer.takeLinkedEdge()) {
            Serial.printf("[trailer] linked %s\n", Trailer.macStr().c_str());
            if (Cfg.trailerAir > 0.5f)                 // gladhands on: air rush
                EngAudio.playShot(&Pack.air, Cfg.volCoupling * 0.6f, DC_SHOT_AIR);
        }
    }

    // ---- steering servo output (iBUS/SBUS receivers only) ----
    {
        float n = (RC.active() && !g_hydDrive) ? ((int)g_steerPwm - 1500) / 500.0f : 0.0f;   // wheels straight while slewing
        n = n < -1 ? -1 : (n > 1 ? 1 : n);
        if (Cfg.steerInv > 0.5f) n = -n;
        float us = n < 0 ? Cfg.steerCtr + n * (Cfg.steerCtr - Cfg.steerMin)
                         : Cfg.steerCtr + n * (Cfg.steerMax - Cfg.steerCtr);
        Servos.steer((int8_t)((int)Cfg.steerOut - 1), (int)us);
    }

    // ---- audio ----
    // THE DOWNSHIFT BLIP. The gearbox shifts itself, but it has to sound like
    // someone is rowing it, and a driver coming DOWN a gear blips the throttle
    // to bring the revs up to meet it. The sim does exactly this:
    //     max(throttle, (shiftT>0 && shiftTo < gear) ? 0.85 : 0)
    // Without it the exhaust just went quiet through every downshift, because
    // the float-shift branch zeroes load - which is right for an UPshift (foot
    // off, fuel cut, revs fall) and wrong for a downshift, where the engine is
    // being driven UP against nothing. That asymmetry is most of what makes a
    // 13-speed recognisable, and we had neither half of it reaching the audio.
    // NO BLIP. The browser feeds the exhaust 0.85 throttle while floating DOWN a
    // gear - a driver bringing the revs up to meet a lower gear - and I ported
    // that. On the board it does not read as a blip, it reads as the engine
    // surging back up mid-shift and it fights the let-off, which is the half
    // that actually sounds like someone shifting. Removed on request: the shift
    // is now a clean lift in BOTH directions.
    float audLoad = Engine.load();
    Audio.setState(Engine.rpm(), audLoad, Engine.boostPsi(),
                   Engine.jakeRetarding(), engineOn);
    Audio.setHorn(bHorn);

    // ---- action one-shots ----
    {   static bool hornPrev = false;          // on the press, not held
        // NO press one-shot here. EngineAudio already loops Pack.horn
        // continuously while hornOn is set, so firing the same recording as a
        // one-shot as well played two copies of it at unrelated phases - the
        // exact flanging this renderer is built to avoid everywhere else - at
        // 1.2x gain for the first second and 0.9x after. The held loop is the
        // one that supports holding the button down, so it wins.
        hornPrev = bHorn;

        static bool parkPrev = false;
        bool parkNow = Engine.parkBrake();
        // POWER-ON IS NOT A BRAKE EVENT - and "power-on" ends when the RECEIVER
        // is up, not at a fixed time. The first fix muted 2.5 s after boot, but
        // RC auto-detect (iBUS -> SBUS -> PWM in 400 ms windows) can take longer
        // than that; until it locks, the gear switch has never been read, so the
        // moment it reported PARK the board saw a fresh "park brake set" and
        // played it (Mat: battery in + ESC on -> park brake sound). Now: silent
        // tracking while there is no receiver signal and for 1.5 s after it first
        // arrives. Bench use (never had a receiver) still sounds normally.
        static uint32_t rcUpSince = 0;
        bool rcNow = RC.active();
        if (!rcNow) rcUpSince = 0;
        else if (!rcUpSince) rcUpSince = millis();
        bool settling = (RC.everSaw() && (!rcNow || millis() - rcUpSince < 1500)) || millis() < 2500;
        if (settling) parkPrev = parkNow;
        if (parkNow != parkPrev) {
            // NO PARK-BRAKE HISS ON A STAGED LAUNCH.
            //
            // Going straight from park to a wide-open launch fired shPbOff
            // right as the truck left, which is wrong twice over: a drag truck
            // is already staged with the brakes off, and the hiss steps on the
            // one moment the engine note matters most.
            //
            // Only suppressed on the RELEASE, and only while the throttle is
            // actually buried - setting the park brake still always sounds,
            // and a normal release at idle still hisses as it should. So this
            // cannot hide a real brake event; it only declines to play one
            // over the top of a launch.
            // THE ONLY PARK-BRAKE SOUND. The RC gear-selector path used to play
            // its own copy too, so every PARK change on the transmitter hissed
            // TWICE. And per the user: no park-brake sounds in RACE MODE at all
            // (staging and pit stops included), nor on a full-throttle launch.
            bool stagedLaunch = (!parkNow && throttle01 > 0.85f);
            Serial.printf("[air] park brake %s%s\n", parkNow ? "SET" : "released", (stagedLaunch || Engine.raceMode()) ? " (muted)" : "");
            if (!stagedLaunch && !Engine.raceMode())
                EngAudio.playShot(parkNow ? &Pack.shPbOn : &Pack.shPbOff,
                                  Cfg.volParkbrake * 0.30f, DC_SHOT_PARK);
        }
        parkPrev = parkNow;

        // Compressor cut-out at 120 psi - the dryer blows off. EngineSim has
        // raised this since day one and nothing ever listened.
        // Dryer purge and brake hiss play in EVERY mode (Mat: "i still need to
        // hear the brakes, there was just a bug"). The bug was the hiss firing
        // ~20 times per release - fixed in EngineSim's peak tracking. Only the
        // PARK-brake sound is muted in race mode / on a staged launch (above).
        // Every air event is logged so odd behaviour can be read off serial.
        if (Engine.takeGovUnload()) {
            Serial.printf("[air] dryer purge at %.0f psi\n", Engine.airPsi());
            EngAudio.playShot(&Pack.shAirDry, Cfg.volAirDry * 0.28f, DC_SHOT_AIR);
        }

        // Scaled by how hard you were actually on the brakes: a dab gives a
        // short pfft, a hard stop gives the full blast.
        float hiss = Engine.takeBrakeHiss();
        if (hiss > 0.05f) {
            Serial.printf("[air] brake release hiss %.2f  (park %d, gear %s, thr %.2f, brk %.2f)\n", hiss,
                          (int)Engine.parkBrake(), inDrive ? "F" : (inReverse ? "R" : "P"), throttle01, brake01);
            EngAudio.playShot(&Pack.air, Cfg.volAirbrake * 0.75f * hiss, DC_SHOT_AIR);
        }
    }
    Audio.setBackup(inReverse);

    Web.poll();
    // ---- battery = fuel -------------------------------------------------
    Batt.poll();
    // BATTERY PROTECTION CUT IN -> "Shake Hands with Danger", once per cut-off
    {   static BattState prevBs = BATT_NONE;
        BattState bs = Batt.state();
        if (bs == BATT_EMPTY && prevBs != BATT_EMPTY) EngAudio.playDanger();
        prevBs = bs; }
    // THE FUEL GAUGE IS THE REAL TANK (Mat 2026-10-31). The battery only protects
    // the pack: at its cut-off the engine starves and stops (setStarve below),
    // but it no longer drives the gauge. The tank empties by what the injectors
    // spray, and REFUEL fills it. Saved so it survives a power cycle: on every
    // shutdown and refuel, and every 3 minutes while running (flash writes are
    // kept rare - each one briefly stalls the audio cache).
    {   static bool wasRun = false; static uint32_t lastSave = 0; static float lastGal = -1;
        bool run = Engine.running(); float gal = Engine.fuelGal();
        bool refuelled = lastGal >= 0 && gal > lastGal + 1.0f;
        if ((wasRun && !run) || refuelled || (run && millis() - lastSave > 180000 && fabsf(gal - lastGal) > 0.5f)) {
            Preferences p; p.begin("fuel", false); p.putFloat("gal", gal); p.end();
            lastSave = millis(); lastGal = gal;
        }
        if (lastGal < 0) lastGal = gal;
        wasRun = run; }
    Engine.setStarve(Batt.starve());
    Servos.poll();
    pedalTrace();
    {   static uint32_t t = 0;
        if ((int32_t)(g_rpmTraceUntil - millis()) > 0 && millis() - t >= 50) { t = millis();
            Serial.printf("[rpm] %lu %.0f thr %.2f lope %.2f hunt %+.2f inj %u mess %.2f\n", (unsigned long)millis(), Engine.rpm(), throttle01,
                          Engine.chopDepth(), Engine.huntNow(), (unsigned)Engine.parts.fuel, Engine.lopeMess()); } }
    heartbeat();
    // Once a second at most, and internally gated so it only spends a flash
    // write when the tank has actually moved 8 psi. Covers yanking the pack
    // without switching the engine off first.
    Engine.maybeSaveAir();
}
