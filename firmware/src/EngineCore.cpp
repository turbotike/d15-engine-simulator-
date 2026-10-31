// =============================================================================
//  EngineCore.cpp - the C15 breathes. Numbers sanity-checked against the real
//  engine: ~258 mg/stroke and ~0.65 kg/s air at 550 hp / 1800 rpm, AFR ~28 at
//  full load, turbine/compressor power balance ~100 kW at rated boost.
// =============================================================================
#include "EngineCore.h"

static inline float clampf_(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

// volumetric efficiency vs rpm: the cam/head breathe best mid-band
static float veOf(float rpm) {
    float x = (rpm - 1400.0f) / 900.0f;
    return clampf_(0.93f - 0.10f * x * x, 0.70f, 0.93f);
}

// full-rack fuel per stroke (mg) shaped like the real torque curve
static float fuelMgFull(float rpm) {
    // c15 torque plateau 1850 lb-ft ~ 2508 Nm -> ~260 mg/stroke
    float t;
    if      (rpm < 1000) t = 0.62f + 0.38f * (rpm - 600.0f) / 400.0f;
    else if (rpm < 1450) t = 1.0f;
    else                 t = 1.0f - 0.215f * (rpm - 1450.0f) / 650.0f;
    return 296.0f * clampf_(t, 0.3f, 1.0f);
}

CoreOut EngineCore::tick(float dt, float rpm, float fuelCmd01, float jake01) {
    CoreOut o = {};
    // sim parity: isFinite guards on the integrator state. Without these one bad frame
    // poisons _map/_wTurbo/_texh forever and every output downstream goes NaN.
    if (!isfinite(_map)    || _map    <= 0)  _map    = P_AMB;
    if (!isfinite(_wTurbo) || _wTurbo <= 0)  _wTurbo = 2000.0f;
    if (!isfinite(_texh))                    _texh   = 750.0f;
    if (!isfinite(_tGas))                    _tGas   = 750.0f;
    if (!isfinite(rpm))       rpm = 650.0f;
    if (!isfinite(fuelCmd01)) fuelCmd01 = 0.0f;
    if (rpm < 50) rpm = 50;
    float w = rpm * 0.10472f;                       // rad/s

    // ---- intake: what the head actually pulls ----
    // intercooler: charge air heats with boost (compression), the IC cools it back
    // by its efficiency. Bigger IC (higher icEff) = denser charge = more power, lower EGT.
    float prcNow  = fmaxf(1.0f, _map / (P_AMB * 0.97f));
    float tCharge = 300.0f + (300.0f * powf(prcNow, 0.286f) - 300.0f) * (1.0f - icEff);
    float rho    = _map / (R_AIR * tCharge);        // charge density (intercooled)
    float ve     = veOf(rpm);
    float airKgS = VD * (rpm / 120.0f) * rho * ve;  // rpm/60/2 intake strokes
    o.airKgS = airKgS;
    float airPerCyl = airKgS / (rpm / 120.0f * 6.0f) + 1e-9f;   // kg trapped per fire

    // ---- injectors: real fuel mass, smoke limited by trapped air ----
    float effTrim = fminf(fuelTrim, fuelCap);                  // injectors can only flow so much
    float mgCmd   = fuelMgFull(rpm) * fuelCmd01 * effTrim * fuelFlow;   // what the rack asks, through THESE injectors
    float fuelKg  = mgCmd * 1e-6f;
    float fuelMax = airPerCyl / AFR_SMOKE;                      // beyond this: smoke
    // boost-referenced SMOKE LIMITER: the ECU won't dump more than ~1.6x the air can take,
    // so lugging it bogs-and-builds instead of overfueling into an instant EGT meltdown.
    // 1.6x of the stoichiometric-ish limit let AFR reach ~10.6, which is
    // black smoke no C15 ECU would ever command - and it is why full rack at
    // 700-800 rpm drove EGT to 1900 F and melted the engine in ~2.5 s instead
    // of just bogging and smoking. 1.28 puts the floor near AFR 13.5.
    // 1.6, the browser's value: fk = Math.min(fk, fmax*1.6). The 1.28 here was a
    // deliberate deviation (AFR floor 13.3 instead of 10.6) and it cost 128 K of
    // EGT at full lug and kept smoke from ever pinning - both of which feed the
    // turbine and therefore the whistle.
    fuelKg = fminf(fuelKg, fuelMax * 1.6f);
    float burnKg  = fminf(fuelKg, fuelMax);
    float rich    = clampf_((fuelKg - fuelMax) / (fuelMax + 1e-9f), 0, 3.0f);  // fuel past the air
    o.smoke01     = clampf_((fuelKg - burnKg) / (fuelMax + 1e-9f) * 3.0f, 0, 1);
    o.afr         = airPerCyl / (fuelKg + 1e-12f);

    // ---- combustion: timing sets efficiency and cylinder pressure ----
    float optAdv  = 2.0f + 4.0f * clampf_((rpm - 600.0f) / 1500.0f, 0, 1);
    // advancing timing ADDS power up to ~stock+2 (real diesel); over-advance past 16 hurts
    float advF    = clampf_((advDeg - 5.0f) / 11.0f, -0.5f, 1.0f);
    float effTim  = clampf_(1.04f + 0.06f * advF, 0.92f, 1.10f);
    if (advDeg > 16.0f) effTim = fmaxf(0.80f, effTim - (advDeg - 16.0f) * 0.02f);
    // MEUI unit injectors: cam-driven, so injection pressure follows rpm.
    // Lugging big fuel at low rpm = weak atomization = smoke + soft power.
    float injP    = clampf_((rpm - 500.0f) / 1500.0f, 0, 1);
    float rack    = clampf_(mgCmd / (fuelMgFull(rpm) + 1.0f), 0, 1.5f);
    float atomEff = 1.0f - (1.0f - injP) * 0.12f * rack;
    o.smoke01     = clampf_(o.smoke01 + (1.0f - atomEff) * 1.6f * rack, 0, 1);
    float effTh   = 0.42f * effTim * atomEff;                    // thermal efficiency
    float wFire   = burnKg * LHV * effTh;                        // J per firing
    float fireHz  = rpm / 120.0f * 6.0f;
    float pIndPwr = wFire * fireHz;                              // indicated power W
    float pFric   = (18.0f + 0.012f * rpm) * w;                  // friction+parasitics
    o.torqueNm    = (pIndPwr - pFric) / w;

    // peak cylinder pressure estimate: boost fill x advance
    o.pcpBar = 105.0f + 1.5f * fmaxf((_map - P_AMB) / 6894.8f, 0.0f) + clampf_(advDeg - 2.0f, 0, 20.0f) * 3.0f;

    // ---- per-cylinder phase: firing pulses (order 1-5-3-6-2-4) ----
    static const uint8_t ORDER[6] = {0, 4, 2, 5, 1, 3};
    // HARD-FREEZE GUARD: this was `while (_phase >= 720) _phase -= 720;`. A NaN exits
    // that loop, but +INFINITY does not - inf-720 is still inf, so it spins forever and
    // locks the Teensy solid. fmodf can't hang, and the finite check stops inf getting in.
    if (!isfinite(_phase)) _phase = 0.0f;
    _phase += rpm / 60.0f * 360.0f * dt;                         // crank deg of 720 cycle
    if (!isfinite(_phase)) _phase = 0.0f;
    _phase = fmodf(_phase, 720.0f);
    if (_phase < 0) _phase += 720.0f;
    int slot = (int)(_phase / 120.0f) % 6;
    float inSlot = fmodf(_phase, 120.0f) / 120.0f;
    for (int i = 0; i < 6; i++) _pulse[i] *= (1.0f - 6.0f * dt);
    _pulse[ORDER[slot]] = fmaxf(_pulse[ORDER[slot]], (inSlot < 0.3f ? 1.0f : 0.0f) * clampf_(wFire / 9000.0f, 0.1f, 1.5f));

    // ---- exhaust energy -> turbine -> shaft ----
    float exhKgS = airKgS + fuelKg * fireHz;
    // injection timing drives EGT: retarded (late) burn dumps heat out the pipe
    // (EGT climbs), advanced (early) burns in-cylinder and runs cooler.
    // Sim formula exactly: clamp(0.72 - (adv-2)*0.012, 0.45, 0.85). The old version was
    // built around optAdv (which the sim computes and never actually uses) with a
    // different base/slope/floor, so the board ran ~6% cooler than the browser at stock
    // timing and melted later than it should.
    (void)optAdv;
    // Advance lowers exhaust heat (correct), but this was the ONLY timing->EGT
    // path, so cranking advance to +10 was a pure win: more power AND less
    // heat, with no penalty anywhere. The header advertises a sweet spot with
    // over-advance punished. Past ~6 deg the benefit now reverses, so there
    // is a real optimum to find instead of 'always max it'.
    // EXACTLY the browser: clamp(0.72 - (adv-2)*0.012, 0.45, 0.85).
    // The `over*0.030` term and the 0.92 ceiling were local inventions - they
    // flipped the slope above 6 deg from -0.012/deg to +0.018/deg, so at 10 deg
    // the port put 19% more heat in the pipe than the browser and at 16 deg, 54%.
    float adv = advDeg - 2.0f;
    float exhFrac = clampf_(0.72f - adv * 0.012f, 0.45f, 0.85f);
    float qExh   = burnKg * LHV * (1.0f - effTh) * exhFrac * fireHz;   // heat to the pipe
    float tExhK  = 400.0f + qExh / (exhKgS * 1150.0f + 1e-9f) * 0.9f
                 + rich * 400.0f + clampf_(effTrim - 1.0f, 0, 1.1f) * 300.0f * clampf_((fuelCmd01 - 0.3f) / 0.5f, 0, 1.0f);   // fat runs HOT (only on the rack)
    // ---- JAKE BRAKE: pumped mass flow, COOL gas -------------------------
    // With the rack at zero, qExh is zero and tExhK falls to its 400 K floor,
    // so pTurb (which scales on max(tGas-560,0)) goes to nothing and the shaft
    // dies. That is right for a plain lift. It is WRONG for a jake: the brake
    // is dumping a compressed charge into the manifold on every cycle, so the
    // turbine keeps being driven.
    //
    // The gas temperature is set by COMPRESSION, not combustion. Adiabatic on
    // a ~17:1 diesel from a ~330 K charge gives roughly 900-950 K at TDC, and
    // it has expanded and mixed by the time it reaches the turbine - call it
    // 700 K. Enough to spin the wheel, nowhere near a firing cylinder, which
    // is why a jake COOLS the pyro instead of heating it.
    if (jake01 > 0.01f) {
        // 1050 K, not 700. The 700 was the gas AFTER expansion and mixing, but
        // a Jacobs brake cracks the exhaust valve right at TDC compression, so
        // the turbine sees it before it has expanded. Adiabatic on a 17:1
        // diesel from a ~330 K charge:  330 * 17^0.4 = 1025 K.
        //
        // pTurb scales on max(tGas-560,0), so 700 K gave a turbine term of 140 -
        // barely over the cutoff, hence only 0.9 psi of boost. 1050 K gives 490,
        // 3.5x stronger, which is real boost and a turbo that audibly spools
        // ON the brake instead of idling through it.
        //
        // Still COOLER than a firing cylinder (1100-1400 K), so the jake keeps
        // bringing the pyro down - which is the whole point of using it.
        const float T_JAKE_K = 1050.0f;
        float blend = clampf_(jake01, 0, 1);
        tExhK = tExhK * (1.0f - blend) + T_JAKE_K * blend;
    }
    _tGas += (tExhK - _tGas) * clampf_(dt * 2.6f, 0, 1);   // fast gas temp -> turbine
    _texh += (tExhK - _texh) * clampf_(dt * 0.3f, 0, 1);   // slow gauge probe
    // turbine driven by exhaust HEAT (instant gas); strong turbine can reach ~90 psi.
    // gated by rpm: below ~680 there's no exhaust energy to spin the wheel (idle lag).
    float pTurb  = exhKgS * 1150.0f * fmaxf(_tGas - 560.0f, 0.0f) * 0.48f * turbMult
                 * clampf_((rpm - 680.0f) / 450.0f, 0, 1);
    // wastegate: past the spring, exhaust bypasses the wheel
    float boostPsiNow = (_map - P_AMB) / 6894.8f;
    if (boostPsiNow > wgPsi) pTurb *= clampf_(1.0f - (boostPsiNow - wgPsi) * 0.5f, 0.05f, 1.0f);
    // compressor absorbs power to make boost
    float prC    = 1.0f + 1.82e-8f * _wTurbo * _wTurbo;              // PR from shaft speed
    float pComp  = airKgS * 1005.0f * 300.0f * (powf(prC, 0.286f) - 1.0f) / 0.72f;
    _wTurbo += (pTurb - pComp) / (J_TURBO * fmaxf(_wTurbo, 800.0f)) * dt;
    // The shaft COASTS. This matters far more than it looks, because the
    // float-shift branch cuts fuel to zero for the whole 1.4 s shift, so this
    // branch runs on EVERY GEAR CHANGE.
    //
    // MEASURED on the bench pulling through the box: boost reached 25-29 psi in
    // each gear and then collapsed to 4-10 psi on every shift, re-spooling from
    // scratch 12 times on the way to 70 mph. The whistle therefore swept up from
    // nothing over and over instead of holding - which is what 'the turbo sucks'
    // was describing. The old 0.55 s time constant pulled the shaft to 1300
    // rad/s (just above the 1000 floor) in about half a second.
    //
    // A 90 mm billet wheel at >100k rpm has real inertia and only bearing drag
    // and windage to slow it: it coasts for SECONDS. 4.0 s keeps most of the
    // shaft speed across a 1.4 s shift, so boost comes back nearly where it
    // left off and the whistle stays up - and a long lift still bleeds down.
    // The comment here already said 'coasts, doesn't stall'; 0.55 s was a stall.
    if (rpm < 780.0f)      _wTurbo += (1000.0f - _wTurbo) * clampf_(dt * 2.2f, 0, 1);
    // 0.55 s, same as the browser. I changed this to 4.0 s thinking the shaft
    // stalled too fast; it does not - the browser uses dt/0.55 verbatim
    // (`wT+=(1300-st.wT)*clamp(dt/0.55,0,1)`), and its only other shaft decays
    // are for a DEAD engine (`*=(1-dt*0.4)`) and a real surge (`*=0.7`).
    // Reverted: the shift-time boost drop is not coming from here.
    if (fuelCmd01 < 0.12f) _wTurbo += (1300.0f - _wTurbo) * clampf_(dt / 0.55f, 0, 1);
    _wTurbo  = clampf_(_wTurbo, 1000.0f, turbCeil);                  // ~160k rpm ceiling
    // _wTurbo is rad/s, so true shaft Hz is w/(2*pi), not w/60 - the old form
    // treated rad/s as rpm and under-reported by 9.55x. Downstream, turboN in
    // EngineAudio normalises (shaftHz-18)/225, which was calibrated against the
    // WRONG units: it saturated at 1.0 for every upgraded turbo, so the big and
    // compound turbos whistled at an identical pitch to each other despite
    // spinning 2-3x faster. See the matching turboN rescale in EngineAudio.cpp.
    // wT/60, EXACTLY as the browser defines it (`shaftHz: st.wT/60`).
    //
    // This was `_wTurbo * 0.1591549` (rad/s -> true Hz, i.e. /2*pi) on the
    // reasoning that the browser's /60 was a units error. It is not a units
    // error that matters: shaftHz is only ever consumed by the audio, and the
    // audio's scale factors are calibrated against /60. Dividing by 2*pi made
    // shaftHz 9.55x too large, which pinned turboN at 1.0 and put the whistle's
    // level, pitch, filter corner and whoosh all on the wrong curve.
    o.shaftHz = _wTurbo / 60.0f;
    // surge: compressor pushing more PR than the engine will swallow
    float surgeFlow = 0.12f * (prC - 1.0f);
    o.surge = (airKgS < surgeFlow) && prC > 1.5f;

    // ---- manifold fills toward compressor outlet (intercooled) ----
    float pTarget = P_AMB * prC * 0.97f;                             // IC pressure drop
    _map += (pTarget - _map) * clampf_(dt / 0.09f, 0, 1);   // manifold fill (golden)
    o.boostPsi = fmaxf(0.0f, (_map - P_AMB) / 6894.8f);

    // ---- EGT readout (pyro in the pipe, degF, filtered like a real probe) ----
    // _texh is ALREADY lagged (dt*0.3) above, same as the sim's st.texh. The extra
    // _egtFilt stage double-lagged it, so EGT crawled where the sim's moves. The sim
    // returns the conversion straight out - do the same.
    o.egtF = (_texh - 273.15f) * 1.8f + 32.0f;
    _egtFilt = o.egtF;                       // kept in sync for anything still reading it
    return o;
}
