// =============================================================================
//  EngineCore.h - physical C15 model: mean-value air path + per-cylinder
//  injection/combustion. The head breathes, injectors meter real fuel mass,
//  and the S410G spins because exhaust energy spins it.
//
//  This is a mean-value engine model (what real ECUs/HIL rigs run) with a
//  per-cylinder combustion layer on top:
//    airflow  = Vd * rpm/2 * density(MAP, T) * VE(rpm)
//    turbine  = exhaust mass flow * cp * Texh * expansion * eff  -> shaft accel
//    boost    = compressor pressure ratio from shaft speed (surge line real)
//    cylinder = fuel mg/stroke (tune cell) burns against the air THAT cylinder
//               actually trapped; smoke-limited; timing shifts efficiency & PCP
// =============================================================================
#pragma once
#include <Arduino.h>

struct CoreOut {
    float torqueNm;      // brake torque at the flywheel
    float boostPsi;      // gauge boost (MAP - ambient)
    float egtF;          // exhaust manifold temp
    float airKgS;        // mass airflow
    float afr;           // air/fuel ratio actually burning
    float smoke01;       // 0..1 overfuel smoke
    float pcpBar;        // peak cylinder pressure estimate
    bool  surge;         // compressor is over the surge line
    float shaftHz;       // wT/60, as the browser defines it (NOT rad/s -> Hz)  (drives the whistle pitch)
};

class EngineCore {
public:
    // tune inputs (wired from the web maps)
    float fuelTrim = 1.0f;      // active cell 0.7..2.0
    float advDeg   = 0.0f;      // active cell -5..+10 deg
    float wgPsi    = 28.0f;     // wastegate spring (boostMax)
    // ---- upgradeable parts (set from EngineSim::Parts; defaults = bone-stock Cat) ----
    float icEff    = 0.72f;     // browser default. stock .72 / upgraded .85 / air-water .93
    float turbMult = 6.5f;      // browser default. stock 6.5 / big 10 / compound 14
    float turbCeil = 22000.0f;  // browser default. stock 22000 / big 32000 / compound 42000
    float fuelFlow = 1.0f;      // injector FLOW vs the stock 57s at the same command (parts decide hp)
    float fuelCap  = 1.75f;     // browser default. stock 1.75 / big 2.0 / dual 2.4

    void  reset() { _map = P_AMB; _texh = 600.0f; _wTurbo = 2000.0f; _phase = 0; }
    // jake01: engine-brake amount, 0..1. The core needs this because a
    // Jacobs brake KEEPS PUMPING GAS THROUGH THE TURBINE. The exhaust valve
    // cracks near TDC compression and dumps the charge into the manifold, so
    // mass flow continues every cycle even with the rack at zero - and the
    // shaft therefore does not spool down the way a plain fuel cut does.
    //
    // Critically the gas is COOL: no fuel is injected, so nothing burns. It is
    // hot from COMPRESSION only - a few hundred degrees, not the ~1000 F of a
    // firing cylinder. That is exactly why holding the jake down a long grade
    // is how a driver brings a pyro back down.
    CoreOut tick(float dt, float rpm, float fuelCmd01, float jake01 = 0.0f);

    float shaftRpm() const { return _wTurbo * 9.5493f; }
    void  stallShaft(float f) { _wTurbo *= f; }   // compressor surge knocks the shaft back
    float firePulse(int cyl) const { return (cyl >= 0 && cyl < 6) ? _pulse[cyl] : 0; }

private:
    // --- physical constants (SI) ---
    static constexpr float VD      = 0.0152f;    // 15.2 L
    static constexpr float P_AMB   = 101325.0f;
    static constexpr float R_AIR   = 287.0f;
    static constexpr float LHV     = 42.8e6f;    // diesel J/kg
    static constexpr float J_TURBO = 20.0e-3f;   // big billet 90mm rotor inertia (real lag) - golden
    static constexpr float AFR_SMOKE = 17.0f;    // below this it just smokes

    float _map    = P_AMB;      // intake manifold pressure (abs, Pa)
    float _texh   = 750.0f;     // exhaust manifold temp (K-ish state, degF out)
    float _wTurbo = 2000.0f;    // turbo shaft speed rad/s
    float _phase  = 0;          // engine phase 0..720 deg
    float _pulse[6] = {0};      // last combustion intensity per cylinder (audio hook)
    float _egtFilt = 750.0f;
    float _tGas    = 750.0f;   // fast turbine gas temp (drives the spool)
};
