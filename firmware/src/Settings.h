// =============================================================================
//  Settings.h - the tunables that used to live in WebLink on the Teensy build
// =============================================================================
//  The Teensy firmware read all of these off `Web.*`, the settings blob the web
//  UI edits. There is no web UI on this board yet, so they live here with the
//  SAME DEFAULTS the Teensy build shipped with - the truck behaves identically
//  out of the box, and when a UI does arrive it only has to write this struct.
//
//  Values copied from WebLink.h lines 43-59. If you change one here, change it
//  there too or the two builds will drift apart.
// =============================================================================
#pragma once
#include <Arduino.h>

struct Settings {
    // ---- RC stick thresholds (microseconds) ----
    float pwmGearPark       = 1300.0f;  // below this on the gear channel = reverse
    float pwmGearForward    = 1700.0f;  // above this = drive
    float pwmGearHysteresis = 40.0f;    // stops a stick on a boundary chattering
    // A 3-position switch thrown park->forward sweeps THROUGH the reverse band,
    // so a change has to settle before it counts. 150 ms is the beier value.
    float gearDebounceMs    = 150.0f;
    float pwmHornLow        = 1600.0f;
    float pwmHornHigh       = 1600.0f;
    float pwmLightsOn       = 1300.0f;  // above this = park + head; >1800 = high beam

    // ---- automatic turn signals off the steering channel ----
    float steerSignalEnabled   = 1.0f;
    float combinedRearLights   = 0.0f;   // US style: rear indicators double as tail/brake (Public V1 board)
    float battProtect          = 1.0f;   // battery = fuel: starve + die at 3.3 V/cell (only with the ESC's BEC present)
    float battTrim             = 1.0f;   // pack-voltage calibration: set so the UI matches your meter

    // ---- added 2026-10-25 (stored in the second web block, WEB2 @300) ----
    // Wireless trailer (DIYGuy wirelessTrailerESP32web, ESP-NOW). The MAC and
    // on/off live in Trailer's own NVS namespace; these two are plain knobs.
    float trailerSync  = 1.0f;   // trailer lights mirror the truck
    float trailerAir   = 1.0f;   // gladhand hiss when the trailer link comes up
    // Steering servo OUTPUT. Only used with an iBUS/SBUS receiver: in PWM mode
    // the servo plugs into the receiver and never sees this board. 0 = off,
    // 1..8 = drive that CH header as the steering servo.
    float steerOut     = 0.0f;
    float steerMin     = 1000.0f, steerCtr = 1500.0f, steerMax = 2000.0f;
    float steerInv     = 0.0f;
    // What the two spare light outputs do (AuxFunc in Lights.h):
    // 0 off, 1 high beam, 2 fog, 3 roof, 4 3rd brake light, 5 beacon 2, 6 cab
    float aux1Func     = 1.0f;
    float aux2Func     = 0.0f;
    // WORK MODE / hydraulics (Hydraulics.h). Off until a mode is picked.
    float hydMode      = 0.0f;                       // 0 none, 1 HIAB crane, 2 dump box
    float hydSrv[4]    = { 0, 0, 0, 0 };             // output header per function: 0 none, 1..8 = CH1..CH8
    float hydIn[4]     = { 1, 2, 0, 0 };             // stick: 0 none, 1 throttle, 2 steering, 3.. = RX CH1..CH16
    float hydInv[4]    = { 0, 0, 0, 0 };             // reverse a function
    float fullStopTime = 1.5f;                       // s: full brakes stop the truck this fast (0 = instant, max 3)
    float hydSwitch    = 0.0f;                       // 0 = WORK MODE button only, 1..16 = RX channel high = work, 17 = always engaged
    float steerSignalThreshold = 150.0f;   // us either side of centre
    float steerSignalHoldMs    = 800.0f;   // latch after you straighten up

    // ---- drivetrain ----
    float throttleExpo = 0.0f;   // 0 = linear, 1 = full cubic
    float trailerLoad  = 0.0f;   // 0 bobtail 8t .. 3 lowboy 42t

    // ESC drag brake strength. 0 = OFF = coast only, which is the shipped
    // default: on an ESC configured for plain forward/reverse, below-neutral IS
    // reverse, so this has to be opted into deliberately.
    // 0.6, not 0: at 0 the ESC never got a brake command (escBrake = brake x
    // dragBrake) - the sound slowed down and the real truck rolled on.
    float dragBrake = 0.6f;     // UNUSED since the reverse-spin fix (kept so the saved layout does not shift)

    // ---- action one-shot volumes ----
    // Same names and defaults as the Teensy's WebLink.h:49-51, so the phone
    // page's existing sliders bind. On the Teensy volStartup/volShutdown/
    // volBlinker were accepted and then read by nothing - here they are real.
    float volStartup   = 1.0f;
    float volShutdown  = 1.0f;
    float volParkbrake = 1.0f;
    float volAirbrake  = 1.0f;
    float volAirDry    = 1.0f;
    float volBlinker   = 1.0f;   // storage-only: no blinker sound in this build
    float volCoupling  = 1.0f;   // storage-only: never existed in either build
    float volBov       = 1.0f;   // new - the Teensy had no slider for the BOV
    float volTick      = 0.55f;  // the Teensy's fixed mixer ch2 gain

    // ---- audio ----
    float masterVol = DC_MASTER_VOL;
};

extern Settings Cfg;
