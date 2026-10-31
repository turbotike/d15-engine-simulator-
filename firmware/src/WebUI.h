// =============================================================================
//  WebUI.h - the Peterbilt dash, hosted by the S3 itself
// =============================================================================
//  On the Teensy board this needed a SECOND CHIP. The Teensy had no radio, so an
//  ESP32-C3 sat alongside it hosting the page, and the two talked over a serial
//  link with its own protocol, its own framing and its own failure modes - the
//  telemetry could go stale, the link could drop, and the page had a "link"
//  flag just to tell you when that had happened.
//
//  None of that exists here. The S3 has WiFi, so it serves the page and reads
//  the sim straight out of memory. The C3, the bridge, WebLink and the whole
//  D[] telemetry array are gone, the same way SdLoop went.
//
//  The page itself (page.h, ~100 KB) is lifted unchanged from the C3 sketch. It
//  builds its tuning sheet dynamically from whatever /settings returns, so the
//  HTML did not have to be touched - exposing a new knob here makes it appear
//  on the phone by itself.
//
//  ONE THING TO WATCH: the audio renderer is pinned to core 0 and sits at
//  ~50% of real time. WiFi normally runs on core 1 alongside the Arduino loop,
//  so they should not fight - but if `xrun` in the heartbeat starts climbing
//  once the radio is up, this is why, and the fix is to drop the web poll rate
//  or move the render's priority up.
// =============================================================================
#pragma once
#include <Arduino.h>

class WebUI {
public:
    // Brings up a WiFi access point and starts serving. Safe to skip entirely -
    // the truck runs fine without it.
    bool begin(const char *ssid = "RCLC-D15", const char *pass = "truckin123");
    void stop();
    bool running() const { return _up; }

    // Call from the main loop. Cheap when idle.
    void poll();

    const char *ip() const { return _ip; }
    uint32_t clients() const;

private:
    bool _up = false;
    char _ip[20] = {0};
};

extern WebUI Web;

// Buttons on the page that act on the truck's control state. WebUI sets
// these, main.cpp consumes them in loop() (same core, so no locking), and
// main writes back what the truck is actually doing for /status.
struct WebControls {
    bool lightNext = false;     // LIGHT_NEXT  (one-shot)
    bool jakeCycle = false;     // JAKE_CYCLE  (one-shot)  off -> LO -> HI -> off
    bool beacon    = false;     // BEACON_ON/OFF (latched, OR'd with the RC aux switch)
    bool workToggle = false;    // WORK_TOGGLE (one-shot) - hydraulics work mode
    // reported back by main
    uint8_t lightModeNow = 0;   // 0 OFF 1 MARK 2 ROOF 3 HEAD 4 FULL
    uint8_t jakeNow      = 0;   // 0 off 1 LO 2 HI
};
extern WebControls WebCtl;
