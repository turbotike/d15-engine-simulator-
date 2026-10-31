// =============================================================================
//  Trailer.h - wireless trailer, compatible with TheDIYGuy999's
//  wirelessTrailerESP32web (github.com/TheDIYGuy999/wirelessTrailerESP32web).
// =============================================================================
//  Same radio and same packet as his sound controller (Rc_Engine_Sound_ESP32
//  src.ino, struct_message / trailerControl()), so an unmodified DIYGuy trailer
//  board works with this truck:
//    * ESP-NOW, unicast to the trailer's MAC (his default FE:49:01:00:00:01;
//      the trailer's own web page at 192.168.4.1 sets it).
//    * WiFi channel 1 - our AP is pinned to 1, and the trailer's AP defaults
//      to 1 too, so the two radios meet without any pairing step.
//    * 10-byte packet, his field order: tail, side, reverse, indicator L/R
//      (0-255 brightness), legs up/down, ramps up/down, beacons (bools).
//    * Sent when something changes, polled every 20 ms like his. We also
//      resend once a second so a trailer switched on AFTER the truck still
//      picks up the current lights (his only sends on change).
//  There is no "pairing" in his protocol: the MAC is the pairing. Every
//  unicast is acknowledged by the trailer's radio, which is how the UI can say
//  LINKED vs NO ANSWER.
// =============================================================================
#pragma once
#include <Arduino.h>

struct TrailerPacket {            // DO NOT reorder - must match his struct_message
    uint8_t tailLight;
    uint8_t sideLight;
    uint8_t reversingLight;
    uint8_t indicatorL;
    uint8_t indicatorR;
    bool    legsUp;
    bool    legsDown;
    bool    rampsUp;
    bool    rampsDown;
    bool    beaconsOn;
};
static_assert(sizeof(TrailerPacket) == 10, "DIYGuy trailer packet is 10 bytes");

enum TrailerFn : uint8_t { TR_LEGS_UP, TR_LEGS_DOWN, TR_RAMPS_UP, TR_RAMPS_DOWN, TR_NFN };

class TrailerLink {
public:
    void begin();                         // after the WiFi AP is up
    // Lights come in as the truck shows them; legs/ramps come from hold().
    void update(uint8_t tail, uint8_t side, uint8_t rev, uint8_t indL, uint8_t indR,
                bool beacon, bool lightsAllowed);

    // Legs/ramps are motors: held, not toggled. The page re-sends every 250 ms
    // while a button is down and we stop the motor 700 ms after the last one,
    // so a dropped "release" (phone locks, WiFi blip) can never leave a leg
    // running into its end stop.
    void hold(TrailerFn fn);
    void release();
    void lightTest(uint32_t ms = 3000) { _testUntil = millis() + ms; }

    // settings (saved in their own NVS namespace, "rclc_trl")
    bool  enabled() const { return _on; }
    void  setEnabled(bool on);
    const uint8_t* mac() const { return _mac; }
    bool  setMac(const char* txt);        // "FE:49:01:00:00:01"
    String macStr() const;

    // 0 off / 1 no answer / 2 linked (an ack within the last 2.5 s)
    int   state() const;
    bool  takeLinkedEdge();               // true once each time the link comes up
    String json() const;

private:
    void  addPeer();
    void  save();
    bool     _started = false, _on = true, _peer = false;
    uint8_t  _mac[6] = { 0xFE, 0x49, 0x01, 0x00, 0x00, 0x01 };
    TrailerPacket _pkt = {}, _sent = {};
    uint32_t _lastSend = 0, _lastPoll = 0, _testUntil = 0;
    uint32_t _holdMs[TR_NFN] = {};
    bool     _wasLinked = false;
};

extern TrailerLink Trailer;
