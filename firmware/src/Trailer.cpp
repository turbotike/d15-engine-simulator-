#include "Trailer.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

TrailerLink Trailer;

static volatile uint32_t s_lastAckMs = 0;
static volatile uint32_t s_acks = 0, s_fails = 0;

// Runs in the WiFi task. Unicast ESP-NOW is ACKed by the receiving radio, so
// SUCCESS here means the trailer really heard it.
static void onSent(const uint8_t*, esp_now_send_status_t st) {
    if (st == ESP_NOW_SEND_SUCCESS) { s_lastAckMs = millis(); s_acks++; }
    else s_fails++;
}

void TrailerLink::begin() {
    Preferences p;
    if (p.begin("rclc_trl", true)) {
        _on = p.getBool("on", true);
        if (p.getBytesLength("mac") == 6) p.getBytes("mac", _mac, 6);
        p.end();
    }
    // ESP-NOW sends on the STA interface (his peer setup leaves ifidx at STA),
    // which only exists in AP+STA mode. The STA never connects to anything; it
    // just sits on the AP's channel 1.
    WiFi.mode(WIFI_AP_STA);
    if (esp_now_init() != ESP_OK) { Serial.println("[trailer] ESP-NOW init FAILED"); return; }
    esp_now_register_send_cb(onSent);
    _started = true;
    addPeer();
    Serial.printf("[trailer] ESP-NOW up, trailer %s %s (DIYGuy wirelessTrailerESP32web)\n",
                  macStr().c_str(), _on ? "enabled" : "disabled");
}

void TrailerLink::addPeer() {
    if (!_started) return;
    if (_peer) { esp_now_del_peer(_mac); _peer = false; }
    if (!_on) return;
    esp_now_peer_info_t pi = {};
    memcpy(pi.peer_addr, _mac, 6);
    pi.channel = 0;                 // = current channel (1)
    pi.encrypt = false;
    pi.ifidx   = WIFI_IF_STA;
    _peer = (esp_now_add_peer(&pi) == ESP_OK);
    if (!_peer) Serial.println("[trailer] could not add the trailer as a peer");
    _lastSend = 0;                  // push the current state straight away
}

void TrailerLink::save() {
    Preferences p;
    if (!p.begin("rclc_trl", false)) return;
    p.putBool("on", _on);
    p.putBytes("mac", _mac, 6);
    p.end();
}

void TrailerLink::setEnabled(bool on) {
    if (on == _on) return;
    if (!on && _peer) { esp_now_del_peer(_mac); _peer = false; }
    _on = on; save(); addPeer();
}

bool TrailerLink::setMac(const char* txt) {
    unsigned v[6];
    if (sscanf(txt, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
    uint8_t m[6];
    for (int i = 0; i < 6; i++) { if (v[i] > 255) return false; m[i] = (uint8_t)v[i]; }
    if (m[0] & 0x01) return false;  // multicast bit set - not a valid unicast address
    if (_peer) { esp_now_del_peer(_mac); _peer = false; }
    memcpy(_mac, m, 6);
    save(); addPeer();
    s_lastAckMs = 0;
    return true;
}

String TrailerLink::macStr() const {
    char b[18];
    snprintf(b, sizeof(b), "%02X:%02X:%02X:%02X:%02X:%02X", _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);
    return String(b);
}

void TrailerLink::hold(TrailerFn fn) {
    if (fn >= TR_NFN) return;
    // up and down of the same motor are never both on
    if (fn == TR_LEGS_UP)    _holdMs[TR_LEGS_DOWN] = 0;
    if (fn == TR_LEGS_DOWN)  _holdMs[TR_LEGS_UP] = 0;
    if (fn == TR_RAMPS_UP)   _holdMs[TR_RAMPS_DOWN] = 0;
    if (fn == TR_RAMPS_DOWN) _holdMs[TR_RAMPS_UP] = 0;
    _holdMs[fn] = millis() | 1;
}
void TrailerLink::release() { for (auto& h : _holdMs) h = 0; }

void TrailerLink::update(uint8_t tail, uint8_t side, uint8_t rev, uint8_t indL, uint8_t indR,
                         bool beacon, bool lightsAllowed) {
    uint32_t now = millis();
    if (now - _lastPoll < 20) return;              // his pollRate
    _lastPoll = now;

    bool held[TR_NFN];
    for (int i = 0; i < TR_NFN; i++) {
        if (_holdMs[i] && now - _holdMs[i] > 700) _holdMs[i] = 0;   // release timeout
        held[i] = _holdMs[i] != 0;
    }
    if (now < _testUntil) {                         // TRAILER LIGHTS button
        bool on = ((now / 500) & 1) == 0;
        tail = side = rev = 255; indL = indR = on ? 255 : 0; lightsAllowed = true;
    }
    if (!lightsAllowed) tail = side = rev = indL = indR = 0;

    _pkt.tailLight = tail; _pkt.sideLight = side; _pkt.reversingLight = rev;
    _pkt.indicatorL = indL; _pkt.indicatorR = indR;
    _pkt.legsUp = held[TR_LEGS_UP];   _pkt.legsDown = held[TR_LEGS_DOWN];
    _pkt.rampsUp = held[TR_RAMPS_UP]; _pkt.rampsDown = held[TR_RAMPS_DOWN];
    _pkt.beaconsOn = beacon;

    if (!_peer) return;
    bool changed = memcmp(&_pkt, &_sent, sizeof(_pkt)) != 0;
    if (changed || now - _lastSend > 1000 || !_lastSend) {
        if (esp_now_send(_mac, (const uint8_t*)&_pkt, sizeof(_pkt)) == ESP_OK) {
            _sent = _pkt; _lastSend = now;
        }
    }
}

int TrailerLink::state() const {
    if (!_on || !_peer) return 0;
    uint32_t a = s_lastAckMs;
    return (a && millis() - a < 2500) ? 2 : 1;
}

bool TrailerLink::takeLinkedEdge() {
    bool l = state() == 2;
    bool edge = l && !_wasLinked;
    _wasLinked = l;
    return edge;
}

String TrailerLink::json() const {
    String j = "{\"on\":"; j += _on ? 1 : 0;
    j += ",\"mac\":\""; j += macStr(); j += "\",\"state\":"; j += state();
    j += ",\"acks\":"; j += (uint32_t)s_acks; j += ",\"fails\":"; j += (uint32_t)s_fails; j += "}";
    return j;
}
