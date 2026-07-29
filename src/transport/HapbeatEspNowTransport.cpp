// ---------------------------------------------------------------------------
// HapbeatEspNowTransport.cpp — radio bring-up, ported from
// hapbeat-device-firmware/src/espnow_receiver.cpp:108-170 (init) and :193-227
// (runtime channel change).
//
// The ordering below is not cosmetic. See the channel verify loop in begin().
// ---------------------------------------------------------------------------

#include "HapbeatEspNowTransport.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "../HapbeatStreamModes.h"   // RSSI_UNKNOWN

namespace hapbeat {
namespace {

// ESP-NOW's receive callback is a plain C function pointer with no user data
// slot, so the target lives in file scope. Only one transport can be active,
// which is also true of esp_now_register_recv_cb() itself.
EspNowFrameFn s_fn      = nullptr;
void*         s_user    = nullptr;
uint8_t       s_channel = 11;
bool          s_ownsWifi = false;
bool          s_started  = false;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
// arduino-esp32 3.x / IDF5: the callback carries an info struct, which also
// exposes the per-packet RSSI that the 2.x signature has no way to reach.
void recvTrampoline(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (!s_fn || !info || !info->src_addr) return;
    int8_t rssi = RSSI_UNKNOWN;
    if (info->rx_ctrl) rssi = (int8_t)info->rx_ctrl->rssi;
    s_fn(info->src_addr, data, len, rssi, s_user);
}
#else
// arduino-esp32 2.0.x: no RSSI is available in this callback at all, so the
// decoder is told so explicitly rather than being handed a fabricated 0.
void recvTrampoline(const uint8_t* mac, const uint8_t* data, int len) {
    if (!s_fn || !mac) return;
    s_fn(mac, data, len, RSSI_UNKNOWN, s_user);
}
#endif

// Re-assert the primary channel until the radio actually reports it.
//
// esp_wifi_set_channel() returning ESP_OK is NOT proof: the STA start /
// disconnect sequence can reset the primary channel back to 1 asynchronously
// *after* the call completes. The symptom is a receiver that randomly hears
// nothing until a reboot happens to win the race. Ported verbatim from
// espnow_receiver.cpp:156-168 (commit 4e41f17).
bool lockChannel(uint8_t ch) {
    uint8_t            prim = 0;
    wifi_second_chan_t sec  = WIFI_SECOND_CHAN_NONE;
    for (int tries = 0; tries < 8; tries++) {
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        delay(10);   // let the driver apply, and any pending STA event settle
        if (esp_wifi_get_channel(&prim, &sec) == ESP_OK && prim == ch) return true;
    }
    return false;
}

} // namespace

bool EspNowTransport::channelValid(uint8_t ch) {
    // Non-overlapping 2.4 GHz channels only, matching the transmitter's picker
    // (espnow_receiver.cpp:29 CHANNEL_LIST).
    return ch == 1 || ch == 6 || ch == 11;
}

bool EspNowTransport::begin(const EspNowTransportConfig& cfg, EspNowFrameFn fn, void* user) {
    if (s_started) return false;
    if (!fn) return false;
    if (!channelValid(cfg.channel)) return false;

    s_fn       = fn;
    s_user     = user;
    s_channel  = cfg.channel;
    s_ownsWifi = cfg.initWifi;

    if (cfg.initWifi) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();   // no association: ESP-NOW owns the channel

        if (cfg.disableWifiSleep) {
            // Modem sleep parks the radio between beacons and adds tens of ms
            // of jitter to an unassociated receiver (espnow_receiver.cpp:125).
            esp_wifi_set_ps(WIFI_PS_NONE);
        }
        esp_wifi_set_max_tx_power(cfg.maxTxPowerQdbm);
        esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);

        if (cfg.longRangeMask) {
            // §7.3: always advertise bgn+LR so an operator can flip the sender
            // to Long Range without reflashing receivers. Adding LR does not
            // affect ordinary bgn reception (espnow_receiver.cpp:138-139).
            esp_wifi_set_protocol(WIFI_IF_STA,
                                  WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                  WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
        }

        // ESP-NOW needs a started Wi-Fi; the channel is locked only afterwards.
        if (esp_now_init() != ESP_OK) {
            s_fn = nullptr;
            s_user = nullptr;
            return false;
        }
    }

    if (esp_now_register_recv_cb(recvTrampoline) != ESP_OK) {
        if (cfg.initWifi) esp_now_deinit();
        s_fn = nullptr;
        s_user = nullptr;
        return false;
    }

    // Only meaningful while unassociated — an AP link owns the channel
    // (espnow_receiver.cpp:157).
    if (WiFi.status() != WL_CONNECTED) {
        lockChannel(s_channel);   // best effort: 8 tries, then run anyway
    }

    s_started = true;
    return true;
}

void EspNowTransport::end() {
    if (!s_started) return;
    esp_now_unregister_recv_cb();
    if (s_ownsWifi) esp_now_deinit();
    s_fn      = nullptr;
    s_user    = nullptr;
    s_started = false;
}

bool EspNowTransport::setChannel(uint8_t ch) {
    if (!channelValid(ch)) return false;
    s_channel = ch;
    if (!s_started) return true;              // applied at begin()
    if (WiFi.status() == WL_CONNECTED) return false;   // the AP link owns it
    return lockChannel(ch);
}

uint8_t EspNowTransport::channel() const {
    return s_channel;
}

} // namespace hapbeat

#endif // ESP32
