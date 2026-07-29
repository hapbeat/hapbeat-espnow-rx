#pragma once
// ---------------------------------------------------------------------------
// HapbeatEspNowTransport.h — bring up the radio and hand raw frames upward.
//
// Two details here cost real debugging time in the firmware, so they are baked
// into this class rather than left to the caller:
//
// 1. THE CHANNEL VERIFY LOOP. esp_wifi_set_channel() can return ESP_OK and then
//    have the channel changed back to 1 underneath you, because the STA
//    start / disconnect sequence resets the primary channel asynchronously
//    *after* your call. The symptom is a receiver that randomly hears nothing
//    until you reboot it. The fix is to set the channel AFTER esp_now_init()
//    and then re-assert it until esp_wifi_get_channel() actually agrees.
//    (hapbeat-device-firmware/src/espnow_receiver.cpp:142-168)
//
// 2. THE LONG-RANGE MASK. Receivers always advertise 11b/g/n + LR, so an
//    operator can flip the transmitter to Long Range for a difficult venue
//    without reflashing a single receiver. Adding LR does not affect ordinary
//    reception. (espnow-stream.md §7.3)
//
// ESP32 family only.
// ---------------------------------------------------------------------------

#include <stdint.h>

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

namespace hapbeat {

// Raw ESP-NOW frame callback. Runs in the Wi-Fi task — treat it as an ISR-ish
// context: no blocking, no logging, no I2C.
using EspNowFrameFn = void (*)(const uint8_t mac[6], const uint8_t* data, int len,
                               int8_t rssi, void* user);

struct EspNowTransportConfig {
    uint8_t channel         = 11;    // 1 / 6 / 11
    bool    longRangeMask   = true;
    bool    disableWifiSleep = true;
    int8_t  maxTxPowerQdbm  = 84;    // quarter-dBm; 84 = 21 dBm
    bool    initWifi        = true;  // false: you already started Wi-Fi + ESP-NOW
};

class EspNowTransport {
public:
    bool begin(const EspNowTransportConfig& cfg, EspNowFrameFn fn, void* user);
    void end();

    // Re-assert a channel at runtime, with the same verify loop as begin().
    bool    setChannel(uint8_t ch);
    uint8_t channel() const;

    static bool channelValid(uint8_t ch);   // 1, 6 or 11
};

} // namespace hapbeat

#endif // ESP32
