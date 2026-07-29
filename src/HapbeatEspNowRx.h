#pragma once
// ---------------------------------------------------------------------------
// HapbeatEspNowRx.h — receive a Hapbeat ESP-NOW broadcast and play it out.
//
// A Hapbeat transmitter takes a line-level feed (a PA send, an instrument, a
// mixer bus), encodes it, and broadcasts it over ESP-NOW. There is no pairing
// and no association: every receiver on the same Wi-Fi channel hears it, so the
// number of listeners is limited only by radio range. This library is the
// receiving half, for any ESP32 board.
//
//   #include <HapbeatEspNowRx.h>
//   HapbeatEspNowRx rx;
//
//   void setup() {
//     auto cfg = hapbeat::boards::GenericI2S(/*bclk*/26, /*lrck*/25, /*dout*/22);
//     cfg.channel = 11;              // must match the transmitter
//     rx.begin(cfg);
//   }
//   void loop() { rx.tick(); }
//
// That is the whole thing: audio flows on a dedicated task, and tick() only
// does deferred housekeeping and callbacks.
//
// ESP32 family only (ESP-NOW + driver/i2s.h).
// ---------------------------------------------------------------------------

#include <stddef.h>
#include <stdint.h>

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include "HapbeatRxConfig.h"
#include "HapbeatStreamModes.h"
#include "protocol/HapbeatStreamDecoder.h"
#include "sink/HapbeatJitterRing.h"
#include "sink/HapbeatI2sSink.h"
#include "transport/HapbeatEspNowTransport.h"

class Print;

class HapbeatEspNowRx {
public:
    // --- lifecycle ----------------------------------------------------------
    // Uses the built-in jitter buffer + I2S output described by `cfg`.
    bool begin(const hapbeat::RxConfig& cfg);

    // Bring your own output. `sink` must outlive this object; nothing in cfg's
    // I2S/task fields is used.
    bool begin(const hapbeat::RxConfig& cfg, hapbeat::FrameSink& sink);

    void end();

    // Call from loop(). Runs deferred fleet-tune application, lock/mode
    // callbacks and lock timeouts. Audio does not depend on it being prompt.
    void tick();

    // Only when cfg.startAudioTask == false: drive the I2S writer yourself.
    void pump();

    // --- operating controls -------------------------------------------------
    bool     setChannel(uint8_t ch);       // 1 / 6 / 11
    uint8_t  channel() const;
    void     setGain(float g);             // 0..1
    float    gain() const;

    // Drop the current transmitter and pick another (the old one is skipped for
    // a cool-off period). Useful when a receiver has locked onto a repeater
    // that is further away than the direct signal.
    void     forceRelock();

    // Accept only relayed packets. For confirming repeater coverage during
    // setup with a single receiver. Never persisted — a reboot clears it.
    void     setRelayTestOnly(bool on);

    // --- state --------------------------------------------------------------
    bool                 locked() const;
    uint8_t              mode() const;
    const char*          modeName() const;
    hapbeat::StreamStats stats() const;

    // --- notifications ------------------------------------------------------
    void setLogger(Print* out);   // nullptr = silent (default)

    // Fired from tick(), so these may block / log / use I2C / write flash.
    void onFleetVolumeMax(hapbeat::FleetVolumeMaxFn fn, void* user = nullptr);
    void onLockChanged(hapbeat::LockChangedFn fn, void* user = nullptr);
    void onModeChanged(hapbeat::ModeChangedFn fn, void* user = nullptr);

    // Fired from the ESP-NOW receive callback: do not block, log or use I2C.
    void onPcmTap(hapbeat::PcmTapFn fn, void* user = nullptr);

    // --- pull output (when you own the DAC) --------------------------------
    // Always writes `frames` stereo frames; underflow is concealed.
    size_t readFrames(int16_t* interleavedOut, size_t frames);

    // ESP-NOW's receive callback is a plain C function, so exactly one instance
    // can be active at a time.
    static HapbeatEspNowRx* instance();

private:
    hapbeat::RxConfig         _cfg;
    hapbeat::StreamDecoder    _decoder;
    hapbeat::JitterRing       _ring;
    hapbeat::I2sSink          _i2s;
    hapbeat::EspNowTransport  _radio;
    hapbeat::FrameSink*       _sink      = nullptr;
    bool                      _ownsRing  = false;
    bool                      _ownsI2s   = false;
    bool                      _started   = false;
    Print*                    _log       = nullptr;
    uint32_t                  _lastLogMs = 0;

    bool startCommon(const hapbeat::RxConfig& cfg, hapbeat::FrameSink& sink);
    static void onFrame(const uint8_t mac[6], const uint8_t* data, int len,
                        int8_t rssi, void* user);
};

#endif // ESP32
