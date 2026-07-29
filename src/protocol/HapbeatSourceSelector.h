#pragma once
// ---------------------------------------------------------------------------
// HapbeatSourceSelector.h — pick exactly one transmitter and stay on it (§7.1).
//
// A venue runs several transmitters at once (one or two audio sources plus
// repeaters). They all broadcast the same programme, so a receiver that mixed
// them would play doubled, smeared audio. Instead we LOCK onto one source MAC
// and decode only that one; the others are tracked for liveness and used as
// failover candidates.
//
// Ported from hapbeat-device-firmware/src/espnow_stream.cpp:253-372.
//
// Selection is deliberately hysteretic — a lock is given up only when the
// source actually goes quiet (lockTimeoutMs), or when a challenger wins a
// 4-second delivery-rate comparison by a clear margin. Without that, two
// marginal sources ping-pong and the audio seams constantly.
//
// RSSI note: arduino-esp32 2.0.x's ESP-NOW receive callback does not expose
// RSSI, so in practice `rssi` is RSSI_UNKNOWN and selection runs on
// first-come + liveness + delivery rate. The RSSI path is kept for toolchains
// that can supply it.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include "HapbeatStreamTypes.h"

namespace hapbeat {

class SourceSelector {
public:
    static const uint8_t MAX_SOURCES = 8;   // espnow_stream.cpp:95

    void begin(const DecoderOptions& opt);
    void setOptions(const DecoderOptions& opt);

    // Called for every accepted packet, before decoding. Returns true when this
    // packet belongs to the locked source and should be decoded.
    // `relayed` is bit7 of the mode byte; `mode` is the masked mode id.
    bool admit(const uint8_t mac[6], int8_t rssi, uint8_t mode, bool relayed,
               uint32_t now_ms);

    // Housekeeping that must not run per packet: release a silent lock and run
    // the delivery-rate window. Call from onPacket() after admit(), and also
    // when idle so a dead lock is released without traffic.
    void tick(uint32_t now_ms);

    // Feed the locked source's loss counters so the rate window can tell a
    // healthy lock from a failing one (§7.1.2).
    void noteLockLoss(uint32_t lost, uint32_t recovered);

    // Force a re-lock, blacklisting the current source for SEL_BLACKLIST_MS.
    void forceRelock(uint32_t now_ms);

    void setRelayTestOnly(bool on);
    bool relayTestOnly() const;

    bool           locked() const;
    bool           lockedRelayed() const;
    const uint8_t* lockedMac() const;          // 6 bytes; valid when locked()
    uint8_t        liveSourceCount(uint32_t now_ms) const;
    uint32_t       handoffs() const;

    // True on the packet where the lock changed — the decoder uses this to
    // reset sequence tracking and arm a hand-off cross-fade.
    bool consumeLockChanged();

private:
    struct Source {
        uint8_t  mac[6];
        bool     used;
        uint32_t last_ms;
        int8_t   rssi;
        uint32_t hyst_start;
        uint8_t  last_mode;
        bool     relayed;
        uint16_t win_count;
        uint32_t blacklist_until;
    };

    Source         _src[MAX_SOURCES] = {};
    int            _locked           = -1;
    DecoderOptions _opt;
    bool           _lockChanged      = false;
    bool           _relayTest        = false;
    uint32_t       _handoffs         = 0;
    uint32_t       _winStart         = 0;
    uint32_t       _lastSwitch       = 0;
    uint32_t       _winLost          = 0;
    uint32_t       _winRecovered     = 0;
    uint8_t        _mode             = 0;

    int  findOrAdd(const uint8_t mac[6], uint32_t now_ms);
    void acquire(int idx, uint32_t now_ms);
    int  pickReLockTarget(uint32_t now_ms) const;
    void evalRateSelection(uint32_t now_ms);
};

} // namespace hapbeat
