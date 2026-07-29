#pragma once
// ---------------------------------------------------------------------------
// HapbeatFleetTune.h — 0xAC "fleet tune" beacon (§3.5).
//
// Lets an operator retune a whole venue of receivers over the air from the
// transmitter, without touching 60 devices. Values are runtime-only and are
// NOT persisted, so a power cycle returns every receiver to its mode defaults
// — the deliberate cure for settings left behind after a show. (The one
// exception is param 5, the volume ceiling, which the application is expected
// to persist; this library has no volume hardware, so it hands it to you.)
//
// Wire: [0]=0xAC [1]=flags(bit7 RELAYED | version) [2]=param [3]=value
//       [4]=epoch (optional; senders always send it)
//
// Epoch rules (DEC-045) — get these wrong and a venue silently keeps a stale
// value forever:
//   * reject ONLY strictly-older epochs, wrap-safe: (int8_t)(e - last) < 0
//   * a repeat of the SAME epoch must be re-applied (the 5 s resend is how a
//     receiver that booted late catches up); every apply is idempotent
//   * latch the epoch only AFTER the value passed its range check, or a bad
//     value burns the epoch and the corrected resend gets rejected
//
// Ported from hapbeat-device-firmware/src/espnow_stream.cpp:942-1037.
// ---------------------------------------------------------------------------

#include <stdint.h>

namespace hapbeat {

enum : uint8_t {
    FLEET_BUFFER_MS    = 1,  // 0 = mode default, else 4..60 ms jitter depth
    FLEET_SELECTION    = 2,  // 0/1 — delivery-rate source selection on/off
    FLEET_LOCK_TIMEOUT = 3,  // ×10 ms, 5..50
    FLEET_RESYNC_GAP   = 4,  // 4..64
    FLEET_VOLUME_MAX   = 5,  // 10..100 % — handed to the application
    FLEET_HP_BUFFER_MS = 6,  // Hapbeat DuoWL v4 only — ignored here
};

// What a beacon asked us to change. Fields are only meaningful when their
// `has*` flag is set.
struct FleetTuneResult {
    bool     applied        = false;
    bool     hasBufferMs    = false;  uint16_t bufferMs    = 0;
    bool     hasSelection   = false;  bool     selection   = true;
    bool     hasLockTimeout = false;  uint32_t lockTimeout = 0;
    bool     hasResyncGap   = false;  uint8_t  resyncGap   = 0;
    bool     hasVolumeMax   = false;  uint8_t  volumeMax   = 0;
};

class FleetTune {
public:
    void reset();

    // Parse and gate one 0xAC frame. Returns what the caller should apply.
    // Unknown param ids and versions are ignored (forward compatibility): a
    // future transmitter must never brick an older receiver.
    FleetTuneResult onBeacon(const uint8_t* data, int len);

private:
    // Index by param id (1..6); [0] unused.
    uint8_t _epoch[7]    = {0};
    bool    _epochSet[7] = {false};

    bool epochAllows(uint8_t param, uint8_t epoch, bool hasEpoch) const;
    void latchEpoch(uint8_t param, uint8_t epoch, bool hasEpoch);
};

} // namespace hapbeat
