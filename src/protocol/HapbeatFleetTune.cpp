// ---------------------------------------------------------------------------
// HapbeatFleetTune.cpp — 0xAC fleet-tune beacon (§3.5, DEC-045).
//
// Ported from hapbeat-device-firmware/src/espnow_stream.cpp:
//   :942-946   fleetLatchEpoch -> latchEpoch
//   :948-1037  espnowStreamOnFleetTune -> onBeacon
//
// This class only parses and gates; it applies nothing. Two checks the firmware
// does at the call site are therefore the CALLER's job:
//   * the beacon must come from the currently-locked source (:952) — otherwise a
//     stray transmitter can retune a receiver that is not listening to it;
//   * every applier must be idempotent, because the sender resends the same
//     epoch every 5 s and this class deliberately lets those through.
//
// No Arduino.h / esp_now.h / millis() here — see HapbeatStreamTypes.h.
// ---------------------------------------------------------------------------

#include "HapbeatFleetTune.h"
#include "../HapbeatStreamModes.h"   // PKT_FLEET_TUNE (0xAC)

namespace hapbeat {

void FleetTune::reset() {
    for (uint8_t i = 0; i < 7; i++) {
        _epoch[i]    = 0;
        _epochSet[i] = false;
    }
}

// :955-976. "Strictly-older reject, same-epoch accept" (relaxed 2026-07-12).
// Rejecting `<= 0` — i.e. also rejecting a repeat of the same epoch — is what
// caused the field reports of "volume_max / buffer override applies once then
// gets stuck": one dropped or out-of-range apply, or a stale-but-same-epoch
// beacon arriving before TX-TX convergence settles, permanently orphaned every
// later resend until the operator changed the value again.
bool FleetTune::epochAllows(uint8_t param, uint8_t epoch, bool hasEpoch) const {
    if (!hasEpoch)          return true;   // len==4 legacy sender: always apply
    if (!_epochSet[param])  return true;   // never seen — epoch 0 is a valid first value
    return (int8_t)(epoch - _epoch[param]) >= 0;   // wrap-safe over the 1-byte counter
}

// :942-946. Called ONLY after the value passed its range check: latching on an
// out-of-range value would burn the epoch, and the corrected resend at the same
// epoch would then be rejected as "not newer" and never applied.
void FleetTune::latchEpoch(uint8_t param, uint8_t epoch, bool hasEpoch) {
    if (!hasEpoch) return;
    _epoch[param]    = epoch;
    _epochSet[param] = true;
}

FleetTuneResult FleetTune::onBeacon(const uint8_t* data, int len) {
    FleetTuneResult r;
    if (!data || len < 4 || data[0] != PKT_FLEET_TUNE) return r;   // :949
    // bit7 of data[1] is the RELAYED flag (§3.4) and must be masked off before
    // the version compare. An unknown version is ignored, never guessed at: a
    // future transmitter must not be able to brick an older receiver (:950).
    if ((data[1] & 0x7F) != 1) return r;

    const uint8_t param = data[2];
    const uint8_t value = data[3];

    // :967-972. Param 6 (mode-9 HP jitter buffer) belongs to Hapbeat DuoWL v4
    // and has no meaning here, so it is excluded from the epoch window as well:
    // it must fall through to `default` WITHOUT latching, exactly like the
    // firmware's non-HP48 builds.
    const bool    hasEpoch = (len >= 5 && param >= 1 && param <= 5);
    const uint8_t epoch    = hasEpoch ? data[4] : 0;
    if (!epochAllows(param, epoch, hasEpoch)) return r;

    switch (param) {
        case FLEET_BUFFER_MS:            // 0 = mode default, else 4..60 ms (:994)
            if (value == 0 || (value >= 4 && value <= 60)) {
                r.applied     = true;
                r.hasBufferMs = true;
                r.bufferMs    = value;
                latchEpoch(param, epoch, hasEpoch);
            }
            break;

        case FLEET_SELECTION:            // any value; non-zero = enabled (:999-1001)
            r.applied      = true;
            r.hasSelection = true;
            r.selection    = (value != 0);
            latchEpoch(param, epoch, hasEpoch);
            break;

        case FLEET_LOCK_TIMEOUT:         // ×10 ms, 5..50 -> 50..500 ms (:1003-1007)
            if (value >= 5 && value <= 50) {
                r.applied        = true;
                r.hasLockTimeout = true;
                r.lockTimeout    = (uint32_t)value * 10;
                latchEpoch(param, epoch, hasEpoch);
            }
            break;

        case FLEET_RESYNC_GAP:           // 4..64 (:1009-1013)
            if (value >= 4 && value <= 64) {
                r.applied      = true;
                r.hasResyncGap = true;
                r.resyncGap    = value;
                latchEpoch(param, epoch, hasEpoch);
            }
            break;

        case FLEET_VOLUME_MAX:           // 10..100 % (:1015-1021)
            if (value >= 10 && value <= 100) {
                r.applied      = true;
                r.hasVolumeMax = true;
                r.volumeMax    = value;
                latchEpoch(param, epoch, hasEpoch);
            }
            break;

        default:                         // unknown param, and param 6 — ignore (:1035)
            break;
    }
    return r;
}

} // namespace hapbeat
