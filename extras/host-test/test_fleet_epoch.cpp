// ---------------------------------------------------------------------------
// test_fleet_epoch.cpp — 0xAC fleet-tune beacon and the DEC-045 epoch gate.
//
// The epoch rules exist because of two real field failures:
//
//  * "applies once, then gets stuck". The gate originally rejected same-epoch
//    resends. Since the transmitter resends the same beacon every ~5 s at the
//    same epoch, a single dropped or invalid apply orphaned that value forever —
//    a whole venue kept a stale buffer/volume until someone changed the setting
//    to bump the epoch. Fix: reject ONLY strictly-older epochs; re-apply
//    same-epoch resends (every applier is idempotent).
//
//  * "burnt epoch". Latching the epoch before range-checking the value meant a
//    bad value consumed the epoch, and the corrected resend at the same epoch
//    was then rejected as "not newer".
//
// Reference: espnow_stream.cpp:947-1035 (fleetLatchEpoch + the epoch gate).
// ---------------------------------------------------------------------------

#include "test_harness.h"
#include "test_wire.h"

#include "protocol/HapbeatFleetTune.h"

using namespace hapbeat;
using namespace tw;

namespace {

FleetTuneResult beacon(FleetTune& ft, uint8_t param, uint8_t value,
                       bool hasEpoch, uint8_t epoch) {
    uint8_t buf[8];
    int len = fleetBeacon(buf, param, value, hasEpoch, epoch);
    return ft.onBeacon(buf, len);
}

} // namespace

// (a) A strictly-older epoch is a stale beacon from an out-of-sync transmitter
// and must not un-apply a newer value this receiver already picked up.
HT_TEST(fleet_strictly_older_epoch_is_rejected) {
    FleetTune ft;
    ft.reset();

    FleetTuneResult r = beacon(ft, FLEET_RESYNC_GAP, 30, true, 5);
    HT_CHECK_EQ_WHY(r.applied, 1, "epoch 5 is the first one seen — it must apply");
    HT_CHECK(r.hasResyncGap);
    HT_CHECK_EQ(r.resyncGap, 30);

    r = beacon(ft, FLEET_RESYNC_GAP, 40, true, 4);
    HT_CHECK_EQ_WHY(r.applied, 0, "epoch 4 is older than the applied epoch 5");
    HT_CHECK_EQ(r.hasResyncGap, 0);

    // Forward again: newer must still get through.
    r = beacon(ft, FLEET_RESYNC_GAP, 50, true, 6);
    HT_CHECK_EQ_WHY(r.applied, 1, "epoch 6 is newer than 5");
    HT_CHECK_EQ(r.resyncGap, 50);
}

// (b) THE DEC-045 regression. The transmitter resends the same beacon every
// ~5 s without bumping the epoch; that resend is how a receiver that booted
// late catches up. Rejecting it is what caused "applies once then sticks".
HT_TEST(fleet_same_epoch_resend_is_reapplied) {
    FleetTune ft;
    ft.reset();

    FleetTuneResult r = beacon(ft, FLEET_VOLUME_MAX, 40, true, 7);
    HT_CHECK_EQ(r.applied, 1);
    HT_CHECK_EQ(r.volumeMax, 40);

    for (int i = 0; i < 3; i++) {
        r = beacon(ft, FLEET_VOLUME_MAX, 40, true, 7);
        HT_CHECKF(r.applied,
                  "resend %d at the SAME epoch was rejected — DEC-045 requires "
                  "strictly-older-reject only, or a late-booting receiver never "
                  "catches up", i);
        HT_CHECK_EQ(r.hasVolumeMax, 1);
        HT_CHECK_EQ(r.volumeMax, 40);
    }
}

// The epoch counter is one byte and rolls over indefinitely; the comparison
// must be wrap-safe (int8_t delta), not a plain <.
HT_TEST(fleet_epoch_comparison_is_wrap_safe) {
    FleetTune ft;
    ft.reset();

    HT_CHECK_EQ(beacon(ft, FLEET_RESYNC_GAP, 20, true, 254).applied, 1);
    HT_CHECK_EQ_WHY(beacon(ft, FLEET_RESYNC_GAP, 24, true, 255).applied, 1,
                    "254 -> 255 is forward");
    HT_CHECK_EQ_WHY(beacon(ft, FLEET_RESYNC_GAP, 28, true, 0).applied, 1,
                    "255 -> 0 is forward (wrap), not a 255-step regression");
    HT_CHECK_EQ_WHY(beacon(ft, FLEET_RESYNC_GAP, 32, true, 1).applied, 1,
                    "0 -> 1 is forward");
    HT_CHECK_EQ_WHY(beacon(ft, FLEET_RESYNC_GAP, 36, true, 250).applied, 0,
                    "250 is behind 1 across the wrap and must be rejected");
}

// (c) An out-of-range value must NOT consume the epoch, or the corrected
// resend at that same epoch is rejected and the value is orphaned forever.
HT_TEST(fleet_invalid_value_does_not_latch_the_epoch) {
    FleetTune ft;
    ft.reset();

    // volume_max is valid over 10..100.
    FleetTuneResult r = beacon(ft, FLEET_VOLUME_MAX, 200, true, 9);
    HT_CHECK_EQ_WHY(r.applied, 0, "200 % is out of range for volume_max");

    r = beacon(ft, FLEET_VOLUME_MAX, 50, true, 9);
    HT_CHECK_EQ_WHY(r.applied, 1,
                    "the corrected value at the SAME epoch must still apply — "
                    "an invalid value must never burn the epoch");
    HT_CHECK_EQ(r.volumeMax, 50);

    // Same for the low end of the range.
    FleetTune ft2;
    ft2.reset();
    HT_CHECK_EQ(beacon(ft2, FLEET_VOLUME_MAX, 5, true, 3).applied, 0);
    HT_CHECK_EQ_WHY(beacon(ft2, FLEET_VOLUME_MAX, 10, true, 3).applied, 1,
                    "10 % is the lower bound and must apply");
}

// (d) A legacy 4-byte beacon has no epoch byte at all and must always apply.
HT_TEST(fleet_beacon_without_epoch_always_applies) {
    FleetTune ft;
    ft.reset();

    for (int i = 0; i < 3; i++) {
        FleetTuneResult r = beacon(ft, FLEET_SELECTION, 0, false, 0);
        HT_CHECKF(r.applied, "4-byte (epoch-less) beacon %d must always apply", i);
        HT_CHECK_EQ(r.hasSelection, 1);
        HT_CHECK_EQ_WHY(r.selection, 0, "value 0 disables rate-based selection");
    }

    FleetTuneResult r = beacon(ft, FLEET_SELECTION, 1, false, 0);
    HT_CHECK_EQ(r.applied, 1);
    HT_CHECK_EQ(r.selection, 1);

    // An epoch-less beacon must not poison the gate for later epoched ones.
    r = beacon(ft, FLEET_SELECTION, 0, true, 0);
    HT_CHECK_EQ_WHY(r.applied, 1, "the first epoched beacon must still apply");
}

// (e) Forward compatibility: a future transmitter must never brick an older
// receiver. Unknown params and unknown versions are ignored, not guessed at.
HT_TEST(fleet_unknown_param_and_version_are_ignored) {
    FleetTune ft;
    ft.reset();

    HT_CHECK_EQ_WHY(beacon(ft, 99, 42, true, 1).applied, 0,
                    "an unknown param id must be ignored");
    HT_CHECK_EQ_WHY(beacon(ft, 0, 42, true, 1).applied, 0,
                    "param 0 is not assigned");

    // Version lives in the low 7 bits of [1]; only version 1 exists.
    uint8_t buf[8];
    int len = fleetBeacon(buf, FLEET_RESYNC_GAP, 20, true, 1, /*version=*/2);
    HT_CHECK_EQ_WHY(ft.onBeacon(buf, len).applied, 0,
                    "an unknown protocol version must be ignored");

    // Wrong packet type entirely.
    len = fleetBeacon(buf, FLEET_RESYNC_GAP, 20, true, 1);
    buf[0] = 0xAA;
    HT_CHECK_EQ_WHY(ft.onBeacon(buf, len).applied, 0, "0xAA is not a fleet beacon");

    // Runt.
    len = fleetBeacon(buf, FLEET_RESYNC_GAP, 20, false, 0);
    HT_CHECK_EQ_WHY(ft.onBeacon(buf, 3).applied, 0, "a beacon shorter than 4 bytes");
    HT_CHECK_EQ_WHY(ft.onBeacon(buf, len).applied, 1, "...but 4 bytes is legal");

    // Version 1 with the RELAYED bit set is still version 1.
    len = fleetBeacon(buf, FLEET_RESYNC_GAP, 24, true, 2, 1, /*relayed=*/true);
    HT_CHECK_EQ_WHY(ft.onBeacon(buf, len).applied, 1,
                    "bit7 is RELAYED, not part of the version — mask before comparing");
}

// Per-param epochs are independent: one param's history must not gate another's.
HT_TEST(fleet_epochs_are_tracked_per_param) {
    FleetTune ft;
    ft.reset();

    HT_CHECK_EQ(beacon(ft, FLEET_RESYNC_GAP, 20, true, 100).applied, 1);

    // A low epoch on a DIFFERENT param is that param's first — it must apply.
    FleetTuneResult r = beacon(ft, FLEET_VOLUME_MAX, 60, true, 2);
    HT_CHECK_EQ_WHY(r.applied, 1, "param 5 has its own epoch history");
    HT_CHECK_EQ(r.volumeMax, 60);

    // ...and param 4's gate is unaffected.
    HT_CHECK_EQ(beacon(ft, FLEET_RESYNC_GAP, 24, true, 99).applied, 0);
}

// Value ranges, straight from §3.5. Out-of-range values are dropped rather than
// clamped: clamping would silently half-apply an operator's mistake.
HT_TEST(fleet_param_value_ranges) {
    FleetTune ft;
    ft.reset();

    // param 1 buffer_ms: 0 (= mode default) or 4..60
    HT_CHECK_EQ(beacon(ft, FLEET_BUFFER_MS, 0, false, 0).applied, 1);
    HT_CHECK_EQ(beacon(ft, FLEET_BUFFER_MS, 3, false, 0).applied, 0);
    FleetTuneResult r = beacon(ft, FLEET_BUFFER_MS, 20, false, 0);
    HT_CHECK_EQ(r.applied, 1);
    HT_CHECK_EQ(r.hasBufferMs, 1);
    HT_CHECK_EQ(r.bufferMs, 20);
    HT_CHECK_EQ(beacon(ft, FLEET_BUFFER_MS, 61, false, 0).applied, 0);

    // param 3 lock_timeout: 5..50, in units of 10 ms
    HT_CHECK_EQ(beacon(ft, FLEET_LOCK_TIMEOUT, 4, false, 0).applied, 0);
    r = beacon(ft, FLEET_LOCK_TIMEOUT, 15, false, 0);
    HT_CHECK_EQ(r.applied, 1);
    HT_CHECK_EQ(r.hasLockTimeout, 1);
    HT_CHECK_EQ_WHY(r.lockTimeout, 150, "param 3 is x10 ms");
    HT_CHECK_EQ(beacon(ft, FLEET_LOCK_TIMEOUT, 51, false, 0).applied, 0);

    // param 4 resync_gap: 4..64
    HT_CHECK_EQ(beacon(ft, FLEET_RESYNC_GAP, 3, false, 0).applied, 0);
    HT_CHECK_EQ(beacon(ft, FLEET_RESYNC_GAP, 64, false, 0).applied, 1);
    HT_CHECK_EQ(beacon(ft, FLEET_RESYNC_GAP, 65, false, 0).applied, 0);

    // param 5 volume_max: 10..100 %
    HT_CHECK_EQ(beacon(ft, FLEET_VOLUME_MAX, 9, false, 0).applied, 0);
    HT_CHECK_EQ(beacon(ft, FLEET_VOLUME_MAX, 100, false, 0).applied, 1);
    HT_CHECK_EQ(beacon(ft, FLEET_VOLUME_MAX, 101, false, 0).applied, 0);

    // param 6 is the DuoWL v4 HP buffer — not this library's business.
    HT_CHECK_EQ_WHY(beacon(ft, FLEET_HP_BUFFER_MS, 12, false, 0).applied, 0,
                    "param 6 is Hapbeat-hardware specific and ignored here");
}

HT_TEST(fleet_reset_clears_epoch_history) {
    FleetTune ft;
    ft.reset();

    HT_CHECK_EQ(beacon(ft, FLEET_RESYNC_GAP, 20, true, 200).applied, 1);
    HT_CHECK_EQ(beacon(ft, FLEET_RESYNC_GAP, 24, true, 100).applied, 0);

    ft.reset();
    HT_CHECK_EQ_WHY(beacon(ft, FLEET_RESYNC_GAP, 24, true, 100).applied, 1,
                    "after reset(), any epoch is the first one seen");
}
