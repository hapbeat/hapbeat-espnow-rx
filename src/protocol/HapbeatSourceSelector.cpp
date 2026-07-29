// ---------------------------------------------------------------------------
// HapbeatSourceSelector.cpp — §7.1 single-source lock.
//
// Ported from hapbeat-device-firmware/src/espnow_stream.cpp:
//   :253-261  rssiKnown / stillBlacklisted
//   :265-292  findOrAddSource      -> findOrAdd
//   :294-306  acquireLock          -> acquire
//   :308-328  pickReLockTarget     -> pickReLockTarget
//   :330-372  evalSourceSelection  -> evalRateSelection
//   :805-849  the per-packet lock flow  -> admit / forceRelock
//   :1059-1066 live source count / locked flags
//
// The constants below are the firmware's shipping values. They are not
// "tunables to taste": they were settled against real venues (two transmitters
// plus repeaters on one channel) and loosening them brings back the ping-pong
// between two marginal sources that the hysteresis exists to stop.
//
// No Arduino.h / esp_now.h / millis() here — see HapbeatStreamTypes.h.
// ---------------------------------------------------------------------------

#include "HapbeatSourceSelector.h"
#include "../HapbeatStreamModes.h"

#include <string.h>

namespace hapbeat {

namespace {

// espnow_stream.cpp:99-101, :127-130. Verbatim.
const uint32_t SOURCE_LIVE_MS     = 1000;   // :99  window for the "live source" count
const int8_t   RSSI_HYST_DB       = 8;      // :100 challenger must beat the lock by…
const uint32_t RSSI_HOLD_MS       = 400;    // :101 …sustained for this long
const uint32_t SEL_WINDOW_MS      = 4000;   // :127 per-source counting window
const uint8_t  SEL_LOSS_GATE_PCT  = 5;      // :128 lock is "bad" at this unrecovered loss %
const uint32_t SEL_BLACKLIST_MS   = 10000;  // :129 switched-away / force-relocked cool-off
const uint32_t SEL_MIN_SWITCH_MS  = 10000;  // :130 min interval between rate-based switches

const uint8_t ZERO_MAC[6] = {0, 0, 0, 0, 0, 0};

inline bool macEq(const uint8_t* a, const uint8_t* b) {
    return memcmp(a, b, 6) == 0;
}

// The firmware keeps the previously-locked MAC in a dedicated global
// (g_last_lock_mac, :213) so that a re-lock counts as a hand-off only when the
// MAC actually changed. This class has no such member and the header is a
// frozen contract, so a released lock is remembered in `_locked` itself as the
// negative encoding -2-slot (locked() stays `_locked >= 0`). Everything that
// asks "are we locked?" therefore keeps using `_locked >= 0` unchanged, and the
// hand-off counter still ignores a same-source resume after a brief silence.
inline int encodeReleased(int slot) { return -2 - slot; }
inline int rememberedSlot(int locked) {
    if (locked >= 0)  return locked;        // still locked
    if (locked <= -2) return -2 - locked;   // released, previous slot remembered
    return -1;                              // never locked
}

} // namespace

void SourceSelector::begin(const DecoderOptions& opt) {
    memset(_src, 0, sizeof(_src));
    _locked       = -1;
    _lockChanged  = false;
    _handoffs     = 0;
    _winStart     = 0;
    _lastSwitch   = 0;
    _winLost      = 0;
    _winRecovered = 0;
    _mode         = 0;
    setOptions(opt);
    // §7.1.3: relay-test is a diagnostic that must never survive a reboot, so it
    // is seeded from the options exactly once, here (espnow_stream.cpp:770).
    _relayTest = opt.relayTestOnly;
}

void SourceSelector::setOptions(const DecoderOptions& opt) {
    _opt = opt;
    // The source table is fixed-size; a caller asking for more sources than the
    // table holds would index past it in findOrAdd().
    if (_opt.maxSources > MAX_SOURCES) _opt.maxSources = MAX_SOURCES;
    if (_opt.maxSources == 0)          _opt.maxSources = 1;
    // _relayTest is deliberately NOT touched here: setRelayTestOnly() owns it,
    // so a runtime setOptions() (e.g. a fleet-tune lock-timeout change) cannot
    // silently cancel an operator's relay test.
}

void SourceSelector::setRelayTestOnly(bool on) { _relayTest = on; }
bool SourceSelector::relayTestOnly() const     { return _relayTest; }

bool SourceSelector::locked() const { return _locked >= 0; }

bool SourceSelector::lockedRelayed() const {
    return _locked >= 0 && _src[_locked].relayed;   // :1065
}

const uint8_t* SourceSelector::lockedMac() const {
    return _locked >= 0 ? _src[_locked].mac : ZERO_MAC;   // :1066-1067
}

uint32_t SourceSelector::handoffs() const { return _handoffs; }

uint8_t SourceSelector::liveSourceCount(uint32_t now_ms) const {
    uint8_t live = 0;
    for (uint8_t i = 0; i < MAX_SOURCES; i++) {                 // :1059-1062
        if (_src[i].used && (now_ms - _src[i].last_ms) < SOURCE_LIVE_MS) live++;
    }
    return live;
}

bool SourceSelector::consumeLockChanged() {
    bool changed = _lockChanged;
    _lockChanged = false;
    return changed;
}

void SourceSelector::noteLockLoss(uint32_t lost, uint32_t recovered) {
    // The firmware snapshots the cumulative stats counters at each window start
    // and subtracts (:341-342, :370-371). Here the decoder reports increments,
    // so the window totals are accumulated directly and zeroed with the window.
    _winLost      += lost;
    _winRecovered += recovered;
}

// :265-292 — never fails: a full table evicts the stalest non-locked entry.
int SourceSelector::findOrAdd(const uint8_t mac[6], uint32_t now_ms) {
    int      free_slot  = -1;
    int      stalest    = -1;
    uint32_t stalest_age = 0;
    for (int i = 0; i < MAX_SOURCES; i++) {
        if (_src[i].used) {
            if (macEq(_src[i].mac, mac)) return i;
            // Slots at or beyond maxSources can only exist if the cap shrank at
            // runtime; they stay matchable above but are not reallocated.
            if (i != _locked && i < (int)_opt.maxSources) {
                uint32_t age = now_ms - _src[i].last_ms;
                if (stalest < 0 || age > stalest_age) { stalest = i; stalest_age = age; }
            }
        } else if (free_slot < 0 && i < (int)_opt.maxSources) {
            free_slot = i;
        }
    }
    int slot = (free_slot >= 0) ? free_slot : stalest;
    if (slot < 0) slot = 0;   // degenerate (every slot is the lock) — shouldn't happen
    memcpy(_src[slot].mac, mac, 6);
    _src[slot].used            = true;
    _src[slot].last_ms         = now_ms;
    _src[slot].rssi            = RSSI_UNKNOWN;
    _src[slot].hyst_start      = 0;
    _src[slot].last_mode       = 0;
    _src[slot].relayed         = false;
    _src[slot].win_count       = 0;
    _src[slot].blacklist_until = 0;   // a reused slot must not inherit a stale blacklist
    return slot;
}

// :294-306. Always restarts the fade (via _lockChanged) so both a switch and a
// same-source resume start from silence, i.e. no click.
void SourceSelector::acquire(int idx, uint32_t now_ms) {
    (void)now_ms;
    int prev = rememberedSlot(_locked);
    // :299 — a hand-off is a change of source, not merely a (re-)lock. Slots are
    // stable per MAC, so comparing slots is equivalent unless the previous slot
    // was evicted and refilled by a different MAC in the meantime (diagnostics
    // only).
    if (prev >= 0 && prev != idx) _handoffs++;
    _locked      = idx;
    _lockChanged = true;   // decoder: reset seq tracking + arm the cross-fade (:303-304)
    for (int i = 0; i < MAX_SOURCES; i++) _src[i].hyst_start = 0;   // :305
}

// :308-328 — strongest LIVE source; first-come when no RSSI is available (which
// is the normal case on arduino-esp32 2.0.x). Returns -1 when nothing qualifies;
// the firmware's `cur` fallback is applied by the caller in admit().
int SourceSelector::pickReLockTarget(uint32_t now_ms) const {
    int    best      = -1;
    int8_t best_rssi = RSSI_UNKNOWN;
    int    fallback  = -1;   // first live, non-blacklisted source (RSSI-unavailable path)
    for (int i = 0; i < MAX_SOURCES; i++) {
        if (!_src[i].used) continue;
        if ((now_ms - _src[i].last_ms) >= _opt.lockTimeoutMs) continue;   // not live
        // Wrap-safe blacklist check (:258-260).
        if (_src[i].blacklist_until != 0 &&
            (int32_t)(_src[i].blacklist_until - now_ms) > 0) continue;    // §7.1.2 cool-off
        if (fallback < 0) fallback = i;
        if (_src[i].rssi == RSSI_UNKNOWN) continue;                       // need RSSI to rank
        if (best < 0 || _src[i].rssi > best_rssi) { best = i; best_rssi = _src[i].rssi; }
    }
    if (best >= 0)     return best;
    if (fallback >= 0) return fallback;
    return -1;
}

// :330-372 — rate-based selection (§7.1.2, DEC-043). A no-op until the window
// closes; then a clearly-better same-mode challenger may take over a lock that
// is actually dropping packets.
void SourceSelector::evalRateSelection(uint32_t now_ms) {
    if ((now_ms - _winStart) < SEL_WINDOW_MS) return;   // window still open

    if (_opt.selectionEnabled && _locked >= 0 &&
        (now_ms - _lastSwitch) >= SEL_MIN_SWITCH_MS) {
        // Packets the locked source should have delivered during the window.
        uint16_t nominal = (uint16_t)((uint32_t)modeInfo(_mode).ratedPps * SEL_WINDOW_MS / 1000);
        uint32_t unrec = (_winLost > _winRecovered) ? (_winLost - _winRecovered) : 0;
        bool lock_bad  = nominal > 0 &&
                         unrec * 100 >= (uint32_t)nominal * SEL_LOSS_GATE_PCT;
        if (lock_bad) {
            uint16_t lock_cnt = _src[_locked].win_count;
            int      best     = -1;
            uint16_t best_cnt = 0;
            for (int i = 0; i < MAX_SOURCES; i++) {
                if (!_src[i].used || i == _locked)   continue;
                if (_src[i].blacklist_until != 0 &&
                    (int32_t)(_src[i].blacklist_until - now_ms) > 0) continue;
                if (_src[i].last_mode != _mode)      continue;   // same-mode only
                if (_src[i].win_count < nominal / 2) continue;   // near-rated only
                if ((uint32_t)_src[i].win_count * 8 <
                    (uint32_t)lock_cnt * 9)          continue;   // ≥12.5% better
                if (best < 0 || _src[i].win_count > best_cnt) {
                    best = i; best_cnt = _src[i].win_count;
                }
            }
            if (best >= 0) {
                int old = _locked;
                acquire(best, now_ms);                                  // counts a hand-off
                _src[old].blacklist_until = now_ms + SEL_BLACKLIST_MS;
                _lastSwitch = now_ms;
            }
        }
    }

    // Reset the window whether or not we switched (:367-371).
    for (int i = 0; i < MAX_SOURCES; i++) _src[i].win_count = 0;
    _winStart     = now_ms;
    _winLost      = 0;
    _winRecovered = 0;
}

// :814-849 — the per-packet half of the lock flow. Housekeeping that must also
// run without traffic (stale-lock release, rate window) lives in tick().
bool SourceSelector::admit(const uint8_t mac[6], int8_t rssi, uint8_t mode,
                           bool relayed, uint32_t now_ms) {
    // §7.1.3 relay test (:814): only RELAYED packets are candidates — origins are
    // not even tracked, so one receiver is enough to prove the repeater path.
    if (_relayTest && !relayed) return false;

    int idx = findOrAdd(mac, now_ms);
    _src[idx].last_ms   = now_ms;
    _src[idx].last_mode = mode;
    _src[idx].relayed   = relayed;
    if (_src[idx].win_count < 0xFFFF) _src[idx].win_count++;
    if (rssi != RSSI_UNKNOWN) _src[idx].rssi = rssi;

    // Release a stale lock so the first survivor re-locks promptly (:826-829).
    // idx's last_ms was just refreshed, so when idx is the lock this cannot fire.
    if (_locked >= 0 && (now_ms - _src[_locked].last_ms) > _opt.lockTimeoutMs) {
        int t = pickReLockTarget(now_ms);
        if (t >= 0 && t != _locked) acquire(t, now_ms);
        else                        _locked = encodeReleased(_locked);
    }

    if (_locked < 0) {
        // Initial / lock-loss re-lock (:831-833). With every live source
        // blacklisted, pickReLockTarget() finds nothing and we fall back to the
        // packet at hand, exactly as the firmware's `return cur` does (:327) —
        // otherwise a blanket blacklist would deadlock the receiver into silence.
        int t = pickReLockTarget(now_ms);
        acquire(t >= 0 ? t : idx, now_ms);
    } else if (_locked != idx) {
        // Another live source: switch only on a sustained RSSI advantage — never
        // mix two sources (:834-848). With no RSSI this branch never fires, so
        // selection is first-come + liveness, per spec §7.1.
        bool stronger = _src[idx].rssi != RSSI_UNKNOWN &&
                        _src[_locked].rssi != RSSI_UNKNOWN &&
                        _src[idx].rssi >= _src[_locked].rssi + RSSI_HYST_DB;
        if (stronger) {
            if (_src[idx].hyst_start == 0) _src[idx].hyst_start = now_ms;
            if ((now_ms - _src[idx].hyst_start) >= RSSI_HOLD_MS) acquire(idx, now_ms);
        } else {
            _src[idx].hyst_start = 0;
        }
    }

    // Decode ONLY the locked source (:858). Everything else is tracked-only.
    if (_locked != idx) return false;
    // Mirrors the firmware's g_mode, which follows the locked stream (:859) and
    // is what the rate window compares challengers against.
    _mode = mode;
    return true;
}

void SourceSelector::tick(uint32_t now_ms) {
    // Same stale-lock release as in admit(), repeated here so a lock whose source
    // died is dropped even when no packets arrive at all (the firmware relies on
    // the next packet for this; a library user may see total silence instead).
    if (_locked >= 0 && (now_ms - _src[_locked].last_ms) > _opt.lockTimeoutMs) {
        int t = pickReLockTarget(now_ms);
        if (t >= 0 && t != _locked) acquire(t, now_ms);
        else                        _locked = encodeReleased(_locked);
    }
    evalRateSelection(now_ms);
}

// :805-810 — blacklist the current lock and drop it, so the next packet selects
// a different source. Deliberately does not re-lock here.
void SourceSelector::forceRelock(uint32_t now_ms) {
    if (_locked < 0) return;
    _src[_locked].blacklist_until = now_ms + SEL_BLACKLIST_MS;
    _locked = encodeReleased(_locked);
}

} // namespace hapbeat
