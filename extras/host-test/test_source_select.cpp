// ---------------------------------------------------------------------------
// test_source_select.cpp — §7.1 "lock onto exactly one transmitter".
//
// A venue runs several transmitters (one or two sources plus repeaters) all
// broadcasting the same programme. A receiver that decoded more than one would
// play doubled, smeared audio, so the selector must admit packets from ONE MAC
// and merely track the rest. The failure mode of a broken lock is not silence —
// it is audio that sounds "thick" and drifts — which is why it needs a test
// rather than a listen.
//
// Ported behaviour: espnow_stream.cpp:253-372, :812-870.
// ---------------------------------------------------------------------------

#include "test_harness.h"
#include "test_wire.h"

#include "protocol/HapbeatSourceSelector.h"

using namespace hapbeat;
using namespace tw;

namespace {

const uint8_t MODE = MODE_SOLID;

bool macSame(const uint8_t* a, const uint8_t* b) {
    return memcmp(a, b, 6) == 0;
}

} // namespace

HT_TEST(select_first_source_wins_and_others_are_rejected) {
    SourceSelector sel;
    sel.begin(defaultOptions());

    uint8_t a[6], b[6];
    makeMac(a, 0xA1);
    makeMac(b, 0xB1);

    HT_CHECK_EQ_WHY(sel.locked(), 0, "nothing is locked before the first packet");

    HT_CHECK_EQ_WHY(sel.admit(a, RSSI_UNKNOWN, MODE, false, 1000), 1,
                    "the first source seen must be locked (first-come, no RSSI)");
    HT_CHECK(sel.locked());
    HT_CHECK_EQ_WHY(macSame(sel.lockedMac(), a), 1, "lockedMac must be source A");
    HT_CHECK_EQ_WHY(sel.consumeLockChanged(), 1, "the lock change must be reported once");
    HT_CHECK_EQ_WHY(sel.consumeLockChanged(), 0, "...and only once");

    // B is live and broadcasting the same programme — it must be tracked but
    // never decoded while A is healthy.
    for (uint32_t t = 1005; t <= 1100; t += 5) {
        HT_CHECKF(sel.admit(b, RSSI_UNKNOWN, MODE, false, t) == false,
                  "t=%u: source B was admitted while A holds the lock — two "
                  "sources would be mixed", (unsigned)t);
        HT_CHECK(sel.admit(a, RSSI_UNKNOWN, MODE, false, t) == true);
        sel.tick(t);
    }

    HT_CHECK_EQ_WHY(macSame(sel.lockedMac(), a), 1, "the lock must not drift to B");
    HT_CHECK_EQ_WHY(sel.liveSourceCount(1100), 2, "both sources are still tracked");
}

// A lock is released only when the source actually goes quiet. lockTimeoutMs is
// the whole hysteresis budget: too short and marginal sources ping-pong.
HT_TEST(select_silent_lock_is_released_after_lock_timeout) {
    SourceSelector sel;
    DecoderOptions opt = defaultOptions();
    HT_CHECK_EQ_WHY(opt.lockTimeoutMs, 150, "shipping LOCK_TIMEOUT_MS (espnow_stream.cpp:98)");
    sel.begin(opt);

    uint8_t a[6], b[6];
    makeMac(a, 0xA2);
    makeMac(b, 0xB2);

    sel.admit(a, RSSI_UNKNOWN, MODE, false, 1000);
    HT_CHECK(sel.locked());

    // A is silent from here on; B keeps broadcasting. Well inside the timeout,
    // B must not take over — that hysteresis is what stops two marginal sources
    // ping-ponging and seaming the audio.
    for (uint32_t t = 1050; t <= 1100; t += 50) {
        HT_CHECKF(sel.admit(b, RSSI_UNKNOWN, MODE, false, t) == false,
                  "t=%u: B stole a lock that is only %u ms silent (timeout %u ms)",
                  (unsigned)t, (unsigned)(t - 1000), (unsigned)opt.lockTimeoutMs);
        sel.tick(t);
    }

    // Past the timeout, the first live survivor must take over promptly.
    bool tookOver = false;
    for (uint32_t t = 1200; t <= 1400 && !tookOver; t += 50) {
        sel.tick(t);
        if (sel.admit(b, RSSI_UNKNOWN, MODE, false, t)) tookOver = true;
    }
    HT_CHECKF(tookOver,
              "B never took over: a lock silent for >%u ms must be released",
              (unsigned)opt.lockTimeoutMs);
    HT_CHECK_EQ_WHY(macSame(sel.lockedMac(), b), 1, "lockedMac must be source B");
    HT_CHECK_EQ_WHY(sel.consumeLockChanged(), 1, "a hand-off must be reported");
    HT_CHECKF(sel.handoffs() >= 1, "expected handoffs >= 1, actual %u",
              (unsigned)sel.handoffs());
}

// idleTick equivalent: a dead lock must be releasable with no traffic at all,
// otherwise a receiver whose only source died keeps a stale lock forever.
HT_TEST(select_lock_released_by_tick_alone) {
    SourceSelector sel;
    DecoderOptions opt = defaultOptions();
    sel.begin(opt);

    uint8_t a[6];
    makeMac(a, 0xA3);

    sel.admit(a, RSSI_UNKNOWN, MODE, false, 1000);
    HT_CHECK(sel.locked());

    sel.tick(1000 + opt.lockTimeoutMs * 10);
    HT_CHECK_EQ_WHY(sel.locked(), 0,
                    "a silent lock must be released by tick() without any packet");
}

// forceRelock is the operator's "this repeater is bad, move" button (CONFIG
// Btn3). Blacklisting is what stops the selector re-picking the same source on
// the very next packet.
HT_TEST(select_force_relock_blacklists_the_old_source) {
    SourceSelector sel;
    sel.begin(defaultOptions());

    uint8_t a[6], b[6];
    makeMac(a, 0xA4);
    makeMac(b, 0xB4);

    HT_CHECK(sel.admit(a, RSSI_UNKNOWN, MODE, false, 1000) == true);
    HT_CHECK(sel.admit(b, RSSI_UNKNOWN, MODE, false, 1010) == false);   // tracked
    sel.consumeLockChanged();

    sel.forceRelock(1020);

    HT_CHECK_EQ_WHY(sel.admit(a, RSSI_UNKNOWN, MODE, false, 1020), 0,
                    "the force-relocked source must be skipped while blacklisted");
    HT_CHECK_EQ_WHY(macSame(sel.lockedMac(), b), 1,
                    "the other live source must take the lock");
    HT_CHECK_EQ_WHY(sel.admit(b, RSSI_UNKNOWN, MODE, false, 1030), 1,
                    "B is now the locked source");
    HT_CHECKF(sel.handoffs() >= 1, "expected handoffs >= 1, actual %u",
              (unsigned)sel.handoffs());
}

// Degenerate case: force-relock with nothing else to move to. Skipping every
// candidate would deadlock the receiver into permanent silence, so the current
// source is allowed back rather than leaving no lock at all.
HT_TEST(select_force_relock_with_one_source_does_not_deadlock) {
    SourceSelector sel;
    sel.begin(defaultOptions());

    uint8_t a[6];
    makeMac(a, 0xA5);

    sel.admit(a, RSSI_UNKNOWN, MODE, false, 1000);
    sel.forceRelock(1010);

    HT_CHECK_EQ_WHY(sel.admit(a, RSSI_UNKNOWN, MODE, false, 1010), 1,
                    "with no alternative, the only source must be re-admitted");
    HT_CHECK(sel.locked());
}

// §7.1.3 relay test: only RELAYED packets are candidates, so a single receiver
// proves the repeater path works. It must never be persisted (a device that
// booted in relay-test mode would look dead), hence the default assertion.
HT_TEST(select_relay_test_only_admits_relayed_packets) {
    HT_CHECK_EQ_WHY(defaultOptions().relayTestOnly, 0,
                    "relayTestOnly MUST default to false — it is never persisted");

    SourceSelector sel;
    sel.begin(defaultOptions());
    HT_CHECK_EQ(sel.relayTestOnly(), 0);

    uint8_t a[6];
    makeMac(a, 0xA6);

    sel.setRelayTestOnly(true);
    HT_CHECK_EQ(sel.relayTestOnly(), 1);

    HT_CHECK_EQ_WHY(sel.admit(a, RSSI_UNKNOWN, MODE, false, 1000), 0,
                    "a direct (non-relayed) packet must be ignored in relay test");
    HT_CHECK_EQ_WHY(sel.locked(), 0,
                    "an origin must not even be tracked in relay test");

    HT_CHECK_EQ_WHY(sel.admit(a, RSSI_UNKNOWN, MODE, true, 1005), 1,
                    "a RELAYED packet must be admitted");
    HT_CHECK(sel.locked());
    HT_CHECK_EQ_WHY(sel.lockedRelayed(), 1, "lockedRelayed must follow bit7");

    sel.setRelayTestOnly(false);
    HT_CHECK_EQ_WHY(sel.admit(a, RSSI_UNKNOWN, MODE, false, 1010), 1,
                    "leaving relay test must restore normal admission");
    HT_CHECK_EQ_WHY(sel.lockedRelayed(), 0,
                    "lockedRelayed must track the LAST packet, not the lock");
}

// A repeater's copy carries bit7; the decoder shows it as "RELAYED" in stats so
// an installer can tell which path is actually feeding the receiver.
HT_TEST(select_locked_relayed_tracks_bit7) {
    SourceSelector sel;
    sel.begin(defaultOptions());

    uint8_t a[6];
    makeMac(a, 0xA7);

    sel.admit(a, RSSI_UNKNOWN, MODE, true, 1000);
    HT_CHECK_EQ(sel.lockedRelayed(), 1);
    sel.admit(a, RSSI_UNKNOWN, MODE, false, 1005);
    HT_CHECK_EQ(sel.lockedRelayed(), 0);
}

// The source table is a fixed 8 slots. More transmitters than that must evict
// the stalest entry, never the lock and never past the end of the array.
HT_TEST(select_source_table_overflow_evicts_without_losing_the_lock) {
    SourceSelector sel;
    sel.begin(defaultOptions());

    uint8_t a[6];
    makeMac(a, 0xA8);
    HT_CHECK(sel.admit(a, RSSI_UNKNOWN, MODE, false, 1000) == true);

    // Twice the table size of strangers, with A staying alive throughout.
    for (int i = 0; i < SourceSelector::MAX_SOURCES * 2; i++) {
        uint8_t other[6];
        makeMac(other, (uint8_t)(0x40 + i));
        uint32_t t = 1000 + (uint32_t)i;
        HT_CHECKF(sel.admit(other, RSSI_UNKNOWN, MODE, false, t) == false,
                  "stranger %d was admitted while A holds the lock", i);
        HT_CHECK(sel.admit(a, RSSI_UNKNOWN, MODE, false, t) == true);
    }

    HT_CHECK_EQ_WHY(macSame(sel.lockedMac(), a), 1,
                    "the locked source must never be evicted from the table");
    HT_CHECKF(sel.liveSourceCount(1100) <= SourceSelector::MAX_SOURCES,
              "liveSourceCount %u exceeds the table size",
              (unsigned)sel.liveSourceCount(1100));
}

HT_TEST(select_live_source_count_ages_out) {
    SourceSelector sel;
    sel.begin(defaultOptions());

    uint8_t a[6], b[6];
    makeMac(a, 0xA9);
    makeMac(b, 0xB9);

    sel.admit(a, RSSI_UNKNOWN, MODE, false, 1000);
    sel.admit(b, RSSI_UNKNOWN, MODE, false, 1000);
    HT_CHECK_EQ(sel.liveSourceCount(1000), 2);

    HT_CHECK_EQ_WHY(sel.liveSourceCount(1000 + 60000), 0,
                    "sources silent for a minute are not live");
}
