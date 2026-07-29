// ---------------------------------------------------------------------------
// test_seq_piggyback.cpp — sequence tracking, loss accounting and recovery.
//
// The seq byte wraps at 256, so every comparison has to be modular. The three
// ways to get this wrong all look like working audio on the bench and only show
// up in a venue:
//   * a wrap counted as a 255-packet loss (stats junk, PLC storm every ~4 s)
//   * a stream splice counted as loss (§7.1.1: a repeater switching upstream or
//     an origin reboot jumps the seq; concealing that gap floods the ring)
//   * an unbounded recovery loop (a long outage running ~127 decodes inside the
//     Wi-Fi receive callback — recoverCap exists to stop exactly that)
//
// Mode 6 (ADPCM 8 kHz mono) is used throughout: it carries up to 3 piggyback
// copies and needs no Opus backend.
// Reference: espnow_stream.cpp:700-722, :663-681.
// ---------------------------------------------------------------------------

#include "test_harness.h"
#include "test_wire.h"

#include "protocol/HapbeatStreamDecoder.h"

using namespace hapbeat;
using namespace tw;

namespace {

const uint8_t NF     = 40;
const int     DBYTES = (NF + 1) / 2;

struct Rig {
    StubSink      sink;
    StreamDecoder dec;
    uint8_t       mac[6];
    uint8_t       audio[DBYTES];

    Rig() {
        makeMac(mac, 0x31);
        fillPattern(audio, DBYTES, 0x51E9);
    }

    void start(const DecoderOptions& opt) { dec.begin(opt, sink); }

    // Plain packet, no piggyback.
    void send(uint8_t seq, uint32_t now) {
        Packet p;
        header(p, MODE_LITE, seq, 0);
        adpcmMonoBody(p, NF, 0, 0, audio);
        dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, now);
    }

    // Packet carrying redundant copies of `pbSeqs`.
    void sendWithPb(uint8_t seq, const uint8_t* pbSeqs, int n, uint32_t now) {
        Packet p;
        header(p, MODE_LITE, seq, (uint8_t)n);
        adpcmMonoBody(p, NF, 0, 0, audio);
        for (int i = 0; i < n; i++) adpcmMonoPb(p, pbSeqs[i], NF, 0, 0, audio);
        dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, now);
    }
};

} // namespace

// The first packet establishes the sequence; it can never be a loss.
HT_TEST(seq_first_packet_does_not_count_loss) {
    Rig r;
    r.start(defaultOptions());

    r.send(200, 1000);

    StreamStats st = r.dec.stats(1000);
    HT_CHECK_EQ(st.packetsReceived, 1);
    HT_CHECK_EQ_WHY(st.packetsLost, 0, "there is nothing to compare the first seq against");
    HT_CHECK_EQ(st.resyncs, 0);
}

HT_TEST(seq_wraps_255_to_0_without_loss) {
    Rig r;
    r.start(defaultOptions());

    r.send(253, 1000);
    r.send(254, 1005);
    r.send(255, 1010);
    r.send(0,   1015);   // wrap
    r.send(1,   1020);
    r.send(2,   1025);

    StreamStats st = r.dec.stats(1025);
    HT_CHECK_EQ(st.packetsReceived, 6);
    HT_CHECK_EQ_WHY(st.packetsLost, 0,
                    "255->0 is consecutive; seq arithmetic must be modulo 256");
    HT_CHECK_EQ(st.resyncs, 0);
    HT_CHECK_EQ(st.maxConsecutiveLost, 0);
}

// A gap that also wraps must still count as the small gap it is.
HT_TEST(seq_gap_across_the_wrap_counts_correctly) {
    Rig r;
    r.start(defaultOptions());

    r.send(254, 1000);   // expected 255
    r.send(2,   1005);   // 255, 0, 1 lost -> gap 3

    StreamStats st = r.dec.stats(1005);
    HT_CHECK_EQ_WHY(st.packetsLost, 3, "gap must be computed as (uint8)(seq - expected)");
    HT_CHECK_EQ(st.maxConsecutiveLost, 3);
    HT_CHECK_EQ(st.resyncs, 0);
}

// §7.1.1: a forward jump of resyncGap or more is a stream splice, not loss.
HT_TEST(seq_large_gap_is_a_resync_not_a_loss) {
    Rig r;
    DecoderOptions opt = defaultOptions();
    r.start(opt);
    HT_CHECK_EQ_WHY(opt.resyncGap, 20, "shipping default (espnow_stream.cpp:131)");

    r.send(0, 1000);                       // expected 1
    r.send((uint8_t)(1 + opt.resyncGap), 1005);   // gap == resyncGap

    StreamStats st = r.dec.stats(1005);
    HT_CHECK_EQ_WHY(st.resyncs, 1, "gap >= resyncGap is a splice");
    HT_CHECK_EQ_WHY(st.packetsLost, 0, "a splice must not inflate the loss counter");
    HT_CHECK_EQ(st.plcConcealed, 0);

    // ...and the boundary below it still counts as loss.
    Rig r2;
    r2.start(opt);
    r2.send(0, 1000);
    r2.send((uint8_t)(1 + opt.resyncGap - 1), 1005);   // gap == resyncGap - 1

    StreamStats st2 = r2.dec.stats(1005);
    HT_CHECK_EQ_WHY(st2.resyncs, 0, "gap < resyncGap must not be a splice");
    HT_CHECK_EQ(st2.packetsLost, opt.resyncGap - 1);
}

// A splice must not go looking for piggyback copies either — it is not loss.
HT_TEST(seq_resync_skips_piggyback_search) {
    Rig r;
    DecoderOptions opt = defaultOptions();
    r.start(opt);

    r.send(0, 1000);                       // expected 1
    const uint8_t pb[3] = {1, 2, 3};
    r.sendWithPb((uint8_t)(1 + opt.resyncGap), pb, 3, 1005);

    StreamStats st = r.dec.stats(1005);
    HT_CHECK_EQ(st.resyncs, 1);
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 0,
                    "no recovery on a splice — those seqs were never sent to us");
    HT_CHECK_EQ_WHY(st.packetsLost, 0, "still not loss");
}

// After a splice, tracking must continue from the new sequence.
HT_TEST(seq_resync_rebases_expected_seq) {
    Rig r;
    DecoderOptions opt = defaultOptions();
    r.start(opt);

    r.send(0, 1000);
    r.send(100, 1005);        // splice
    r.send(101, 1010);        // consecutive with the new base
    r.send(102, 1015);

    StreamStats st = r.dec.stats(1015);
    HT_CHECK_EQ(st.resyncs, 1);
    HT_CHECK_EQ_WHY(st.packetsLost, 0, "expected seq must rebase to seq+1 after a splice");
}

HT_TEST(seq_resync_gap_is_runtime_tunable) {
    Rig r;
    DecoderOptions opt = defaultOptions();
    r.start(opt);
    r.dec.setResyncGap(8);    // 0xAC param 4

    r.send(0, 1000);          // expected 1
    r.send(9, 1005);          // gap 8

    StreamStats st = r.dec.stats(1005);
    HT_CHECK_EQ_WHY(st.resyncs, 1, "setResyncGap(8) must make a gap of 8 a splice");
    HT_CHECK_EQ(st.packetsLost, 0);
}

// A late/duplicate packet (backwards seq) must not produce a ~255 loss burst.
HT_TEST(seq_backwards_gap_is_not_counted_as_loss) {
    Rig r;
    r.start(defaultOptions());

    r.send(50, 1000);         // expected 51
    r.send(10, 1005);         // (uint8)(10-51) = 215 -> >= 128, neither loss nor splice

    StreamStats st = r.dec.stats(1005);
    HT_CHECK_EQ_WHY(st.packetsLost, 0, "a backwards seq must never count 215 losses");
    HT_CHECK_EQ(st.resyncs, 0);
    HT_CHECK(!r.sink.overflow);
}

// Piggyback beats concealment: if a copy of the missing frame is in the packet,
// the real audio must be used.
HT_TEST(seq_piggyback_is_preferred_over_silence) {
    Rig r;
    r.start(defaultOptions());

    r.send(0, 1000);                       // expected 1
    r.sink.clear();

    const uint8_t pb[2] = {1, 2};
    r.sendWithPb(3, pb, 2, 1005);          // gap 2, both copies present

    StreamStats st = r.dec.stats(1005);
    HT_CHECK_EQ(st.packetsLost, 2);
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 2, "both missing frames were carried");
    HT_CHECK_EQ_WHY(r.sink.frames, (uint32_t)NF * 2 * 3,
                    "two recovered frames at full length, then the primary");
}

// Only the missing seqs may be recovered; a copy of an already-played frame
// must not be replayed.
HT_TEST(seq_piggyback_for_already_played_frame_is_ignored) {
    Rig r;
    r.start(defaultOptions());

    r.send(0, 1000);
    r.send(1, 1005);                       // expected 2
    r.sink.clear();

    const uint8_t pb[2] = {0, 1};          // both already played
    r.sendWithPb(2, pb, 2, 1010);          // gap 0

    StreamStats st = r.dec.stats(1010);
    HT_CHECK_EQ(st.packetsLost, 0);
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 0, "nothing was missing");
    HT_CHECK_EQ_WHY(r.sink.frames, (uint32_t)NF * 2, "only the primary frame plays");
}

// recoverCap bounds the work done inside the receive callback: for a gap larger
// than the cap, only the NEWEST recoverCap frames are reconstructed. The older
// ones are dropped on purpose — the jitter ring would have trimmed them anyway.
HT_TEST(seq_gap_over_recover_cap_recovers_only_the_newest) {
    Rig r;
    DecoderOptions opt = defaultOptions();
    r.start(opt);
    HT_CHECK_EQ_WHY(opt.recoverCap, 12, "shipping OPUS_RECOVER_CAP (espnow_stream.cpp:109)");
    HT_CHECK_EQ_WHY(opt.silenceCapFrames, 64, "shipping SILENCE_CAP (espnow_stream.cpp:103)");

    r.send(0, 1000);                       // expected 1
    r.sink.clear();

    // seq 16: gap 15 — over the cap of 12, but under resyncGap 20 so it is loss.
    const uint8_t gap = 15;
    const uint8_t pb[3] = {13, 14, 15};    // copies of the three newest
    r.sendWithPb(16, pb, 3, 1005);

    StreamStats st = r.dec.stats(1005);
    HT_CHECK_EQ_WHY(st.packetsLost, gap, "the whole gap is still reported as lost");
    HT_CHECK_EQ(st.maxConsecutiveLost, gap);
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 3, "the three carried copies must be used");

    // Frames out: (recoverCap - 3) concealment gaps at silenceCapFrames each,
    // 3 recovered frames, then the primary. Anything larger means the cap is
    // not being applied and a real outage would spin the receive callback.
    const uint32_t perFrame = (uint32_t)NF * 2;                 // 80
    const uint32_t silence  = (uint32_t)(opt.recoverCap - 3) * opt.silenceCapFrames;
    HT_CHECK_EQ_WHY(r.sink.frames, silence + 3 * perFrame + perFrame,
                    "recovery must run exactly recoverCap times, newest first");
}

// The recovery walk must land exactly on the newest frames, not on an offset
// window: a copy of the oldest missing frame is outside the cap and must not
// resurrect it out of order.
HT_TEST(seq_recover_window_starts_at_seq_minus_cap) {
    Rig r;
    DecoderOptions opt = defaultOptions();
    r.start(opt);

    r.send(0, 1000);                       // expected 1
    r.sink.clear();

    // gap 15, window = seq-12 .. seq-1 = 4..15. A copy of seq 2 is outside it.
    const uint8_t pb[3] = {2, 3, 15};
    r.sendWithPb(16, pb, 3, 1005);

    StreamStats st = r.dec.stats(1005);
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 1,
                    "only seq 15 is inside the recoverCap window (4..15)");
}

HT_TEST(seq_stats_reset_clears_counters) {
    Rig r;
    r.start(defaultOptions());

    r.send(0, 1000);
    r.send(5, 1005);
    HT_CHECK(r.dec.stats(1005).packetsLost > 0);

    r.dec.resetStats();

    StreamStats st = r.dec.stats(1005);
    HT_CHECK_EQ(st.packetsReceived, 0);
    HT_CHECK_EQ(st.packetsLost, 0);
    HT_CHECK_EQ(st.maxConsecutiveLost, 0);
    HT_CHECK_EQ(st.piggybackRecovered, 0);
}
