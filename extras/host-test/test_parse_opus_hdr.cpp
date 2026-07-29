// ---------------------------------------------------------------------------
// test_parse_opus_hdr.cpp — THE regression test of this library.
//
// The Opus body carries its frame length in ONE byte:
//     [4]   opus_len
//     [5..] frame
//     then pb_count x [prev_seq(1)][len(1)][data]
//
// contracts/specs/espnow-stream.md §3.3 says uint16 LE. That section is stale.
// The shipping transmitter writes a single byte (audio_source.cpp:798, :802-803)
// and the shipping receiver reads a single byte (espnow_stream.cpp:505, :515).
// Anyone "fixing" this file to match the spec breaks every Opus mode — and the
// failure mode is total silence, not distortion, because the mis-read length
// fails the bounds check and the packet is dropped whole.
//
// These tests run in an Opus-DISABLED build by injecting a fake backend through
// setOpusBackend(): the wire format is what is under test, not libopus.
// ---------------------------------------------------------------------------

#include "test_harness.h"
#include "test_wire.h"

#include "protocol/HapbeatStreamDecoder.h"

using namespace hapbeat;
using namespace tw;

namespace {

// First byte 0x11 is deliberate: with a uint16 read at [4] the length becomes
// 0x1104, which overruns the packet and drops it. That is what makes the
// "expected 1 decode call, actual 0" failure below unambiguous.
const uint8_t FRAME_A[4] = {0x11, 0x22, 0x33, 0x44};
const uint8_t FRAME_B[3] = {0xA1, 0xA2, 0xA3};
const uint8_t FRAME_C[5] = {0x51, 0x52, 0x53, 0x54, 0x55};

void sendOpus(StreamDecoder& dec, const uint8_t mac[6], uint8_t mode,
              uint8_t seq, const uint8_t* frame, uint8_t frameLen,
              uint32_t now) {
    Packet p;
    header(p, mode, seq, 0);
    opusBody(p, frame, frameLen);
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, now);
}

} // namespace

HT_TEST(opus_len_is_one_byte) {
    StubSink sink;
    FakeOpus fake;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);
    dec.setOpusBackend(&fake);

    uint8_t mac[6]; makeMac(mac, 0x21);

    sendOpus(dec, mac, MODE_FAST, 0, FRAME_A, 4, 1000);   // lock + burn the fade

    HT_CHECK_EQ_WHY(dec.stats(1000).modeUnsupported, 0,
                    "a backend given via setOpusBackend() must be used even in a "
                    "build without libopus");
    HT_CHECKF(fake.decodeCalls >= 1,
              "decode() was never called: opus_len at [4] is ONE byte, not "
              "uint16 LE — a uint16 read gives 0x%04X here and drops the packet",
              (unsigned)(FRAME_A[0] << 8 | 4));
    if (fake.decodeCalls == 0) return;   // the rest would only add noise

    sink.clear();
    int before = fake.decodeCalls;

    Packet p;
    header(p, MODE_FAST, 1, 0);
    opusBody(p, FRAME_A, 4);
    HT_CHECK_EQ_WHY(p.len, 4 + 1 + 4, "header(4) + opus_len(1) + frame(4)");

    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1005);

    HT_CHECK_EQ_WHY(fake.decodeCalls, before + 1, "exactly one frame per packet");
    HT_CHECK_EQ_WHY(fake.callLen[before], 4,
                    "the decoder must be handed opus_len bytes, read from [4] as u8");
    HT_CHECK_BYTES(fake.callData[before], FRAME_A, 4, "opus frame bytes (start at [5])");
    HT_CHECK_EQ_WHY(sink.frames, 80, "mode 1 decodes 80 samples/ch into 80 stereo frames");
}

// The piggyback entry header is [prev_seq(1)][len(1)] — the same one-byte
// length. A uint16 read here mis-locates every entry after the first.
HT_TEST(opus_piggyback_entry_is_seq_plus_one_byte_len) {
    StubSink sink;
    FakeOpus fake;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);
    dec.setOpusBackend(&fake);

    uint8_t mac[6]; makeMac(mac, 0x22);

    sendOpus(dec, mac, MODE_FAST, 10, FRAME_A, 4, 1000);   // expected seq 11
    sink.clear();
    int before = fake.decodeCalls;

    // seq 11 and 12 are lost; seq 13 carries copies of both.
    Packet p;
    header(p, MODE_FAST, 13, 2);
    opusBody(p, FRAME_A, 4);
    HT_CHECK_EQ_WHY(p.len, 9, "pb region starts right after the primary frame");
    opusPb(p, 11, FRAME_B, 3);
    opusPb(p, 12, FRAME_C, 5);
    HT_CHECK_EQ_WHY(p.len, 9 + (2 + 3) + (2 + 5),
                    "each pb entry is prev_seq(1) + len(1) + data");

    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1010);

    StreamStats st = dec.stats(1010);
    HT_CHECK_EQ(st.packetsLost, 2);
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 2, "both pb copies must be found");
    HT_CHECK_EQ_WHY(fake.decodeCalls, before + 3, "pb 11, pb 12, then the primary");

    // Order and content: recovery runs in sequence order, oldest first. The
    // SECOND entry is the one that proves the stride.
    HT_CHECK_EQ_WHY(fake.callLen[before], 3, "first recovered frame is seq 11 (3 bytes)");
    HT_CHECK_BYTES(fake.callData[before], FRAME_B, 3, "seq 11 frame bytes");
    HT_CHECK_EQ_WHY(fake.callLen[before + 1], 5, "second recovered frame is seq 12 (5 bytes)");
    HT_CHECK_BYTES(fake.callData[before + 1], FRAME_C, 5, "seq 12 frame bytes");
    HT_CHECK_EQ_WHY(fake.callLen[before + 2], 4, "then the packet's own frame");
}

HT_TEST(opus_rejects_length_past_end_of_packet) {
    StubSink sink;
    FakeOpus fake;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);
    dec.setOpusBackend(&fake);

    uint8_t mac[6]; makeMac(mac, 0x23);

    sendOpus(dec, mac, MODE_FAST, 0, FRAME_A, 4, 1000);
    sink.clear();
    int    beforeCalls = fake.decodeCalls;
    uint32_t beforePkts = dec.stats(1000).packetsReceived;

    // Declares 200 bytes of frame, carries 4.
    Packet p;
    header(p, MODE_FAST, 1, 0);
    p.u8(200);
    p.raw(FRAME_A, 4);
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1005);

    HT_CHECK_EQ_WHY(fake.decodeCalls, beforeCalls,
                    "an opus_len past the packet end must drop the packet");
    HT_CHECK_EQ_WHY(dec.stats(1005).packetsReceived, beforePkts,
                    "a dropped packet must not be counted as received");
    HT_CHECK_EQ(sink.frames, 0);

    // A pb entry whose declared length overruns must be skipped, not read.
    Packet q;
    header(q, MODE_FAST, 1, 1);
    opusBody(q, FRAME_A, 4);
    q.u8(0);       // prev_seq
    q.u8(250);     // len — far past the end
    q.u8(0xEE);
    dec.onPacket(mac, q.b, q.len, RSSI_UNKNOWN, 1006);
    HT_CHECK(!sink.overflow);
}

// With no backend the packet must be dropped and counted, never decoded as
// noise. This is what keeps a plain (Opus-less) build silent on an Opus stream.
HT_TEST(opus_without_backend_counts_mode_unsupported) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);
    // deliberately no setOpusBackend()

    uint8_t mac[6]; makeMac(mac, 0x24);
    sendOpus(dec, mac, MODE_FAST, 0, FRAME_A, 4, 1000);
    sendOpus(dec, mac, MODE_FAST, 1, FRAME_A, 4, 1005);

    StreamStats st = dec.stats(1005);
    HT_CHECKF(st.modeUnsupported >= 1,
              "expected modeUnsupported >= 1, actual %u — an undecodable mode "
              "must be counted, not silently ignored", (unsigned)st.modeUnsupported);
    HT_CHECK_EQ_WHY(sink.frames, 0, "no backend must mean no audio, not noise");
    HT_CHECK_EQ_WHY(hapbeat::opusAvailable(), 0,
                    "the host test builds without -DHAPBEAT_ESPNOW_RX_OPUS");
    HT_CHECK_EQ_WHY(hapbeat::modeSupported(MODE_SOLID), 1,
                    "ADPCM modes are supported in every build");
}

// A gap with no piggyback copy is concealed, not skipped: dropping the frame
// outright leaves a hole the jitter ring then has to paper over.
HT_TEST(opus_gap_without_piggyback_uses_plc) {
    StubSink sink;
    FakeOpus fake;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);
    dec.setOpusBackend(&fake);

    uint8_t mac[6]; makeMac(mac, 0x25);

    sendOpus(dec, mac, MODE_FAST, 0, FRAME_A, 4, 1000);   // expected 1
    sink.clear();
    fake.concealCalls = 0;

    sendOpus(dec, mac, MODE_FAST, 2, FRAME_A, 4, 1005);   // seq 1 lost, no pb

    StreamStats st = dec.stats(1005);
    HT_CHECK_EQ(st.packetsLost, 1);
    HT_CHECK_EQ(st.piggybackRecovered, 0);
    HT_CHECK_EQ_WHY(fake.concealCalls, 1, "one lost frame -> one PLC frame");
    HT_CHECK_EQ_WHY(st.plcConcealed, 1, "PLC must be counted in stats");
}

// Mono and stereo Opus modes need differently shaped decoders; a mode switch
// must re-create the backend or the first stereo frame decodes as noise.
HT_TEST(opus_mode_switch_resets_backend_channels) {
    StubSink sink;
    FakeOpus fake;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);
    dec.setOpusBackend(&fake);

    uint8_t mac[6]; makeMac(mac, 0x26);

    sendOpus(dec, mac, MODE_FAST, 0, FRAME_A, 4, 1000);     // mono
    HT_CHECK_EQ_WHY(fake.channels(), 1, "mode 1 (FAST) is Opus mono");

    fake.samplesPerFrame = 160;                              // HIFI frame length
    sendOpus(dec, mac, MODE_HIFI, 1, FRAME_A, 4, 1010);      // stereo
    HT_CHECK_EQ_WHY(fake.channels(), 2, "mode 5 (HIFI) is Opus stereo");
    HT_CHECK_EQ_WHY(dec.mode(), MODE_HIFI, "mode() must follow the wire mode id");
}
