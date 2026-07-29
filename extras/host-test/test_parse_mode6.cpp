// ---------------------------------------------------------------------------
// test_parse_mode6.cpp — modes 6 LITE / 7 TURBO (IMA-ADPCM 8 kHz mono).
//
// Same codec as mode 0 but a different body geometry, which is exactly why it
// needs its own test: the state block is 3 bytes (not 6) and the data offset is
// 8 (not 11), so a decoder that reuses the stereo offsets produces plausible
// garbage rather than an obvious failure.
//
// Layout (espnow_stream.cpp:44-45, :725-729, :640-671):
//   [4]     num_frames        (8 kHz mono samples)
//   [5,6]   predictor (LE)
//   [7]     step_index
//   [8..]   ceil(num_frames/2) bytes, 2 samples per byte, low nibble first
//   then pb_count x [prev_seq(1)][pred_lo][pred_hi][step][data] = 4+ceil(nf/2)
//
// Output is nf*2 stereo frames: x2 linear interpolation to 16 kHz, L=R.
// ---------------------------------------------------------------------------

#include "test_harness.h"
#include "test_wire.h"

#include "protocol/HapbeatStreamDecoder.h"

using namespace hapbeat;
using namespace tw;

namespace {

const uint8_t NF     = 40;                 // 5 ms @8 kHz — the shipping frame size
const int     DBYTES = (NF + 1) / 2;       // 20

// Reference: decode mono, then x2 interpolate and duplicate L=R exactly as
// decodeAdpcmMonoToRing() does (espnow_stream.cpp:645-660). The final mid
// sample duplicates the last sample (there is no nf-th sample to average with).
void referenceMono(const uint8_t* data, int nf, int16_t pred, uint8_t step,
                   int16_t* outStereo) {
    int16_t mono[128];
    AdpcmState st;
    st.predictor  = pred;
    st.step_index = step;
    adpcmDecodeBlockMono(data, mono, (size_t)nf, &st);

    for (int i = 0; i < nf; i++) {
        int cur = mono[i];
        int nxt = (i + 1 < nf) ? mono[i + 1] : cur;
        int16_t mid = (int16_t)((cur + nxt) >> 1);
        outStereo[i * 4 + 0] = (int16_t)cur;
        outStereo[i * 4 + 1] = (int16_t)cur;
        outStereo[i * 4 + 2] = mid;
        outStereo[i * 4 + 3] = mid;
    }
}

void prime(StreamDecoder& dec, StubSink& sink, const uint8_t mac[6],
           uint8_t mode, uint8_t seq, const uint8_t* payload) {
    Packet p;
    header(p, mode, seq, 0);
    adpcmMonoBody(p, NF, 0, 0, payload);
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1000);
    sink.clear();
}

} // namespace

HT_TEST(mode6_body_offsets_and_exact_pcm) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x11);
    uint8_t audio[DBYTES];
    fillPattern(audio, DBYTES, 0x600D);

    prime(dec, sink, mac, MODE_LITE, 20, audio);

    const int16_t pred = -777;
    const uint8_t step = 23;

    Packet p;
    header(p, MODE_LITE, 21, 0);
    adpcmMonoBody(p, NF, pred, step, audio);
    HT_CHECK_EQ_WHY(p.len, 8 + DBYTES,
                    "mono state is 3 bytes at offset 5; data starts at offset 8");

    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1005);

    HT_CHECK_EQ_WHY(sink.frames, (uint32_t)NF * 2,
                    "8 kHz mono upsamples x2 into 16 kHz stereo frames");

    int16_t expect[NF * 4];
    referenceMono(audio, NF, pred, step, expect);
    HT_CHECK_PCM(sink.pcm, expect, NF * 4, "mode6 pcm");

    // L must equal R on every frame — the haptic output is mono content.
    for (int f = 0; f < NF * 2; f++) {
        HT_CHECKF(sink.pcm[f * 2] == sink.pcm[f * 2 + 1],
                  "frame %d: L=%d R=%d — mono must be duplicated to both channels",
                  f, (int)sink.pcm[f * 2], (int)sink.pcm[f * 2 + 1]);
    }

    HT_CHECK_EQ_WHY(dec.mode(), MODE_LITE, "mode() must follow the wire mode id");
}

// Mode 7 is byte-identical on the wire; only the jitter depth differs.
HT_TEST(mode7_uses_the_same_wire_as_mode6) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x12);
    uint8_t audio[DBYTES];
    fillPattern(audio, DBYTES, 0x7777);

    prime(dec, sink, mac, MODE_TURBO, 0, audio);

    Packet p;
    header(p, MODE_TURBO, 1, 0);
    adpcmMonoBody(p, NF, 55, 4, audio);
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1005);

    int16_t expect[NF * 4];
    referenceMono(audio, NF, 55, 4, expect);
    HT_CHECK_EQ(sink.frames, (uint32_t)NF * 2);
    HT_CHECK_PCM(sink.pcm, expect, NF * 4, "mode7 pcm");
    HT_CHECK_EQ(dec.mode(), MODE_TURBO);
}

HT_TEST(mode6_odd_num_frames_sizes_body_with_ceil) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x13);
    const uint8_t oddNf = 41;
    const int     bytes = (oddNf + 1) / 2;   // 21
    uint8_t audio[64];
    fillPattern(audio, bytes, 0x0DD);

    Packet p;
    header(p, MODE_LITE, 0, 0);
    p.u8(oddNf);
    p.u16le(0);
    p.u8(0);
    p.raw(audio, bytes);
    HT_CHECK_EQ_WHY(p.len, 8 + bytes, "odd nf must round the byte count up");

    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1000);
    HT_CHECK_EQ(sink.frames, (uint32_t)oddNf * 2);
}

HT_TEST(mode6_rejects_truncated_body) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x14);
    uint8_t audio[DBYTES];
    fillPattern(audio, DBYTES, 9);

    prime(dec, sink, mac, MODE_LITE, 0, audio);
    uint32_t basePackets = dec.stats(1000).packetsReceived;

    // Declares 40 samples (20 bytes) but carries 19.
    Packet p;
    header(p, MODE_LITE, 1, 0);
    p.u8(NF);
    p.u16le(0);
    p.u8(0);
    p.raw(audio, DBYTES - 1);
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1001);

    HT_CHECK_EQ_WHY(sink.frames, 0, "a truncated mono body must be dropped");
    HT_CHECK_EQ(dec.stats(1001).packetsReceived, basePackets);
}

// The mono piggyback entry has its own geometry (4 + ceil(nf/2) bytes each, up
// to 3 of them). Reading it with the stereo stride silently mis-parses entry 2.
HT_TEST(mode6_piggyback_entry_stride) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x15);
    uint8_t a[DBYTES], b[DBYTES], c[DBYTES];
    fillPattern(a, DBYTES, 0xA0);
    fillPattern(b, DBYTES, 0xB0);
    fillPattern(c, DBYTES, 0xC0);

    prime(dec, sink, mac, MODE_LITE, 100, a);   // expected 101

    // 101 and 102 lost; 103 arrives carrying copies of both.
    Packet p;
    header(p, MODE_LITE, 103, 2);
    adpcmMonoBody(p, NF, 0, 0, c);
    HT_CHECK_EQ_WHY(p.len, 8 + DBYTES, "pb region starts at 8 + ceil(nf/2)");
    adpcmMonoPb(p, 101, NF, 11, 2, a);
    adpcmMonoPb(p, 102, NF, 22, 5, b);
    HT_CHECK_EQ_WHY(p.len, 8 + DBYTES + 2 * (4 + DBYTES),
                    "each mono pb entry is prev_seq(1) + state(3) + data");

    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1010);

    StreamStats st = dec.stats(1010);
    HT_CHECK_EQ(st.packetsLost, 2);
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 2, "both pb entries must be found");
    HT_CHECK_EQ_WHY(sink.frames, (uint32_t)NF * 2 * 3,
                    "two recovered frames plus the primary");

    // Recovery order matters: 101 then 102, then the primary. Verifying the
    // SECOND entry's audio proves the stride (a wrong stride still finds #1).
    int16_t expect101[NF * 4], expect102[NF * 4], expect103[NF * 4];
    referenceMono(a, NF, 11, 2, expect101);
    referenceMono(b, NF, 22, 5, expect102);
    referenceMono(c, NF, 0, 0, expect103);

    HT_CHECK_PCM(sink.pcm, expect101, NF * 4, "recovered seq 101");
    HT_CHECK_PCM(sink.pcm + NF * 4, expect102, NF * 4, "recovered seq 102");
    HT_CHECK_PCM(sink.pcm + NF * 8, expect103, NF * 4, "primary seq 103");
}

// A pb_count larger than the packet actually carries must stop at the packet
// end rather than walking off it.
HT_TEST(mode6_pb_count_larger_than_packet_is_bounded) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x16);
    uint8_t audio[DBYTES];
    fillPattern(audio, DBYTES, 0x5A);

    prime(dec, sink, mac, MODE_LITE, 0, audio);   // expected 1

    Packet p;
    header(p, MODE_LITE, 3, 3);          // claims 3 pb entries...
    adpcmMonoBody(p, NF, 0, 0, audio);
    adpcmMonoPb(p, 1, NF, 0, 0, audio);  // ...but carries one
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1010);

    StreamStats st = dec.stats(1010);
    HT_CHECK_EQ(st.packetsLost, 2);
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 1,
                    "only the pb entry that actually fits may be used");
    HT_CHECK(!sink.overflow);
}
