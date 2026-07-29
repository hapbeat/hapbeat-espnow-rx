// ---------------------------------------------------------------------------
// test_parse_mode0.cpp — mode 0 SOLID (IMA-ADPCM 16 kHz stereo), the
// transmitter's default mode and therefore the one that MUST work in every
// build, Opus or not.
//
// Layout under test (espnow_stream.cpp:42-43, :690-737):
//   [4]      num_frames        (1..64; MAX_FRAMES guard)
//   [5..10]  predL(LE) stepL predR(LE) stepR
//   [11..]   1 ADPCM byte per stereo frame
//   then pb_count x [prev_seq(1)][state(6)][data(num_frames)]
//
// A shifted state or data offset still "decodes" — it just produces noise — so
// the value assertions here are what actually pin the offsets down.
// ---------------------------------------------------------------------------

#include "test_harness.h"
#include "test_wire.h"

#include "protocol/HapbeatStreamDecoder.h"

using namespace hapbeat;
using namespace tw;

namespace {

const uint8_t NF = 64;   // MAX_FRAMES: the largest legal mode-0 packet

// Reference decode of a mode-0 body, using the codec directly. If the decoder
// reads state or data from the wrong offset, this will not match.
void referenceStereo(const uint8_t* data, uint8_t numFrames,
                     int16_t predL, uint8_t stepL,
                     int16_t predR, uint8_t stepR, int16_t* out) {
    AdpcmState l, r;
    l.predictor = predL; l.step_index = stepL;
    r.predictor = predR; r.step_index = stepR;
    adpcmDecodeBlockStereo(data, out, numFrames, &l, &r);
}

// Locks the decoder onto `mac` and burns the 48-frame hand-off fade, so the
// packet the test actually inspects is reproduced unattenuated.
void prime(StreamDecoder& dec, StubSink& sink, const uint8_t mac[6],
           uint8_t seq, const uint8_t* payload) {
    Packet p;
    header(p, 0, seq, 0);
    adpcmStereoBody(p, NF, 0, 0, 0, 0, payload);
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1000);
    sink.clear();
}

} // namespace

HT_TEST(mode0_body_offsets_and_exact_pcm) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x01);
    uint8_t audio[NF];
    fillPattern(audio, NF, 0xC0FFEE);

    prime(dec, sink, mac, 10, audio);

    // Non-trivial state: a decoder that ignored the packet's state (or read it
    // from the wrong offset) would produce a completely different waveform.
    const int16_t predL = -1234; const uint8_t stepL = 17;
    const int16_t predR =  4321; const uint8_t stepR = 42;

    Packet p;
    header(p, 0, 11, 0);
    adpcmStereoBody(p, NF, predL, stepL, predR, stepR, audio);
    HT_CHECK_EQ_WHY(p.len, 11 + NF, "mode-0 data must start at offset 11");

    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1010);

    HT_CHECK_EQ_WHY(sink.frames, NF, "one stereo frame out per ADPCM byte in");

    int16_t expect[NF * 2];
    referenceStereo(audio, NF, predL, stepL, predR, stepR, expect);
    HT_CHECK_PCM(sink.pcm, expect, NF * 2, "mode0 pcm");

    StreamStats st = dec.stats(1010);
    HT_CHECK_EQ(st.packetsReceived, 2);
    HT_CHECK_EQ(st.packetsLost, 0);
    HT_CHECK_EQ(st.mode, 0);
}

HT_TEST(mode0_rejects_bad_num_frames) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x02);
    uint8_t audio[NF];
    fillPattern(audio, NF, 7);

    prime(dec, sink, mac, 0, audio);
    uint32_t basePackets = dec.stats(1000).packetsReceived;

    // num_frames = 0 — nothing to decode.
    Packet zero;
    header(zero, 0, 1, 0);
    adpcmStereoBody(zero, 0, 0, 0, 0, 0, audio);
    dec.onPacket(mac, zero.b, zero.len, RSSI_UNKNOWN, 1001);

    // num_frames = 65 > MAX_FRAMES: would overrun the decode scratch buffer.
    Packet big;
    header(big, 0, 1, 0);
    big.u8(65);
    adpcmStereoState(big, 0, 0, 0, 0);
    big.raw(audio, NF);
    big.u8(0);                        // 65 bytes of "data"
    dec.onPacket(mac, big.b, big.len, RSSI_UNKNOWN, 1002);

    HT_CHECK_EQ_WHY(sink.frames, 0, "malformed num_frames must produce no audio");
    HT_CHECK_EQ_WHY(dec.stats(1002).packetsReceived, basePackets,
                    "a rejected packet must not count as received");
}

HT_TEST(mode0_rejects_truncated_body) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x03);
    uint8_t audio[NF];
    fillPattern(audio, NF, 11);

    prime(dec, sink, mac, 0, audio);
    uint32_t basePackets = dec.stats(1000).packetsReceived;

    // Claims 64 frames but carries only 63 bytes of data: reading frame 63
    // would be one byte past the packet.
    Packet p;
    header(p, 0, 1, 0);
    p.u8(NF);
    adpcmStereoState(p, 0, 0, 0, 0);
    p.raw(audio, NF - 1);
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1001);

    HT_CHECK_EQ_WHY(sink.frames, 0, "a truncated body must be dropped, not decoded");
    HT_CHECK_EQ(dec.stats(1001).packetsReceived, basePackets);

    // A header-only runt must not be read past either.
    uint8_t runt[5] = {0xAA, 0x00, 2, 0, NF};
    dec.onPacket(mac, runt, 5, RSSI_UNKNOWN, 1002);
    HT_CHECK_EQ(sink.frames, 0);
}

// Mode 0 carries pb 1: exactly one redundant copy of the previous frame, and
// the firmware only uses it for a single-packet gap (espnow_stream.cpp:715-719).
HT_TEST(mode0_piggyback_recovers_single_gap) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x04);
    uint8_t audioA[NF], audioB[NF];
    fillPattern(audioA, NF, 0xAAAA);
    fillPattern(audioB, NF, 0xBBBB);

    prime(dec, sink, mac, 5, audioA);      // expected seq is now 6

    // seq 7 arrives; seq 6 was lost but rides along as the piggyback copy.
    const int16_t pbPredL = 100; const uint8_t pbStepL = 3;
    const int16_t pbPredR = -200; const uint8_t pbStepR = 9;

    Packet p;
    header(p, 0, 7, 1);
    adpcmStereoBody(p, NF, 0, 0, 0, 0, audioB);
    HT_CHECK_EQ_WHY(p.len, 11 + NF, "piggyback must start at 11 + num_frames");
    adpcmStereoPb(p, 6, NF, pbPredL, pbStepL, pbPredR, pbStepR, audioA);
    HT_CHECK_EQ_WHY(p.len, 11 + NF + 7 + NF,
                    "stereo pb entry is prev_seq(1) + state(6) + data(N)");

    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1010);

    StreamStats st = dec.stats(1010);
    HT_CHECK_EQ_WHY(st.packetsLost, 1, "seq 6 is a real loss");
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 1, "seq 6 must be rebuilt from the pb copy");
    HT_CHECK_EQ_WHY(sink.frames, NF * 2, "recovered frame then primary frame");

    // The recovered audio comes first and must decode with the PB state.
    int16_t expect[NF * 2];
    referenceStereo(audioA, NF, pbPredL, pbStepL, pbPredR, pbStepR, expect);
    HT_CHECK_PCM(sink.pcm, expect, NF * 2, "mode0 piggyback pcm");
}

// prev_seq is a filter, not decoration: a copy of some other packet must not be
// spliced in as if it were the missing one.
HT_TEST(mode0_piggyback_ignored_when_prev_seq_mismatches) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x05);
    uint8_t audio[NF];
    fillPattern(audio, NF, 3);

    prime(dec, sink, mac, 5, audio);       // expected 6

    Packet p;
    header(p, 0, 7, 1);
    adpcmStereoBody(p, NF, 0, 0, 0, 0, audio);
    adpcmStereoPb(p, 99, NF, 0, 0, 0, 0, audio);   // wrong prev_seq
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1010);

    StreamStats st = dec.stats(1010);
    HT_CHECK_EQ(st.packetsLost, 1);
    HT_CHECK_EQ_WHY(st.piggybackRecovered, 0,
                    "a pb entry for a different seq must not be used");
}

// Mode 0 is the one mode that must decode with no Opus backend attached.
HT_TEST(mode0_needs_no_opus_backend) {
    StubSink sink;
    StreamDecoder dec;
    dec.begin(defaultOptions(), sink);

    uint8_t mac[6]; makeMac(mac, 0x06);
    uint8_t audio[NF];
    fillPattern(audio, NF, 5);

    Packet p;
    header(p, 0, 0, 0);
    adpcmStereoBody(p, NF, 0, 0, 0, 0, audio);
    dec.onPacket(mac, p.b, p.len, RSSI_UNKNOWN, 1000);

    HT_CHECK_EQ_WHY(sink.frames, NF, "ADPCM must decode without Opus");
    HT_CHECK_EQ_WHY(dec.stats(1000).modeUnsupported, 0,
                    "mode 0 is never 'unsupported'");
}
