// ---------------------------------------------------------------------------
// test_adpcm.cpp — IMA-ADPCM decode.
//
// Guards two things:
//  1. Bit-exactness against the transmitter's encoder. The expected values here
//     were computed by hand from the IMA algorithm (step table + index table),
//     not captured from our own decoder, so a "cleanup" of the arithmetic that
//     changes output is caught.
//  2. Table-bounds safety. step_index arrives straight off the wire with no
//     validation (espnow_stream.cpp loads it from the packet body and from
//     every piggyback entry), so a corrupt or mixed-fleet packet can present
//     step_index > 88. The decoder must clamp BEFORE its first table read;
//     ima_adpcm.cpp:38-42 documents exactly this.
// ---------------------------------------------------------------------------

#include "test_harness.h"
#include "codec/HapbeatImaAdpcm.h"

using namespace hapbeat;

// Hand-worked reference, starting from predictor=0 / step_index=0:
//   nibble 4  : step 7  -> diff 0+7=7        pred    7   idx 0+2 = 2
//   nibble 0  : step 9  -> diff 1            pred    8   idx 2-1 = 1
//   nibble 12 : step 8  -> diff -(1+8)=-9    pred   -1   idx 1+2 = 3
//   nibble 15 : step 10 -> diff -(1+2+5+10)  pred  -19   idx 3+8 = 11
//   nibble 7  : step 21 -> diff 2+5+10+21=38 pred   19   idx 11+8 = 19
HT_TEST(adpcm_known_nibble_sequence) {
    const uint8_t nibbles[5]  = {4, 0, 12, 15, 7};
    const int16_t expected[5] = {7, 8, -1, -19, 19};

    AdpcmState st;   // defaults: predictor 0, step_index 0
    for (int i = 0; i < 5; i++) {
        int16_t got = adpcmDecodeSample(nibbles[i], &st);
        HT_CHECKF(got == expected[i],
                  "nibble[%d]=%u: expected predictor %d, actual %d",
                  i, nibbles[i], (int)expected[i], (int)got);
    }
    HT_CHECK_EQ_WHY(st.step_index, 19, "step index must track INDEX_TABLE");
}

HT_TEST(adpcm_predictor_clamps_to_int16) {
    // Drive the predictor hard positive: nibble 7 is the largest positive step.
    AdpcmState st;
    st.predictor  = 0;
    st.step_index = 88;          // largest step (32767)
    for (int i = 0; i < 8; i++) adpcmDecodeSample(7, &st);
    HT_CHECK_EQ_WHY(st.predictor, 32767, "predictor must saturate, not wrap");

    st.predictor  = 0;
    st.step_index = 88;
    for (int i = 0; i < 8; i++) adpcmDecodeSample(15, &st);   // 15 = 7 negated
    HT_CHECK_EQ_WHY(st.predictor, -32768, "predictor must saturate negative");
}

// The regression that matters most: a wire-supplied step_index of 200 must not
// index STEP_TABLE[200]. If the clamp is moved to after the table read (or
// dropped), this reads ~186 bytes past a 89-entry int16 table.
HT_TEST(adpcm_out_of_range_step_index_is_clamped) {
    AdpcmState st;
    st.predictor  = 0;
    st.step_index = 200;          // impossible on the wire, possible in a corrupt packet

    int16_t got = adpcmDecodeSample(0, &st);

    // Clamped to 88 -> step 32767 -> diff = 32767>>3 = 4095, idx 88-1 = 87.
    HT_CHECK_EQ_WHY(got, 4095, "step_index must be clamped to 88 BEFORE the table read");
    HT_CHECK_EQ_WHY(st.step_index, 87, "index update must run on the clamped value");

    // 0xFF is what a zeroed/garbage byte most often looks like.
    st.predictor  = 0;
    st.step_index = 255;
    got = adpcmDecodeSample(0, &st);
    HT_CHECK_EQ(got, 4095);
    HT_CHECK_EQ(st.step_index, 87);
}

HT_TEST(adpcm_step_index_never_leaves_0_88) {
    // Walk every nibble from every reachable index and assert the invariant.
    for (int start = 0; start <= 88; start++) {
        for (uint8_t n = 0; n < 16; n++) {
            AdpcmState st;
            st.predictor  = 0;
            st.step_index = (uint8_t)start;
            adpcmDecodeSample(n, &st);
            HT_CHECKF(st.step_index <= 88,
                      "start=%d nibble=%u produced step_index %u (> 88)",
                      start, n, st.step_index);
        }
    }
}

// Stereo packing: 1 byte per frame, low nibble = L, high nibble = R.
// Getting this backwards swaps the channels on every mode-0 stream.
HT_TEST(adpcm_stereo_nibble_order_low_is_left) {
    const uint8_t in[2] = {0x04, 0x40};   // frame0: L=4 R=0 | frame1: L=0 R=4
    int16_t out[4] = {0, 0, 0, 0};
    AdpcmState l, r;

    adpcmDecodeBlockStereo(in, out, 2, &l, &r);

    // L sees nibble 4 then 0 -> 7, 8 (the known sequence above).
    HT_CHECK_EQ_WHY(out[0], 7, "low nibble must feed the LEFT channel");
    HT_CHECK_EQ_WHY(out[2], 8, "left channel state must persist across frames");
    // R sees nibble 0 then 4: step 7 -> diff 0 -> pred 0 (idx 0-1 -> 0), then 7.
    HT_CHECK_EQ_WHY(out[1], 0, "high nibble must feed the RIGHT channel");
    HT_CHECK_EQ_WHY(out[3], 7, "right channel state must be independent of left");
}

// Mono packing: 1 byte per 2 samples, low nibble first.
HT_TEST(adpcm_mono_nibble_order_low_first) {
    const uint8_t in[2] = {0x04, 0xFC};   // 4, 0, 12, 15
    int16_t out[4] = {0, 0, 0, 0};
    AdpcmState st;

    adpcmDecodeBlockMono(in, out, 4, &st);

    HT_CHECK_EQ_WHY(out[0], 7, "low nibble of byte 0 is sample 0");
    HT_CHECK_EQ_WHY(out[1], 8, "high nibble of byte 0 is sample 1");
    HT_CHECK_EQ_WHY(out[2], -1, "low nibble of byte 1 is sample 2");
    HT_CHECK_EQ_WHY(out[3], -19, "high nibble of byte 1 is sample 3");
}

// Odd sample counts happen whenever num_frames is odd: the last byte's high
// nibble is padding and must not be decoded into out[samples].
HT_TEST(adpcm_mono_odd_sample_count_ignores_pad_nibble) {
    const uint8_t in[2] = {0x04, 0x7C};   // samples: 12, (pad 7)
    int16_t out[4] = {0, 0, 0, 0x5A5A};
    AdpcmState st;

    adpcmDecodeBlockMono(in, out, 3, &st);

    HT_CHECK_EQ(out[0], 7);
    HT_CHECK_EQ(out[1], 8);
    HT_CHECK_EQ_WHY(out[2], -1, "third sample comes from the low nibble of byte 1");
    HT_CHECK_EQ_WHY(out[3], 0x5A5A, "the pad nibble must not be written to out[3]");
}
