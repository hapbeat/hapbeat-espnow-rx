// ---------------------------------------------------------------------------
// HapbeatImaAdpcm.cpp — IMA-ADPCM decode (see header).
//
// Ported from hapbeat-device-firmware/src/ima_adpcm.cpp (decode half only).
// Tables and arithmetic are byte-identical to the firmware; the transmitter's
// encoder mirrors this exactly, so any deviation desynchronises the predictor.
// ---------------------------------------------------------------------------

#include "HapbeatImaAdpcm.h"

namespace hapbeat {
namespace {

// IMA ADPCM step size table (89 entries).
const int16_t STEP_TABLE[89] = {
    7,     8,     9,     10,    11,    12,    13,    14,
    16,    17,    19,    21,    23,    25,    28,    31,
    34,    37,    41,    45,    50,    55,    60,    66,
    73,    80,    88,    97,    107,   118,   130,   143,
    157,   173,   190,   209,   230,   253,   279,   307,
    337,   371,   408,   449,   494,   544,   598,   658,
    724,   796,   876,   963,   1060,  1166,  1282,  1411,
    1552,  1707,  1878,  2066,  2272,  2499,  2749,  3024,
    3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,
    7132,  7845,  8630,  9493,  10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
    32767
};

// Step index adjustment per nibble value.
const int8_t INDEX_TABLE[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

const uint8_t STEP_INDEX_MAX = 88;

} // namespace

int16_t adpcmDecodeSample(uint8_t nibble, AdpcmState* st) {
    // Clamp BEFORE the table read, not only after the update. step_index is
    // loaded straight out of packet bytes (the per-packet ADPCM state that lets
    // a receiver join mid-stream), so a corrupt or foreign-fleet packet can hand
    // us a value > 88 that would read past STEP_TABLE on the very first sample.
    // The firmware only had the post-update clamp until device-firmware
    // 5dc0918; this library takes external input directly, so the pre-lookup
    // clamp is mandatory here.
    if (st->step_index > STEP_INDEX_MAX) st->step_index = STEP_INDEX_MAX;
    int step = STEP_TABLE[st->step_index];

    int diff = step >> 3;
    if (nibble & 1) diff += step >> 2;
    if (nibble & 2) diff += step >> 1;
    if (nibble & 4) diff += step;
    if (nibble & 8) diff = -diff;

    int pred = st->predictor + diff;
    if (pred > 32767)  pred = 32767;
    if (pred < -32768) pred = -32768;
    st->predictor = (int16_t)pred;

    int idx = st->step_index + INDEX_TABLE[nibble & 0x0F];
    if (idx < 0)                    idx = 0;
    if (idx > (int)STEP_INDEX_MAX)  idx = STEP_INDEX_MAX;
    st->step_index = (uint8_t)idx;

    return st->predictor;
}

void adpcmDecodeBlockStereo(const uint8_t* in, int16_t* out, size_t frames,
                            AdpcmState* left, AdpcmState* right) {
    // 1 byte per stereo frame: low nibble L, high nibble R.
    for (size_t i = 0; i < frames; i++) {
        uint8_t byte = in[i];
        out[i * 2]     = adpcmDecodeSample(byte & 0x0F, left);
        out[i * 2 + 1] = adpcmDecodeSample((byte >> 4) & 0x0F, right);
    }
}

void adpcmDecodeBlockMono(const uint8_t* in, int16_t* out, size_t samples,
                          AdpcmState* st) {
    // 1 byte per 2 samples: low nibble N, high nibble N+1. An odd `samples`
    // leaves the high nibble of the last byte unused (matches the encoder).
    for (size_t i = 0; i < samples; i += 2) {
        uint8_t byte = in[i / 2];
        out[i] = adpcmDecodeSample(byte & 0x0F, st);
        if (i + 1 < samples) {
            out[i + 1] = adpcmDecodeSample((byte >> 4) & 0x0F, st);
        }
    }
}

} // namespace hapbeat
