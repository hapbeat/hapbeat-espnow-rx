#pragma once
// ---------------------------------------------------------------------------
// HapbeatImaAdpcm.h — IMA-ADPCM decoder (decode only, no dependencies).
//
// Ported from hapbeat-device-firmware/src/ima_adpcm.{h,cpp}. The encoder side
// is intentionally not included: this is a receive-only library.
//
// Wire nibble packing (must match the transmitter):
//   stereo (mode 0): 1 byte per frame — low nibble = L, high nibble = R
//   mono (mode 6/7): 1 byte per 2 samples — low nibble = sample N,
//                    high nibble = sample N+1
// ---------------------------------------------------------------------------

#include <stddef.h>
#include <stdint.h>

namespace hapbeat {

// Decoder state carried in every packet so a receiver can start mid-stream.
struct AdpcmState {
    int16_t predictor  = 0;
    uint8_t step_index = 0;
};

// Decode one 4-bit nibble, advancing `st`.
int16_t adpcmDecodeSample(uint8_t nibble, AdpcmState* st);

// `in` holds `frames` bytes (1 byte/frame). `out` receives `frames * 2` int16
// values, interleaved L,R.
void adpcmDecodeBlockStereo(const uint8_t* in, int16_t* out, size_t frames,
                            AdpcmState* left, AdpcmState* right);

// `in` holds ceil(samples/2) bytes. `out` receives `samples` mono int16 values.
void adpcmDecodeBlockMono(const uint8_t* in, int16_t* out, size_t samples,
                          AdpcmState* st);

} // namespace hapbeat
