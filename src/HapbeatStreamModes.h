#pragma once
// ---------------------------------------------------------------------------
// HapbeatStreamModes.h — stream mode table (framework independent)
//
// The transmitter puts a mode id in every 0xAA packet. Sender and receiver MUST
// agree on codec / sample rate / channels for that id, so this table is a
// verbatim copy of the firmware's:
//   hapbeat-device-firmware/src/espnow_stream.cpp:61-72  (RX_MODE)
//   hapbeat-device-firmware/src/espnow_stream.cpp:174-185 (MODE_RATED_PPS)
//   hapbeat-device-firmware/src/espnow_stream.cpp:52-53   (STREAM_MODE_NAME)
// Do not "improve" these numbers — they are a wire contract.
//
// This library always outputs 16 kHz interleaved stereo PCM16, whatever the
// wire rate/channels of the mode (8 kHz mono modes are upsampled ×2 and
// duplicated L=R, matching the firmware).
// ---------------------------------------------------------------------------

#include <stdint.h>

namespace hapbeat {

// ESP-NOW payload types we understand. 0xAB exists in the spec but is NOT
// implemented by any shipping firmware — every mode rides on 0xAA + mode_id.
enum : uint8_t { PKT_STREAM = 0xAA, PKT_FLEET_TUNE = 0xAC };

enum : uint8_t { CODEC_ADPCM = 0, CODEC_OPUS = 1 };

static const uint8_t  MODE_COUNT   = 10;
static const uint16_t OUTPUT_RATE  = 16000;  // this library's PCM output rate
static const int8_t   RSSI_UNKNOWN = -128;

// Mode ids, named as the transmitter UI shows them.
enum : uint8_t {
    MODE_SOLID    = 0,  // IMA-ADPCM 16k stereo   — transmitter default
    MODE_FAST     = 1,  // Opus 8k mono   5 ms
    MODE_BALANCED = 2,  // Opus 8k mono   5 ms
    MODE_SMOOTH   = 3,  // Opus 8k mono  10 ms
    MODE_STEREO   = 4,  // Opus 8k stereo 10 ms
    MODE_HIFI     = 5,  // Opus 16k stereo 10 ms
    MODE_LITE     = 6,  // IMA-ADPCM 8k mono 5 ms
    MODE_TURBO    = 7,  // same wire as LITE, shallow jitter buffer
    MODE_FINE     = 8,  // Opus 16k mono 10 ms
    MODE_SOLID48  = 9,  // Opus 48k stereo 10 ms (decoded here at 16 kHz)
};

struct ModeInfo {
    uint8_t     codec;       // CODEC_ADPCM / CODEC_OPUS
    uint8_t     channels;    // channels on the wire (1 or 2)
    uint16_t    outSamples;  // samples/ch per frame at 16 kHz (Opus PLC length)
    uint8_t     piggyback;   // sender's pb depth (informative; wire pb_count rules)
    uint16_t    bufFrames;   // recommended jitter depth, in 16 kHz stereo frames
    uint16_t    ratedPps;    // nominal packets/sec (denominator for rate select)
    const char* name;        // "SOLID" .. "SOLID48"
};

// Out-of-range ids return mode 0 / "?" rather than reading past the table.
const ModeInfo& modeInfo(uint8_t id);
const char*     modeName(uint8_t id);

// Recommended jitter buffer depth in ms for a mode (16 frames == 1 ms @16 kHz).
inline uint16_t suggestedBufferMs(uint8_t id) { return modeInfo(id).bufFrames / 16; }

// True when this build can actually decode `id`. Opus modes need the optional
// Opus backend (-DHAPBEAT_ESPNOW_RX_OPUS); ADPCM modes always work.
bool modeSupported(uint8_t id);

} // namespace hapbeat
