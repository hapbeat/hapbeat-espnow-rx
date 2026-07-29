// ---------------------------------------------------------------------------
// HapbeatStreamModes.cpp — the stream mode table (see header).
//
// Verbatim merge of three firmware tables into one row per mode:
//   STREAM_MODE_NAME  hapbeat-device-firmware/src/espnow_stream.cpp:52-53
//   RX_MODE           hapbeat-device-firmware/src/espnow_stream.cpp:61-72
//   MODE_RATED_PPS    hapbeat-device-firmware/src/espnow_stream.cpp:174-185
// These are a wire contract with the transmitter (audio_source.cpp MODE_DEFS):
// no value here may be "tuned" locally or the fleet stops interoperating.
// ---------------------------------------------------------------------------

#include "HapbeatStreamModes.h"
#include "codec/HapbeatOpusBackend.h"

namespace hapbeat {
namespace {

// codec       ch  out@16k  pb  bufFrames  pps  name
// pb is the *sender's* redundancy depth and is documentation only: the parser
// follows the wire pb_count (capped at 3), exactly as the firmware does.
const ModeInfo kModes[MODE_COUNT] = {
    { CODEC_ADPCM, 2,   0, 1,  480, 250, "SOLID"    },  // 0 ADPCM 16k stereo,  30 ms buffer
    { CODEC_OPUS,  1,  80, 3,  192, 200, "FAST"     },  // 1 Opus 8k mono   5 ms, 12 ms
    { CODEC_OPUS,  1,  80, 3,  288, 200, "BALANCED" },  // 2 Opus 8k mono   5 ms, 18 ms
    { CODEC_OPUS,  1, 160, 2,  448, 100, "SMOOTH"   },  // 3 Opus 8k mono  10 ms, 28 ms
    { CODEC_OPUS,  2, 160, 2,  288, 100, "STEREO"   },  // 4 Opus 8k stereo 10 ms, 18 ms (pb 2: pb 3 exceeds 250 B)
    { CODEC_OPUS,  2, 160, 2,  448, 100, "HIFI"     },  // 5 Opus 16k stereo 10 ms, 28 ms
    { CODEC_ADPCM, 1,  80, 3,  256, 200, "LITE"     },  // 6 ADPCM 8k mono→16k 5 ms, 16 ms
    { CODEC_ADPCM, 1,  80, 3,   64, 200, "TURBO"    },  // 7 same wire as LITE, 4 ms buffer
    { CODEC_OPUS,  1, 160, 2,  448, 100, "FINE"     },  // 8 Opus 16k mono 10 ms, 28 ms
    { CODEC_OPUS,  2, 480, 2, 1920, 100, "SOLID48"  },  // 9 Opus 48k stereo 10 ms (decoded at 16k here)
};

} // namespace

const ModeInfo& modeInfo(uint8_t id) {
    // mode_id comes off the wire with only bit7 (RELAYED) stripped, so anything
    // 0..127 can arrive. Never index past the table; mode 0 is the safe default
    // (it is also the transmitter default).
    return kModes[id < MODE_COUNT ? id : 0];
}

const char* modeName(uint8_t id) {
    return id < MODE_COUNT ? kModes[id].name : "?";
}

bool modeSupported(uint8_t id) {
    if (id >= MODE_COUNT) return false;   // unknown mode: drop, don't guess
    if (kModes[id].codec == CODEC_OPUS) return opusAvailable();
    return true;                          // ADPCM needs no optional dependency
}

} // namespace hapbeat
