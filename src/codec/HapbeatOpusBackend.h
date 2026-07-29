#pragma once
// ---------------------------------------------------------------------------
// HapbeatOpusBackend.h — optional Opus decode.
//
// Opus is OFF by default so the library has zero dependencies and still plays
// the transmitter's default mode (SOLID / IMA-ADPCM). Without it, Opus-mode
// packets are counted in StreamStats::modeUnsupported and dropped — the
// receiver stays silent instead of playing noise.
//
// To enable it, add libopus to your project and register the shipped backend
// FROM YOUR SKETCH, before begin():
//
//   ; platformio.ini
//   lib_deps = https://github.com/pschatzmann/arduino-libopus.git
//
//   // sketch
//   #include <codec/HapbeatOpusBackendImpl.h>
//   void setup() { hapbeat::useOpusBackend(); rx.begin(cfg); }
//
// Why from the sketch and not from inside the library: PlatformIO compiles each
// library in isolation with only its own declared dependencies on the include
// path. A dependency listed in the project's lib_deps is NOT visible while
// compiling this library, so an `#include <opus.h>` in one of our .cpp files
// fails no matter which lib_ldf_mode you pick. Sketch translation units DO get
// the project include paths, so putting the implementation in a header that the
// sketch includes is what makes an optional dependency work at all. The
// alternative — declaring libopus in library.json — would force every
// ADPCM-only user to download and build it.
//
// The decoder is created at 16 kHz regardless of the mode's wire rate: libopus
// resamples internally, so an 8 kHz or 48 kHz stream decodes straight to this
// library's 16 kHz output with no resampler of our own. This mirrors what the
// firmware does (espnow_stream.cpp:441-470).
// ---------------------------------------------------------------------------

#include <stdint.h>

namespace hapbeat {

class OpusBackend {
public:
    virtual ~OpusBackend() {}

    // (Re)create the decoder for `channels` (1 or 2) at 16 kHz. Returns false
    // if allocation failed; the caller then treats Opus modes as unsupported.
    virtual bool reset(uint8_t channels) = 0;

    // Decode one frame. `out` holds up to `maxSamples` samples PER CHANNEL.
    // Returns samples/ch decoded, or <= 0 on error.
    virtual int decode(const uint8_t* data, int len, int16_t* out, int maxSamples) = 0;

    // Packet-loss concealment: synthesize `samples` samples/ch.
    virtual int conceal(int16_t* out, int samples) = 0;

    virtual uint8_t channels() const = 0;
};

// Install the backend the library should use. Pass nullptr to remove it.
// Call before HapbeatEspNowRx::begin(). Normally you don't call this directly —
// hapbeat::useOpusBackend() in HapbeatOpusBackendImpl.h does it for you.
void registerOpusBackend(OpusBackend* backend);

// The registered backend, or nullptr when none was installed.
OpusBackend* defaultOpusBackend();

// True when Opus modes can actually be decoded right now.
bool opusAvailable();

} // namespace hapbeat
