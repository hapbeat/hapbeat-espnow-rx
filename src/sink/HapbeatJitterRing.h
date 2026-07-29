#pragma once
// ---------------------------------------------------------------------------
// HapbeatJitterRing.h — the jitter buffer between the radio and the DAC.
//
// This is the part that is easy to get wrong, so it ships in the library rather
// than in an example. Three things it has to do:
//
// 1. Absorb Wi-Fi burst jitter. Measured RF loss on a clean channel is a few
//    percent arriving in short bursts, so a buffer shallower than the burst
//    runs dry and clicks.
//
// 2. Correct clock drift. The transmitter's ADC clock and this receiver's DAC
//    clock are independent crystals (~0.1-0.5 % apart). With a fixed buffer the
//    fill level walks steadily to empty or full and dropouts appear every few
//    seconds, forever. We nudge by ±1 frame per block to hold fill near target.
//
// 3. De-click an under-run. When it does run dry, hold the last frame and decay
//    it to silence rather than emitting a hard zero edge.
//
// Behaviour mirrors hapbeat-device-firmware/src/audio_stream.cpp; the default
// geometry is that firmware's per-mode table (see HapbeatStreamModes).
//
// Lock-free single-producer / single-consumer: write() is called from the
// ESP-NOW receive callback, read() from the audio task. Exactly one thread each.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <atomic>
#include "../protocol/HapbeatStreamTypes.h"

namespace hapbeat {

class JitterRing : public FrameSink {
public:
    ~JitterRing();

    // `capacityFrames` must be a power of two and larger than any max depth
    // you intend to use. 4096 frames = 256 ms @16 kHz is the firmware default.
    bool begin(uint32_t capacityFrames);
    void end();

    // --- FrameSink (producer side) -----------------------------------------
    void     writeFrames(const int16_t* interleaved, uint16_t frames) override;
    void     setJitterFrames(uint32_t prebuf, uint32_t target, uint32_t max) override;
    void     reprime() override;
    uint32_t droppedFrames() const override;
    uint32_t fillFrames() const override;

    // --- consumer side ------------------------------------------------------
    // Fills `frames` stereo frames, applying `gain`. Returns frames actually
    // sourced from the ring; the remainder is hold-and-decay or silence, so the
    // output buffer is always fully written.
    uint32_t read(int16_t* interleavedOut, uint32_t frames, float gain);

    // Estimated playout delay from the current fill level.
    uint32_t estDelayMs() const;

private:
    int16_t* _buf      = nullptr;
    uint32_t _capacity = 0;      // frames (power of two)
    uint32_t _mask     = 0;

    std::atomic<uint32_t> _w{0};
    std::atomic<uint32_t> _r{0};

    uint32_t _prebuf = 480;
    uint32_t _target = 480;
    uint32_t _max    = 720;

    std::atomic<uint32_t> _dropped{0};
    bool     _priming   = true;
    int16_t  _holdL     = 0;
    int16_t  _holdR     = 0;
    uint16_t _decay     = 0;
    int32_t  _driftAcc  = 0;
};

} // namespace hapbeat
