// ---------------------------------------------------------------------------
// HapbeatJitterRing.cpp — jitter buffer between the radio and the DAC.
//
// Behaviour is ported from hapbeat-device-firmware/src/audio_stream.cpp:
//   ringWrite / ringRead / ringAvailable   :252-279  (lock-free SPSC indices)
//   pre-buffer gate (s_prebuf_done)        :81, :551, :626
//   hard-trim + ±1 frame/block drift       :630-655
//   under-run hold-and-decay (63/64)       :659-681
//   max<=target guard                      :884
// That file is welded to duowl_v4 / sw_eq / Preferences / the 48 kHz HP ring /
// the UDP format=2 path, so this is a re-implementation of the same behaviour
// rather than a lift of the code. The numbers (63/64 decay, target/4+32
// deadband, one frame per block) are carried over verbatim: they are tuned
// against real transmitters, not derived.
//
// Threading: single producer (ESP-NOW receive callback) calls writeFrames();
// single consumer (audio task) calls read(). Only the consumer ever advances
// _r, which is what makes the trims below safe — see reprime() for why the
// "drop buffered audio" of a splice is also done on the consumer side.
// ---------------------------------------------------------------------------

#include "HapbeatJitterRing.h"
#include "../HapbeatStreamModes.h"

#include <stdlib.h>
#include <string.h>

namespace hapbeat {

namespace {

// Frames per millisecond at the library's fixed 16 kHz output rate.
const uint32_t kFramesPerMs = OUTPUT_RATE / 1000;

// How long the under-run conceal keeps holding+decaying before it declares the
// source gone and emits flat silence. 63/64 per frame reaches integer zero on
// its own well inside this, so the cap only bounds the "concealed" accounting
// during a long outage (otherwise droppedFrames() would climb forever while
// nothing is being transmitted at all).
const uint16_t kConcealCapFrames = 512;   // 32 ms @16 kHz

inline int16_t clip16(int32_t v) {
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

// Under-run conceal: hold the last frame and decay it toward silence. A hard
// zero clicks; holding the last sample is inaudible for a brief gap, and the
// fade means a source that truly stopped ends in silence rather than a DC
// offset held against the actuator (firmware audio_stream.cpp:668-670).
// Free function because the class layout is fixed and this needs no members
// beyond the four it is handed.
inline void concealFrame(int16_t& holdL, int16_t& holdR, uint16_t& decay,
                         std::atomic<uint32_t>& dropped) {
    if (decay >= kConcealCapFrames) {
        holdL = 0;
        holdR = 0;
        return;   // fully faded: steady silence, not counted as a loss
    }
    holdL = (int16_t)((int32_t)holdL * 63 / 64);
    holdR = (int16_t)((int32_t)holdR * 63 / 64);
    decay++;
    dropped.fetch_add(1, std::memory_order_relaxed);
}

} // namespace

JitterRing::~JitterRing() {
    end();
}

bool JitterRing::begin(uint32_t capacityFrames) {
    end();
    // Power of two is not a nicety: the read/write paths index with a mask so
    // the indices can free-run and wrap as plain uint32 counters.
    if (capacityFrames < 64) return false;
    if (capacityFrames & (capacityFrames - 1)) return false;

    _buf = (int16_t*)malloc((size_t)capacityFrames * 2 * sizeof(int16_t));
    if (!_buf) return false;
    memset(_buf, 0, (size_t)capacityFrames * 2 * sizeof(int16_t));

    _capacity = capacityFrames;
    _mask     = capacityFrames - 1;

    _w.store(0, std::memory_order_relaxed);
    _r.store(0, std::memory_order_relaxed);
    _dropped.store(0, std::memory_order_relaxed);

    _priming  = true;
    _holdL    = 0;
    _holdR    = 0;
    _decay    = 0;
    _driftAcc = 0;
    return true;
}

void JitterRing::end() {
    if (_buf) free(_buf);
    _buf      = nullptr;
    _capacity = 0;
    _mask     = 0;
    _priming  = true;
}

// --- producer side ---------------------------------------------------------

void JitterRing::writeFrames(const int16_t* interleaved, uint16_t frames) {
    if (!_buf || !interleaved || frames == 0) return;

    const uint32_t w    = _w.load(std::memory_order_relaxed);
    const uint32_t r    = _r.load(std::memory_order_acquire);
    const uint32_t free = _capacity - (w - r);

    uint32_t n = frames;
    if (n > free) {
        // Ring full: drop the newest, same as the firmware's ringWrite
        // (audio_stream.cpp:262). The reader is not keeping up or the sender
        // burst exceeded the ring; either way the delay must not grow.
        _dropped.fetch_add(n - free, std::memory_order_relaxed);
        n = free;
    }
    for (uint32_t i = 0; i < n; i++) {
        const uint32_t idx = ((w + i) & _mask) * 2;
        _buf[idx]     = interleaved[i * 2];
        _buf[idx + 1] = interleaved[i * 2 + 1];
    }
    // Release: the samples above must be visible before the consumer can see
    // the new write index.
    _w.store(w + n, std::memory_order_release);
}

void JitterRing::setJitterFrames(uint32_t prebuf, uint32_t target, uint32_t max) {
    // max must exceed target or the hard-trim would trim down to a fill that it
    // immediately exceeds again (firmware audio_stream.cpp:884).
    if (max <= target) max = target + 1;

    if (_capacity) {
        // A depth the ring can never reach would leave read() priming forever.
        const uint32_t cap = _capacity - 1;
        if (prebuf > cap) prebuf = cap;
        if (target > cap) target = cap;
        if (max    > cap) max    = cap;
    }

    if (prebuf == _prebuf && target == _target && max == _max) return;

    _prebuf = prebuf;
    _target = target;
    _max    = max;
    reprime();   // adopt the new depth now instead of when the ring next drains
}

void JitterRing::reprime() {
    // Only re-arms the gate here. The actual "drop buffered audio" happens in
    // read(), which trims the fill down to _prebuf as it leaves priming — the
    // consumer is the only context allowed to advance _r, and reprime() is
    // called from the receive callback (source handoff / mode change), so
    // touching _r here would race the audio task.
    //
    // Plain bool, like the firmware's volatile s_prebuf_done (audio_stream.cpp:81):
    // it gates WHEN playback starts, never the data safety, which comes from the
    // atomic indices alone.
    _priming  = true;
    _driftAcc = 0;
}

uint32_t JitterRing::droppedFrames() const {
    return _dropped.load(std::memory_order_relaxed);
}

uint32_t JitterRing::fillFrames() const {
    if (!_buf) return 0;
    return _w.load(std::memory_order_acquire) - _r.load(std::memory_order_acquire);
}

uint32_t JitterRing::estDelayMs() const {
    return fillFrames() / kFramesPerMs;
}

// --- consumer side ---------------------------------------------------------

uint32_t JitterRing::read(int16_t* out, uint32_t frames, float gain) {
    if (!out || frames == 0) return 0;
    if (!_buf) {
        memset(out, 0, (size_t)frames * 2 * sizeof(int16_t));
        return 0;
    }

    uint32_t r    = _r.load(std::memory_order_relaxed);
    uint32_t fill = _w.load(std::memory_order_acquire) - r;

    // --- pre-buffer gate ---------------------------------------------------
    // Starting playback the instant the first packet lands guarantees an
    // immediate under-run, because there is nothing absorbing the next burst.
    // Conceal (hold + decay) rather than emitting hard zeros so a splice into
    // priming does not click.
    if (_priming) {
        if (fill < _prebuf) {
            for (uint32_t i = 0; i < frames; i++) {
                concealFrame(_holdL, _holdR, _decay, _dropped);
                out[i * 2]     = clip16((int32_t)(_holdL * gain));
                out[i * 2 + 1] = clip16((int32_t)(_holdR * gain));
            }
            return 0;
        }
        _priming = false;
        // Discard whatever is stale above the start depth. After a reprime the
        // ring can hold a whole previous source's audio; playing it out would
        // add its full length to the delay.
        if (fill > _prebuf) {
            const uint32_t drop = fill - _prebuf;
            r += drop;
            fill -= drop;
            _r.store(r, std::memory_order_release);
            _dropped.fetch_add(drop, std::memory_order_relaxed);
        }
    }

    // --- hard trim (safety valve) ------------------------------------------
    // The writer got briefly far ahead (burst, or a stalled reader). Cut back
    // to target so playback delay never accumulates. Only fires above _max, so
    // a normal burst never touches it.
    if (fill > _max) {
        const uint32_t drop = fill - _target;
        r += drop;
        fill -= drop;
        _r.store(r, std::memory_order_release);
        _dropped.fetch_add(drop, std::memory_order_relaxed);
    }

    // --- clock drift correction --------------------------------------------
    // The transmitter's ADC clock and this DAC clock are independent crystals
    // ~0.1-0.5 % apart, so the fill level walks steadily toward empty or full
    // and dropouts appear every few seconds, forever. Nudge by at most ONE
    // frame per block (62 us @16 kHz — inaudible, and haptics low-pass it),
    // outside a deadband wider than Wi-Fi jitter so we do not fight normal
    // jitter. Anything faster than one frame per block is audible.
    const uint32_t dead = _target / 4 + 32;
    int drift = 0;
    if (fill > _target + dead)                    drift = +1;   // drop one input frame
    else if (fill > 0 && fill + dead < _target)   drift = -1;   // duplicate one output frame

    if (drift > 0) {
        r += 1;
        fill -= 1;
        _r.store(r, std::memory_order_release);
        _dropped.fetch_add(1, std::memory_order_relaxed);
        _driftAcc += 1;   // diagnostic: net frames the drift spring has moved
    }

    // --- fill the block -----------------------------------------------------
    uint32_t sourced = 0;
    for (uint32_t i = 0; i < frames; i++) {
        if (drift < 0 && i == frames - 1) {
            // Depth rebuild: a real-time sender can never send ahead, so depth
            // eaten by an outage is otherwise permanent. Repeat the last frame.
            _decay = 0;
        } else {
            if (fill == 0) {
                // Re-read the write index before conceding: the producer may
                // have appended since the snapshot at the top of this call.
                fill = _w.load(std::memory_order_acquire) - r;
            }
            if (fill > 0) {
                const uint32_t idx = (r & _mask) * 2;
                _holdL = _buf[idx];
                _holdR = _buf[idx + 1];
                r++;
                fill--;
                sourced++;
                _decay = 0;
            } else {
                concealFrame(_holdL, _holdR, _decay, _dropped);
            }
        }
        out[i * 2]     = clip16((int32_t)(_holdL * gain));
        out[i * 2 + 1] = clip16((int32_t)(_holdR * gain));
    }
    if (drift < 0) _driftAcc -= 1;

    _r.store(r, std::memory_order_release);
    return sourced;
}

} // namespace hapbeat
