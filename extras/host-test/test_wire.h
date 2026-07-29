#pragma once
// ---------------------------------------------------------------------------
// test_wire.h — packet builders and test doubles shared by the parse tests.
//
// The builders write the AS-BUILT wire format (the one the shipping firmware
// actually emits), so they are themselves part of the regression contract:
//   TX  hapbeat-transmitter-firmware/src/audio_source.cpp:798, :802-803
//   RX  hapbeat-device-firmware/src/espnow_stream.cpp:505, :515, :690-737
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <string.h>

#include "HapbeatStreamModes.h"
#include "protocol/HapbeatStreamTypes.h"
#include "codec/HapbeatImaAdpcm.h"
#include "codec/HapbeatOpusBackend.h"

namespace tw {

// --- packet buffer ----------------------------------------------------------
struct Packet {
    uint8_t b[512];
    int     len;

    Packet() : len(0) { memset(b, 0, sizeof(b)); }

    void u8(uint8_t v) { b[len++] = v; }
    void u16le(uint16_t v) { u8((uint8_t)(v & 0xFF)); u8((uint8_t)(v >> 8)); }
    void raw(const uint8_t* p, int n) { memcpy(b + len, p, (size_t)n); len += n; }
};

// [0]=0xAA [1]=RELAYED<<7|mode [2]=seq [3]=pb_count
inline void header(Packet& p, uint8_t mode, uint8_t seq, uint8_t pbCount,
                   bool relayed = false) {
    p.len = 0;
    p.u8(0xAA);
    p.u8((uint8_t)((relayed ? 0x80 : 0x00) | (mode & 0x7F)));
    p.u8(seq);
    p.u8(pbCount);
}

// --- ADPCM stereo (mode 0 SOLID) --------------------------------------------
// [4]=num_frames [5..10]=predL(LE),stepL,predR(LE),stepR [11..]=1 byte/frame
inline void adpcmStereoState(Packet& p, int16_t predL, uint8_t stepL,
                             int16_t predR, uint8_t stepR) {
    p.u16le((uint16_t)predL);
    p.u8(stepL);
    p.u16le((uint16_t)predR);
    p.u8(stepR);
}

inline void adpcmStereoBody(Packet& p, uint8_t numFrames,
                            int16_t predL, uint8_t stepL,
                            int16_t predR, uint8_t stepR,
                            const uint8_t* data) {
    p.u8(numFrames);
    adpcmStereoState(p, predL, stepL, predR, stepR);
    p.raw(data, numFrames);          // data starts at offset 11
}

// piggyback entry: [prev_seq][state 6][data numFrames] = 7 + N bytes
inline void adpcmStereoPb(Packet& p, uint8_t prevSeq, uint8_t numFrames,
                          int16_t predL, uint8_t stepL,
                          int16_t predR, uint8_t stepR,
                          const uint8_t* data) {
    p.u8(prevSeq);
    adpcmStereoState(p, predL, stepL, predR, stepR);
    p.raw(data, numFrames);
}

// --- ADPCM mono (modes 6 LITE / 7 TURBO) ------------------------------------
// [4]=num_frames [5,6]=pred(LE) [7]=step [8..]=ceil(nf/2) bytes, 2 samples each
inline int adpcmMonoDataBytes(int numFrames) { return (numFrames + 1) / 2; }

inline void adpcmMonoBody(Packet& p, uint8_t numFrames, int16_t pred,
                          uint8_t step, const uint8_t* data) {
    p.u8(numFrames);
    p.u16le((uint16_t)pred);
    p.u8(step);
    p.raw(data, adpcmMonoDataBytes(numFrames));   // data starts at offset 8
}

// piggyback entry: [prev_seq][pred_lo][pred_hi][step][data] = 4 + ceil(nf/2)
inline void adpcmMonoPb(Packet& p, uint8_t prevSeq, uint8_t numFrames,
                        int16_t pred, uint8_t step, const uint8_t* data) {
    p.u8(prevSeq);
    p.u16le((uint16_t)pred);
    p.u8(step);
    p.raw(data, adpcmMonoDataBytes(numFrames));
}

// --- Opus (modes 1-5, 8, 9) -------------------------------------------------
// [4]=opus_len — ONE byte. Writing a uint16 here is the退行 test_parse_opus_hdr
// exists to catch; see the note at the top of HapbeatStreamDecoder.h.
inline void opusBody(Packet& p, const uint8_t* frame, uint8_t frameLen) {
    p.u8(frameLen);
    p.raw(frame, frameLen);
}

// piggyback entry: [prev_seq][len][data]
inline void opusPb(Packet& p, uint8_t prevSeq, const uint8_t* frame,
                   uint8_t frameLen) {
    p.u8(prevSeq);
    p.u8(frameLen);
    p.raw(frame, frameLen);
}

// --- 0xAC fleet-tune beacon --------------------------------------------------
// [0]=0xAC [1]=RELAYED<<7|version [2]=param [3]=value [4]=epoch (optional)
inline int fleetBeacon(uint8_t* out, uint8_t param, uint8_t value,
                       bool hasEpoch, uint8_t epoch, uint8_t version = 1,
                       bool relayed = false) {
    out[0] = 0xAC;
    out[1] = (uint8_t)((relayed ? 0x80 : 0x00) | (version & 0x7F));
    out[2] = param;
    out[3] = value;
    if (!hasEpoch) return 4;
    out[4] = epoch;
    return 5;
}

// --- deterministic filler ---------------------------------------------------
// A fixed LCG rather than rand(): the same bytes on every platform, so a
// failure message is reproducible.
inline void fillPattern(uint8_t* p, int n, uint32_t seed) {
    uint32_t s = seed ? seed : 1;
    for (int i = 0; i < n; i++) {
        s = s * 1664525u + 1013904223u;
        p[i] = (uint8_t)(s >> 24);
    }
}

// --- FrameSink double -------------------------------------------------------
// Records everything the decoder pushes downstream so a test can assert on
// frame counts and exact PCM without any audio hardware.
class StubSink : public hapbeat::FrameSink {
public:
    // 8192 frames = 512 ms @16 kHz. Deliberately small enough that a StubSink
    // is cheap on the stack; the widest test case produces < 1000 frames.
    static const uint32_t CAP_FRAMES = 8192;

    StubSink() { clear(); reprimes = 0; jitterCalls = 0;
                 lastPrebuf = lastTarget = lastMax = 0; }

    void clear() {
        frames          = 0;
        writeCalls      = 0;
        lastWriteFrames = 0;
        overflow        = false;
    }

    void writeFrames(const int16_t* interleaved, uint16_t n) {
        writeCalls++;
        lastWriteFrames = n;
        if (frames + n > CAP_FRAMES) { overflow = true; return; }
        memcpy(pcm + frames * 2, interleaved, (size_t)n * 2 * sizeof(int16_t));
        frames += n;
    }

    void setJitterFrames(uint32_t prebuf, uint32_t target, uint32_t max) {
        jitterCalls++;
        lastPrebuf = prebuf; lastTarget = target; lastMax = max;
    }

    void reprime() { reprimes++; }

    uint32_t droppedFrames() const { return 0; }
    uint32_t fillFrames() const { return frames; }

    // interleaved L,R — sample index i of frame f is pcm[f*2 + i]
    int16_t  pcm[CAP_FRAMES * 2];
    uint32_t frames;
    uint32_t writeCalls;
    uint16_t lastWriteFrames;
    bool     overflow;

    uint32_t reprimes;
    uint32_t jitterCalls;
    uint32_t lastPrebuf, lastTarget, lastMax;
};

// --- OpusBackend double -----------------------------------------------------
// Lets the Opus wire-format tests run in an Opus-DISABLED build: we only care
// that the right bytes reach decode(), not what libopus makes of them.
class FakeOpus : public hapbeat::OpusBackend {
public:
    static const int MAX_CALLS = 32;

    FakeOpus() : samplesPerFrame(80), decodeCalls(0), concealCalls(0),
                 resetCalls(0), lastMaxSamples(0), _ch(1) {
        memset(callLen, 0, sizeof(callLen));
        memset(callData, 0, sizeof(callData));
    }

    bool reset(uint8_t channels) {
        _ch = channels;
        resetCalls++;
        return true;
    }

    int decode(const uint8_t* data, int len, int16_t* out, int maxSamples) {
        if (decodeCalls < MAX_CALLS) {
            callLen[decodeCalls] = len;
            int n = len < 32 ? len : 32;
            memcpy(callData[decodeCalls], data, (size_t)n);
        }
        decodeCalls++;
        lastMaxSamples = maxSamples;

        int n = samplesPerFrame < maxSamples ? samplesPerFrame : maxSamples;
        // Tag the output with the first frame byte so a test can tell which
        // frame (primary vs a piggyback copy) produced which audio.
        int16_t tag = (int16_t)(len > 0 ? data[0] : 0);
        for (int i = 0; i < n * _ch; i++) out[i] = tag;
        return n;
    }

    int conceal(int16_t* out, int samples) {
        concealCalls++;
        for (int i = 0; i < samples * _ch; i++) out[i] = 0;
        return samples;
    }

    uint8_t channels() const { return _ch; }

    int     samplesPerFrame;
    int     decodeCalls;
    int     concealCalls;
    int     resetCalls;
    int     lastMaxSamples;
    int     callLen[MAX_CALLS];
    uint8_t callData[MAX_CALLS][32];

private:
    uint8_t _ch;
};

// Default options with the shipping values; tests tweak fields from here.
inline hapbeat::DecoderOptions defaultOptions() {
    hapbeat::DecoderOptions o;
    return o;
}

inline void makeMac(uint8_t out[6], uint8_t last) {
    out[0] = 0x24; out[1] = 0x6F; out[2] = 0x28;
    out[3] = 0x00; out[4] = 0x00; out[5] = last;
}

} // namespace tw
