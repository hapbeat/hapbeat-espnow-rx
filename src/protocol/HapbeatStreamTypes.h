#pragma once
// ---------------------------------------------------------------------------
// HapbeatStreamTypes.h — shared protocol-layer types (framework independent).
//
// Nothing in protocol/ may include <Arduino.h>, <esp_now.h> or call millis():
// the caller passes a monotonic `now_ms`, so the whole layer builds and runs on
// a host PC (see extras/host-test/).
// ---------------------------------------------------------------------------

#include <stdint.h>

namespace hapbeat {

// --- Where decoded audio goes ----------------------------------------------
// Implemented by sink/HapbeatJitterRing (default) or by the application.
class FrameSink {
public:
    virtual ~FrameSink() {}

    // 16 kHz interleaved stereo PCM16. `frames` counts stereo frames.
    virtual void writeFrames(const int16_t* interleaved, uint16_t frames) = 0;

    // Re-tune the jitter buffer (mode switch, or 0xAC param 1). 16 kHz frames.
    virtual void setJitterFrames(uint32_t prebuf, uint32_t target, uint32_t max) = 0;

    // Drop buffered audio and re-prime (source handoff / stream splice).
    virtual void reprime() = 0;

    // Diagnostics for stats(); return 0 if not tracked.
    virtual uint32_t droppedFrames() const { return 0; }
    virtual uint32_t fillFrames()    const { return 0; }
};

// --- Statistics -------------------------------------------------------------
// Mirrors the firmware's EspnowStreamStats (espnow_stream.cpp:1054-1071) plus
// modeUnsupported, which counts packets dropped because this build lacks Opus.
struct StreamStats {
    uint32_t packetsReceived     = 0;  // accepted from the locked source
    uint32_t packetsLost         = 0;  // seq gaps on the locked source
    uint32_t piggybackRecovered  = 0;  // packets rebuilt from a redundant copy
    uint32_t plcConcealed        = 0;  // Opus PLC frames synthesized
    uint32_t resyncs             = 0;  // stream splices (§7.1.1), not losses
    uint32_t droppedFrames       = 0;  // ring overruns
    uint32_t handoffs            = 0;  // source-MAC switches
    uint32_t modeUnsupported     = 0;  // packets dropped: no decoder for that mode
    uint16_t maxConsecutiveLost  = 0;  // largest single seq gap seen
    uint8_t  sourceCount         = 0;  // distinct live source MACs
    bool     locked              = false;
    bool     lockedRelayed       = false;  // locked source's last packet had bit7
    uint8_t  lockedMac[6]        = {0, 0, 0, 0, 0, 0};
    uint8_t  mode                = 0;
    uint32_t estDelayMs          = 0;  // ring fill → estimated playout delay
};

// --- Callbacks --------------------------------------------------------------
// All three are invoked from dispatchDeferred() (i.e. your loop), never from
// the ESP-NOW receive callback, so they may block, log, touch I2C or write NVS.
using FleetVolumeMaxFn = void (*)(uint8_t percent, void* user);
using LockChangedFn    = void (*)(const uint8_t mac[6], bool relayed, bool locked, void* user);
using ModeChangedFn    = void (*)(uint8_t modeId, void* user);

// Raw PCM tap. Called synchronously from onPacket() — do NOT block, log or
// touch I2C in it. For full ownership of the output path, pass your own
// FrameSink to begin() instead.
using PcmTapFn = void (*)(const int16_t* interleaved, uint16_t frames, void* user);

// --- Tuning -----------------------------------------------------------------
// Defaults are the firmware's shipping values; the file:line comments point at
// the constant they mirror in hapbeat-device-firmware/src/espnow_stream.cpp.
struct DecoderOptions {
    uint8_t  maxSources       = 8;      // :95   MAX_SOURCES (hard cap, see .cpp)
    uint32_t lockTimeoutMs    = 150;    // :98   LOCK_TIMEOUT_MS (0xAC param 3)
    uint8_t  resyncGap        = 20;     // :131  RESYNC_GAP     (0xAC param 4)
    bool     selectionEnabled = true;   // :134  rate-based select (0xAC param 2)
    bool     relayTestOnly    = false;  // §7.1.3 — MUST start false, never persist
    bool     acceptFleetTune  = true;
    uint16_t fadeFrames       = 48;     // :102  FADE_FRAMES (3 ms @16 kHz)
    uint32_t silenceCapFrames = 64;     // :103  SILENCE_CAP
    uint8_t  recoverCap       = 12;     // :109  OPUS_RECOVER_CAP
};

} // namespace hapbeat
