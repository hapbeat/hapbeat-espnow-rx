#pragma once
// ---------------------------------------------------------------------------
// HapbeatStreamDecoder.h — the protocol core: parse 0xAA/0xAC, track sequence,
// recover losses, decode to 16 kHz stereo PCM16.
//
// Framework independent by design (no Arduino.h, no esp_now.h, no millis()):
// the caller supplies `now_ms`, so the whole thing compiles and runs on a host
// PC against captured packets. See extras/host-test/.
//
// ---- WIRE FORMAT (as built — this is what the shipping firmware sends) -----
//
// Common header, every 0xAA packet:
//   [0] type      = 0xAA
//   [1] mode byte = RELAYED(bit7) | mode_id(bit6..0)
//   [2] seq       (wraps at 256)
//   [3] pb_count  (number of redundant previous frames appended)
//
// ADPCM body (mode 0 stereo, modes 6/7 mono):
//   [4]              num_frames
//   [5..]            state: 6 bytes stereo (predL,stepL,predR,stepR) / 3 mono
//   [11] or [8]      ADPCM data
//   then pb_count × [prev_seq(1)][state][data]
//
// Opus body (modes 1-5, 8, 9):
//   [4]              opus_len — ONE byte, not uint16
//   [5..]            Opus frame
//   then pb_count × [prev_seq(1)][len(1)][data]
//
// NOTE: contracts/specs/espnow-stream.md §3.3 documents the Opus length fields
// as uint16 LE. That is stale — both the transmitter (audio_source.cpp:798,
// :802-803) and the receiver (espnow_stream.cpp:505, :515) use a single byte.
// Implement from this file, not from that section.
//
// ---- THREADING -------------------------------------------------------------
// onPacket() must be called from ONE thread only (on ESP32: the ESP-NOW receive
// callback). There are no locks; the decoder, sequence state and source lock
// are protected purely by that single-writer rule, exactly as the firmware does
// it (espnow_stream.cpp:5-9). Control calls from other threads (forceRelock,
// setRelayTestOnly) only raise flags that onPacket() consumes.
//
// Anything that may block, log, use I2C or write flash happens in
// dispatchDeferred(), which you call from your main loop.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include "HapbeatStreamTypes.h"
#include "HapbeatSourceSelector.h"
#include "HapbeatFleetTune.h"
#include "../HapbeatStreamModes.h"

namespace hapbeat {

class OpusBackend;

class StreamDecoder {
public:
    void begin(const DecoderOptions& opt, FrameSink& sink);

    // Supply an Opus decoder. Without one, Opus modes count as unsupported.
    void setOpusBackend(OpusBackend* backend);

    // ---- the single entry point (call from ONE thread) ---------------------
    // Handles both 0xAA stream packets and 0xAC fleet-tune beacons; anything
    // else is ignored. `rssi` may be RSSI_UNKNOWN.
    void onPacket(const uint8_t mac[6], const uint8_t* data, int len,
                  int8_t rssi, uint32_t now_ms);

    // Release a silent lock even when no packets are arriving. Safe to call
    // from the same thread as onPacket().
    void idleTick(uint32_t now_ms);

    // ---- deferred work (call from your loop, NOT from the RX callback) -----
    void dispatchDeferred(uint32_t now_ms);
    void onFleetVolumeMax(FleetVolumeMaxFn fn, void* user);
    void onLockChanged(LockChangedFn fn, void* user);
    void onModeChanged(ModeChangedFn fn, void* user);
    void onPcmTap(PcmTapFn fn, void* user);

    // ---- control (any thread; these only set flags) ------------------------
    void forceRelock();
    void setRelayTestOnly(bool on);
    void setResyncGap(uint8_t gap);
    void setLockTimeoutMs(uint32_t ms);

    // ---- state -------------------------------------------------------------
    uint8_t     mode() const { return _mode; }
    bool        locked() const { return _sel.locked(); }
    StreamStats stats(uint32_t now_ms) const;
    void        resetStats();

private:
    FrameSink*     _sink = nullptr;
    OpusBackend*   _opus = nullptr;
    SourceSelector _sel;
    FleetTune      _fleet;
    DecoderOptions _opt;
    StreamStats    _stats;

    uint8_t  _mode          = 0;
    bool     _seqInit       = false;
    uint8_t  _expectedSeq   = 0;
    uint16_t _fadeRemaining = 0;

    // Cross-thread request flags, consumed inside onPacket()/dispatchDeferred().
    volatile bool    _reqRelock    = false;
    volatile bool    _reqRelayTest = false;
    volatile bool    _relayTestOn  = false;
    volatile uint8_t _pendingVolMax = 0;   // 0xAC param 5, applied in your loop
    volatile bool    _pendingLockEvt = false;
    volatile bool    _pendingModeEvt = false;

    FleetVolumeMaxFn _volMaxFn = nullptr;  void* _volMaxUser = nullptr;
    LockChangedFn    _lockFn   = nullptr;  void* _lockUser   = nullptr;
    ModeChangedFn    _modeFn   = nullptr;  void* _modeUser   = nullptr;
    PcmTapFn         _tapFn    = nullptr;  void* _tapUser    = nullptr;

    void applyMode(uint8_t mode);
    void feedWithFade(const int16_t* interleaved, uint16_t frames);
    void feedSilence(uint32_t frames);

    void decodeAdpcmPacket(const uint8_t* data, int len, uint8_t seq, uint8_t pb);
    void decodeOpusPacket(const uint8_t* data, int len, uint8_t seq, uint8_t pb);
};

} // namespace hapbeat
