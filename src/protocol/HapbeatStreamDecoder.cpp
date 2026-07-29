// ---------------------------------------------------------------------------
// HapbeatStreamDecoder.cpp — ported from
//   hapbeat-device-firmware/src/espnow_stream.cpp
// with the Hapbeat-specific parts (NVS, OLED, buttons, volume hardware,
// AIC3204, the DuoWL v4 mode-9 HP48 delegation) left behind.
//
// Framework independent: no Arduino.h, no esp_now.h, no millis(). Every time
// value arrives as `now_ms` so this file also builds on a host PC.
//
// The constants and the ORDER of the checks below are a wire contract with the
// shipping transmitter, not style choices. See the notes on §7.1.1 resync and
// on the recovery cap: getting either one "tidier" is audible.
// ---------------------------------------------------------------------------

#include "HapbeatStreamDecoder.h"

#include <string.h>

#include "../codec/HapbeatImaAdpcm.h"
#include "../codec/HapbeatOpusBackend.h"

namespace hapbeat {
namespace {

// ---- Packet layout (espnow_stream.cpp:39-45) -------------------------------
const uint16_t MAX_FRAMES        = 64;  // :41  ADPCM frames/packet guard
const int      ADPCM_STATE_OFF   = 5;   // :42  stereo state (predL,stepL,predR,stepR)
const int      ADPCM_DATA_OFF    = 11;  // :43  stereo ADPCM data
const int      ADPCM_M_STATE_OFF = 5;   // :44  mono state (pred lo,hi + step)
const int      ADPCM_M_DATA_OFF  = 8;   // :45  mono ADPCM data
const int      MAX_PIGGYBACK     = 3;   // :511 / :663 wire pb_count is capped at 3

// Largest Opus frame any mode can produce, per channel. Mode 9 (SOLID48)
// declares outSamples = 480, which is what decodeOpusPLC() asks the decoder to
// conceal (espnow_stream.cpp:491) — so the scratch has to hold 480/ch even
// though a 48 kHz frame decoded at our 16 kHz output only yields 160.
const int OPUS_MAX_OUT = 480;
static_assert(OPUS_MAX_OUT >= 480,
              "mode 9 SOLID48 declares outSamples=480; PLC asks for that many samples/ch");
static_assert(OPUS_MAX_OUT >= 160,
              "modes 3-5/8 decode 10 ms @16 kHz = 160 samples/ch");

// ---- Decode scratch --------------------------------------------------------
// File scope, exactly like the firmware's s_pcm / s_mono8k / s_opus_st
// (espnow_stream.cpp:217-236): onPacket() is documented as single-threaded and
// the ESP-NOW receive callback is a single global, so one instance is decoding
// at a time. They live here rather than in the class because the header is a
// frozen contract. No dynamic allocation anywhere.
int16_t       s_pcm[MAX_FRAMES * 2];          // stereo ADPCM, nf frames
int16_t       s_mono8k[MAX_FRAMES];           // mono ADPCM @8k
int16_t       s_monoSt[MAX_FRAMES * 4];       // …upsampled ×2 + L=R  → nf*2 frames
int16_t       s_opusMono[OPUS_MAX_OUT];
int16_t       s_opusStereo[OPUS_MAX_OUT * 2];
const int16_t s_zero[MAX_FRAMES * 2] = {0};

// Fleet-tune buffer override tracking. applyFleetBuffer() in the firmware
// (:900-908) is idempotent — a repeat of the same value must NOT re-prime the
// ring, because the transmitter resends the beacon every 5 s and every reprime
// is an audible click. The header has no member to keep it in, so it is kept
// here, keyed on the owning decoder so a second instance (host tests) simply
// re-applies instead of silently inheriting the first one's state.
const StreamDecoder* s_bufOwner = nullptr;
uint16_t             s_bufMs    = 0;

inline uint16_t rd16(const uint8_t* p) {  // espnow_stream.cpp:245
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

inline bool macEq(const uint8_t* a, const uint8_t* b) {
    return memcmp(a, b, 6) == 0;
}

// Decode `nf` mono ADPCM samples @8 kHz, linear-interpolate ×2 to 16 kHz and
// duplicate L=R into `outSt`. Produces nf*2 stereo frames.
// espnow_stream.cpp:638-651.
void adpcmMonoTo16kStereo(const uint8_t* adpcm, int nf, AdpcmState* st,
                          int16_t* out8k, int16_t* outSt) {
    adpcmDecodeBlockMono(adpcm, out8k, (size_t)nf, st);
    // out[2i] = m[i], out[2i+1] = avg(m[i], m[i+1]); the last mid-sample repeats
    // m[nf-1] (there is no m[nf] to average with).
    for (int i = 0; i < nf; i++) {
        int     cur  = out8k[i];
        int     nxt  = (i + 1 < nf) ? out8k[i + 1] : cur;
        int16_t mid  = (int16_t)((cur + nxt) >> 1);
        int     base = i * 4;
        outSt[base]     = (int16_t)cur;
        outSt[base + 1] = (int16_t)cur;
        outSt[base + 2] = mid;
        outSt[base + 3] = mid;
    }
}

// How a forward sequence delta must be treated.
enum GapKind { GAP_IN_ORDER, GAP_STALE, GAP_RESYNC, GAP_LOSS };

// §7.1.1. The order matters: a large forward jump is a stream SPLICE (a repeater
// switched upstream, or the transmitter rebooted), not loss. Counting it as loss
// and running recovery would emit `gap` concealed frames of noise at every
// splice. Deltas ≥128 are stale/duplicate packets and are ignored outright.
inline GapKind classifyGap(uint8_t gap, uint8_t resyncGap) {
    if (gap == 0)   return GAP_IN_ORDER;
    if (gap >= 128) return GAP_STALE;
    // resyncGap 0 would make every packet a "splice"; treat it as 1 so a broken
    // setter can never take the whole stream out.
    return (gap >= (resyncGap ? resyncGap : 1)) ? GAP_RESYNC : GAP_LOSS;
}

}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void StreamDecoder::begin(const DecoderOptions& opt, FrameSink& sink) {
    _opt = opt;
    // §7.1.3: relay-test is a bring-up aid and must never come back after a
    // reboot, so it is forced off here regardless of what the caller passed.
    _opt.relayTestOnly = false;

    _sink = &sink;
    _sel.begin(_opt);
    _sel.setRelayTestOnly(false);
    _fleet.reset();

    _stats         = StreamStats();
    _mode          = 0;
    _seqInit       = false;
    _expectedSeq   = 0;
    _fadeRemaining = 0;

    _reqRelock      = false;
    _reqRelayTest   = false;
    _relayTestOn    = false;
    _pendingVolMax  = 0;
    _pendingLockEvt = false;
    _pendingModeEvt = false;

    s_bufOwner = this;
    s_bufMs    = 0;

    // Start on mode 0's geometry, which is what the firmware primes the ring
    // with at init (espnow_stream.cpp:785, RING_* == RX_MODE[0].buf_frames).
    const ModeInfo& mi  = modeInfo(0);
    const uint32_t  tgt = mi.bufFrames;
    sink.setJitterFrames(tgt, tgt, tgt + tgt / 2);
}

void StreamDecoder::setOpusBackend(OpusBackend* backend) {
    _opus = backend;
    // If a mode is already selected, shape the new backend for it now; otherwise
    // the first packet of an Opus mode would decode with the wrong channel count.
    if (_opus && modeInfo(_mode).codec == CODEC_OPUS) {
        _opus->reset(modeInfo(_mode).channels);
    }
}

void StreamDecoder::onFleetVolumeMax(FleetVolumeMaxFn fn, void* user) { _volMaxFn = fn; _volMaxUser = user; }
void StreamDecoder::onLockChanged(LockChangedFn fn, void* user)       { _lockFn   = fn; _lockUser   = user; }
void StreamDecoder::onModeChanged(ModeChangedFn fn, void* user)       { _modeFn   = fn; _modeUser   = user; }
void StreamDecoder::onPcmTap(PcmTapFn fn, void* user)                 { _tapFn    = fn; _tapUser    = user; }

// Control entry points only raise flags; onPacket() (the single writer of lock
// and sequence state) consumes them. Same convention as the firmware's
// g_force_relock / g_relay_test (espnow_stream.cpp:201-202).
void StreamDecoder::forceRelock() { _reqRelock = true; }

void StreamDecoder::setRelayTestOnly(bool on) {
    _relayTestOn  = on;
    _reqRelayTest = true;
}

void StreamDecoder::setResyncGap(uint8_t gap) {
    if (gap == 0) return;  // 0 would classify every packet as a splice
    _opt.resyncGap = gap;
}

void StreamDecoder::setLockTimeoutMs(uint32_t ms) {
    _opt.lockTimeoutMs = ms;
    _sel.setOptions(_opt);
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

// Feed decoded 16 kHz stereo PCM16 onward, applying the hand-off fade-in ramp.
// espnow_stream.cpp:375-388.
void StreamDecoder::feedWithFade(const int16_t* interleaved, uint16_t frames) {
    if (!interleaved || frames == 0) return;

    // The ramp is applied in place, as in the firmware. Every caller passes one
    // of this file's scratch buffers (never a const object), so the cast is
    // safe; the parameter is const only because FrameSink/PcmTapFn are.
    int16_t* pcm = const_cast<int16_t*>(interleaved);

    if (_fadeRemaining > 0) {
        const uint16_t total = _opt.fadeFrames ? _opt.fadeFrames : 1;
        for (uint16_t i = 0; i < frames && _fadeRemaining > 0; i++) {
            // Gain runs 1/N .. N/N so the last faded frame lands exactly at
            // unity — otherwise the following full-scale frame steps and clicks.
            float fg = (float)(total - _fadeRemaining + 1) / (float)total;
            pcm[i * 2]     = (int16_t)(pcm[i * 2] * fg);
            pcm[i * 2 + 1] = (int16_t)(pcm[i * 2 + 1] * fg);
            _fadeRemaining--;
        }
    }

    if (_tapFn) _tapFn(pcm, frames, _tapUser);
    if (_sink)  _sink->writeFrames(pcm, frames);
}

// Bridge a lost packet with silence. Bounded: a long gap must not dump a long
// silence, which would only inflate playout delay — the ring trims it anyway.
// espnow_stream.cpp:393-400. Zeros skip the fade ramp (it would be a no-op).
void StreamDecoder::feedSilence(uint32_t frames) {
    if (frames > _opt.silenceCapFrames) frames = _opt.silenceCapFrames;
    while (frames > 0) {
        uint32_t chunk = (frames < MAX_FRAMES) ? frames : MAX_FRAMES;
        if (_tapFn) _tapFn(s_zero, (uint16_t)chunk, _tapUser);
        if (_sink)  _sink->writeFrames(s_zero, (uint16_t)chunk);
        frames -= chunk;
    }
}

// ---- Mode switch: jitter depth + decoder shape (espnow_stream.cpp:403-468) --
void StreamDecoder::applyMode(uint8_t mode) {
    if (mode >= MODE_COUNT) return;

    _mode = mode;
    const ModeInfo& mi = modeInfo(mode);

    const uint32_t tgt = mi.bufFrames;
    if (_sink) {
        _sink->setJitterFrames(tgt, tgt, tgt + tgt / 2);
        _sink->reprime();  // refill to the new depth
    }
    // The buffer is now at the mode default; a standing 0xAC override re-applies
    // on its next 5 s resend (§3.5).
    s_bufOwner = this;
    s_bufMs    = 0;

    if (mi.codec == CODEC_OPUS && _opus) {
        // Mono⇄stereo needs a differently shaped decoder; the backend also
        // resets decoder state, which is what we want across a mode change.
        // NOTE: mode 9 (SOLID48) is decoded here as plain 16 kHz stereo. The
        // firmware's ESPNOW_HP48 48 kHz dual-rate delegation is DuoWL v4
        // hardware-specific and is deliberately not ported.
        _opus->reset(mi.channels);
    }

    _seqInit        = false;  // sequence tracking restarts on the new mode
    _pendingModeEvt = true;
}

// ---------------------------------------------------------------------------
// Opus (modes 1-5, 8, 9) — espnow_stream.cpp:470-550
// ---------------------------------------------------------------------------

void StreamDecoder::decodeOpusPacket(const uint8_t* data, int len, uint8_t seq, uint8_t pb) {
    if (len < 6) return;

    // opus_len is ONE byte (:505 receiver / audio_source.cpp:798 sender).
    // contracts/specs/espnow-stream.md §3.3 says uint16 LE — that section is
    // stale; reading two bytes here desynchronises every Opus packet.
    const int opusLen = data[4];
    const int fpos    = 5;
    if (fpos + opusLen > len) return;
    const uint8_t* frame = data + fpos;

    // Piggyback entries: [prev_seq(1)][len(1)][data(len)] × pb_count, ≤3.
    uint8_t        pbSeq[MAX_PIGGYBACK];
    const uint8_t* pbDat[MAX_PIGGYBACK];
    int            pbLen[MAX_PIGGYBACK];
    int            pbn = 0;
    int            pos = fpos + opusLen;
    for (int k = 0; k < (int)pb && k < MAX_PIGGYBACK; k++) {
        if (pos + 2 > len) break;
        uint8_t ps = data[pos];
        int     pl = data[pos + 1];
        pos += 2;
        if (pos + pl > len) break;
        pbSeq[pbn] = ps;
        pbDat[pbn] = data + pos;
        pbLen[pbn] = pl;
        pbn++;
        pos += pl;
    }

    const ModeInfo& mi = modeInfo(_mode);

    // Decode one frame and push it out. Stereo decoders write interleaved L/R
    // directly; mono decoders decode then duplicate L=R (:472-487).
    auto feedFrame = [this](const uint8_t* f, int flen) {
        if (!_opus) return;
        if (_opus->channels() == 2) {
            int n = _opus->decode(f, flen, s_opusStereo, OPUS_MAX_OUT);
            if (n > 0) feedWithFade(s_opusStereo, (uint16_t)n);
        } else {
            int n = _opus->decode(f, flen, s_opusMono, OPUS_MAX_OUT);
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    s_opusStereo[i * 2]     = s_opusMono[i];
                    s_opusStereo[i * 2 + 1] = s_opusMono[i];
                }
                feedWithFade(s_opusStereo, (uint16_t)n);
            }
        }
    };

    // Packet-loss concealment: synthesize one frame of this mode's duration
    // (:489-500). A mode whose declared length is not a legal Opus frame size
    // (mode 9's 480 @16 kHz) simply fails in the backend and stays silent —
    // which is the desired graceful behaviour, not noise.
    auto plcFrame = [this, &mi]() {
        if (!_opus) return;
        int want = mi.outSamples;
        if (want <= 0 || want > OPUS_MAX_OUT) want = OPUS_MAX_OUT;
        if (_opus->channels() == 2) {
            int n = _opus->conceal(s_opusStereo, want);
            if (n > 0) { feedWithFade(s_opusStereo, (uint16_t)n); _stats.plcConcealed++; }
        } else {
            int n = _opus->conceal(s_opusMono, want);
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    s_opusStereo[i * 2]     = s_opusMono[i];
                    s_opusStereo[i * 2 + 1] = s_opusMono[i];
                }
                feedWithFade(s_opusStereo, (uint16_t)n);
                _stats.plcConcealed++;
            }
        }
    };

    _stats.packetsReceived++;

    uint32_t lost = 0, recovered = 0;
    if (_seqInit) {
        const uint8_t gap  = (uint8_t)(seq - _expectedSeq);
        const GapKind kind = classifyGap(gap, _opt.resyncGap);
        if (kind == GAP_RESYNC) {
            // Stream splice: no loss count, no piggyback search, no PLC. Just
            // fade back in over the seam.
            _stats.resyncs++;
            _fadeRemaining = _opt.fadeFrames;
        } else if (kind == GAP_LOSS) {
            _stats.packetsLost += gap;
            lost = gap;
            if (gap > _stats.maxConsecutiveLost) _stats.maxConsecutiveLost = gap;

            // Recover IN SEQ ORDER (piggyback if this packet carries the frame,
            // else PLC). Capped to the NEWEST recoverCap packets: an outage of
            // 100+ would otherwise spin ~127 PLC decodes inside the receive
            // callback, and the ring would trim the oldest of them anyway.
            uint8_t start = _expectedSeq;
            if (gap > _opt.recoverCap) start = (uint8_t)(seq - _opt.recoverCap);
            for (uint8_t s = start; s != seq; s++) {
                int found = -1;
                // Absolute prev_seq match — never re-derive the sequence number.
                for (int k = 0; k < pbn; k++) {
                    if (pbSeq[k] == s) { found = k; break; }
                }
                if (found >= 0) {
                    feedFrame(pbDat[found], pbLen[found]);
                    _stats.piggybackRecovered++;
                    recovered++;
                } else {
                    plcFrame();
                }
            }
        }
        // GAP_STALE / GAP_IN_ORDER: nothing to recover.
    }
    _seqInit     = true;
    _expectedSeq = (uint8_t)(seq + 1);

    feedFrame(frame, opusLen);

    if (lost || recovered) _sel.noteLockLoss(lost, recovered);
}

// ---------------------------------------------------------------------------
// IMA-ADPCM (mode 0 stereo, modes 6/7 mono) — espnow_stream.cpp:616-739
// ---------------------------------------------------------------------------

void StreamDecoder::decodeAdpcmPacket(const uint8_t* data, int len, uint8_t seq, uint8_t pb) {
    if (len < 5) return;
    const int numFrames = data[4];
    if (numFrames == 0 || numFrames > (int)MAX_FRAMES) return;

    const bool mono      = (modeInfo(_mode).channels == 1);
    const int  dataOff   = mono ? ADPCM_M_DATA_OFF : ADPCM_DATA_OFF;
    const int  dataBytes = mono ? (numFrames + 1) / 2   // 2 samples per byte
                                : numFrames;            // 1 frame per byte
    if (dataOff + dataBytes > len) return;

    auto feedMono = [this, numFrames](const uint8_t* adpcm, AdpcmState* st) {
        adpcmMonoTo16kStereo(adpcm, numFrames, st, s_mono8k, s_monoSt);
        feedWithFade(s_monoSt, (uint16_t)(numFrames * 2));  // ×2 upsample @16 kHz
    };

    _stats.packetsReceived++;

    uint32_t lost = 0, recovered = 0;
    if (_seqInit) {
        const uint8_t gap  = (uint8_t)(seq - _expectedSeq);
        const GapKind kind = classifyGap(gap, _opt.resyncGap);
        if (kind == GAP_RESYNC) {
            _stats.resyncs++;                   // §7.1.1 splice, not loss
            _fadeRemaining = _opt.fadeFrames;
        } else if (kind == GAP_LOSS) {
            _stats.packetsLost += gap;
            lost = gap;
            if (gap > _stats.maxConsecutiveLost) _stats.maxConsecutiveLost = gap;

            if (mono) {
                // LITE/TURBO carry up to 3 piggyback copies. Each entry is fixed
                // size because every frame shares the packet's num_frames:
                // [prev_seq(1)][pred_lo][pred_hi][step][data(ceil(nf/2))].
                const int      dbytes = (numFrames + 1) / 2;
                uint8_t        pbSeq[MAX_PIGGYBACK];
                const uint8_t* pbEnt[MAX_PIGGYBACK];
                int            pbn = 0;
                int            pos = ADPCM_M_DATA_OFF + dbytes;  // just past the body
                for (int k = 0; k < (int)pb && k < MAX_PIGGYBACK; k++) {
                    if (pos + 4 + dbytes > len) break;
                    pbSeq[pbn] = data[pos];
                    pbEnt[pbn] = data + pos + 1;  // → [pred_lo][pred_hi][step][data…]
                    pbn++;
                    pos += 4 + dbytes;
                }
                // Same newest-N cap as the Opus path, for the same reason.
                uint8_t start = _expectedSeq;
                if (gap > _opt.recoverCap) start = (uint8_t)(seq - _opt.recoverCap);
                for (uint8_t s = start; s != seq; s++) {
                    int found = -1;
                    for (int k = 0; k < pbn; k++) {
                        if (pbSeq[k] == s) { found = k; break; }
                    }
                    if (found >= 0) {
                        AdpcmState st;
                        st.predictor  = (int16_t)rd16(pbEnt[found]);
                        st.step_index = pbEnt[found][2];
                        feedMono(pbEnt[found] + 3, &st);
                        _stats.piggybackRecovered++;
                        recovered++;
                    } else {
                        feedSilence((uint32_t)numFrames * 2);  // one 8k frame → ×2 @16k
                    }
                }
            } else {
                // SOLID ships pb 1, so only the immediately preceding packet is
                // reconstructable — hence gap == 1 only.
                bool ok = false;
                if (gap == 1 && pb >= 1) {
                    const int pbOff   = ADPCM_DATA_OFF + numFrames;  // 11 + N
                    const int pbTotal = 7 + numFrames;               // seq(1)+state(6)+data(N)
                    if (pbOff + pbTotal <= len && data[pbOff] == _expectedSeq) {
                        AdpcmState pl, pr;
                        pl.predictor  = (int16_t)rd16(data + pbOff + 1);
                        pl.step_index = data[pbOff + 3];
                        pr.predictor  = (int16_t)rd16(data + pbOff + 4);
                        pr.step_index = data[pbOff + 6];
                        adpcmDecodeBlockStereo(data + pbOff + 7, s_pcm, (size_t)numFrames, &pl, &pr);
                        feedWithFade(s_pcm, (uint16_t)numFrames);
                        _stats.piggybackRecovered++;
                        recovered++;
                        ok = true;
                    }
                }
                if (!ok) feedSilence((uint32_t)gap * (uint32_t)numFrames);
            }
        }
    }
    _seqInit     = true;
    _expectedSeq = (uint8_t)(seq + 1);

    if (mono) {
        AdpcmState st;
        st.predictor  = (int16_t)rd16(data + ADPCM_M_STATE_OFF);
        st.step_index = data[ADPCM_M_STATE_OFF + 2];
        feedMono(data + ADPCM_M_DATA_OFF, &st);
    } else {
        AdpcmState sl, sr;
        sl.predictor  = (int16_t)rd16(data + ADPCM_STATE_OFF);
        sl.step_index = data[ADPCM_STATE_OFF + 2];
        sr.predictor  = (int16_t)rd16(data + ADPCM_STATE_OFF + 3);
        sr.step_index = data[ADPCM_STATE_OFF + 5];
        adpcmDecodeBlockStereo(data + ADPCM_DATA_OFF, s_pcm, (size_t)numFrames, &sl, &sr);
        feedWithFade(s_pcm, (uint16_t)numFrames);
    }

    if (lost || recovered) _sel.noteLockLoss(lost, recovered);
}

// ---------------------------------------------------------------------------
// Packet entry point — espnow_stream.cpp:791-871 (+ :948-1037 for 0xAC)
// ---------------------------------------------------------------------------

void StreamDecoder::onPacket(const uint8_t mac[6], const uint8_t* data, int len,
                             int8_t rssi, uint32_t now_ms) {
    if (!mac || !data || len < 4) return;

    // ---- 0xAC fleet tune (§3.5) -------------------------------------------
    if (data[0] == PKT_FLEET_TUNE) {
        if (!_opt.acceptFleetTune) return;
        // Only from the source we are actually listening to, so a stray
        // transmitter in the next room cannot retune this receiver.
        if (!_sel.locked() || !macEq(mac, _sel.lockedMac())) return;

        FleetTuneResult r = _fleet.onBeacon(data, len);
        if (!r.applied) return;

        bool selOpt = false;
        if (r.hasBufferMs) {
            // Idempotent: the same value must not re-prime (see s_bufMs).
            if (s_bufOwner != this || s_bufMs != r.bufferMs) {
                s_bufOwner = this;
                s_bufMs    = r.bufferMs;
                uint32_t f = (r.bufferMs == 0)
                                 ? modeInfo(_mode).bufFrames
                                 : (uint32_t)r.bufferMs * (OUTPUT_RATE / 1000);  // 16 frames/ms
                if (_sink) {
                    _sink->setJitterFrames(f, f, f + f / 2);
                    _sink->reprime();
                }
            }
        }
        if (r.hasSelection)   { _opt.selectionEnabled = r.selection;   selOpt = true; }
        if (r.hasLockTimeout) { _opt.lockTimeoutMs    = r.lockTimeout; selOpt = true; }
        if (r.hasResyncGap)   { _opt.resyncGap        = r.resyncGap; }
        if (r.hasVolumeMax) {
            // Applying a volume ceiling means codec I2C and flash on real
            // hardware — never from here. Stash; the loop consumes it.
            _pendingVolMax = r.volumeMax;
        }
        if (selOpt) _sel.setOptions(_opt);
        return;
    }

    // ---- 0xAA stream -------------------------------------------------------
    if (data[0] != PKT_STREAM || len < 5) return;

    const uint8_t raw     = data[1];
    const bool    relayed = (raw & 0x80) != 0;  // DEC-043 RELAYED
    const uint8_t mode    = raw & 0x7F;         // MASK BEFORE the range check (§3.4):
                                                // checking the raw byte would drop every
                                                // relayed packet as an "unknown mode" and
                                                // silently kill repeater coverage.
    if (mode >= MODE_COUNT) return;
    const uint8_t seq = data[2];
    const uint8_t pb  = data[3];

    // Consume cross-thread control requests here, where the single writer runs.
    if (_reqRelayTest) {
        _reqRelayTest = false;
        const bool was = _sel.relayTestOnly();
        _sel.setRelayTestOnly(_relayTestOn);
        // Turning it ON must drop a lock that may be an origin, so the filter
        // re-selects a relayed source (espnow_stream.cpp:888-894).
        if (_relayTestOn && !was) _reqRelock = true;
    }
    if (_reqRelock) {
        _reqRelock = false;
        _sel.forceRelock(now_ms);
        _seqInit = false;
    }

    const bool take = _sel.admit(mac, rssi, mode, relayed, now_ms);
    if (_sel.consumeLockChanged()) {
        _seqInit        = false;
        _fadeRemaining  = _opt.fadeFrames;  // start the new source from silence
        _pendingLockEvt = true;
    }

    if (take) {
        if (mode != _mode) applyMode(mode);
        if (modeInfo(mode).codec == CODEC_OPUS) {
            if (!_opus) {
                // No Opus in this build: drop and stay silent rather than
                // feeding the DAC garbage.
                _stats.modeUnsupported++;
            } else {
                decodeOpusPacket(data, len, seq, pb);
            }
        } else {
            decodeAdpcmPacket(data, len, seq, pb);
        }
    }

    // Housekeeping last: the rate-selection window then also sees this packet's
    // losses, and a lock released here is picked up on the next packet.
    _sel.tick(now_ms);
    if (_sel.consumeLockChanged()) {
        _seqInit        = false;
        _fadeRemaining  = _opt.fadeFrames;
        _pendingLockEvt = true;
    }
}

void StreamDecoder::idleTick(uint32_t now_ms) {
    _sel.tick(now_ms);
    if (_sel.consumeLockChanged()) {
        _seqInit        = false;
        _fadeRemaining  = _opt.fadeFrames;
        _pendingLockEvt = true;
    }
}

// ---------------------------------------------------------------------------
// Deferred work (runs in the caller's loop, never in the receive callback)
// ---------------------------------------------------------------------------

void StreamDecoder::dispatchDeferred(uint32_t now_ms) {
    (void)now_ms;  // lock timeouts are idleTick()'s job; don't double-tick here

    // Read-and-clear, like espnowStreamFleetVolMaxPct() (:1048-1052): a beacon
    // must be consumed exactly once, or a local volume change made between two
    // 5 s beacons is instantly reverted by the same stale value.
    uint8_t vol = _pendingVolMax;
    if (vol != 0) {
        _pendingVolMax = 0;
        if (_volMaxFn) _volMaxFn(vol, _volMaxUser);
    }

    if (_pendingLockEvt) {
        _pendingLockEvt = false;
        if (_lockFn) {
            uint8_t    mac[6] = {0, 0, 0, 0, 0, 0};
            const bool lk     = _sel.locked();
            if (lk) memcpy(mac, _sel.lockedMac(), 6);
            _lockFn(mac, _sel.lockedRelayed(), lk, _lockUser);
        }
    }

    if (_pendingModeEvt) {
        _pendingModeEvt = false;
        if (_modeFn) _modeFn(_mode, _modeUser);
    }
}

// ---------------------------------------------------------------------------
// Stats — espnow_stream.cpp:1054-1071
// ---------------------------------------------------------------------------

StreamStats StreamDecoder::stats(uint32_t now_ms) const {
    StreamStats s = _stats;
    s.mode          = _mode;
    s.sourceCount   = _sel.liveSourceCount(now_ms);
    s.handoffs      = _sel.handoffs();          // owned by the selector
    s.locked        = _sel.locked();
    s.lockedRelayed = _sel.lockedRelayed();
    if (s.locked) memcpy(s.lockedMac, _sel.lockedMac(), 6);
    else          memset(s.lockedMac, 0, 6);
    s.droppedFrames = _sink ? _sink->droppedFrames() : 0;
    s.estDelayMs    = _sink ? (_sink->fillFrames() * 1000U) / OUTPUT_RATE : 0;
    return s;
}

void StreamDecoder::resetStats() {
    // Counters only; handoffs live in the selector and keep counting.
    _stats = StreamStats();
}

}  // namespace hapbeat
