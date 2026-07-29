// ---------------------------------------------------------------------------
// HapbeatEspNowRx.cpp — assembles transport → decoder → ring → I2S.
//
// The layering here matches the firmware's split: espnow_receiver.cpp owns the
// radio, espnow_stream.cpp owns the protocol, audio_stream.cpp owns the ring and
// the drain task. This class is only the wiring plus the 2 s status line from
// espnowStreamTick() (espnow_stream.cpp:1073-1102).
// ---------------------------------------------------------------------------

#include "HapbeatEspNowRx.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "codec/HapbeatOpusBackend.h"

namespace {
// ESP-NOW's receive callback is a plain C function, so only one receiver can be
// wired to it at a time; begin() enforces that rather than silently stealing
// the callback from an earlier instance.
HapbeatEspNowRx* s_instance = nullptr;

const uint32_t LOG_INTERVAL_MS = 2000;   // espnow_stream.cpp:1076
} // namespace

HapbeatEspNowRx* HapbeatEspNowRx::instance() {
    return s_instance;
}

// ---------------------------------------------------------------------------
// lifecycle
// ---------------------------------------------------------------------------

bool HapbeatEspNowRx::startCommon(const hapbeat::RxConfig& cfg, hapbeat::FrameSink& sink) {
    _cfg  = cfg;
    _log  = cfg.log;
    _sink = &sink;

    _decoder.setOpusBackend(hapbeat::defaultOpusBackend());
    _decoder.begin(cfg.decoder, sink);

    hapbeat::EspNowTransportConfig radio;
    radio.channel          = cfg.channel;
    radio.longRangeMask    = cfg.longRangeMask;
    radio.disableWifiSleep = cfg.disableWifiSleep;
    radio.maxTxPowerQdbm   = cfg.maxTxPowerQdbm;
    radio.initWifi         = cfg.initWifi;

    if (!_radio.begin(radio, &HapbeatEspNowRx::onFrame, this)) {
        return false;
    }

    _started   = true;
    _lastLogMs = millis();
    return true;
}

bool HapbeatEspNowRx::begin(const hapbeat::RxConfig& cfg) {
    if (_started) return false;
    if (s_instance && s_instance != this) return false;
    s_instance = this;

    if (!_ring.begin(cfg.ringFrames)) {
        s_instance = nullptr;
        return false;
    }
    _ownsRing = true;

    // Audio output is optional: with no DOUT pin the receiver still decodes and
    // reports, which is how boards::NoAudio() is used to prove the channel is
    // right before any amplifier is wired.
    if (cfg.i2s.dout >= 0) {
        hapbeat::I2sSinkConfig s;
        s.pins         = cfg.i2s;
        s.port         = cfg.i2sPort;
        s.sampleRate   = hapbeat::OUTPUT_RATE;
        s.bits         = cfg.i2sBits;
        s.dmaBufCount  = cfg.dmaBufCount;
        s.dmaBufLen    = cfg.dmaBufLen;
        s.startTask    = cfg.startAudioTask;
        s.taskCore     = cfg.taskCore;
        s.taskPriority = cfg.taskPriority;
        s.taskStack    = cfg.taskStack;

        if (!_i2s.begin(s, _ring)) {
            _ring.end();
            _ownsRing  = false;
            s_instance = nullptr;
            return false;
        }
        _i2s.setGain(cfg.gain);
        _ownsI2s = true;
    }

    if (!startCommon(cfg, _ring)) {
        end();
        return false;
    }
    return true;
}

bool HapbeatEspNowRx::begin(const hapbeat::RxConfig& cfg, hapbeat::FrameSink& sink) {
    if (_started) return false;
    if (s_instance && s_instance != this) return false;
    s_instance = this;

    // Caller owns the output path entirely: no ring, no I2S, no audio task.
    if (!startCommon(cfg, sink)) {
        s_instance = nullptr;
        return false;
    }
    return true;
}

void HapbeatEspNowRx::end() {
    // Reverse of begin(): stop the producer first so nothing is still writing
    // into the ring while it is being torn down.
    _radio.end();
    if (_ownsI2s) {
        _i2s.end();
        _ownsI2s = false;
    }
    if (_ownsRing) {
        _ring.end();
        _ownsRing = false;
    }
    _sink      = nullptr;
    _started   = false;
    s_instance = nullptr;
}

// ---------------------------------------------------------------------------
// receive path
// ---------------------------------------------------------------------------

void HapbeatEspNowRx::onFrame(const uint8_t mac[6], const uint8_t* data, int len,
                              int8_t rssi, void* user) {
    HapbeatEspNowRx* self = static_cast<HapbeatEspNowRx*>(user);
    if (!self) return;
    // Straight into the decoder from the Wi-Fi task — no queue hop, matching the
    // firmware's shortest receive→playback path (espnow_receiver.cpp:42-58).
    self->_decoder.onPacket(mac, data, len, rssi, millis());
}

void HapbeatEspNowRx::tick() {
    if (!_started) return;
    const uint32_t now = millis();

    _decoder.dispatchDeferred(now);   // fleet-tune + lock/mode callbacks
    _decoder.idleTick(now);           // release a silent lock with no traffic

    if (!_log) return;                // nullptr logger == completely silent
    if ((now - _lastLogMs) < LOG_INTERVAL_MS) return;
    _lastLogMs = now;

    const hapbeat::StreamStats s = _decoder.stats(now);
    const uint32_t total    = s.packetsReceived + s.packetsLost;
    const uint32_t lossX10  = total ? (s.packetsLost * 1000U) / total : 0;

    char line[224];
    if (s.locked) {
        snprintf(line, sizeof(line),
                 "[hapbeat-rx] lock=%02X:%02X:%02X:%02X:%02X:%02X mode=%s src=%u "
                 "pkt=%lu lost=%lu(%lu.%lu%%) recov=%lu maxgap=%u drop=%lu "
                 "handoff=%lu dly=%lums plc=%lu rsync=%lu rly=%d unsup=%lu",
                 s.lockedMac[0], s.lockedMac[1], s.lockedMac[2],
                 s.lockedMac[3], s.lockedMac[4], s.lockedMac[5],
                 hapbeat::modeName(s.mode), s.sourceCount,
                 (unsigned long)s.packetsReceived, (unsigned long)s.packetsLost,
                 (unsigned long)(lossX10 / 10), (unsigned long)(lossX10 % 10),
                 (unsigned long)s.piggybackRecovered, s.maxConsecutiveLost,
                 (unsigned long)s.droppedFrames, (unsigned long)s.handoffs,
                 (unsigned long)s.estDelayMs, (unsigned long)s.plcConcealed,
                 (unsigned long)s.resyncs, (int)s.lockedRelayed,
                 (unsigned long)s.modeUnsupported);
    } else {
        snprintf(line, sizeof(line),
                 "[hapbeat-rx] no lock, src=%u pkt=%lu unsup=%lu (waiting on ch %u)",
                 s.sourceCount, (unsigned long)s.packetsReceived,
                 (unsigned long)s.modeUnsupported, _radio.channel());
    }
    _log->println(line);
}

void HapbeatEspNowRx::pump() {
    if (_ownsI2s) _i2s.pump();
}

// ---------------------------------------------------------------------------
// controls
// ---------------------------------------------------------------------------

bool HapbeatEspNowRx::setChannel(uint8_t ch) {
    // Runtime only — nothing here persists it. A board that wants the channel
    // to survive a reboot stores it itself and passes it in RxConfig.
    if (!_radio.setChannel(ch)) return false;
    _cfg.channel = ch;
    return true;
}

uint8_t HapbeatEspNowRx::channel() const { return _radio.channel(); }

void HapbeatEspNowRx::setGain(float g) {
    if (g < 0.0f) g = 0.0f;
    if (g > 1.0f) g = 1.0f;
    _cfg.gain = g;
    if (_ownsI2s) _i2s.setGain(g);
}

float HapbeatEspNowRx::gain() const { return _cfg.gain; }

void HapbeatEspNowRx::forceRelock()             { _decoder.forceRelock(); }
void HapbeatEspNowRx::setRelayTestOnly(bool on) { _decoder.setRelayTestOnly(on); }

// ---------------------------------------------------------------------------
// state / notifications
// ---------------------------------------------------------------------------

bool        HapbeatEspNowRx::locked() const   { return _decoder.locked(); }
uint8_t     HapbeatEspNowRx::mode() const     { return _decoder.mode(); }
const char* HapbeatEspNowRx::modeName() const { return hapbeat::modeName(_decoder.mode()); }

hapbeat::StreamStats HapbeatEspNowRx::stats() const {
    return _decoder.stats(millis());
}

void HapbeatEspNowRx::setLogger(Print* out) { _log = out; }

void HapbeatEspNowRx::onFleetVolumeMax(hapbeat::FleetVolumeMaxFn fn, void* user) {
    _decoder.onFleetVolumeMax(fn, user);
}
void HapbeatEspNowRx::onLockChanged(hapbeat::LockChangedFn fn, void* user) {
    _decoder.onLockChanged(fn, user);
}
void HapbeatEspNowRx::onModeChanged(hapbeat::ModeChangedFn fn, void* user) {
    _decoder.onModeChanged(fn, user);
}
void HapbeatEspNowRx::onPcmTap(hapbeat::PcmTapFn fn, void* user) {
    _decoder.onPcmTap(fn, user);
}

size_t HapbeatEspNowRx::readFrames(int16_t* interleavedOut, size_t frames) {
    if (!_ownsRing || !interleavedOut || frames == 0) return 0;
    return _ring.read(interleavedOut, (uint32_t)frames, _cfg.gain);
}

#endif // ESP32
