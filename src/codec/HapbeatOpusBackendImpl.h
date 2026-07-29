#pragma once
// ---------------------------------------------------------------------------
// HapbeatOpusBackendImpl.h — the actual Opus decoder, header-only.
//
// INCLUDE THIS FROM YOUR SKETCH, not from the library:
//
//   #include <HapbeatEspNowRx.h>
//   #include <codec/HapbeatOpusBackendImpl.h>
//
//   void setup() {
//     hapbeat::useOpusBackend();     // before begin()
//     rx.begin(cfg);
//   }
//
// and add libopus to your project:
//
//   lib_deps = https://github.com/pschatzmann/arduino-libopus.git
//
// It is header-only on purpose. PlatformIO compiles each library with only its
// own declared dependencies on the include path, so <opus.h> is unreachable
// from the library's own .cpp files however you set lib_ldf_mode. A sketch
// translation unit does get the project include paths, so compiling the
// decoder there is what makes the dependency genuinely optional.
//
// Ported from hapbeat-device-firmware/src/espnow_stream.cpp:441-500.
// ---------------------------------------------------------------------------

#include <opus.h>

#include "HapbeatOpusBackend.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#include <esp_heap_caps.h>
#define HAPBEAT_RX_OPUS_INTERNAL_ALLOC 1
#else
#include <stdlib.h>
#endif

namespace hapbeat {

class OpusBackendImpl : public OpusBackend {
public:
    ~OpusBackendImpl() { release(); }

    bool reset(uint8_t channels) override {
        if (channels != 1 && channels != 2) return false;

        // Same shape as before → a state reset is enough, and avoids churning
        // the heap on every source handoff or mode re-select.
        if (_dec && _ch == channels) {
            opus_decoder_ctl(_dec, OPUS_RESET_STATE);
            return true;
        }

        release();

        // Always 16 kHz, whatever the mode's wire rate: libopus resamples
        // internally, so 8 k / 48 k streams land straight on this library's
        // 16 kHz output and we carry no resampler of our own. Same choice the
        // firmware makes (espnow_stream.cpp:441-470).
        const opus_int32 kDecodeRate = 16000;

        int size = opus_decoder_get_size((int)channels);
        if (size <= 0) return false;

#if defined(HAPBEAT_RX_OPUS_INTERNAL_ALLOC)
        // Keep decoder state in internal SRAM. On PSRAM-equipped boards the
        // default allocator hands out external RAM, whose wait states make
        // opus_decode measurably slower — and this runs on the audio path.
        void* mem = heap_caps_malloc((size_t)size,
                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!mem) return false;
        _dec = (OpusDecoder*)mem;
        if (opus_decoder_init(_dec, kDecodeRate, (int)channels) != OPUS_OK) {
            heap_caps_free(mem);
            _dec = nullptr;
            return false;
        }
        _ownsRawMemory = true;
#else
        int err = 0;
        _dec = opus_decoder_create(kDecodeRate, (int)channels, &err);
        if (!_dec || err != OPUS_OK) {
            if (_dec) { opus_decoder_destroy(_dec); _dec = nullptr; }
            return false;
        }
        _ownsRawMemory = false;
#endif
        _ch = channels;
        return true;
    }

    int decode(const uint8_t* data, int len, int16_t* out, int maxSamples) override {
        if (!_dec || !data || len <= 0 || !out || maxSamples <= 0) return -1;
        return opus_decode(_dec, data, len, out, maxSamples, 0);
    }

    int conceal(int16_t* out, int samples) override {
        // A NULL frame asks libopus for packet-loss concealment.
        if (!_dec || !out || samples <= 0) return -1;
        return opus_decode(_dec, NULL, 0, out, samples, 0);
    }

    uint8_t channels() const override { return _ch; }

private:
    void release() {
        if (!_dec) { _ch = 0; return; }
        if (_ownsRawMemory) {
#if defined(HAPBEAT_RX_OPUS_INTERNAL_ALLOC)
            heap_caps_free(_dec);
#else
            free(_dec);
#endif
        } else {
            opus_decoder_destroy(_dec);
        }
        _dec = nullptr;
        _ch  = 0;
        _ownsRawMemory = false;
    }

    OpusDecoder* _dec = nullptr;
    uint8_t      _ch  = 0;
    bool         _ownsRawMemory = false;   // opus_decoder_init on our own buffer
};

// Install the decoder. Safe to call more than once. Returns the backend so you
// can check it against nullptr if you want to.
inline OpusBackend* useOpusBackend() {
    static OpusBackendImpl instance;   // built on first use, never freed
    registerOpusBackend(&instance);
    return &instance;
}

} // namespace hapbeat
