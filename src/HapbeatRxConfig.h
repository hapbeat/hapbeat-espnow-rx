#pragma once
// ---------------------------------------------------------------------------
// HapbeatRxConfig.h — everything HapbeatEspNowRx::begin() needs.
//
// The defaults are chosen so that `HapbeatEspNowRx rx; rx.begin(cfg);` with
// nothing but I2S pins filled in behaves like a Hapbeat receiver: channel 11,
// long-range mask on, modem sleep off, 16 kHz stereo out, dedicated audio task.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include "protocol/HapbeatStreamTypes.h"
#include "sink/HapbeatI2sSink.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
class Print;

namespace hapbeat {

struct RxConfig {
    // --- radio -------------------------------------------------------------
    uint8_t channel          = 11;    // must match the transmitter (1/6/11)
    bool    longRangeMask    = true;  // also receive 802.11 LR frames
    bool    disableWifiSleep = true;  // modem sleep adds tens of ms of jitter
    int8_t  maxTxPowerQdbm   = 84;
    bool    initWifi         = true;  // false if you already own Wi-Fi/ESP-NOW

    // --- output ------------------------------------------------------------
    I2sPins  i2s;                     // leave unset to run without audio output
    int      i2sPort     = 0;
    uint8_t  i2sBits     = 16;
    uint16_t dmaBufCount = 4;
    uint16_t dmaBufLen   = 128;
    uint32_t ringFrames  = 4096;      // power of two; 256 ms @16 kHz
    float    gain        = 0.8f;

    // --- audio task --------------------------------------------------------
    bool     startAudioTask = true;   // false: drive it yourself with pump()
    uint8_t  taskCore       = 1;
    uint8_t  taskPriority   = 19;
    uint32_t taskStack      = 4096;

    // --- protocol ----------------------------------------------------------
    DecoderOptions decoder;

    // --- diagnostics -------------------------------------------------------
    Print* log = nullptr;             // nullptr = completely silent (default)
};

// Ready-made configurations. Fill in what you need afterwards.
namespace boards {

// Any board wired to an external I2S DAC / class-D amp.
RxConfig GenericI2S(int bclk, int lrck, int dout);

// No audio output at all: decode and observe only. Use this to check that the
// channel is right and a transmitter is being heard, before any wiring exists.
RxConfig NoAudio();

} // namespace boards
} // namespace hapbeat

#endif // ESP32
