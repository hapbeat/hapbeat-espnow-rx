// ---------------------------------------------------------------------------
// HapbeatRxBoards.cpp — ready-made RxConfig factories.
//
// Everything except the pins is already at the Hapbeat receiver's shipping
// values via RxConfig's member initialisers, so these only fill in the wiring.
// ---------------------------------------------------------------------------

#include "HapbeatRxConfig.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

namespace hapbeat {
namespace boards {

RxConfig GenericI2S(int bclk, int lrck, int dout) {
    RxConfig cfg;
    cfg.i2s.bclk = bclk;
    cfg.i2s.lrck = lrck;
    cfg.i2s.dout = dout;
    cfg.i2s.mclk = -1;   // most class-D amps free-run off BCLK
    return cfg;
}

RxConfig NoAudio() {
    RxConfig cfg;
    // Pins stay at -1, so begin() skips the I2S sink entirely. Without a sink
    // there is nothing for the audio task to drain, so don't start one.
    cfg.startAudioTask = false;
    return cfg;
}

} // namespace boards
} // namespace hapbeat

#endif // ESP32
