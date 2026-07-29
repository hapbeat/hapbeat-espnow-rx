#pragma once
// ---------------------------------------------------------------------------
// HapbeatI2sSink.h — drain the jitter ring into an I2S DAC / amplifier.
//
// Runs a dedicated high-priority task pinned to one core. i2s_write() blocks
// until the DMA queue has room, which paces the loop at exactly the audio rate
// — no vTaskDelay, no drift of our own. This mirrors the firmware's
// taskAudioEspnow (main.cpp:154-165, :782), which sits at priority 19, above
// the UI task (5) and the Arduino loop (1) so a frame is never late because
// something else was busy.
//
// If you would rather own the output yourself, don't use this class: pass your
// own FrameSink to HapbeatEspNowRx::begin(), or pull frames with readFrames().
//
// ESP32 family only (uses driver/i2s.h).
// ---------------------------------------------------------------------------

#include <stdint.h>

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include "HapbeatJitterRing.h"

namespace hapbeat {

struct I2sPins {
    int bclk = -1;
    int lrck = -1;
    int dout = -1;
    int mclk = -1;   // -1 = not used; most class-D amps don't need MCLK
};

struct I2sSinkConfig {
    I2sPins  pins;
    int      port         = 0;
    uint32_t sampleRate   = 16000;
    uint8_t  bits         = 16;    // 32 left-justifies the 16-bit sample
    uint16_t dmaBufCount  = 4;
    uint16_t dmaBufLen    = 128;   // frames per DMA buffer
    bool     startTask    = true;
    uint8_t  taskCore     = 1;
    uint8_t  taskPriority = 19;
    uint32_t taskStack    = 4096;
};

class I2sSink {
public:
    bool begin(const I2sSinkConfig& cfg, JitterRing& ring);
    void end();

    // Only needed when cfg.startTask == false: call it often enough to keep the
    // DMA queue fed (it blocks inside i2s_write, so a tight loop is correct).
    void pump();

    void  setGain(float g);
    float gain() const;

private:
    JitterRing*   _ring = nullptr;
    I2sSinkConfig _cfg;
    void*         _task    = nullptr;
    volatile bool _running = false;
    volatile float _gain   = 0.8f;

    static void taskEntry(void* arg);
};

} // namespace hapbeat

#endif // ESP32
