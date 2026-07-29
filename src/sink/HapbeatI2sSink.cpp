// ---------------------------------------------------------------------------
// HapbeatI2sSink.cpp — drain the jitter ring into an I2S DAC / amplifier.
//
// Ported from hapbeat-device-firmware:
//   src/audio_player.cpp:184-218  installI2sPort() — driver config + pins
//   src/audio_player.cpp:162-178  writeI2sStereo16() — 32-bit slot packing
//   src/audio_player.cpp:479-483  the blocking i2s_write that paces the loop
//   src/main.cpp:154-165, :782    taskAudioEspnow — prio 19, Core 1, 4096 B
//
// The one thing to preserve: i2s_write() with portMAX_DELAY blocks until the
// DMA queue has room, which paces this loop at exactly the audio rate. Adding a
// vTaskDelay would make the loop run to its own clock and fight the ring's
// drift correction. High priority on a dedicated core is what makes the small
// DMA queue (and therefore the low latency) safe: UI work can never delay a
// refill.
// ---------------------------------------------------------------------------

#include "HapbeatI2sSink.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace hapbeat {

namespace {

// Frames moved per i2s_write. Bounded so the conversion scratch below fits a
// 4096 B task stack even when the caller configures large DMA buffers; a bigger
// dmaBufLen just means several writes per DMA buffer, which costs nothing.
const uint32_t kBlockFrames = 128;

inline i2s_port_t portOf(int port) {
    return (port == 1) ? I2S_NUM_1 : I2S_NUM_0;
}

inline int pinOrNoChange(int pin) {
    return (pin < 0) ? I2S_PIN_NO_CHANGE : pin;
}

} // namespace

bool I2sSink::begin(const I2sSinkConfig& cfg, JitterRing& ring) {
    if (_running) return false;

    _cfg  = cfg;
    _ring = &ring;
    _task = nullptr;

    const i2s_port_t port = portOf(cfg.port);

    i2s_config_t c = {};
    c.mode             = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
    c.sample_rate      = cfg.sampleRate;
    c.bits_per_sample  = (cfg.bits == 32) ? I2S_BITS_PER_SAMPLE_32BIT
                                          : I2S_BITS_PER_SAMPLE_16BIT;
    c.channel_format   = I2S_CHANNEL_FMT_RIGHT_LEFT;      // stereo L+R
    c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    c.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    c.dma_buf_count    = cfg.dmaBufCount;
    c.dma_buf_len      = cfg.dmaBufLen;
    // APLL is deliberately off: the ESP32-S3 has none (the driver warns and
    // falls back), and on parts that do have it the fractional divider is
    // accurate enough here — the ring corrects residual clock error anyway.
    c.use_apll         = false;
    c.tx_desc_auto_clear = true;

    if (i2s_driver_install(port, &c, 0, nullptr) != ESP_OK) {
        _ring = nullptr;
        return false;
    }

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = pinOrNoChange(cfg.pins.mclk);   // most class-D amps need none
    pins.bck_io_num   = pinOrNoChange(cfg.pins.bclk);
    pins.ws_io_num    = pinOrNoChange(cfg.pins.lrck);
    pins.data_out_num = pinOrNoChange(cfg.pins.dout);
    pins.data_in_num  = I2S_PIN_NO_CHANGE;              // TX only

    if (i2s_set_pin(port, &pins) != ESP_OK) {
        i2s_driver_uninstall(port);
        _ring = nullptr;
        return false;
    }

    _running = true;

    if (cfg.startTask) {
        TaskHandle_t h = nullptr;
        if (xTaskCreatePinnedToCore(taskEntry, "hapbeat_i2s", cfg.taskStack, this,
                                    cfg.taskPriority, &h, cfg.taskCore) != pdPASS) {
            _running = false;
            i2s_driver_uninstall(port);
            _ring = nullptr;
            return false;
        }
        _task = h;
    }
    return true;
}

void I2sSink::end() {
    if (!_running) return;
    _running = false;   // the task observes this and self-terminates

    if (_task) {
        // The task clears _task immediately before vTaskDelete(NULL), so this
        // waits for it to actually be gone before the driver disappears from
        // under a blocked i2s_write. Read through a volatile view so the poll
        // is not hoisted out of the loop. i2s_write returns within one DMA
        // period while the driver is installed, so this settles quickly; the
        // bound only exists so a wedged driver cannot hang end() forever.
        volatile void* const* task = (volatile void* const*)&_task;
        for (int i = 0; i < 200 && *task != nullptr; i++) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        _task = nullptr;
    }

    const i2s_port_t port = portOf(_cfg.port);
    i2s_zero_dma_buffer(port);   // leave the amp at 0, not the last sample
    i2s_driver_uninstall(port);
    _ring = nullptr;
}

void I2sSink::pump() {
    if (!_running || !_ring) return;

    uint32_t frames = _cfg.dmaBufLen ? _cfg.dmaBufLen : kBlockFrames;
    if (frames > kBlockFrames) frames = kBlockFrames;

    int16_t pcm[kBlockFrames * 2];
    _ring->read(pcm, frames, _gain);   // always fills; underflow is concealed

    const i2s_port_t port = portOf(_cfg.port);
    size_t written = 0;

    if (_cfg.bits == 32) {
        // 32-bit slots carrying 16-bit audio in the high half. Some codecs need
        // the wider slot to keep BCLK above their PLL floor (audio_player.cpp:146-156).
        int32_t buf32[kBlockFrames * 2];
        const uint32_t n = frames * 2;
        for (uint32_t i = 0; i < n; i++) buf32[i] = ((int32_t)pcm[i]) << 16;
        i2s_write(port, buf32, (size_t)n * sizeof(int32_t), &written, portMAX_DELAY);
    } else {
        i2s_write(port, pcm, (size_t)frames * 2 * sizeof(int16_t), &written,
                  portMAX_DELAY);
    }
}

void I2sSink::setGain(float g) {
    if (g < 0.0f) g = 0.0f;
    if (g > 8.0f) g = 8.0f;   // headroom for deliberate boost; read() clips
    _gain = g;
}

float I2sSink::gain() const {
    return _gain;
}

void I2sSink::taskEntry(void* arg) {
    I2sSink* self = static_cast<I2sSink*>(arg);
    while (self->_running) {
        self->pump();   // blocks inside i2s_write → self-paced, no delay needed
    }
    self->_task = nullptr;   // publish termination to end()
    vTaskDelete(nullptr);
}

} // namespace hapbeat

#endif // ESP32
