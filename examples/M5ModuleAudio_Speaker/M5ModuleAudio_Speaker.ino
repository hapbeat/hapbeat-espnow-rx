// M5ModuleAudio_Speaker — receive the Hapbeat ESP-NOW stream on an M5Stack
// (Core / Core2 / CoreS3) with the M5Stack Module Audio (ES8388) attached, and
// play it out of the module's speaker/line output.
//
// Division of labour, and the reason this sketch is shaped the way it is:
//   * M5Module-Audio initialises the CODEC over I2C only (board-correct pins,
//     STM32 side of the module). The 5-argument begin() overload does NOT
//     install an I2S driver.
//   * This library owns I2S. It runs its own high-priority writer task with
//     small DMA buffers, which is what keeps the playout delay low.
// The transmitter firmware splits the two the same way, on the capture side:
//   hapbeat-transmitter-firmware/src/audio_source.cpp:1192-1215
//
// Controls:
//   BtnA tap        cycle the Wi-Fi channel 1 -> 6 -> 11 (must match the TX)
//   BtnA long press force a re-lock (drop this transmitter, pick another —
//                   useful when locked to a repeater that is further away)
//   BtnB / BtnC     gain -0.1 / +0.1
//
// -----------------------------------------------------------------------------
// NOT VERIFIED ON HARDWARE. Two things vary by board generation and must be
// checked on your unit:
//   1. The module's A/B I2S switch. Position A is assumed here.
//   2. Which M-Bus pin carries BCLK and which carries MCLK. They are SWAPPED
//      between board families, and the two sources available here disagree for
//      the ESP32-S3: the transmitter firmware uses BCK=mbus_pin22 / MCK=pin24 on
//      CoreS3 (audio_source.cpp:1185-1187), while M5Module_Audio.cpp:60-66 uses
//      BCK=pin24 / MCK=pin22 under CONFIG_IDF_TARGET_ESP32S3. If you get silence
//      or noise, set SWAP_BCLK_MCLK to 1 and try again.
// Requires: M5Unified + https://github.com/m5stack/M5Module-Audio.git
// -----------------------------------------------------------------------------

#include <M5Unified.h>
#include <M5Module_Audio.h>
#include <HapbeatEspNowRx.h>

// Flip this if there is no sound (see the note above).
#ifndef SWAP_BCLK_MCLK
#define SWAP_BCLK_MCLK 0
#endif

static const uint8_t CHANNELS[] = {1, 6, 11};
static uint8_t g_chIndex = 2;  // start on 11, the Hapbeat fleet default

M5ModuleAudio  audio;
HapbeatEspNowRx rx;
bool g_codecOk = false;

static void drawStatus() {
    const hapbeat::StreamStats s = rx.stats();
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(4, 4);

    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.printf("ch %u   gain %.1f\n", (unsigned)rx.channel(), rx.gain());

    if (!g_codecOk) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.println("MODULE AUDIO\nNOT FOUND");
        return;
    }

    if (s.locked) {
        M5.Display.setTextColor(TFT_GREEN);
        M5.Display.printf("LOCK %02X:%02X%s\n", s.lockedMac[4], s.lockedMac[5],
                          s.lockedRelayed ? " R" : "");
        M5.Display.setTextColor(TFT_WHITE);
        M5.Display.printf("mode %s\n", rx.modeName());
    } else {
        M5.Display.setTextColor(TFT_YELLOW);
        M5.Display.printf("no lock (src %u)\n", (unsigned)s.sourceCount);
        M5.Display.setTextColor(TFT_WHITE);
        M5.Display.println("check TX channel");
    }
    M5.Display.printf("pkt %lu lost %lu\n",
                      (unsigned long)s.packetsReceived, (unsigned long)s.packetsLost);
    M5.Display.printf("delay ~%lu ms\n", (unsigned long)s.estDelayMs);
}

void setup() {
    auto mcfg = M5.config();
    M5.begin(mcfg);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(4, 4);
    M5.Display.println("Hapbeat RX\ncodec init...");

    const int sda = M5.getPin(m5::pin_name_t::in_i2c_sda);
    const int scl = M5.getPin(m5::pin_name_t::in_i2c_scl);

    // CODEC only: I2C pins + address + speed. No I2S is installed by this call.
    // The ES8388 can miss I2C right after power-up, so retry a few times
    // (same treatment as audio_source.cpp:1196-1204).
    delay(100);
    for (int attempt = 0; attempt < 5 && !g_codecOk; ++attempt) {
        g_codecOk = audio.begin(Wire, (uint8_t)sda, (uint8_t)scl, 0x33, 400000);
        if (!g_codecOk) delay(120);
    }

    if (g_codecOk) {
        audio.setSpeakerOutput(DAC_OUTPUT_ALL);   // OUT1 (headphone) + OUT2 (speaker)
        audio.setSpeakerVolume(70);               // 0..100, analog side
        audio.setMute(false);
        audio.setBitsSample(ES_MODULE_DAC, BIT_LENGTH_16BITS);
        audio.setSampleRate(SAMPLE_RATE_16K);     // this library always outputs 16 kHz
    }

    // I2S pins from the M-Bus. mbus_pin23 = data ESP32 -> module DAC,
    // mbus_pin21 = LRCK. BCLK/MCLK assignment: see the header note.
#if SWAP_BCLK_MCLK
    const int bclk = M5.getPin(m5::pin_name_t::mbus_pin24);
    const int mclk = M5.getPin(m5::pin_name_t::mbus_pin22);
#else
    const int bclk = M5.getPin(m5::pin_name_t::mbus_pin22);
    const int mclk = M5.getPin(m5::pin_name_t::mbus_pin24);
#endif
    const int lrck = M5.getPin(m5::pin_name_t::mbus_pin21);
    const int dout = M5.getPin(m5::pin_name_t::mbus_pin23);

    hapbeat::RxConfig cfg = hapbeat::boards::GenericI2S(bclk, lrck, dout);
    cfg.i2s.mclk = mclk;   // the ES8388 needs a master clock; class-D amps do not
    cfg.channel  = CHANNELS[g_chIndex];
    cfg.gain     = 0.8f;
    cfg.log      = &Serial;

    Serial.begin(115200);
    if (!rx.begin(cfg)) {
        M5.Display.fillScreen(TFT_BLACK);
        M5.Display.setCursor(4, 4);
        M5.Display.setTextColor(TFT_RED);
        M5.Display.println("rx.begin FAILED");
        while (true) delay(1000);
    }
    drawStatus();
}

void loop() {
    M5.update();
    rx.tick();

    // BtnA: long press = re-lock, tap = next channel. The long press is latched
    // so it fires once, and suppresses the tap that would follow on release.
    static bool relocked = false;
    if (M5.BtnA.pressedFor(800) && !relocked) {
        relocked = true;
        rx.forceRelock();
        drawStatus();
    }
    if (M5.BtnA.wasReleased()) {
        if (!relocked) {
            g_chIndex = (g_chIndex + 1) % (sizeof(CHANNELS) / sizeof(CHANNELS[0]));
            rx.setChannel(CHANNELS[g_chIndex]);
            drawStatus();
        }
        relocked = false;
    }

    if (M5.BtnB.wasPressed()) {
        float g = rx.gain() - 0.1f;
        rx.setGain(g < 0.0f ? 0.0f : g);
        drawStatus();
    }
    if (M5.BtnC.wasPressed()) {
        float g = rx.gain() + 0.1f;
        rx.setGain(g > 1.0f ? 1.0f : g);
        drawStatus();
    }

    static uint32_t last = 0;
    if (millis() - last >= 1000) {
        last = millis();
        drawStatus();
    }
}
