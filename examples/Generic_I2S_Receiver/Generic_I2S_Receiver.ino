// Generic_I2S_Receiver — play the Hapbeat ESP-NOW stream on any I2S DAC.
//
// Tested-shape wiring for the usual breakout boards (MAX98357A class-D amp,
// PCM5102 / UDA1334 line-out DACs). Only three signals are needed; none of
// these parts requires MCLK.
//
//   ESP32 pin        breakout pin        notes
//   ---------------  ------------------  ------------------------------------
//   BCLK  (26)       BCLK / BCK / SCK    bit clock
//   LRCK  (25)       LRC / LRCK / WS     word select (left/right)
//   DOUT  (22)       DIN / DATA / SDIN   data, ESP32 -> DAC
//   3V3              VIN / VCC           MAX98357A also runs at 5 V
//   GND              GND                 common ground is mandatory
//
//   MAX98357A: leave SD floating (mono, L+R averaged) — it has no stereo out.
//   PCM5102:   tie FLT/DEMP/XSMT per its datasheet (XSMT high = un-muted).
//
// If nothing comes out, run the Minimal_LevelMeter example first: it tells you
// whether the problem is the radio (channel) or the wiring.

#include <HapbeatEspNowRx.h>

// --- board wiring ------------------------------------------------------------
#define PIN_BCLK 26
#define PIN_LRCK 25
#define PIN_DOUT 22

// Must match the transmitter. The Hapbeat fleet default is 11.
static const uint8_t CHANNEL = 11;

HapbeatEspNowRx rx;

void setup() {
    Serial.begin(115200);
    delay(300);

    hapbeat::RxConfig cfg = hapbeat::boards::GenericI2S(PIN_BCLK, PIN_LRCK, PIN_DOUT);
    cfg.channel = CHANNEL;
    cfg.gain    = 0.8f;    // 0..1, digital; back off if the amp clips
    cfg.log     = &Serial; // remove for a completely silent library

    if (!rx.begin(cfg)) {
        Serial.println("begin() failed — check the I2S pins and the port");
        while (true) delay(1000);
    }
    Serial.printf("receiving on channel %u\n", (unsigned)rx.channel());
}

void loop() {
    // Audio is drained into I2S by a dedicated high-priority task, so a late
    // tick() never interrupts playback — it only defers callbacks and the
    // lock/mode bookkeeping.
    rx.tick();
}
