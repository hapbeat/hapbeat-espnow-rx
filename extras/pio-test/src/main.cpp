// Multi-board build/flash test for hapbeat-espnow-rx.
//
// Same content as examples/Generic_I2S_Receiver, plus a once-a-second status
// line on the serial monitor so a build can be smoke-tested without a DAC
// attached (it will report "no lock" until a transmitter is heard).
//
// Wiring (MAX98357A / PCM5102 / UDA1334 or similar):
//   BCLK -> BCLK/BCK/SCK, LRCK -> LRC/WS, DOUT -> DIN/DATA, plus 3V3 and a
//   common GND. Override the pins from platformio.ini if yours differ.

#include <Arduino.h>
#include <HapbeatEspNowRx.h>

#if defined(HAPBEAT_ESPNOW_RX_OPUS)
// Opus modes (1-5, 8, 9). The implementation is a header compiled here, in the
// sketch, because that is the only translation unit that sees the project's
// lib_deps include paths — see codec/HapbeatOpusBackend.h for why.
#include <codec/HapbeatOpusBackendImpl.h>
#endif

#ifndef PIN_BCLK
#define PIN_BCLK 26
#endif
#ifndef PIN_LRCK
#define PIN_LRCK 25
#endif
#ifndef PIN_DOUT
#define PIN_DOUT 22
#endif
#ifndef HB_CHANNEL
#define HB_CHANNEL 11  // must match the transmitter
#endif

HapbeatEspNowRx rx;

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("hapbeat-espnow-rx test");

    hapbeat::RxConfig cfg = hapbeat::boards::GenericI2S(PIN_BCLK, PIN_LRCK, PIN_DOUT);
    cfg.channel = HB_CHANNEL;
    cfg.gain    = 0.8f;
    cfg.log     = &Serial;

#if defined(HAPBEAT_ESPNOW_RX_OPUS)
    hapbeat::useOpusBackend();   // must be before begin()
#endif

    if (!rx.begin(cfg)) {
        Serial.println("begin() failed");
        while (true) delay(1000);
    }
    Serial.printf("receiving on channel %u (I2S bclk=%d lrck=%d dout=%d)\n",
                  (unsigned)rx.channel(), PIN_BCLK, PIN_LRCK, PIN_DOUT);
}

void loop() {
    // Audio runs on its own task; tick() only handles callbacks and lock state.
    rx.tick();

    static uint32_t last = 0;
    if (millis() - last < 1000) return;
    last = millis();

    const hapbeat::StreamStats s = rx.stats();
    if (s.locked) {
        Serial.printf("lock=%02X:%02X mode=%s pkt=%lu lost=%lu recov=%lu plc=%lu "
                      "rsync=%lu drop=%lu delay=%lums\n",
                      s.lockedMac[4], s.lockedMac[5], rx.modeName(),
                      (unsigned long)s.packetsReceived, (unsigned long)s.packetsLost,
                      (unsigned long)s.piggybackRecovered, (unsigned long)s.plcConcealed,
                      (unsigned long)s.resyncs, (unsigned long)s.droppedFrames,
                      (unsigned long)s.estDelayMs);
    } else {
        Serial.printf("no lock (sources=%u) — check the channel (now %u)\n",
                      (unsigned)s.sourceCount, (unsigned)rx.channel());
    }
}
