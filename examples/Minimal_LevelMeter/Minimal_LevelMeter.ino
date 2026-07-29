// Minimal_LevelMeter — no wiring at all: is the channel right, and is anyone
// transmitting?
//
// Flash this on a bare ESP32 board with nothing attached and open the serial
// monitor at 115200. It decodes the Hapbeat ESP-NOW stream but produces no
// audio; once a second it prints whether it is locked onto a transmitter and
// how loud the decoded signal is. Run this FIRST — the single most common
// reason a receiver stays silent is a channel mismatch, and that is invisible
// once a DAC is in the picture.
//
// Reading the output:
//   "lock=... rms=..."        -> the stream is arriving and decoding. Wire up a
//                               DAC next (see Generic_I2S_Receiver).
//   "no lock (sources=0)"     -> nothing heard at all. Wrong channel, out of
//                               range, or the transmitter is not streaming.
//   "no lock (sources=1..)"   -> packets ARE arriving but none is being locked:
//                               relay-test mode is on, or the source was just
//                               dropped by forceRelock() and is cooling off.
//
// Works with the transmitter's default mode (SOLID = IMA-ADPCM), so no Opus
// build flag and no extra library are needed.

#include <math.h>
#include <HapbeatEspNowRx.h>

// Must match the transmitter. The Hapbeat fleet default is 11.
static const uint8_t CHANNEL = 11;

HapbeatEspNowRx rx;

// --- level accumulation ------------------------------------------------------
// onPcmTap runs inside the ESP-NOW receive callback: no Serial, no delay(), no
// I2C, no allocation. All it may do is arithmetic on plain variables.
// The float accumulator is written by the callback and reset by loop(); a lost
// update at the exact 1 s boundary would nudge one meter reading and nothing
// else, which is why no locking is used here.
static volatile float    g_sumSq  = 0.0f;
static volatile uint32_t g_frames = 0;

static void pcmTap(const int16_t* interleaved, uint16_t frames, void* /*user*/) {
    float sum = 0.0f;
    for (uint16_t i = 0; i < frames; ++i) {
        const float l = (float)interleaved[i * 2] * (1.0f / 32768.0f);  // left only
        sum += l * l;
    }
    g_sumSq  = g_sumSq + sum;
    g_frames = g_frames + frames;
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("Hapbeat ESP-NOW receiver — level meter (no audio output)");

    // NoAudio(): decode and observe only, no I2S peripheral is touched.
    hapbeat::RxConfig cfg = hapbeat::boards::NoAudio();
    cfg.channel = CHANNEL;

    rx.onPcmTap(pcmTap);
    if (!rx.begin(cfg)) {
        Serial.println("begin() failed — ESP-NOW/Wi-Fi could not be started");
        while (true) delay(1000);
    }
    Serial.printf("listening on channel %u\n", (unsigned)rx.channel());
}

void loop() {
    rx.tick();

    static uint32_t last = 0;
    const uint32_t now = millis();
    if (now - last < 1000) return;
    last = now;

    // Snapshot and clear the accumulator for the next second.
    const float    sumSq  = g_sumSq;
    const uint32_t frames = g_frames;
    g_sumSq  = 0.0f;
    g_frames = 0;
    const float rms = (frames > 0) ? sqrtf(sumSq / (float)frames) : 0.0f;

    const hapbeat::StreamStats s = rx.stats();
    if (s.locked) {
        Serial.printf("lock=%02X:%02X mode=%s pkt=%lu lost=%lu recov=%lu plc=%lu rsync=%lu rms=%.3f%s\n",
                      s.lockedMac[4], s.lockedMac[5], rx.modeName(),
                      (unsigned long)s.packetsReceived, (unsigned long)s.packetsLost,
                      (unsigned long)s.piggybackRecovered, (unsigned long)s.plcConcealed,
                      (unsigned long)s.resyncs, rms,
                      s.lockedRelayed ? " (via repeater)" : "");
        if (s.modeUnsupported > 0) {
            // Opus modes need libopus + hapbeat::useOpusBackend() (see README).
            Serial.printf("  %lu packets dropped: this build cannot decode that mode\n",
                          (unsigned long)s.modeUnsupported);
        }
    } else {
        Serial.printf("no lock (sources=%u) — check the channel (now %u); "
                      "the transmitter must match\n",
                      (unsigned)s.sourceCount, (unsigned)rx.channel());
    }
}
