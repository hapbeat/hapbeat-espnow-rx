# Host regression tests

PC-side tests for the framework-independent half of the library
(`src/protocol/`, `src/codec/`). No board, no PlatformIO, no libopus.

```sh
cd extras/host-test
make            # build + run
make clean
make run ARGS=opus     # only tests whose name contains "opus"
```

Exit status is 0 when everything passes, 1 otherwise, so this drops straight
into CI. Any g++ with C++11 works.

On Windows the MinGW toolchain installs the binary as `mingw32-make`, so from
git-bash use `mingw32-make` in place of `make` above (verified with
MinGW.org GCC 9.2.0 + make 3.82).

Building at all is itself part of the contract: `src/protocol/` must never
include `<Arduino.h>` or `<esp_now.h>`, and must never call `millis()` — the
caller passes `now_ms`. If that rule is broken, this Makefile stops compiling.

Opus is intentionally **not** enabled (`-DHAPBEAT_ESPNOW_RX_OPUS` is absent).
The Opus wire-format tests inject a fake decoder through
`StreamDecoder::setOpusBackend()`, so the packet layout is verified without
libopus being installed anywhere.

## Why these tests exist

The ESP-NOW stream has no handshake and no error reporting. Get a byte offset
wrong and the receiver does not glitch — it goes **completely silent**, because
every mis-parsed length fails a bounds check and the packet is dropped whole.
There is nothing to debug from the outside. So the wire format is pinned here.

| File | Guards against |
|---|---|
| `test_adpcm.cpp` | IMA-ADPCM output drifting from the transmitter's encoder, and an out-of-range `step_index` from a corrupt packet reading past the 89-entry step table. |
| `test_parse_mode0.cpp` | Mode 0 SOLID (ADPCM 16 kHz stereo) offsets: `num_frames` at 4, 6-byte state at 5, data at 11, `[prev_seq][state 6][data]` piggyback. This is the transmitter's default mode. |
| `test_parse_mode6.cpp` | Modes 6/7 (ADPCM 8 kHz mono) offsets: 3-byte state at 5, data at 8, `ceil(nf/2)` bytes, and the `4 + ceil(nf/2)` piggyback stride. Reusing the stereo offsets here produces plausible garbage, not an error. |
| `test_parse_opus_hdr.cpp` | **`opus_len` being read as uint16 LE.** `contracts/specs/espnow-stream.md` §3.3 says uint16; the shipping firmware on both ends uses one byte. This is the single most likely regression in the library, and it silences every Opus mode. |
| `test_seq_piggyback.cpp` | Sequence wrap (255→0) counted as a 255-packet loss; a stream splice (`gap >= resyncGap`, §7.1.1) counted as loss and concealed; and an unbounded recovery loop — `recoverCap` limits how much work a single packet can trigger inside the Wi-Fi receive callback. |
| `test_source_select.cpp` | Decoding more than one transmitter at once (§7.1). A venue runs several sources broadcasting the same programme; mixing them sounds thick and smeared rather than obviously broken. Also covers force-relock blacklisting and relay-test mode. |
| `test_fleet_epoch.cpp` | The DEC-045 epoch rules: reject *strictly older* epochs only, re-apply same-epoch resends (otherwise a receiver that booted late never catches up), and never latch an epoch on a value that failed its range check (otherwise the corrected resend is orphaned forever). |

Supporting files: `test_harness.h` (assertions and self-registration),
`test_wire.h` (packet builders, `StubSink`, `FakeOpus`), `test_main.cpp`
(runner).

## Adding a test

```cpp
#include "test_harness.h"
#include "test_wire.h"

HT_TEST(my_case) {
    HT_CHECK_EQ_WHY(got, want, "why this matters");
}
```

`make` picks up any `test_*.cpp` in this directory automatically; tests
self-register, so there is no list to update. Failures print expected vs
actual — keep it that way, "assertion failed" is useless when the thing under
test is a byte offset.

## `vectors/`

Empty for now. Drop real captures here as `.bin` (one ESP-NOW payload per file,
or a length-prefixed stream) and add a test that replays them: capture from a
transmitter with a known input signal, record the decoded PCM once it is
verified by ear, and the pair becomes a golden-vector regression that covers
the codec end to end rather than field by field.
