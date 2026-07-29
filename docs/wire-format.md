# Hapbeat ESP-NOW ストリーム — ワイヤ形式（AS-BUILT）

出荷中のファームウェアが実際に送受しているバイト列。他言語 / 他 MCU で
実装し直すときはこのページを正とする。

> ## ⚠️ 仕様書 §3.3 は stale
>
> `hapbeat-contracts` の `specs/espnow-stream.md` §3.3 は Opus の長さフィールドを
> **uint16 LE** と記述しているが、**実装は 1 バイト**。仕様書どおりに書くと
> パースがずれて音が出ない。
>
> - 送信: `hapbeat-transmitter-firmware/src/audio_source.cpp:798, :802-803`
> - 受信: `hapbeat-device-firmware/src/espnow_stream.cpp:505, :515`
>
> 本ページ（実装由来）を正とすること。

すべて ESP-NOW ブロードキャスト（宛先 `FF:FF:FF:FF:FF:FF`）。ACK も再送も無い。
1 パケットの上限は `ESP_NOW_MAX_DATA_LEN` = 250 バイトで、どのモードの
最悪ケースもこの範囲に収まるようピギーバック深さが決めてある。

パケット種別は先頭 1 バイト:

| type | 意味 |
|---|---|
| `0xAA` | 音声ストリーム |
| `0xAC` | フリートチューンビーコン（§3.5） |

`0xAB` は仕様上存在するが、**どの出荷ファームウェアも実装していない**。
全モードが `0xAA` + mode_id に乗る。

---

## 1. 共通ヘッダ（すべての 0xAA パケット）

| offset | size | 内容 |
|---:|---:|---|
| 0 | 1 | `0xAA` |
| 1 | 1 | mode バイト = `RELAYED(bit7) \| mode_id(bit6..0)` |
| 2 | 1 | `seq` — 256 で折り返す |
| 3 | 1 | `pb_count` — 後続のピギーバック（冗長な過去フレーム）数 |

body は mode_id のコーデックで決まる（下表）。

- **bit7 (RELAYED)** はリピーターが中継したことを示す（DEC-043）。mode_id を読む
  ときは必ず `& 0x7F` でマスクする。
- ピギーバックのパース深さは **wire の `pb_count`** が決める。ただし受信側は
  **3 個で打ち切る**（`espnow_stream.cpp:513`, `:665`）。モード表の `piggyback`
  列は送信側の実際の深さで、ドキュメント用。

### モード表

`espnow_stream.cpp:61-72`（RX_MODE）/ `:174-185`（MODE_RATED_PPS）の逐語コピー。

| id | 名前 | codec | ch | out_samples | pb | buf_frames | rated pps |
|---:|---|---|---:|---:|---:|---:|---:|
| 0 | SOLID | ADPCM | 2 | — | 1 | 480 (30 ms) | 250 (4 ms) |
| 1 | FAST | Opus | 1 | 80 | 3 | 192 (12 ms) | 200 (5 ms) |
| 2 | BALANCED | Opus | 1 | 80 | 3 | 288 (18 ms) | 200 (5 ms) |
| 3 | SMOOTH | Opus | 1 | 160 | 2 | 448 (28 ms) | 100 (10 ms) |
| 4 | STEREO | Opus | 2 | 160 | 2 | 288 (18 ms) | 100 (10 ms) |
| 5 | HIFI | Opus | 2 | 160 | 2 | 448 (28 ms) | 100 (10 ms) |
| 6 | LITE | ADPCM | 1 | 80 | 3 | 256 (16 ms) | 200 (5 ms) |
| 7 | TURBO | ADPCM | 1 | 80 | 3 | 64 (4 ms) | 200 (5 ms) |
| 8 | FINE | Opus | 1 | 160 | 2 | 448 (28 ms) | 100 (10 ms) |
| 9 | SOLID48 | Opus | 2 | 480 | 2 | 1920 | 100 (10 ms) |

`ch` はワイヤ上のチャンネル数、`out_samples` は 16 kHz 換算のフレームあたり
サンプル数/ch（Opus PLC の生成長）、`buf_frames` は 16 kHz ステレオフレーム単位の
推奨ジッタ深さ（16 フレーム = 1 ms）。

mode 9 (SOLID48) は Hapbeat DuoWL v4 の 48 kHz HP 経路向け。それ以外の受信機は
16 kHz にデコードして触覚だけ鳴らす（graceful degradation）。

---

## 2. ADPCM body（mode 0 / 6 / 7）

IMA-ADPCM。デコーダ状態を毎パケットに載せるので、途中から受信し始めても鳴る。

### ステレオ（mode 0 SOLID）

| offset | size | 内容 |
|---:|---:|---|
| 4 | 1 | `num_frames` N（1..64。`MAX_FRAMES` 超は破棄） |
| 5 | 2 | predictor L（int16 LE） |
| 7 | 1 | step_index L |
| 8 | 2 | predictor R（int16 LE） |
| 10 | 1 | step_index R |
| 11 | N | ADPCM データ（1 バイト/フレーム: low nibble = L, high nibble = R） |

続いて `pb_count` 個: `[prev_seq(1)][state(6)][data(N)]`（1 エントリ = 7 + N バイト）。

### モノラル（mode 6 LITE / 7 TURBO）

| offset | size | 内容 |
|---:|---:|---|
| 4 | 1 | `num_frames` N |
| 5 | 2 | predictor（int16 LE） |
| 7 | 1 | step_index |
| 8 | ceil(N/2) | ADPCM データ（1 バイト = 2 サンプル: low nibble が先） |

続いて `pb_count` 個: `[prev_seq(1)][pred_lo][pred_hi][step][data(ceil(N/2))]`。

モノラルは 8 kHz なので、16 kHz 出力へは ×2 アップサンプル + L=R 複製。

定数（`espnow_stream.cpp:42-45`）:
`ADPCM_STATE_OFF = 5` / `ADPCM_DATA_OFF = 11`（ステレオ）、
`ADPCM_M_STATE_OFF = 5` / `ADPCM_M_DATA_OFF = 8`（モノラル）。

---

## 3. Opus body（mode 1-5 / 8 / 9）

| offset | size | 内容 |
|---:|---:|---|
| 4 | 1 | **`opus_len` — 1 バイト**（仕様書の uint16 LE は誤り） |
| 5 | opus_len | Opus フレーム |

続いて `pb_count` 個（最大 3 個までパース）: `[prev_seq(1)][len(1)][data(len)]`。

`5 + opus_len > len` のパケットは破棄（`espnow_stream.cpp:507`）。

デコーダは**モードのワイヤレートに関わらず 16 kHz で生成**する。libopus が
内部でリサンプルするので、8 kHz でも 48 kHz でも自前のリサンプラなしに
16 kHz 出力へ落ちる（`espnow_stream.cpp:441-470`）。

---

## 4. シーケンス処理と損失回復

`seq` は 1 パケットごとに +1、256 で折り返す。`gap = (uint8_t)(seq - expected)` として:

| 条件 | 扱い |
|---|---|
| `gap == 0` | 通常 |
| `gap >= RESYNC_GAP && gap < 128` | **ストリームの継ぎ目**（リピーターの上流切替 / 送信元再起動）。損失として数えず、`resyncs++` と短いフェードだけ。PLC もピギーバック探索もしない（§7.1.1） |
| `0 < gap < 128` | 損失。`packets_lost += gap`。欠落 seq を**古い順に**ピギーバックから探し、無ければ PLC（Opus）/ 無音（ADPCM）で埋める |
| `gap >= 128` | 過去の seq（重複・順序逆転）。無視 |

`RESYNC_GAP` の既定は 20（0xAC param 4 で変更可）。

Opus の回復は 1 ギャップあたり `OPUS_RECOVER_CAP = 12` パケットで打ち切る
（受信コールバック内で ~127 回の PLC を回さないため）。9 パケットも入れば
リング上限に達するので、古い側を捨てても実質失うものは無い。

---

## 5. ソース選択（§7）

会場に複数の送信機・リピーターがいる。全部が同じ番組を流しているので、混ぜると
音が二重になる。受信機は **MAC 1 つにロックして、それだけをデコードする**。
他のソースは生存確認とフェイルオーバー候補として追跡するだけ。

主な定数（`espnow_stream.cpp:95-135`）:

| 定数 | 既定 | 意味 |
|---|---:|---|
| `MAX_SOURCES` | 8 | 追跡できるソース MAC 数 |
| `LOCK_TIMEOUT_MS` | 150 | ロック中のソースが無音になったら解放（0xAC param 3） |
| `SOURCE_LIVE_MS` | 1000 | 「生きているソース」と数える窓 |
| `SEL_WINDOW_MS` | 4000 | 配信レート比較の窓 |
| `SEL_LOSS_GATE_PCT` | 5 | 未回復損失がレートのこの % 以上ならロックは「不調」 |
| `SEL_BLACKLIST_MS` | 10000 | 切り替え元 / force-relock したソースのクールオフ |
| `SEL_MIN_SWITCH_MS` | 10000 | レート起因の切り替えの最小間隔 |
| `FADE_FRAMES` | 48 | ハンドオフ時のクロスフェード（3 ms @16 kHz） |
| `SILENCE_CAP` | 64 | 1 ギャップに挿入する無音の上限フレーム |

意図的にヒステリシスを効かせてある。境界にいる 2 台の間で往復すると、音の継ぎ目が
延々と出るため。

**RSSI について**: arduino-esp32 2.0.x の ESP-NOW 受信コールバックは RSSI を
渡してこない。実際には RSSI 不明（`RSSI_UNKNOWN = -128`）で、先着 + 生存 +
配信レートだけで選択が回る。

---

## 6. 0xAC フリートチューンビーコン（§3.5）

会場の受信機 60 台を触らずに、送信機から一括で再調整するための片方向ビーコン。

| offset | size | 内容 |
|---:|---:|---|
| 0 | 1 | `0xAC` |
| 1 | 1 | `RELAYED(bit7) \| version(bit6..0)`。version は 1 のみ。他は無視 |
| 2 | 1 | param id |
| 3 | 1 | value |
| 4 | 1 | epoch（任意。現行の送信機は常に付ける） |

param:

| id | 範囲 | 意味 |
|---:|---|---|
| 1 | 0 = モード既定 / 4..60 | ジッタバッファ深さ [ms] |
| 2 | 0 / 1 | 配信レートによるソース選択の on/off |
| 3 | 5..50（×10 ms） | ロックタイムアウト |
| 4 | 4..64 | RESYNC_GAP |
| 5 | 10..100 | 音量上限 [%] — アプリケーションが永続化する想定 |
| 6 | — | Hapbeat DuoWL v4 の HP バッファ（DEC-046）。他機は無視 |

適用は**現在ロック中のソースからのビーコンのみ**（直接・中継どちらでも可）。
値は永続化しない — 電源を入れ直せばモード既定に戻る。イベント後に設定が
残らないのは意図的な仕様。param 5 だけは例外でアプリ側が保存する想定。

### エポック規則（DEC-045）— 間違えると会場全体が古い値で固まる

- **厳密に古いエポックだけ拒否**。ラップセーフに `(int8_t)(epoch - last) < 0`。
- **同一エポックの再送は再適用する**。5 秒ごとの再送は、後から起動した受信機が
  追いつくための仕組み。適用は全て冪等なので再適用しても副作用は無い。
- **エポックのラッチは値が範囲チェックを通った後**。先にラッチすると、不正値が
  エポックを焼いて、その後の正しい再送が全部拒否される。
- `len == 4`（エポックバイト無し）は旧送信機。常に適用する。

かつては同一エポックも拒否していた（`<= 0`）ため、1 回の取りこぼしで
「一度は効くがその後ずっと効かない」状態に陥っていた（2026-07-11/12 の
現場報告の構造的な原因）。2026-07-12 に上記へ緩和。

---

## 参照元

- 受信: `hapbeat-device-firmware/src/espnow_stream.cpp`
- 送信: `hapbeat-transmitter-firmware/src/audio_source.cpp`
- 仕様（一部 stale）: `hapbeat-contracts/specs/espnow-stream.md`
