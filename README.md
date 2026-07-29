# HapbeatEspNowRx — ESP-NOW 音声ブロードキャスト受信ライブラリ

Hapbeat 送信機がライン入力（PA のセンド・楽器・ミキサーのバス）をエンコードして
ESP-NOW でブロードキャストしている音声を、**任意の ESP32 ボードで受信して I2S に出す**
ための受信側ライブラリ。

ペアリングも接続確立も ACK も無い一斉同報なので、**受信台数の上限は電波の届く範囲だけ**。
1 台でも 60 台でも送信側の処理は変わらない。

> 📚 ドキュメント: <https://devtools.hapbeat.com/>

対応: ESP32 ファミリのみ（ESP-NOW + `driver/i2s.h` を使用）。

> **状態**: 0.1.0 は初版。プロトコル部分は Hapbeat 受信機ファームウェアからの移植で、
> ワイヤ形式は送受で一致を確認済み。ただし**本ライブラリ自体の実機動作は未検証**の
> 項目が残っているため、導入時は自分のボードで確認してから使うこと。

## インストール

**PlatformIO**:

```ini
lib_deps = hapbeat/espnow-rx@^0.1.0
```

**Arduino IDE**: ライブラリマネージャで **"HapbeatEspNowRx"** を検索してインストール
（またはリポジトリの ZIP を *スケッチ → ライブラリをインクルード → .ZIP形式のライブラリをインストール*）。

依存ライブラリはゼロ。Opus モードを使う場合のみ後述の opt-in が必要。

## Quick start

```cpp
#include <HapbeatEspNowRx.h>

HapbeatEspNowRx rx;

void setup() {
  auto cfg = hapbeat::boards::GenericI2S(/*bclk*/26, /*lrck*/25, /*dout*/22);
  cfg.channel = 11;          // ★ 送信機と必ず一致させる
  rx.begin(cfg);
}

void loop() {
  rx.tick();                 // 音声は専用タスクで流れる。tick() は後処理だけ
}
```

出力は常に **16 kHz インターリーブド ステレオ PCM16**。8 kHz モノラルのモードは
×2 アップサンプル + L=R 複製で、Hapbeat ファームウェアと同じ挙動になる。

## ★ チャンネルを送信機と合わせる

**動かない原因のほとんどはこれ。** ESP-NOW は Wi-Fi のチャンネルが一致していないと
1 パケットも届かない。

- Hapbeat 送信機の既定は **チャンネル 11**（本ライブラリの既定も 11）。
- 送信機側で変更しているなら `cfg.channel` を同じ値にする（1 / 6 / 11）。
- 受信側でルーターに Wi-Fi 接続すると、STA の接続先チャンネルに引きずられる。
  ESP-NOW 受信専用で使うなら Wi-Fi 接続はしないこと（`cfg.initWifi = true` のまま、
  ライブラリに radio を任せる）。

`rx.stats().sourceCount` が 0 のままなら、まずここを疑う。

## ストリームモード一覧

送信機は 0xAA パケットの中にモード ID を入れて送る。ID ごとのコーデック / レート /
チャンネル数は送受で一致していなければならない wire 契約（`src/HapbeatStreamModes.h`）。

| ID | 名前 | コーデック | ワイヤ形式 | フレーム長 | 推奨ジッタ深さ | 依存 |
|---:|---|---|---|---:|---:|---|
| 0 | SOLID | IMA-ADPCM | 16 kHz ステレオ | 4 ms | 30 ms | なし（送信機の既定） |
| 1 | FAST | Opus | 8 kHz モノラル | 5 ms | 12 ms | Opus |
| 2 | BALANCED | Opus | 8 kHz モノラル | 5 ms | 18 ms | Opus |
| 3 | SMOOTH | Opus | 8 kHz モノラル | 10 ms | 28 ms | Opus |
| 4 | STEREO | Opus | 8 kHz ステレオ | 10 ms | 18 ms | Opus |
| 5 | HIFI | Opus | 16 kHz ステレオ | 10 ms | 28 ms | Opus |
| 6 | LITE | IMA-ADPCM | 8 kHz モノラル | 5 ms | 16 ms | なし |
| 7 | TURBO | IMA-ADPCM | 8 kHz モノラル | 5 ms | 4 ms | なし（低遅延優先・途切れやすい） |
| 8 | FINE | Opus | 16 kHz モノラル | 10 ms | 28 ms | Opus |
| 9 | SOLID48 | Opus | 48 kHz ステレオ | 10 ms | — | Opus（本ライブラリは 16 kHz にデコード） |

**ADPCM 系（0 / 6 / 7）は依存ゼロで再生できる。** 送信機の既定は SOLID なので、
Opus を入れなくても素の状態で音は出る。**Opus 系（1〜5 / 8 / 9）は opt-in**。
Opus を有効にしていないビルドで Opus モードのパケットを受けると、ノイズを鳴らさずに
破棄し `StreamStats::modeUnsupported` を数える（＝無音になる）。

## Opus を有効にする

libopus をプロジェクトに追加する。

```ini
lib_deps =
    hapbeat/espnow-rx@^0.1.0
    https://github.com/pschatzmann/arduino-libopus.git
```

**そのうえで、スケッチ側でバックエンドを登録する。**

```cpp
#include <HapbeatEspNowRx.h>
#include <codec/HapbeatOpusBackendImpl.h>   // 実装はヘッダオンリー

void setup() {
  hapbeat::useOpusBackend();                // begin() より前に呼ぶ
  rx.begin(cfg);
}
```

`hapbeat::opusAvailable()` / `hapbeat::modeSupported(id)` で、実行時にデコード可否を
確認できる。

<details>
<summary>なぜライブラリ内で完結せず、スケッチ側で include するのか</summary>

PlatformIO は各ライブラリを**そのライブラリ自身が宣言した依存だけ**を include パスに
乗せてコンパイルする。プロジェクトの `lib_deps` に libopus を書いても、本ライブラリの
`.cpp` をコンパイルしている間はそのパスが見えないため、ライブラリ内に
`#include <opus.h>` を置くと `lib_ldf_mode` を何にしても解決できない
（`chain+` / `deep+` でも同じ）。

一方スケッチの翻訳単位にはプロジェクトの include パスが効く。そこでデコーダ実装を
ヘッダに置き、スケッチから include してもらう形にした。`library.json` に libopus を
依存として宣言してしまう方法もあるが、それだと ADPCM しか使わない人にも
ダウンロードとビルドを強制することになる。

</details>

> **注意（各自で確認してください）**: `pschatzmann/arduino-libopus` は作者が
> retired（メンテナンス終了）を表明しており、リポジトリに LICENSE ファイルが
> 置かれていない。本ライブラリは同 repo に依存を固定しておらず、Opus バックエンドは
> `hapbeat::OpusBackend` インターフェース越しに差し替えられる。採用可否とライセンス
> 条件は利用者側で最新の状態を確認して判断すること（ここでは法的な助言はしない）。

## サンプル

| サンプル | 内容 |
|---|---|
| `Minimal_LevelMeter` | 音声出力なし。受信できているか・どのモードか・レベルをシリアルに出すだけ。配線前の疎通確認用 |
| `Generic_I2S_Receiver` | 外付け I2S DAC / D 級アンプに出す最小構成 |
| `M5Core2_Speaker` | M5Stack Core2 の内蔵スピーカーに出す |

## 動かないときの切り分け

`rx.stats()` を見ながら上から順に。

1. **`sourceCount == 0`（送信機が 1 台も見えていない）**
   → チャンネル不一致がほぼ確実。送信機の設定と `cfg.channel` を突き合わせる。
   受信機側で Wi-Fi に接続していないかも確認する。
2. **`sourceCount > 0` なのに `locked == false`（見えているのにロックしない）**
   → `setRelayTestOnly(true)` を入れっぱなしにしていないか（中継パケットしか
   受け付けなくなる）。あるいは直前に `forceRelock()` した送信機が
   クールオフ（ブラックリスト）期間中の可能性。
3. **ロックしているのに無音、`modeUnsupported` が増える**
   → 送信機が Opus モードで、こちらが Opus 無効ビルド。上記の opt-in を入れるか、
   送信機を SOLID / LITE に戻す。
4. **音は出るが途切れる**
   → ジッタ深さが足りていない可能性。TURBO は設計上「低遅延優先で途切れ上等」。
   `stats().droppedFrames` / `packetsLost` と `estDelayMs` を併せて見る。

## ライセンス

MIT © Hapbeat

バイト単位のワイヤ仕様は [docs/wire-format.md](docs/wire-format.md) を参照。
