# Changelog

**HapbeatEspNowRx**（`hapbeat-espnow-rx`）の主要な変更点をまとめます。

## 0.1.0 — 初の公開リリース

Hapbeat 送信機の ESP-NOW 音声ブロードキャストを、任意の ESP32 ボードで受信して
I2S に出す受信側ライブラリ。Hapbeat 受信機ファームウェア
（`hapbeat-device-firmware/src/espnow_stream.cpp` ほか）からの移植で、Hapbeat 固有の
ハードウェア依存（NVS 設定・OLED・ボタン・音量ハード・コーデック IC）は持ち込んでいません。

### 受信・再生
- ESP-NOW 0xAA ストリームパケットの受信、シーケンス追跡、ピギーバック冗長フレームに
  よる損失回復、Opus PLC による欠落補間。
- 出力は常に 16 kHz インターリーブド ステレオ PCM16。8 kHz モノラルのモードは
  ×2 アップサンプル + L=R 複製（ファームウェアと同じ挙動）。
- ドリフト補正付きジッタバッファ。送信機の ADC クロックと受信機の DAC クロックは
  別水晶（±0.1〜0.5 %）なので、固定長バッファでは数秒おきに枯渇/溢れが起きる。
  1 ブロックあたり ±1 フレームで fill を目標値に寄せ、アンダーラン時は
  ホールド＋減衰でクリックを避ける。
- 専用の高優先度 I2S 出力タスク（既定 core 1 / prio 19）。`i2s_write()` のブロックで
  ペーシングするので独自のスリープや時刻計算は無し。
- 出力は差し替え可能: 独自の `FrameSink` を `begin()` に渡す、`readFrames()` で
  引き取る、`onPcmTap()` で覗く、のいずれも可能。

### ストリームモード
- 10 モード（SOLID / FAST / BALANCED / SMOOTH / STEREO / HIFI / LITE / TURBO /
  FINE / SOLID48）のテーブルはファームウェアの逐語コピー。
- **IMA-ADPCM 系（0 / 6 / 7）は依存ゼロ**で再生可能。送信機の既定 SOLID が
  これに当たるため、素の状態でも音が出る。
- **Opus 系は opt-in**（`-DHAPBEAT_ESPNOW_RX_OPUS`）。無効ビルドで Opus モードを
  受けた場合はノイズを鳴らさず破棄し、`StreamStats::modeUnsupported` を数える。

### マルチ送信機環境
- ソース選択（§7.1）: 会場に複数の送信機・リピーターがあっても 1 台の MAC に
  ロックして、それだけをデコードする。無音タイムアウトと 4 秒の配信レート比較に
  よるヒステリシス付きなので、境界条件で 2 台の間を往復して音が乱れない。
- `forceRelock()`（現在のソースを一定時間ブラックリストして選び直す）、
  `setRelayTestOnly()`（中継パケットのみ受け付けてリピーター到達範囲を確認する。
  永続化しないので再起動で解除）。
- 0xAC フリートチューンビーコン（§3.5）に対応。エポック規則は DEC-045 準拠で
  「厳密に古いエポックだけ拒否・同一エポックの再送は再適用」。値の範囲チェックを
  通ってからエポックをラッチするので、不正値がエポックを焼いて以降の訂正再送を
  取りこぼす事故が起きない。
- 音量上限（param 5）はアプリケーションへコールバックで渡す（本ライブラリは
  音量ハードウェアを持たないため）。

### 無線まわり
- チャンネル verify ループ: `esp_wifi_set_channel()` は ESP_OK を返した後に
  STA の start/disconnect シーケンスで非同期にチャンネル 1 へ戻されることがある。
  `esp_now_init()` の後に設定し、`esp_wifi_get_channel()` が一致するまで再設定する。
- Long Range マスクを既定で有効（通常の受信には影響しない）。運用中に送信機だけを
  LR に切り替えても、受信機を焼き直さずに済む。

### 移植にあたっての注意
- `src/protocol/` は `Arduino.h` / `esp_now.h` / `millis()` を使わない。時刻は
  引数 `now_ms` で受け取るので、ホスト PC 上でビルド・テストできる
  （`extras/host-test/`）。
- ワイヤ形式は実装が正。`hapbeat-contracts` の `specs/espnow-stream.md` §3.3 は
  Opus の長さフィールドを uint16 LE と書いているが、送受とも実際は 1 バイト。
  詳細は [docs/wire-format.md](docs/wire-format.md)。

### 未対応（今後の予定）
- 受信機からの送信（テレメトリ・ACK）は一切なし（設計上の一斉同報）。
- Hapbeat DuoWL v4 の HP 48k 経路（0xAC param 6）は解釈せず無視する。
