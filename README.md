# UNOR4_Chtbi-T_EFI

Arduino UNO R4（RA4M1）/ 互換環境上で動作するエコラン車両用ECUプログラム

## 概要

- メイン周期処理: [`Routine`](src/main.cpp)
  - (AGTimerライブラリ [`AGTimer.init`](lib/AGTimer_R4_Library/src/AGTimerR4.h) による24usec毎処理)
- 割り込み:
  - 車速入力: [`WH_PULSE_ISR`](src/main.cpp)
  - クランク角 A: Arduino attachInterrupt + `ReadNe_ISR`（パルスでクランク角更新）
  - カム角: [`G_PULSE_ISR`](src/main.cpp)
- 高速 GPIO: [`fastestdigitalWrite` / `fastestdigitalRead`](src/fastestdigitalRW.hpp)
- フリータスク: FreeRTOS で 500ms 周期 [`statusTask`](src/main.cpp)

## ディレクトリ構成（抜粋）

- [src/main.cpp](src/main.cpp) : コアロジック
- [src/fastestdigitalRW.hpp](src/fastestdigitalRW.hpp) : ボード別最速 GPIO
- [lib/AGTimer_R4_Library](lib/AGTimer_R4_Library) : 周期タイマ
- [platformio.ini](platformio.ini) : ビルド環境定義
- microSD/ : 走行ログや MAP 用 CSV（将来拡張）
- log/ : 記録例
- [document/arduino-workflow.puml](document/arduino-workflow.puml) : PlantUML ワークフロー図（詳細版）

## 対応ボード / ビルド

PlatformIO 環境: [platformio.ini](platformio.ini)

| Env | ボード | 備考 |
|-----|--------|------|
| `uno_r4_minima` | Arduino UNO R4 Minima | デフォルト |
| `rmc_ra4m1_20` | カスタム RA4M1 (`-D rmc_ra4m1_20`) | SD動作分岐あり |
| `uno_r3` | ATmega328P | 高速GPIO分岐あり |

ビルド例:

```sh
pio run -e uno_r4_minima
pio run -e uno_r3
pio run -t upload
pio device monitor -b 115200
```

## 主要定数 / パラメータ

| 項目 | 定義 | 説明 |
|------|------|------|
| ROUTINE_CYCLE_US | 24 | メイン周期 (µs) |
| IGNITION_HOLD_US | 5000 | 点火出力保持時間 |
| PERIMETER_MM | 1548 | タイヤ周長(mm) |
| TACHO_RPM_MAX | 6000 | 上限保護 |

## ピン割り当て

- WH_IN / G_IN / STR_IN / ENGOFF_IN は `74HC14` によるシュミットトリガ回路でチャタリング防止・反転入力
- INJ_OUT / IGN_OUT / STR_OUT / DISRESET_OUT は `Nch MOSFET` による LOW アクティブ。  
- 詳細は [src/main.cpp](src/main.cpp) 参照。

| 信号 | 物理ピン | 説明 |
|------|----------|------|
| NE_A_IN | 2 | クランク角 A (1deg/パルス) |
| NE_B_IN | 8 | クランク角 B (位相判定) |
| NE_Z_IN | 9 | クランク角0deg基準 |
| WH_IN | 3 | 車軸パルス入力 <BR> 1回転で1パルスIN |
| G_IN | 5 | カムパルス入力 <BR> クランク角720°毎に1パルス入力 |
| STR_IN | 6 | エンジンスタートスイッチ |
| ENGOFF_IN | 7 | キルスイッチ |
| INJ_OUT | A0 | 燃料噴射 (LOW=ON) |
| IGN_OUT | A1 | 点火 (LOW=ON) |
| STR_OUT | A2 | スタータリレー |
| DISRESET_OUT | A3 | リセットランプ |
| MA735_CS | 10 | MA735 SPI CS |

LOW アクティブ出力注意 (INJ/IGN/STR/DISRESET)。

## 処理フロー概要

1. 割り込みで角度/速度更新  
   - `ReadNe_ISR`: NE_A パルスで `Ne_deg` ±1 更新（NE_Bの位相参照）  
   - `G_PULSE_ISR`: カムパルス同期フラグ設定  

   - 角度モデルと usecperdig
     - `usecperdig` は NE_A パルス間隔 (µs/deg) をクランク割り込み [`ReadNe_ISR`](src/main.cpp) 内で直接測定し更新。  
     - MA735 使用時は `usecperdig` を補間に使わない（`Ne_deg` を直接取得）。  
     - 推定補間時: `Ne_deg += ROUTINE_CYCLE_US / usecperdig` (エンコーダ無効かつ MA735無効時)。

     - 簡易平滑: 移動平均 (3:1)

       ```text
       usecperdig = (prev*3 + dt) / 4
       ```

     - 異常除外: `dt == 0` や 過大 (例 > 100000µs) は無視。

2. 周期関数 [`Routine`](src/main.cpp):
   - スタート/キル状態評価
   - カム同期タイムアウト→`cycleReset`
   - マップ更新: [`updateEngineMap`](src/main.cpp)
   - 噴射開始条件 (角度 >= `INJ_STR_CA`)
   - 噴射時間経過で OFF & 燃料量積算
   - 点火進角計算 & 保持時間後 OFF

3. 500ms タスク [`statusTask`](src/main.cpp):
   - 状態計算 (燃費, 稼働時間)
   - シリアル出力 (タブ/CSV)

## 燃料噴射計算

噴射時間: `calculatedINJ_time` (0.1ms単位) → 実際 µs: `injDuration = calculatedINJ_time * 100`  
燃料量近似:

```text
gasml += ( (Δt * 0.0000007) + 0.0015 ) / 1.5073
```

係数は実測燃費から逆算したインジェクタ流量補正。

## 点火制御

進角: `calculatedIGN_CA`  
条件: `Ne_deg >= (360 - calculatedIGN_CA)` で点火 LOW  
保持: `IGNITION_HOLD_US` 経過で HIGH 戻し。

## MAP

- デフォルト MAP: `defaultMap` (RPM 昇順) → 最初の `rpm` 超過前エントリ採用。

- 列順 (SD読込も同一):  

  ```text
  rpm,inj_time(x0.1msec),ign_ca(deg)
  ```

- 拡張:  
  - SD 読み込み有効化: フラグ `SDMapEnabled = true;` + `parseCSV()` 実装  
  - AFR 補正: A/Fセンサによる燃料噴射量補正のため、`Increase_Fuel` 分岐位置あり（未実装）
  - 将来: `INJ_STR_CA` 列追加予定 (現状 0 固定)。

## 高速GPIO

[`fastestdigitalRW.hpp`](src/fastestdigitalRW.hpp):

- AVR: `sbi/cbi` 直接制御
- UNO R4 (RA4M1): レジスタ `R_PORTx->PODR_b`
- その他: フォールバック `digitalWrite/digitalRead`

クリティカル区間 (割り込み内) の遅延最小化に寄与。

## タイマ

AGTimer: [`AGTimer.init(period_us, callback)`](lib/AGTimer_R4_Library/src/AGTimerR4.h)  
本プロジェクトでは 24µs 周期で [`Routine`](src/main.cpp) 呼び出し。  
周波数変更は `ROUTINE_CYCLE_US` を調整。

## FreeRTOS

- 監視タスク: `statusTask` (500ms)  
- 追加タスクは `xTaskCreate` で拡張可能。スタック 128 words は余裕少 → 拡張時は増量推奨。

## ログ / 出力

USB シリアル (タブ区切り) / `Serial1` (CSV)。  
出力フィールド: RPM, INJ(ms), IGN_CA, speed(km/h, 0.1分解能・停止時は最終パルス経過で減衰→約8sで0.0), distance(km), fuel(ml), km/L, worktime(s), Ne_deg。

## PlantUML 図の参照

- 図ファイル: `document/Arduino_ECU_Workflow.puml`  
- VS Code でのプレビュー: PlantUML 拡張(例: `jebbs.plantuml`)を使用して開く。  
- コマンド例 (Windows PowerShell):

```powershell
java -jar "$env:USERPROFILE\.vscode\extensions\jebbs.plantuml-2.18.1\plantuml.jar" -tsvg document/Arduino_ECU_Workflow.puml
```

- 図に含まれる主な数値注記:
  - ROUTINE_CYCLE_US = 24µs
  - PERIMETER_MM = 1548mm
  - 速度上限 99.9km/h (内部999)
  - カム同期タイムアウト ≈ 50ms
  - RPMゼロ化 ≈ 1.2s無信号
  - 速度強制ゼロ ≈ 8s無信号
  - 点火保持 5ms

## 拡張アイデア (TODO)

- MAP内パラメータ選択・燃料噴射・点火処理高速化  
  (現状では処理遅れに起因すると思われる過大な進角角度を設定している)
- SD から MAP 読込実装 (`parseCSV`)
- AFR センサ補正ロジック (`updateAFR`)
- クランク角推定のドリフト補正（非エンコーダ時）
- 例外検出 (センサ断線・異常 RPM)
- フラッシュ書き込みによる学習補正保存
- 単位/係数の物理モデル化（燃料密度, 噴射流量）

## ビルドオプションフラグ

| フラグ | 影響 |
|--------|------|
| `uno_r4_minima` | 自動 (PlatformIO env) |
| `rmc_ra4m1_20` | SD 初期化ブロック有効 |
| `uno_r3` | AVR 高速 I/O 経路使用 |

## ライセンス

- 本体: リポジトリ LICENSE (MIT)
- AGTimer ライブラリ: 同梱 MIT (作者表記参照)

## 安全上の注意

実車/燃焼系制御へ適用する際は下記を検討:

- ウォッチドッグ / フェールセーフ / 過回転保護など追加必須
- 電源ノイズ対策 (車載 12V → 5V/3.3V 安定化)
- I/O レベルと駆動回路(インジェクタ / イグナイタ)の絶縁
