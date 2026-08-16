// http://akiracing.com/2018/01/12/arduino_tachometer/
// ECU Shield は入出力がHIGH/LOWが反転しているので注意
#include <Arduino.h>
#include "fastestdigitalRW.hpp"

// =====================================================================
// インジェクタ単体特性試験用スケッチ（INJ_testブランチ改良版）
//
// 元コードからの変更点:
//  1) for(int i=0; i++; i<500) のバグを修正（条件式と更新式が入れ替わっており
//     元コードは1発も噴射せずに抜けていた）
//  2) パルス幅・パルス数・パルス間隔をSerial経由で変更可能にし、
//     無効噴射時間の直線近似法（複数パルス幅でのスイープ）を
//     書き込み直しなしで実施できるようにした
//  3) 試験中にキルスイッチを戻すと即中断する安全策を追加
//  4) 完了時にサマリを表示し、オシロ/重量計での記録と対応付けやすくした
//  5) main.cppをmainブランチの現行版（パルスエンコーダ+ISR方式）へ復元した際に、
//     ピン配置・ヘッダ名（fastestdigitalRW.hpp）を追従させた。
//     本試験に不要なセンサ系ピン（旧HW_IN/G_IN/RESET_IN/SD_IN）は削除。
//
// platformio.ini の `uno_r4_minima_inj_test` 環境でこのファイルのみをビルドする。
//   ビルド/書き込み: pio run -e uno_r4_minima_inj_test -t upload
// =====================================================================

float    INJ_time_ms     = 4.0;   // インジェクタ噴射パルス幅 [ms]（0.1ms単位推奨）
uint16_t INJ_count        = 3000; // 1回の計測で打つパルス数
uint16_t INJ_interval_ms  = 100;  // パルス間隔 [ms]（噴射時間を含まないOFF区間）

uint8_t STR_IN = 6;       // スタータボタン入力
uint8_t ENGOFF_IN = 7;    // キルスイッチ入力
uint8_t INJ_OUT = A0;     // インジェクタ出力
uint8_t IGN_OUT = A1;     // イグニッション出力
uint8_t STR_OUT = A2;     // スタータ出力
uint8_t DISRESET_OUT = A3; // リセット状態出力

bool testRunning = false;

void printSettings() {
  Serial.println(F("---- 現在の設定 ----"));
  Serial.print(F("パルス幅[ms] : ")); Serial.println(INJ_time_ms, 1);
  Serial.print(F("パルス数     : ")); Serial.println(INJ_count);
  Serial.print(F("間隔[ms]     : ")); Serial.println(INJ_interval_ms);
  Serial.println(F("設定変更: W<ms> / N<回数> / I<ms>  (例: W2.5 / N4000 / I50)"));
  Serial.println(F("開始: キルスイッチOFF(導通) + スタータON(導通) で噴射バースト開始"));
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() < 2) return;

  char cmd = toupper(line.charAt(0));
  float val = line.substring(1).toFloat();

  switch (cmd) {
    case 'W':
      if (val > 0) INJ_time_ms = val;
      break;
    case 'N':
      if (val > 0) INJ_count = (uint16_t)val;
      break;
    case 'I':
      if (val >= 0) INJ_interval_ms = (uint16_t)val;
      break;
    default:
      Serial.println(F("不明なコマンドです（W/N/I のいずれかを使用）"));
      return;
  }
  printSettings();
}

void setup() {
  Serial.begin(115200);

  digitalWrite(INJ_OUT, HIGH);
  digitalWrite(IGN_OUT, HIGH);
  digitalWrite(STR_OUT, HIGH);
  digitalWrite(DISRESET_OUT, HIGH);

  pinMode(STR_IN, INPUT_PULLUP);
  pinMode(ENGOFF_IN, INPUT_PULLUP);
  pinMode(INJ_OUT, OUTPUT);
  pinMode(IGN_OUT, OUTPUT);
  pinMode(STR_OUT, OUTPUT);
  pinMode(DISRESET_OUT, OUTPUT);

  delay(300);
  Serial.println(F("=== インジェクタ特性試験モード ==="));
  printSettings();
}

void loop() {
  handleSerial();

  // キルスイッチOFF(導通=LOW) & スタータボタン押下(導通=LOW) でバースト開始
  if (!testRunning &&
      digitalRead(ENGOFF_IN) == LOW &&
      digitalRead(STR_IN) == LOW) {

    testRunning = true;
    unsigned long pulse_us = (unsigned long)(INJ_time_ms * 1000.0);

    Serial.println(F("=== 噴射テスト開始 ==="));
    Serial.print(F("パルス幅: ")); Serial.print(INJ_time_ms, 1); Serial.println(F(" ms"));
    Serial.print(F("パルス数: ")); Serial.println(INJ_count);
    Serial.println(F("(オシロはINJ_OUTの立下りエッジでトリガしてください)"));

    uint16_t firedCount = 0;
    for (uint16_t i = 0; i < INJ_count; i++) {
      fastestdigitalWrite(INJ_OUT, LOW);    // 噴射ON（トリガ基準エッジ）
      delayMicroseconds(pulse_us);
      fastestdigitalWrite(INJ_OUT, HIGH);   // 噴射OFF
      firedCount = i + 1;
      delay(INJ_interval_ms);

      if (firedCount % 200 == 0) {
        Serial.println(firedCount);
      }

      // 安全策: 試験中にキルスイッチが戻されたら即中断
      if (digitalRead(ENGOFF_IN) == HIGH) {
        Serial.println(F("!! キルスイッチ復帰のため中断 !!"));
        break;
      }
    }

    Serial.print(F("=== 完了: パルス幅 "));
    Serial.print(INJ_time_ms, 1);
    Serial.print(F(" ms x "));
    Serial.print(firedCount);
    Serial.println(F(" 発 ==="));
    Serial.println(F("計測値（重量計・メスシリンダ）を記録してください。"));
    Serial.println(F("スタータボタンを離してから次の設定へ進んでください。"));

    // スタータボタンが離されるまで待機（意図しない連続再トリガ防止）
    while (digitalRead(STR_IN) == LOW) {
      delay(10);
    }
    testRunning = false;
    printSettings();
  }
}
