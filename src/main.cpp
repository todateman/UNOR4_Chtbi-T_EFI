// http://akiracing.com/2018/01/12/arduino_tachometer/
// ECU Shield は入出力がHIGH/LOWが反転しているので注意
//#define CSV_PARSER_DONT_IMPORT_SD
#include <Arduino.h>
#include "fastestDigitalRW.hpp"

uint8_t INJ_time = 10;  // インジェクタ噴射時間(ms)
// uint8_t INJ_time = 6;   // インジェクタ噴射時間(ms)
// uint8_t INJ_time = 3;   // インジェクタ噴射時間(ms)
// uint8_t INJ_time = 1;   // インジェクタ噴射時間(ms)

uint8_t HW_IN = 2;      // 駆動軸パルスセンサ入力・外部割込み
uint8_t G_IN = 3;       // カム角センサ入力・外部割込み
uint8_t STR_IN = 5;     // スタータボタン入力
uint8_t ENGOFF_IN = 6;  // キルスイッチ入力
uint8_t RESET_IN = 7;   // 走行距離・燃費リセットボタン入力
uint8_t SD_IN = 9;      // microSD挿入チェック
uint8_t INJ_OUT = A0;   // インジェクタ出力
uint8_t IGN_OUT = A1;   // イグニッション出力
uint8_t STR_OUT = A2;   // スタータ出力
uint8_t DISRESET_OUT = A3;      // リセット状態出力(リセットされていればOFF)

void setup() {
  Serial.begin(115200);  // シリアル通信を開始

  // 出力オフ
  digitalWrite(INJ_OUT, HIGH);
  digitalWrite(IGN_OUT, HIGH);
  digitalWrite(STR_OUT, HIGH);
  digitalWrite(DISRESET_OUT, HIGH);

  pinMode(HW_IN, INPUT_PULLUP);
  pinMode(G_IN, INPUT_PULLUP);
  pinMode(STR_IN, INPUT_PULLUP);
  pinMode(ENGOFF_IN, INPUT_PULLUP);
  pinMode(RESET_IN, INPUT_PULLUP);
  pinMode(SD_IN, INPUT_PULLUP);
  pinMode(INJ_OUT, OUTPUT);
  pinMode(IGN_OUT, OUTPUT);
  pinMode(STR_OUT, OUTPUT);
  pinMode(DISRESET_OUT, OUTPUT);

  /*
  delay(100);
  digitalWrite(INJ_OUT, LOW);   // インジェクタON
  delay(100);
  digitalWrite(INJ_OUT, HIGH);  // インジェクタOFF
  digitalWrite(IGN_OUT, LOW);   // イグニッションON
  delay(100);
  digitalWrite(IGN_OUT, HIGH);  // イグニッションOFF
  digitalWrite(STR_OUT, LOW);   // スタータON
  delay(100);
  digitalWrite(STR_OUT, HIGH);  // スタータOFF
  */

}

void loop() {
  if ( digitalRead(ENGOFF_IN) == LOW && digitalRead(STR_IN) == LOW ){  // キルスイッチOFF & スタータボタン押す 
    for(int i=0; i++; i<500){
      Serial.println(i);
      fastestDigitalWrite(INJ_OUT, LOW);   // 噴射ON
      delayMicroseconds(INJ_time * 1000);
      fastestDigitalWrite(INJ_OUT, HIGH);  // 噴射OFF
      delay(100);
    }
    Serial.print("Finish! ");
    Serial.print(INJ_time);
    Serial.println("ms");
  }
}