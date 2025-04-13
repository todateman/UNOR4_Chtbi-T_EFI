// http://akiracing.com/2018/01/12/arduino_tachometer/
// ECU Shield は入出力がHIGH/LOWが反転しているので注意
//#define CSV_PARSER_DONT_IMPORT_SD
#include <Arduino.h>
//#include <CSV_Parser.h>  // https://github.com/michalmonday/CSV-Parser-for-Arduino
#include <SPI.h>
#include "SD.h"
#include "fastestdigitalRW.hpp"
#include "AGTimerR4.h"
#include <Arduino_FreeRTOS.h>

// ステータス出力タスクのハンドル
TaskHandle_t UpdatemainloopHandle;

uint8_t IGN_Standard = 0;  // 点火時期基準[CA] 全体のMAPの基準になる点火時期。0でTDC,+XXで進角,-XXで遅角

volatile unsigned long tachoBefore = 0;     // カム角センサーの前回の反応時の時間
volatile unsigned long tachoAfter = 0;      // カム角センサーの今回の反応時の時間
volatile unsigned long tachoWidth = 0;      // カム一回転の時間　tachoAfter - tachoBefore
volatile unsigned long speedBefore = 0;     // 車軸パルスセンサーの前回の反応時の時間
volatile unsigned long speedAfter = 0;      // 車軸パルスセンサーの今回の反応時の時間
volatile unsigned long speedWidth = 0;      // 車軸一回転の時間　speedAfter - speedBefore
volatile int16_t Ne_deg = 0;                // 磁気エンコーダのクランク角(deg)
volatile int8_t Ne_rev = 0;                 // クランク回転数(クランク角計算用)
volatile unsigned long timeNow_INJ_ON = 0;  // 噴射開始時の時間
volatile unsigned long timeNow_INJ_OFF = 0; // 噴射終了時の時間
volatile unsigned long timeNow_IGN_ON = 0;  // 点火開始時の時間
volatile unsigned long timeNow_IGN_OFF = 0; // 噴射終了時の時間
volatile unsigned long distancemm = 0;      // 走行距離積算(mm)
volatile uint16_t distance = 0;   // 走行距離積算(m)
volatile unsigned long speed = 0; // 速度(km/h)
volatile uint16_t tachoRpm = 0;   // エンジンの回転数(rpm)
volatile uint8_t INJ_time = 0;    // インジェクタ噴射時間(*0.1ms) 速度向上のため10倍して表記(1.2ms->12)
volatile int16_t IGN_CA = 0;      // 点火時期[CA]
volatile uint8_t INJ_Status = 1;  // 噴射ステータス(0: 無効 1: OFF 2:ON)
volatile uint8_t IGN_Status = 1;  // 点火ステータス(0: 無効 1: OFF 2:ON)
volatile float AFR_value = 0.0;   // A/Fセンサーの値
volatile bool INJ_His = false;    // 1サイクル中の噴射履歴
volatile bool IGN_His = false;    // 1サイクル中の点火履歴
volatile bool G_Pulse = false;    // カムパルス信号
volatile bool G_Pulse_Flag = false;  // カムパルス信号の立ち上がりフラグ
volatile bool CycleReset = false; // サイクルリセットフラグ
bool OLED = false;                // OLED有効/無効
bool Encoder = true;             // MA735磁気エンコーダパルス有効/無効
bool MA735SPI = false;             // MA735磁気エンコーダSPI有効/無効
bool AFR = false;                 // A/Fセンサ有効/無効
bool Increase_Fuel = false;       // 燃料増量係数有効/無効
bool Serial_ON = true;            // Serial(USBシリアル)有効/無効
bool Serial1_ON = true;           // Serial1(メーター・ロガーへのシリアル)有効/無効

uint8_t NE_A_IN = 2;              // P105 クランクAパルスセンサ入力・外部割込み
uint8_t WH_IN = 3;                // P104 駆動軸パルスセンサ入力・外部割込み
uint8_t G_IN = 5;                 // P102 カム角センサ入力
uint8_t STR_IN = 6;               // P106 スタータボタン入力
uint8_t ENGOFF_IN = 7;            // P107 キルスイッチ入力
uint8_t NE_B_IN = 8;              // P304 クランクBパルスセンサ入力
uint8_t NE_Z_IN = 9;              // P303 クランクZパルスセンサ入力
uint8_t MA735_CS = 10;            // P112 MA735磁気エンコーダSPIチップセレクト
uint8_t INJ_OUT = A0;             // P014 インジェクタ出力
uint8_t IGN_OUT = A1;             // P000 イグニッション出力
uint8_t STR_OUT = A2;             // P001 スタータ出力
uint8_t DISRESET_OUT = A3;        // P002 リセット状態出力(リセットされていればOFF)
float usecperdig = 1.0;           // クランク1°当たりの時間(usec)
uint16_t perimeter = 1548;        // 車軸1回転当たりの周長(mm)
uint8_t INJ_STR_CA = 65;          // 燃料噴射タイミング角度(deg)を設定
float gasml = 0.0;                // 積算燃料消費量(ml)
float INJ_timems = 0.0;           // 燃焼噴射時間(msec)
float dispergas = 0.0;            // 燃費(km/l)
unsigned long starttime = 0;      // 走行開始時間(msec)
uint16_t worktime = 0;            // 走行時間(sec)
bool Launch = false;              // 走行開始状態
const uint8_t Routine_Cycle = 24; // 燃料噴射・点火制御の実行サイクル(usec)
float Stoichi = 11.69;            // 目標空燃比(理想空燃比を設定する場合、ガソリン：14.70, CN燃料：13.75
                                  //           出力空燃比を設定する場合、ガソリン：12.50, CN燃料：11.69付近？)
float _Stoichi = 0.0;             // 理想空燃比の逆数(燃料増量係数計算用)
float Fuel_boost_factor = 1.0;    // 燃料増量係数

// framework-arduinorenesas-uno 1.3.2ベースのライブラリではSPI.cppで宣言済みのため不要なのでコメントアウト
// ArduinoSPI SPI(MISO1, MOSI1, SCK1, FORCE_SPI1_MODE);   // RMC-RA4M1のmicroSD用SPIを選択(SPIの各端子はpins_arduino.hで定義)
// ArduinoSPI SPI(MISO, MOSI, SCK, FORCE_SPI_MODE);       // Arduino UNO R4のSPIを使用(MISO=12,MOSI=11,SCK=13)
SPISettings MA735settings = SPISettings(12000000, MSBFIRST, SPI_MODE0);    // MA735磁気エンコーダSPIの設定

String MAPFILE = "RPM.CSV";  // 点火MAPファイル名
bool SDMAP = false;
uint16_t rpm1[20]; //要素記憶用の配列を作成
uint8_t inj1[20];
int8_t  ign1[20];
uint8_t row;

// 点火MAP
void INJ_IGN() {
  if (tachoRpm < 400) {
    INJ_time = 90;
    IGN_CA = 0;
  }
  else if (tachoRpm < 800) {
    INJ_time = 90;
    IGN_CA = 5;
  }
  else if (tachoRpm < 1200) {
    INJ_time = 90;
    IGN_CA = 7;
  }
  else if (tachoRpm < 1600) {
    INJ_time = 90;
    IGN_CA = 9;
  }
  else if (tachoRpm < 2000) {
    INJ_time = 90;
    IGN_CA = 11;
  }
  else if (tachoRpm < 2400) {
    INJ_time = 90;
    IGN_CA = 13;
  }
  else if (tachoRpm < 2800) {
    INJ_time = 90;
    IGN_CA = 15;
  }
  else if (tachoRpm < 3200) {
    INJ_time = 90;
    IGN_CA = 17;
  }
  else if (tachoRpm < 3600) {
    INJ_time = 90;
    IGN_CA = 17;
  }
  else if (tachoRpm < 4000) {
    INJ_time = 90;
    IGN_CA = 17;
  }
  else if (tachoRpm < 4400) {
    INJ_time = 90;
    IGN_CA = 20;
  }
  else if (tachoRpm < 4800) {
    INJ_time = 90;
    IGN_CA = 20;
  }
  else if (tachoRpm < 5200) {
    INJ_time = 90;
    IGN_CA = 20;
  }
  else if (tachoRpm < 5600) {
    INJ_time = 90;
    IGN_CA = 20;
  }
  else if (tachoRpm < 6000) {
    INJ_time = 90;
    IGN_CA = 20;
  }
  else {
    INJ_time = 0;
    IGN_CA = 0;
  }
  if (fastestdigitalRead(STR_IN) == LOW){    // スタータボタン(D6, P106)を押したとき
    INJ_time = 100;
    IGN_CA = 0;
  }
}


// SDの点火MAP
void INJ_IGN_SD() {
  for (uint8_t i = 0; i < row; i++) { //整理したデータをタブで区切って表示
    if (tachoRpm < rpm1[i]) {
      INJ_time = inj1[i];
      IGN_CA = ign1[i];
      break;
    }
  }
  if (tachoRpm >= rpm1[row]){         // MAP以上の回転数(オーバーレブ)の場合
    INJ_time = 0;
    IGN_CA = 0;
  }
  if (fastestdigitalRead(STR_IN) == LOW){    // スタータボタン(D6, P106)を押したとき
    INJ_time = 50;
    IGN_CA = 0;
  }
}

// エンジン回転処理リセット
void Cycle_Reset() {
  if (SDMAP) {                              // SD内の点火MAPが使用できる場合
    INJ_IGN_SD();                           // SDから読んだ点火MAP
  }
  else {
    INJ_IGN();                              // 本コード内の点火MAP
  }

  //if (INJ_time > 0) {
  if (tachoRpm <= 6000) {
    INJ_Status = 1;                         // 噴射OFF
    IGN_Status = 1;                         // 点火OFF
  }
  else {
    INJ_Status = 0;                         // 噴射無効
    IGN_Status = 0;                         // 点火無効
    timeNow_INJ_ON = NULL;
    timeNow_INJ_OFF = NULL;
  }
  //INJ_Status = 1;                         // 噴射ステータスOFF
  //IGN_Status = 1;                         // 点火ステータスOFF
  INJ_His = false;                          // 噴射履歴をリセット
  IGN_His = false;                          // 点火履歴をリセット

  /*
  if (Encoder || MA735SPI) {                // エンコーダパルス or MA735SPIが有効の場合
    Serial.print("Cycle_Reset: ");
    Serial.println(Ne_deg);
  }
  */
}

// 車軸パルスから走行距離・速度を算出
void WH_PULSE() {
  speedAfter = micros();                    // 現在の時刻を記録
  speedWidth = speedAfter - speedBefore;    // 前回と今回の時間の差(車軸1回転当たりの時間 usec)を計算
  speed = 3600 * perimeter / speedWidth;    // 速度(km/h = 3600 * km/sec = 3600 * mm/usec)を計算
  speedBefore = speedAfter;                 // 今回の値を前回の値に代入する
  distancemm += perimeter;                  //走行距離(mm)を積算
  distance = distancemm * 0.001;            //走行距離(m)を積算
}

// エンコーダパルスからクランク角の読み取り
void ReadNe(){
  if (!fastestdigitalRead(NE_B_IN)) {       // クランクBパルス(D8, P304)がOFFの場合
    Ne_deg += 1;                              // 1パルス毎にクランク角を360°/360=1°ずつ加算
  } else {                                  // クランクBパルス(D8, P304)がONの場合
    Ne_deg -= 1;                              // 1パルス毎にクランク角を360°/360=1°ずつ減算   
  }

  if (fastestdigitalRead(NE_Z_IN)) {        // クランクZパルス(D9, P303)がONの場合 = クランク1回転の場合
    tachoAfter = micros();                    // 現在の時刻を記録
    tachoWidth = tachoAfter - tachoBefore;    // 前回と今回の時間の差(カムシャフト1回転当たりの時間 usec)を計算
    tachoBefore = tachoAfter;                 // 今回の値を前回の値に代入する
    tachoRpm = (60000000.0 / tachoWidth);     //クランクの回転数[rpm]を計算
    
    if (Ne_deg > 360 && G_Pulse_Flag) {       // クランク角が360°より大きい&カムパルスセンサ立ち上がりフラグONの場合
      Ne_deg = 0;                               // クランク角を0°にリセット
      G_Pulse_Flag = false;                     // カムパルスセンサの立ち上がりフラグをOFF
      CycleReset = true;                        // サイクルリセットフラグON
    }
  }
}

// MA735磁気エンコーダSPIで角度を読み取る
int16_t readMA735SPI() {
  static uint16_t _rd = 0;                  // 16bit(0-65535)の前回の角度
  static unsigned long _rd_time = 0;        // 前回の角度を取得した時間

  SPI.beginTransaction(MA735settings);
  fastestdigitalWrite(MA735_CS, LOW);
  uint16_t rd = SPI.transfer16(0);          // 16bit(0-65535)で現在の角度を取得
  fastestdigitalWrite(MA735_CS, HIGH);
  SPI.endTransaction();

  long diff_rd = rd - _rd;                  // クランク角の差分を計算
  if (diff_rd < -32767) {                   // クランク角の差分が-180°以上の場合(=正転でクランク角が上死点を超えた場合)
    unsigned long rd_time = micros();         // 現在の時間を取得
    Ne_rev++;                                 // 回転数(1回転,2回転,...)をカウントアップ

    // 回転数の計算
    if (diff_rd < 262) {                    // 10000rpm(=1.44deg/24usec)相当以下の回転数の場合(チャタリング防止)
      tachoRpm = (60000000.0 / (rd_time - _rd_time));     // クランクの回転数[rpm]を計算
    }
    _rd_time = rd_time;                     // 現在の時間を前回の時間に設定

    if (G_Pulse_Flag) {                      // カムパルスセンサ立ち上がりフラグONの場合
      Ne_rev = 0;                               // クランク回転数をリセット
      G_Pulse_Flag = false;                     // カムパルスセンサの立ち上がりフラグをOFF
      // Serial.println("G_Pulse_Flag: false");
      CycleReset = true;                        // サイクルリセットフラグON
    }
  }
  else if (diff_rd > 32767) {               // クランク角の差分が180°以上の場合(=逆転でクランク角が上死点を超えた場合)
    Ne_rev--;                                 // 回転数(1回転,2回転,...)をカウントダウン
  }
  _rd = rd;                                 // 現在の角度を前回の角度に設定

  return rd / 65535 * 360 + 360 * Ne_rev;     // 0-720(deg)に変換
}

// カム角センサーから回転数計算
void tachometer() {
  tachoAfter = micros();                    // 現在の時刻を記録
  tachoWidth = tachoAfter - tachoBefore;    // 前回と今回の時間の差(カムシャフト1回転当たりの時間 usec)を計算
  tachoBefore = tachoAfter;                 // 今回の値を前回の値に代入する

  if (!Encoder && !MA735SPI) {              // 磁気エンコーダパルス と MA735磁気エンコーダSPIの両方が無効の場合
    Ne_deg = 0;                               // クランク角を上死点にリセット
    // ゼロ除算対策
    if (tachoWidth > 0) {
      tachoRpm = (60000000.0 / tachoWidth) * 2; //クランクの回転数[rpm]を計算
      usecperdig = tachoWidth / 720.0;          // クランク1°当たりの時間(usec)
    } else {
      tachoRpm = 0;                             // クランクの回転数[rpm]を0にリセット
      usecperdig = 1.0;                         // クランク1°当たりの時間(usec)を0.001にリセット
    }
    CycleReset = true;                        // サイクルリセットフラグON
  }
}

// AFRセンサーの値を取得
void getAFR() {
  uint16_t ad = R_ADC0->ADDR[21];                   // A4ポート(P101, AN021)のA/D変換結果を取得
  AFR_value = map (ad, 0, 16383, 1000, 2000) * 0.01;  // A/D変換結果を10.00-20.00の空燃比に変換
  if (Increase_Fuel) {                                // 燃料増量係数の補正が有効な場合
    Fuel_boost_factor = AFR_value * _Stoichi;           // 目標空燃比から求める燃料増量係数を計算
  }

}

// 燃料噴射・点火制御
void Routine() {
  if (AFR) {                            // A/Fセンサが有効の場合
    getAFR();                             // A/Fセンサの値を取得
  }
  
  if (MA735SPI) {                       // MA735磁気エンコーダSPIが有効の場合
    Ne_deg = readMA735SPI();              // MA735磁気エンコーダSPIの値を取得
  }

  if (!Encoder && !MA735SPI) {          // 磁気エンコーダパルス or MA735磁気エンコーダSPIの両方が無効の場合
    // ゼロ除算対策
    if (usecperdig > 0.001) {  // 小さすぎる値でも除算しない
      Ne_deg += Routine_Cycle / usecperdig;
    }
  }
  
  if (fastestdigitalRead(G_IN) == LOW){ // カムパルス(D5, P102)がONの場合
    if (!G_Pulse) {
      tachometer();
      G_Pulse = true;
      G_Pulse_Flag = true;
      // Serial.println("G_Pulse_Flag: true");
    }
  }
  if (fastestdigitalRead(G_IN) == HIGH){ // カムパルス(D5, P102)がOFFの場合
    if (G_Pulse) {
      G_Pulse = false;
    }
  }

  if (CycleReset) {                       // サイクルリセットフラグがONの場合
    Cycle_Reset();                          // エンジン運転のリセット処理
    CycleReset = false;                     // サイクルリセットフラグをリセット
  }

  // 噴射ONステータス時
  if ( INJ_Status == 2 ) {
    if ( micros() - timeNow_INJ_ON >= INJ_time * 100 * Fuel_boost_factor ){  // 噴射開始から噴射時間(燃料増量係数で補正)が経過したら
      timeNow_INJ_OFF = micros();                 // 噴射終了時の時刻を記録
      //digitalWrite(INJ_OUT, HIGH);              // 噴射OFF
      fastestdigitalWrite(INJ_OUT, HIGH);         // 噴射OFF
      INJ_Status = 1;                             // 噴射無効ステータス
      gasml += ( (timeNow_INJ_OFF - timeNow_INJ_ON) * 0.0000007 + 0.0015 ) / 1.5073;  // 燃料消費量(ml)を積算 2024.10.13の全国大会CN燃料結果で燃料消費量を補正
      /*
      if (Encoder || MA735SPI) {                  // 磁気エンコーダパルス or MA735SPIが有効の場合 
        Serial.print("INJ_OFF: ");
        Serial.println(Ne_deg);
      }
      */
    }
  }

  // 点火ONステータス時
  if ( IGN_Status == 2 ) {
    if ( micros() - timeNow_IGN_ON >= 5000){    // 点火維持時間(5000us)が経過したら
      timeNow_IGN_OFF = micros();                 // 点火終了時の時刻を記録
      //digitalWrite(IGN_OUT, HIGH);              // 点火OFF
      fastestdigitalWrite(IGN_OUT, HIGH);         // 点火OFF
      IGN_Status = 1;                             // 点火無効ステータス
      /*
      if (Encoder || MA735SPI) {                  // 磁気エンコーダパルス or MA735SPIが有効の場合
        Serial.print("IGN_OFF: ");
        Serial.println(Ne_deg);
      }
      */
    }
  }

  if (fastestdigitalRead(ENGOFF_IN) == LOW){  // キルスイッチピン(D7, P107)がOFFの場合
    // スタータボタンを押したとき
    if (fastestdigitalRead(STR_IN) == LOW){     // スタータボタン(D6, P106)を押したとき
      //digitalWrite(STR_OUT, LOW);               // スタータON
      fastestdigitalWrite(STR_OUT, LOW);          // スタータON
      //Serial.println("STR_ON");
      Launch = true;                              // 走行開始ON
    }
    else {                                      // スタータボタン(D6, P106)が押されていない場合
      //digitalWrite(STR_OUT, HIGH);              // スタータOFF
      fastestdigitalWrite(STR_OUT, HIGH);         // スタータOFF
    }

    // 噴射OFFステータス時
    if ( INJ_Status == 1 && !INJ_His ) {
      if ( Ne_deg >= INJ_STR_CA ){                // 上死点から燃料噴射タイミングの角度に達したら
        timeNow_INJ_ON = micros();                  // 噴射開始時の時刻を記録
        INJ_His = true;                             // 噴射履歴あり
        //digitalWrite(INJ_OUT, LOW);               // 噴射ON
        fastestdigitalWrite(INJ_OUT, LOW);          // 噴射ON
        INJ_Status = 2;                             // 噴射ONステータス
        /*
        if (Encoder || MA735SPI) {                  // 磁気エンコーダパルス or MA735SPIが有効の場合
          Serial.print("INJ_ON:  ");
          Serial.println(Ne_deg);
        }
        */
      }
    }
  
    //点火OFFステータス時
    if ( IGN_Status == 1 && !IGN_His ) {  
      if ( Ne_deg >= 360 - IGN_CA ){              // 圧縮→膨張サイクルの上死点から進角角度に達したら
        timeNow_IGN_ON = micros();                  // 点火開始時の時刻を記録
        IGN_His = true;                             // 点火履歴あり
        //digitalWrite(IGN_OUT, LOW);               // 点火ON
        fastestdigitalWrite(IGN_OUT, LOW);          // 点火ON
        IGN_Status = 2;                             // 点火ONステータス
        /*
        if (Encoder || MA735SPI) {                  // 磁気エンコーダパルス or MA735SPIが有効の場合
          Serial.print("IGN_ON:  ");
          Serial.println(Ne_deg);
        }
        */
      }
    }
  }
}

// シリアル送信
void Serialsend() {
  if (Serial_ON){                             // USBシリアルが有効なら
    Serial.print(tachoRpm);
    Serial.print("\t");
    Serial.print(INJ_timems, 1);
    Serial.print("\t");
    Serial.print(IGN_CA);
    Serial.print("\t");
    Serial.print(speed);
    Serial.print("\t");
    Serial.print(distance);
    Serial.print("\t");
    Serial.print(gasml, 1);
    Serial.print("\t");
    Serial.print(dispergas, 1);
    Serial.print("\t");
    Serial.print(worktime);
    Serial.print("\t");
    if (!Encoder && !MA735SPI) {
      Serial.print("(");
    }
    Serial.print(Ne_deg);
    Serial.print("'");
    if (!Encoder && !MA735SPI) {
      Serial.print(")");
    }
    if (Encoder && MA735SPI) {            // 磁気エンコーダパルス or MA735SPIが有効の場合 
      Serial.print("\t");
      //Serial.print(G_Pulse, HEX);
      Serial.print(G_Pulse_Flag, HEX);
    }
    if (AFR) {                            // A/Fセンサが有効の場合
      Serial.print("\t");
      Serial.print(AFR_value, 1);
    }
    Serial.println();
  }
  if (Serial1_ON){                             // 外部シリアルが有効なら
    Serial1.print(tachoRpm);
    Serial1.print(",");
    Serial1.print(INJ_timems, 1);
    Serial1.print(",");
    Serial1.print(IGN_CA);
    Serial1.print(",");
    Serial1.print(speed);
    Serial1.print(",");
    Serial1.print(distance);
    Serial1.print(",");
    Serial1.print(gasml, 1);
    Serial1.print(",");
    Serial1.print(dispergas, 1);
    Serial1.print(",");
    Serial1.print(worktime);
    if (AFR) {                            // A/Fセンサが有効の場合
      Serial.print(",");
      Serial.print(AFR_value, 1);
    }
    Serial1.println();
  }
}

// 点火MAPファイル読み込み
void parseCSV() {
  File file = SD.open(MAPFILE, FILE_READ);  // ファイルを開く

  String line;                // 行を一時的に記憶する変数
  int row_inheader = 0;       // ヘッダーも含めた行数を記憶する変数
  while (file.available()) {  // データが有ればループ
    SDMAP = true;             // SDMAPを有効にする
    char ch = file.read();    // データを1byteずつ読み取り
    line += String(ch);       // 1byteずつ繋げていく
    if (ch == '\n') {         // '\n'が読み込まれた所で一旦読み取りをやめる
      Serial.print(line);     // 行全体を画面に表示
      line.trim();            // 行に含まれる不要な空白を取り除く

      int column = 0;         // 行に含まれる要素番号カウントする変数
      int num_end = 0;        // 要素の終わりの列番号を記憶する変数
      int num_start = 0;      // 要素の始まりの列番号を記憶する変数

      if (0 < row_inheader) {       // ヘッダーを読み込まないように除外する
        while (num_end != -1) {     // 範囲内に','がなくなるまで繰り返し
          num_end = line.indexOf(',', num_start);       // num_strから','がないか確認。あれば列番号をnum_endに記憶
          // Serial.print('[' + String(num_end) + ']'); // 正しく認識できているか確認用
          String part = line.substring(num_start, num_end); // ','で区切られた要素を行から切り出す
          // Serial.print('(' + part + ')');            // 正しく認識できているか確認用
          if (column == 0) {          // 行の最初の要素
            rpm1[row_inheader - 1] = part.toFloat(); // 配列の"ヘッダーを除いた行数"番目にデータをfloat型に変換して代入
          } 
          else if (column == 1) {     // 行の2番目の要素
            inj1[row_inheader - 1] = part.toFloat(); // 配列の"ヘッダーを除いた行数"番目にデータをfloat型に変換して代入
          } 
          else if (column == 2) {     // 行の3番目の要素
            ign1[row_inheader - 1] = part.toInt();   // 配列の"ヘッダーを除いた行数"番目にデータをint型に変換して代入
          }
          num_start = num_end + 1;    // ','分で1を足して、次の要素を区切る','を探し始めるスタート地点を代入
          column++;                   // 要素数に1を加算
        }
      }
      line = "\0";    //変数を初期化
      row_inheader++; //行数に1を加算
    }
    row = row_inheader - 1; //ヘッダーを除いた行数をrowに代入
  }
  file.close();                    //ファイルを閉じる
}


// 点火MAPファイルの表示
void printData() {
  for (uint8_t i = 0; i < row; i++) //整理したデータをタブで区切って表示
  {
    Serial.println(String(rpm1[i]) + '\t' + String(inj1[i]) + '\t' + String(ign1[i]));
  }
}

// ステータス出力
void mainloop(void *pvParameters) {
  for(;;){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    if (Launch) {                               // 走行開始ONの場合
      fastestdigitalWrite(DISRESET_OUT, LOW);     // リセット状態出力をONにする(リセット忘れ防止のため)
      if (starttime == 0) {                       // 走行時間が0の場合
        starttime = millis();                     // 走行開始時間を現在の時間にする
      }
      worktime = (millis() - starttime) * 0.001;  // 走行時間を秒に変換する
    }
    else {
      fastestdigitalWrite(DISRESET_OUT, HIGH);    // リセット状態出力をOFFにする
      worktime = 0;                               // 走行時間を0にする
      starttime = 0;                              // 走行開始時間を0にする
      distancemm = 0;                             // 走行距離を0にする
      distance = 0;                               // 走行距離を0にする
      gasml = 0.0;                                // 積算燃料消費量を0にする
    }

    INJ_timems = INJ_time * 0.1;          // 燃料噴射時間をmsecに変換
    dispergas = distance  / gasml;        // 燃費(m/ml = km/l)を計算

    // シリアル送信
    Serialsend();

    if (micros() - tachoBefore >= 1200 * 1000 ) {  // 50rpm以下(前回のカムパルスONから1.2sec以上経過)の場合
      tachoRpm = 0;
      usecperdig = 1.0;
      INJ_time = 0.0;
      IGN_CA = 0;
    }

    if (micros() - speedBefore > 3600 * perimeter){speed = 0;}  // 速度1km/h以下の時は速度を0にする 

    vTaskDelayUntil(&xLastWakeTime, 500);                  // 500ms停止
  }
}

void setup() {
  Serial.begin(115200);  // シリアル通信を開始
  Serial1.begin(115200);  // シリアル通信を開始

  pinMode(WH_IN, INPUT_PULLUP);
  pinMode(G_IN, INPUT_PULLUP);
  pinMode(STR_IN, INPUT_PULLUP);
  pinMode(ENGOFF_IN, INPUT_PULLUP);
  pinMode(NE_A_IN, INPUT);
  pinMode(NE_B_IN, INPUT);
  pinMode(NE_Z_IN, INPUT);
  pinMode(INJ_OUT, OUTPUT);
  pinMode(IGN_OUT, OUTPUT);
  pinMode(STR_OUT, OUTPUT);
  pinMode(DISRESET_OUT, OUTPUT);

  // 出力オフ
  digitalWrite(INJ_OUT, HIGH);
  digitalWrite(IGN_OUT, HIGH);
  digitalWrite(STR_OUT, HIGH);
  digitalWrite(DISRESET_OUT, HIGH);

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
  digitalWrite(DISRESET_OUT, LOW);    // DISRESET_OUT ON
  delay(100);
  digitalWrite(DISRESET_OUT, HIGH);  // DISRESET_OUT OFF

#ifdef rmc_ra4m1_20
  if ( SD.begin( 2e6, CS1 ) ) {       // microSDを認識させる。
                                      //通信速度Hz(e6=10の6乗), RMC-RA4M1本体のmicroSDはCS1(pins_arduino.hで定義) Arduino UNO R4は「CS」にする
    delay(100);
    digitalWrite(DISRESET_OUT, HIGH);  // DISRESET_OUT OFF

    Serial.print(F("Initializing SD card..."));

    // CSVファイルを読み込んで配列に代入する
    parseCSV();
  
    // データを表示して確認する
    //printData();

  }
  else {
    delay(100);
    digitalWrite(DISRESET_OUT, HIGH);  // DISRESET_OUT OFF
    Serial.println(F("Don't use SD card"));
  }
#endif  

  // MA735磁気エンコーダSPIの設定
  if (MA735SPI) {
    SPI.begin();
    pinMode(MA735_CS, OUTPUT);
    digitalWrite(MA735_CS, HIGH);

    Ne_deg = readMA735SPI();  // クランク角(deg)を読み取る
  }

  if (AFR) {  // A/Fセンサが接続されていれば
    _Stoichi = 1 / Stoichi;                     // 理想空燃比の逆数を計算
    R_PFS->PORT[1].PIN[1].PmnPFS_b.PMR = 0;     // A4ポート(P101, AN021)を入力に設定
    R_PFS->PORT[1].PIN[1].PmnPFS_b.ASEL = 1;    // A4ポート(P101, AN021)をアナログ入力ポートに設定 
    R_PFS->PORT[1].PIN[1].PmnPFS_b.PMR = 1;     // 0:汎用入出力 1:周辺機能用
    R_ADC0->ADANSA_b[1].ANSA5 = 1;              // A4ポートでA/D変換する

    R_ADC0->ADCSR_b.ADCS = 2;  // 連続スキャンモードに設定
    R_ADC0->ADCSR_b.ADST = 1;  // A/D変換開始
  } 


  if (Encoder) {             // 磁気エンコーダパルスが有効の場合
    attachInterrupt(digitalPinToInterrupt(NE_A_IN), ReadNe, RISING);   // 外部割り込み（NE_A_IN）
  }
  
  attachInterrupt(digitalPinToInterrupt(WH_IN), WH_PULSE, FALLING);   // 外部割り込み（WH_IN）


  AGTimer.init(Routine_Cycle, Routine);  // 24use周期で燃料噴射・点火制御を実行
  AGTimer.start();

  // ステータス出力のタスク
  xTaskCreate(
    mainloop,           // タスク関数
    "mainloop",         // タスクの名前
    128,                // タスクのスタックサイズ
    NULL,               // タスク関数に渡す引数
    2,                  // タスクの優先度
    &UpdatemainloopHandle // タスクハンドル
  );
  // タスク実行
  vTaskStartScheduler();
}

void loop() {
 delay(1);
}