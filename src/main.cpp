// http://akiracing.com/2018/01/12/arduino_tachometer/
// ECU Shield は入出力がHIGH/LOWが反転しているので注意
//#define CSV_PARSER_DONT_IMPORT_SD
#include <Arduino.h>
//#include <CSV_Parser.h>  // https://github.com/michalmonday/CSV-Parser-for-Arduino
#include <SPI.h>
#include "SD.h"
#include "fastestDigitalRW.hpp"
#include "AGTimerR4.h"
#include <Wire.h>
#include <U8x8lib.h>

//U8X8_SH1107_64X128_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);  // 1.3"
U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);  // 0.91"

uint8_t IGN_Standard = 0;  // 点火時期基準[CA] 全体のMAPの基準になる点火時期。0でTDC,+XXで進角,-XXで遅角

volatile unsigned long tachoBefore = 0;  // クランクセンサーの前回の反応時の時間
volatile unsigned long tachoAfter = 0;  // クランクセンサーの今回の反応時の時間
volatile unsigned long tachoWidth = 0;  // クランク一回転の時間　tachoAfter - tachoBefore
volatile unsigned long tachoWidth_b = 0;  // 前回のクランク一回転の時間
volatile unsigned long timeNow_INJ_ON = 0;  // 噴射開始時の時間
volatile unsigned long timeNow_INJ_OFF = 0; // 噴射終了時の時間
volatile unsigned long timeNow_IGN_ON = 0;  // 点火開始時の時間
volatile unsigned long timeNow_IGN_OFF = 0; // 噴射終了時の時間
volatile uint16_t tachoRpm = 0;  // エンジンの回転数[rpm]
volatile float INJ_time = 0;  // インジェクタ噴射時間[ms]
volatile int8_t  IGN_CA = 0;  // 点火時期[CA]
volatile uint8_t INJ_Status = 1;  // 噴射ステータス(0: 無効 1: OFF 2:ON)
volatile uint8_t IGN_Status = 1;  // 点火ステータス(0: 無効 1: OFF 2:ON)
volatile bool INJ_His = false;    // 1サイクル中の噴射履歴
volatile bool IGN_His = false;    // 1サイクル中の点火履歴
bool OLED = false;                // OLED有効/無効

//uint8_t NE_IN = 2;  // クランク角センサ入力・外部割込み
uint8_t G_IN = 3;     // カム角センサ入力・外部割込み
uint8_t STR_IN = 5;   // スタータボタン入力
uint8_t IN_6 = 6;     // 拡張入力
uint8_t LOG_IN = 7;   // ログのON/OFF
uint8_t SD_IN = 9;    // microSD挿入チェック
uint8_t INJ_OUT = A0; // インジェクタ出力
uint8_t IGN_OUT = A1; // イグニッション出力
uint8_t STR_OUT = A2; // スタータ出力
uint8_t OUT_A3 = A3;  // スタータ出力
unsigned long usecperdig = 0;  // クランク1°当たりの時間(usec)
int8_t  TDC_P = -50;    //カム角センサと上死点の位相差を補正


const uint8_t chipSelect = 10;  // 10ピンをSSとする
File logFile;
char fileName[16];       // ファイル名
uint8_t fileNum = 0;         // ファイル連番
String MAPFILE = "RPM.CSV";  // 点火MAPファイル名
bool SDMAP = false;
float rpm1[20]; //要素記憶用の配列を作成
float inj1[20];
uint8_t ign1[20];
uint8_t row;

// 点火MAP
void INJ_IGN() {
  if (tachoRpm < 400) {
    INJ_time = 4.0;
    IGN_CA = 0;
  }
  else if (tachoRpm < 800) {
    INJ_time = 4.0;
    IGN_CA = 0;
  }
  else if (tachoRpm < 1200) {
    INJ_time = 4.0;
    IGN_CA = 0;
  }
  else if (tachoRpm < 1600) {
    INJ_time = 4.0;
    IGN_CA = 0;
  }
  else if (tachoRpm < 2000) {
    INJ_time = 4.0;
    IGN_CA = 0;
  }
  else if (tachoRpm < 2400) {
    INJ_time = 4.5;
    IGN_CA = 20;
  }
  else if (tachoRpm < 2800) {
    INJ_time = 4.0;
    IGN_CA = 40;
  }
  else if (tachoRpm < 3200) {
    INJ_time = 4.0;
    IGN_CA = 55;
  }
  else if (tachoRpm < 3600) {
    INJ_time = 4.0;
    IGN_CA = 65;
  }
  else if (tachoRpm < 4000) {
    INJ_time = 3.0;
    IGN_CA = 65;
  }
  else if (tachoRpm < 4400) {
    INJ_time = 3.0;
    IGN_CA = 70;
  }
  else if (tachoRpm < 4800) {
    INJ_time = 2.7;
    IGN_CA = 75;
  }
  else if (tachoRpm < 5200) {
    INJ_time = 2.5;
    IGN_CA = 80;
  }
  else if (tachoRpm < 5600) {
    INJ_time = 2.5;
    IGN_CA = 80;
  }
  else if (tachoRpm < 6000) {
    INJ_time = 2.5;
    IGN_CA = 80;
  }
  else {
    INJ_time = 0;
    IGN_CA = 0;
  }
  if (digitalRead(STR_IN) == LOW){  // スタータボタンを押したとき
    INJ_time = 10.0;
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
  if (tachoRpm >= rpm1[row]){       // MAP以上の回転数(オーバーレブ)の場合
    INJ_time = 0.0;
    IGN_CA = 0;
  }
  if (digitalRead(STR_IN) == LOW){  // スタータボタンを押したとき
    INJ_time = 10.0;
    IGN_CA = 0;
  }
}


void SendStatus() {
  if (Serial){  // USBシリアルが有効なら
    Serial.print(tachoRpm);
    Serial.print("\t");
    Serial.print(INJ_time,1);
    Serial.print("\t");
    Serial.println(IGN_CA);
  }
  if (Serial1){  // 外部シリアルが有効なら
  Serial1.println(tachoRpm);
  }

  if(OLED) {  // OLEDが接続されていれば
    u8x8.setCursor(4, 0);
    u8x8.print(tachoRpm);
    u8x8.print("  ");
    u8x8.setCursor(4, 1);
    u8x8.print(INJ_time,1);
    u8x8.setCursor(4, 2);
    u8x8.print(IGN_CA);
    u8x8.print(" ");
  }

  /*
  char Info_b[110];
  char Rpm_b[8];
  char INJ_b[5];

  dtostrf(tachoRpm, 6, 2, Rpm_b);
  dtostrf(INJ_time, 3, 2, INJ_b);
  sprintf_P(Info_b, PSTR("%s[rpm]\tBefore: %lu[us]\tAfter: %lu[us]\tWidth: %lu[us]\t1dig: %lu[us]\tINJ: %s[ms]\tIGN: %d[CA]"), Rpm_b, tachoBefore, tachoAfter, tachoWidth, usecperdig, INJ_b, IGN_CA);
  Serial.print(Info_b);
  SoftSerial.println(Rpm_b);  // UARTで回転数を出力
  sprintf_P(Info_b, PSTR("\tINJON: %lu[us]\tINJOFF: %lu[us]\tIGNON: %lu[us]\tIGNOFF: %lu[us]"), timeNow_INJ_ON, timeNow_INJ_OFF, timeNow_IGN_ON, timeNow_IGN_OFF);
  Serial.println(Info_b);
  
    if (digitalRead(LOG_IN) == LOW){  // 7ピンがONの場合
    if(!SD.exists(fileName)) {  // ログファイルが無かったらヘッダを書き込む
      logFile = SD.open(fileName, FILE_WRITE);
      if (logFile){
        logFile.println(F("rpm,Before[us],After[us],Width[us],INJ[ms],IGN[CA],IGN ON"));
      }
    }

    logFile = SD.open(fileName, FILE_WRITE);
    if (logFile){
      logFile.print(Rpm_b);
      logFile.print(F(","));
      logFile.print(tachoBefore);
      logFile.print(F(","));
      logFile.print(tachoAfter);
      logFile.print(F(","));
      logFile.print(tachoWidth);
      logFile.print(F(","));
      logFile.print(INJ_b);
      logFile.print(F(","));
      logFile.println(IGN_CA);
    }
    logFile.close();
  }
  */
}

// 回転数判定・点火制御
void tachometer() {
  char Info_b[110];
  char Rpm_b[8];
  char INJ_b[5];

  tachoAfter = micros();  // 現在の時刻を記録
  tachoWidth = tachoAfter - tachoBefore;  // 前回と今回の時間の差(カムシャフト1回転当たりの時間)を計算
  tachoRpm = (60000000.0 / tachoWidth) * 2;     //クランクの回転数[rpm]を計算
  usecperdig = tachoWidth / 360 / 2;  // クランク1°当たりの時間(usec)
  
  if (SDMAP) {     // SD内の点火MAPが使用できる場合
    INJ_IGN_SD();  // SDから読んだ点火MAP
  }
  else {
    INJ_IGN();     // 本コード内の点火MAP
  }

  //if (INJ_time > 0) {
  if (tachoRpm <= 6000) {
    INJ_Status = 1;  // 噴射OFF
    IGN_Status = 1;  // 点火OFF
  }
  else {
    INJ_Status = 0;  // 噴射無効
    IGN_Status = 0;  // 点火無効
    timeNow_INJ_ON = NULL;
    timeNow_INJ_OFF = NULL;
  }
  //INJ_Status = 1;              // 噴射ステータスOFF
  //IGN_Status = 1;              // 点火ステータスOFF
  INJ_His = false;               // 噴射履歴をリセット
  IGN_His = false;               // 点火履歴をリセット
  
  tachoBefore = tachoAfter;  // 今回の値を前回の値に代入する
  tachoWidth_b = tachoWidth;
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
  for (int i = 0; i < row; i++) //整理したデータをタブで区切って表示
  {
    Serial.println(String(rpm1[i]) + '\t' + String(inj1[i]) + '\t' + String(ign1[i]));
  }
}


void setup() {
  Serial.begin(115200);  // シリアル通信を開始
  Serial1.begin(115200);  // シリアル通信を開始

  Wire.begin();

  //pinMode(NE_IN, INPUT_PULLUP);
  pinMode(G_IN, INPUT_PULLUP);
  pinMode(STR_IN, INPUT_PULLUP);
  pinMode(LOG_IN, INPUT_PULLUP);
  pinMode(SD_IN, INPUT_PULLUP);
  pinMode(INJ_OUT, OUTPUT);
  pinMode(IGN_OUT, OUTPUT);
  pinMode(STR_OUT, OUTPUT);
  pinMode(OUT_A3, OUTPUT);

  digitalWrite(INJ_OUT, HIGH);
  digitalWrite(IGN_OUT, HIGH);
  digitalWrite(STR_OUT, HIGH);
  digitalWrite(OUT_A3, HIGH);

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
  digitalWrite(OUT_A3, LOW);    // OUT_A3 ON

  if (digitalRead(SD_IN) == LOW){  // microSDが挿入されているとき
    delay(100);
    digitalWrite(OUT_A3, HIGH);  // OUT_A3 OFF

    Serial.print(F("Initializing SD card..."));
    pinMode(chipSelect, OUTPUT);
   
    if (!SD.begin(chipSelect)) {
      Serial.println(F("initialization failed!"));
      //while (1);
    }
    Serial.println(F("initialization done."));

    if (digitalRead(LOG_IN) == LOW){  // 7ピンがONの場合
      Serial.print("LOG:ON\tfilename:");
      while(1){      
        sprintf_P(fileName, PSTR("LOG%04d.CSV"), fileNum);

        if(!SD.exists(fileName)) {
          Serial.println(fileName);
          break;
        }
        fileNum++;
      }
    }
    else{  // 7ピンがOFFの場合
      Serial.println("LOG:OFF");
    }
    
    // CSVファイルを読み込んで配列に代入する
    parseCSV();
  
    // データを表示して確認する
    //printData();

  }
  else {
    delay(100);
    digitalWrite(OUT_A3, HIGH);  // OUT_A3 OFF
    Serial.print(F("Don't use SD card"));
  }

  // OLEDの接続確認
  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() == 0) {
    OLED = true;
  }
  Wire.end();

  if(OLED) {  // OLEDが接続されていれば
    u8x8.begin();
    u8x8.setPowerSave(0);
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    //u8x8.setFont(u8x8_font_profont29_2x3_r);
    u8x8.setInverseFont(0);
    u8x8.drawString(0,0,"RPM:");
    u8x8.drawString(0,1,"INJ:");
    u8x8.drawString(0,2,"IGN:");
  }

  
  //attachInterrupt(digitalPinToInterrupt(NE_IN), NE_PULSE,   FALLING); // 外部割り込み（NE_IN）
  attachInterrupt(digitalPinToInterrupt(G_IN),  tachometer, FALLING); // 外部割り込み（G_IN）

  AGTimer.init(1000000, SendStatus); //  1sec周期でステータスを表示
  AGTimer.start();

}


void loop() {
  // スタータボタンを押したとき
  if (digitalRead(STR_IN) == LOW){
    //digitalWrite(STR_OUT, LOW);  // スタータON
    fastestDigitalWrite(STR_OUT, LOW);  // スタータON
    //Serial.println("STR_ON");
  }
  else {
    //digitalWrite(STR_OUT, HIGH);  // スタータOFF
    fastestDigitalWrite(STR_OUT, HIGH);  // スタータOFF
  }

  // 噴射OFFステータス時
  if ( INJ_Status == 1 && !INJ_His ) {
    if ( micros() - tachoAfter >= usecperdig * (TDC_P + 110) ){  // 上死点から[回転数に応じた1°当たりの時間]*60°以上経過したら
      timeNow_INJ_ON = micros();     // 噴射開始時の時刻を記録
      INJ_His = true;               // 噴射履歴あり
      //digitalWrite(INJ_OUT, LOW);   // 噴射ON
      fastestDigitalWrite(INJ_OUT, LOW);   // 噴射ON
      INJ_Status = 2;               // 噴射ONステータス
      //Serial.println("INJ_ON");
    }
  }

  // 噴射ONステータス時
  if ( INJ_Status == 2 ) {
    if ( micros() - timeNow_INJ_ON >= INJ_time * 1000 ){  // 噴射開始から噴射時間が経過したら
      timeNow_INJ_OFF = micros();    // 噴射終了時の時刻を記録
      //digitalWrite(INJ_OUT, HIGH);  // 噴射OFF
      fastestDigitalWrite(INJ_OUT, HIGH);  // 噴射OFF
      INJ_Status = 1;               // 噴射無効ステータス
      //Serial.println("INJ_OFF");
    }
  }
  
  //点火OFFステータス時
  if ( IGN_Status == 1 && !IGN_His )  {  
    if ( micros() - tachoAfter >= usecperdig * (360 + TDC_P - IGN_CA) ){  // 圧縮→膨張サイクルの上死点から進角角度分の時間が経過したら
      timeNow_IGN_ON = micros();     // 点火開始時の時刻を記録
      IGN_His = true;               // 点火履歴あり
      //digitalWrite(IGN_OUT, LOW);   // 点火ON
      fastestDigitalWrite(IGN_OUT, LOW);   // 点火ON
      IGN_Status = 2;               // 点火ONステータス
      //Serial.println("IGN_ON");
    }
  }

  // 点火ONステータス時
  if ( IGN_Status == 2 ) {
    if ( micros() - timeNow_IGN_ON >= 5000){  // 点火維持時間(5000us)が経過したら
      timeNow_IGN_OFF = micros();    // 点火終了時の時刻を記録
      //digitalWrite(IGN_OUT, HIGH);  // 点火OFF
      fastestDigitalWrite(IGN_OUT, HIGH);  // 点火OFF
      IGN_Status = 1;               // 点火無効ステータス
      //Serial.println("IGN_OFF");
    }
  }

  delayMicroseconds(24);  // 24us停止
}