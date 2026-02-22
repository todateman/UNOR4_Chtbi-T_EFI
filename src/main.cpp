#include <Arduino.h>
#include <SPI.h>
#include "SD.h"
#include "fastestdigitalRW.hpp"  :contentReference[oaicite:0]{index=0}
#include "AGTimerR4.h"
#include <Arduino_FreeRTOS.h>

// IRAM_ATTR が未定義の場合は空定義を追加（ESP32等でなければ不要）
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

//-----------------------------------------------------------------------------
// 前方宣言（setup()で使う割り込み関数の宣言）
//-----------------------------------------------------------------------------
void IRAM_ATTR WH_PULSE_ISR();
void IRAM_ATTR ReadNe_ISR();
void IRAM_ATTR NE_Z_RISE_ISR();
void IRAM_ATTR G_PULSE_ISR();

//-----------------------------------------------------------------------------
// 定数・設定
//-----------------------------------------------------------------------------

#define ROUTINE_CYCLE_US     24
#define STATUS_TASK_DELAY_MS 500

// 燃料噴射量計算定数（インジェクタキャリブレーション）
#define INJ_FLOW_ML_PER_US   0.0000007f  // インジェクタ流量（ml/µs）
#define INJ_DEAD_TIME_ML     0.0015f     // インジェクタ不感時間相当燃料量（ml/cycle）
#define INJ_FUEL_DENSITY     1.5073f     // 燃料密度補正係数

#define PERIMETER_MM         1548UL   // [mm]
#define TACHO_RPM_MAX        6000
#define IGNITION_HOLD_US     5000

const uint8_t NE_A_IN      = 2;   // クランク角エンコーダAパルス(360°で360パルス)
const uint8_t NE_B_IN      = 8;   // クランク角エンコーダBパルス(360°で360パルス)
const uint8_t NE_Z_IN      = 9;   // クランク角エンコーダBパルス(360°で1パルス)
const uint8_t WH_IN        = 3;   // 車軸パルス（タイヤ1周で1パルス）
const uint8_t G_IN         = 5;   // カム角センサ（クランク2周で1パルス）
const uint8_t STR_IN       = 6;   // スタートスイッチ（エンジン始動）
const uint8_t ENGOFF_IN    = 7;   // エンジンキルスイッチ（エンジン停止）
const uint8_t MA735_CS     = 10;  // MA735 SPIセンサのCSピン（SPI通信に使用）
const uint8_t INJ_OUT      = A0;  // 燃料噴射出力（ON:LOW, OFF:HIGH）
const uint8_t IGN_OUT      = A1;  // 点火出力（ON:LOW, OFF:HIGH）
const uint8_t STR_OUT      = A2;  // スタータ出力（ON:LOW, OFF:HIGH）
const uint8_t DISRESET_OUT = A3;  // リセットランプ出力（ON:LOW, OFF:HIGH）

bool EncoderEnabled   = true;     // クランク角エンコーダ使用フラグ
bool MA735SPIEnabled  = false;    // MA735 SPIセンサ使用フラグ
bool AFREnabled       = false;    // A/Fセンサ使用フラグ
bool Increase_Fuel    = false;    // A/F補正フラグ（未実装）
bool SDMapEnabled     = false;    // SDカードMAP使用フラグ
bool SerialUSBEnabled = true;     // USBシリアル出力フラグ
bool Serial1Enabled   = true;     // Serial1(toメーター・ロガー)出力フラグ

SPISettings ma735Settings(12000000, MSBFIRST, SPI_MODE0);

//-----------------------------------------------------------------------------
// グローバル変数（volatile指定）
//-----------------------------------------------------------------------------
volatile unsigned long tachoBefore  = 0;  // NE_Z_INの立ち上がりで更新
volatile unsigned long tachoAfter   = 0;  // NE_Z_INの立ち下がりで更新
volatile unsigned long tachoWidth   = 0;  // NE_Z_INのパルス幅
volatile uint16_t      tachoRpm     = 0;  // NE_Z_INの回転数（RPM）
volatile int16_t       Ne_deg       = 0;  // クランク角度（CA）
volatile int16_t       Ne_rev       = 0;  // クランク回転数（回転数）

volatile unsigned long speedBefore  = 0;  // WH_INの立ち上がりで更新
volatile unsigned long speedAfter   = 0;  // WH_INの立ち下がりで更新
volatile unsigned long speedWidth   = 0;  // WH_INのパルス幅
volatile unsigned long distancemm   = 0;  // WH_INの走行距離（mm）
volatile uint16_t      distance     = 0;  // WH_INの走行距離（km）
volatile unsigned long speed        = 0;  // WH_INの速度（0.1 km/h 単位）0.0～99.9km/h→内部0～999
// 注意: パルスが止まると最後の1回転周期で算出した速度が保持されるため下限値(例:1.4km/h)から0へ落ちない。
// `statusTask` 内で最終パルスからの経過時間を使った減衰再計算を行い停車時に 0.0 へ近づける。

volatile bool ENG_ON                 = false; // エンジンONフラグ（キルスイッチに連動）
volatile uint8_t  calculatedINJ_time = 0; // 燃料噴射時間（x0.1ms）
volatile int16_t  calculatedIGN_CA   = 0; // 点火タイミング進角角度（CA）
uint8_t  start_INJ_time              = 50;  // 始動時の燃料噴射時間（x0.1ms）
int16_t  start_IGN_CA                = 75;  // 始動時の点火タイミング進角角度（CA）
volatile int16_t  INJ_STR_CA         = 0; // 燃料噴射タイミング角度（CA）
volatile uint8_t  INJ_Status         = 1; // 燃料噴射状態（0:OFF, 1:ON, 2:ON_HOLD）
volatile uint8_t  IGN_Status         = 1; // 点火状態（0:OFF, 1:ON, 2:ON_HOLD）

volatile unsigned long timeNow_INJ_ON  = 0; // 燃料噴射ON時間（us）
volatile unsigned long timeNow_INJ_OFF = 0; // 燃料噴射OFF時間（us）
volatile unsigned long timeNow_IGN_ON  = 0; // 点火ON時間（us）
volatile unsigned long timeNow_IGN_OFF = 0; // 点火OFF時間（us）

volatile bool INJ_His = false;            // 燃料噴射履歴（ON/OFF）
volatile bool IGN_His = false;            // 点火履歴（ON/OFF）

volatile bool G_Pulse      = false;       // G_INのパルス状態（立ち上がり）
volatile bool G_Pulse_Flag = false;       // G_INのパルスフラグ（立ち上がり）
volatile bool CycleReset   = false;       // サイクルリセットフラグ（立ち上がり）

bool Launch = false;                      // スタートフラグ（エンジン始動）
bool startState = HIGH;                   // スタートスイッチ状態（OFF=HIGH, ON=LOW）
bool lastStartState = HIGH;               // スタートスイッチの前回状態   
bool STR_IN_state = false;                // エンジン始動状態
volatile float gasml       = 0.0;         // 燃料消費量（ml）
volatile float INJ_timems  = 0.0;         // 燃料噴射時間（ms）
volatile float dispergas   = 0.0;         // 燃費（km/L）
unsigned long starttime    = 0;           // エンジン始動時間（ms）
volatile uint16_t worktime = 0;           // エンジン稼働時間（秒）

volatile float usecperdig = 1.0;          // NE_A_INの1パルスあたりの時間（us）

volatile unsigned long injTotalUs   = 0;  // 燃料噴射累積時間（us）
volatile uint32_t      injCycleCount = 0; // 燃料噴射サイクル数

//-----------------------------------------------------------------------------
// MAPテーブル（SD未使用の場合のデフォルトMAP）
//-----------------------------------------------------------------------------
struct MapEntry {
  uint16_t rpm;
  uint8_t inj_time;
  uint16_t ign_ca;
};

const MapEntry defaultMap[] = {
  {400, 90, 115},
  {800, 90, 115},
  {1200, 90, 115},
  {1600, 90, 120},
  {2000, 90, 120},
  {2400, 85, 140},
  {2800, 85, 140},
  {3200, 75, 150},
  {3600, 72, 170},
  {4000, 68, 175},
  {4400, 65, 185},
  {4800, 63, 185},
  {5200, 57, 185},
  {5600, 57, 185},
  {6000, 57, 185}
};
const uint8_t defaultMapSize = sizeof(defaultMap) / sizeof(defaultMap[0]);

//-----------------------------------------------------------------------------
// 関数宣言（詳細実装は下部）
//-----------------------------------------------------------------------------
void updateEngineMap();
void cycleReset();
void Routine();
int16_t readMA735SPI();
void updateAFR();  // AFR関連は必要に応じて実装
void parseCSV();   // SDカード用

//-----------------------------------------------------------------------------
// スタブ実装（リンクエラー解消用）
//-----------------------------------------------------------------------------
// updateAFR()：AFRセンサ処理（未使用の場合は空実装）
void updateAFR() {
  // AFRセンサ未実装の場合は何もしない
}

// parseCSV()：SDカードからMAP読み込み（未使用の場合は空実装）
void parseCSV() {
  // SDカードMAP読み込み未実装の場合は何もしない
}

//-----------------------------------------------------------------------------
// 割り込みルーチン
//-----------------------------------------------------------------------------

// WH_PULSE_ISR：車軸パルス割込み（速度・走行距離の更新）
void IRAM_ATTR WH_PULSE_ISR() {
  unsigned long now = micros();
  speedWidth = now - speedBefore;
  // 速度計算: km/h = (3600 * 周長(mm)) / パルス間隔(µs) / 1000(mm→m) / 1000(m→km)
  // 0.1km/h分解能にするため10倍 → (36000 * PERIMETER_MM) / dt
  unsigned long dt = (speedWidth ? speedWidth : 1);
  unsigned long calc = (36000UL * PERIMETER_MM) / dt; // 0.1km/h単位
  if (calc > 999) calc = 999; // 上限 99.9km/h
  speed = calc;
  speedBefore = now;
  distancemm += PERIMETER_MM;
  distance = distancemm / 1000;
}

// ReadNe_ISR：クランク角更新割込み（NE_A_INの立ち上がりで1度カウント）
void IRAM_ATTR ReadNe_ISR() {
  if (!fastestdigitalRead(NE_B_IN))
    Ne_deg += 1;
  else
    Ne_deg -= 1;

  // 点火タイミングトリガー（クランク角ベース・度単位精度）
  if (ENG_ON && IGN_Status == 1 && !IGN_His) {
    if (Ne_deg >= (360 - calculatedIGN_CA)) {
      timeNow_IGN_ON = micros();
      IGN_His = true;
      fastestdigitalWrite(IGN_OUT, LOW);
      IGN_Status = 2;
    }
  }

  // 燃料噴射タイミングトリガー（クランク角ベース・度単位精度）
  if (ENG_ON && INJ_Status == 1 && !INJ_His) {
    if (Ne_deg >= INJ_STR_CA) {
      timeNow_INJ_ON = micros();
      INJ_His = true;
      fastestdigitalWrite(INJ_OUT, LOW);
      INJ_Status = 2;
    }
  }
}

// NE_Z_RISE_ISR：クランク角エンコーダZパルス割込み（回転数・角度基準の更新）
void IRAM_ATTR NE_Z_RISE_ISR() {
  unsigned long now = micros();
  tachoWidth = now - tachoBefore;
  tachoBefore = now;
  if (tachoWidth > 0) {
    uint16_t _tachoRpm = (uint16_t)(60000000UL / tachoWidth);
    if (_tachoRpm < 10000) {
      tachoRpm = _tachoRpm;
    }
  }
  if (Ne_deg > 360 && G_Pulse_Flag) {
    Ne_deg = 0;
    G_Pulse_Flag = false;
    CycleReset = true;
  }
}

// G_PULSE_ISR：カム角センサ割込み（G_IN CHANGE）
// CHANGE割込みでピン状態を読み取るため、読み取りタイミングとエッジ間の競合は
// カム信号の変化速度に対して十分小さく実用上問題ない
void IRAM_ATTR G_PULSE_ISR() {
  if (fastestdigitalRead(G_IN) == LOW) {  // 立ち下がり（アクティブ）
    if (!G_Pulse) {
      G_Pulse = true;
      G_Pulse_Flag = true;
    }
  } else {                                // 立ち上がり（リリース）
    G_Pulse = false;
  }
}

//-----------------------------------------------------------------------------
// MA735 SPIによる角度取得
//-----------------------------------------------------------------------------
int16_t readMA735SPI() {
  static uint16_t last_rd = 0;
  static unsigned long last_rd_time = 0;
  SPI.beginTransaction(ma735Settings);
  fastestdigitalWrite(MA735_CS, LOW);
  uint16_t rd = SPI.transfer16(0);
  fastestdigitalWrite(MA735_CS, HIGH);
  SPI.endTransaction();

  long diff = (long)rd - (long)last_rd;
  if (diff < -32767) {
    unsigned long now = micros();
    Ne_rev++;
    if (diff < 262)
      tachoRpm = (uint16_t)(60000000.0 / (now - last_rd_time));
    last_rd_time = now;
    if (G_Pulse_Flag) {
      Ne_rev = 0;
      G_Pulse_Flag = false;
      CycleReset = true;
    }
  } else if (diff > 32767) {
    Ne_rev--;
  }
  last_rd = rd;
  int16_t angle = (rd / 65535) * 360 + 360 * Ne_rev;
  return angle;
}

//-----------------------------------------------------------------------------
// エンジンMAP更新
//-----------------------------------------------------------------------------
void updateEngineMap() {
  // スタータONの場合
  if (startState == LOW) {
    calculatedINJ_time = start_INJ_time;
    calculatedIGN_CA   = start_IGN_CA;
  }
  // スタータOFFの場合
  else {
    if (SDMapEnabled) {
      // SDカードMAP読み込みの場合（parseCSV()でロードしたデータを利用）
      // ここでは未実装（必要なら実装）
    } else {
      for (uint8_t i = 0; i < defaultMapSize; i++) {
        if (tachoRpm < defaultMap[i].rpm) {
          calculatedINJ_time = defaultMap[i].inj_time;
          calculatedIGN_CA   = defaultMap[i].ign_ca;
          return;
        }
      }
      calculatedINJ_time = 0;
      calculatedIGN_CA   = 0;
      return;
    }
  }
}

//-----------------------------------------------------------------------------
// サイクルリセット
//-----------------------------------------------------------------------------
void cycleReset() {
  updateEngineMap();
  if (tachoRpm > TACHO_RPM_MAX) {
    timeNow_INJ_ON  = 0;
    timeNow_INJ_OFF = 0;
  }
  INJ_Status = 1;
  IGN_Status = 1;
  INJ_His = false;
  IGN_His = false;
  G_Pulse = false;
  G_Pulse_Flag = false;
}

//-----------------------------------------------------------------------------
// Routine(): AGTimerより周期実行されるリアルタイム処理
//-----------------------------------------------------------------------------
void Routine() {
  // スタートスイッチの状態を更新(OFF=HIGH, ON=LOW)
  startState = fastestdigitalRead(STR_IN);

  // A/Fセンサの更新
  if (AFREnabled) {
    updateAFR();
  }
  
  // MA735 SPIセンサの更新
  if (MA735SPIEnabled) {
    Ne_deg = readMA735SPI();
  }
  
  // クランク角センサを使用しない場合、回転数からクランク角を概算
  if (!EncoderEnabled && !MA735SPIEnabled) {
    if (usecperdig > 1e-3)
      Ne_deg += (int16_t)(ROUTINE_CYCLE_US / usecperdig);
  }
  
  // サイクル同期が取れない場合のタイムアウト／再リセット機構(始動不良対策)
  static unsigned long lastZMicros = 0;
  if (G_Pulse_Flag) {
    lastZMicros = micros();
  }
  // カムパルスが来ずに一定時間経過したら、自動的に cycleReset() を繰り返し呼び出す
  if (micros() - lastZMicros > 50000UL) {
    cycleReset();
    lastZMicros = micros();
  }
  
  // エンジン燃焼サイクルのリセット
  if (CycleReset) {
    cycleReset();
    CycleReset = false;
  }

  // スタートスイッチの状態がOFF->ONに変化した場合
  if (lastStartState == HIGH && startState == LOW) {
    if (fastestdigitalRead(STR_IN) == LOW) {
      cycleReset();
    }
  }
  lastStartState = startState;
  
  // 燃料噴射終了タイミング管理
  if (INJ_Status == 2) {
    unsigned long injDuration = calculatedINJ_time * 100UL;
    if (Increase_Fuel) {
      // AFRによる補正（実装必要なら）
    }
    if (micros() - timeNow_INJ_ON >= injDuration) {
      timeNow_INJ_OFF = micros();
      fastestdigitalWrite(INJ_OUT, HIGH);
      INJ_Status = 1;
      injTotalUs += (timeNow_INJ_OFF - timeNow_INJ_ON);
      injCycleCount++;
    }
  }
  
  // 点火保持終了タイミング管理
  if (IGN_Status == 2) {
    if (micros() - timeNow_IGN_ON >= IGNITION_HOLD_US) {
      timeNow_IGN_OFF = micros();
      fastestdigitalWrite(IGN_OUT, HIGH);
      IGN_Status = 1;
    }
  }
  
  // キルスイッチの状態を確認
  if (fastestdigitalRead(ENGOFF_IN) == LOW) {   // キルスイッチがONの場合（運転状態）
    if (startState == LOW) {                      // スタートボタンON場合
      STR_IN_state = true;
      fastestdigitalWrite(STR_OUT, LOW);
      Launch = true;
      ENG_ON = true;
      if (starttime == 0)
        starttime = millis();
    } else {
      STR_IN_state = false;
      fastestdigitalWrite(STR_OUT, HIGH);
    }    
  }
  else {                                      // キルスイッチがOFFの場合（停止状態）
    ENG_ON = false;
  }
}

//-----------------------------------------------------------------------------
// statusTask(): 500ms間隔でシステム状態を出力
//-----------------------------------------------------------------------------
void statusTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    if (Launch) {
      fastestdigitalWrite(DISRESET_OUT, LOW);
      worktime = (millis() - starttime) / 1000;
    } else {
      fastestdigitalWrite(DISRESET_OUT, HIGH);
      worktime = 0;
      starttime = 0;
      distancemm = 0;
      distance = 0;
      gasml = 0.0;
      injTotalUs = 0;
      injCycleCount = 0;
    }
    INJ_timems = calculatedINJ_time * 0.1;
    gasml = (injTotalUs * INJ_FLOW_ML_PER_US + injCycleCount * INJ_DEAD_TIME_ML) / INJ_FUEL_DENSITY;
    if (gasml > 0.0)
      dispergas = (float)distance / gasml;
    else
      dispergas = 0.0;
    
    // 停止減衰: 最後のパルスから時間が経つほど再計算速度を小さくする (1回転未満で停止した場合の見かけ速度低下)
    unsigned long age = micros() - speedBefore; // 最終パルスからの経過(us)
    if (age > speedWidth && speedWidth > 0) {   // 新しいパルスが来ていない区間
      unsigned long decay = (36000UL * PERIMETER_MM) / age; // 0.1km/h単位
      if (decay < speed) {
        if (decay > 999) decay = 999;
        speed = decay < 1 ? 0 : decay; // 0.0未満は 0.0 とする
      }
    }

    if (SerialUSBEnabled) {
      Serial.print(tachoRpm);
      Serial.print("\t");
      Serial.print(INJ_timems, 1);
      Serial.print("\t");
      Serial.print(calculatedIGN_CA);
      Serial.print("\t");
      Serial.print(speed / 10.0f, 1); // 0.1km/h表示
      Serial.print("\t");
      Serial.print(distance);
      Serial.print("\t");
      Serial.print(gasml, 1);
      Serial.print("\t");
      Serial.print(dispergas, 1);
      Serial.print("\t");
      Serial.print(worktime);
      Serial.print("\t");
      Serial.print(Ne_deg);
      Serial.println();
    }
    
    if (Serial1Enabled) {
      Serial1.print(tachoRpm);
      Serial1.print(",");
      Serial1.print(INJ_timems, 1);
      Serial1.print(",");
      Serial1.print(calculatedIGN_CA);
      Serial1.print(",");
      Serial1.print(speed / 10.0f, 1); // 0.1km/h表示
      Serial1.print(",");
      Serial1.print(distance);
      Serial1.print(",");
      Serial1.print(gasml, 1);
      Serial1.print(",");
      Serial1.print(dispergas, 1);
      Serial1.print(",");
      Serial1.print(worktime);
      Serial1.println();
    }
    
    if (micros() - tachoBefore >= 1200000UL) {
      tachoRpm = 0;
      usecperdig = 1.0;
      calculatedINJ_time = 0;
      calculatedIGN_CA = 0;
    }
    // 無信号ゼロ化: 減衰後さらに 8 秒相当以上経過で強制 0 (約 8s = 8,000,000us)
    if (micros() - speedBefore > 8000000UL) {
      speed = 0;
    }
    
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(STATUS_TASK_DELAY_MS));
  }
}

//-----------------------------------------------------------------------------
// setup()
//-----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  pinMode(WH_IN, INPUT_PULLUP);
  pinMode(G_IN, INPUT_PULLUP);
  pinMode(STR_IN, INPUT_PULLUP);
  pinMode(ENGOFF_IN, INPUT_PULLUP);
  pinMode(NE_A_IN, INPUT_PULLUP);
  pinMode(NE_B_IN, INPUT_PULLUP);
  pinMode(NE_Z_IN, INPUT_PULLUP);
  pinMode(INJ_OUT, OUTPUT);
  pinMode(IGN_OUT, OUTPUT);
  pinMode(STR_OUT, OUTPUT);
  pinMode(DISRESET_OUT, OUTPUT);
  
  digitalWrite(INJ_OUT, HIGH);
  digitalWrite(IGN_OUT, HIGH);
  digitalWrite(STR_OUT, HIGH);
  digitalWrite(DISRESET_OUT, HIGH);
  
  digitalWrite(DISRESET_OUT, LOW);
  delay(100);
  digitalWrite(DISRESET_OUT, HIGH);
  
#ifdef rmc_ra4m1_20
  if (SD.begin(2000000UL, CS)) {
    delay(100);
    digitalWrite(DISRESET_OUT, HIGH);
    Serial.println(F("SD card initialized."));
    parseCSV();
  } else {
    delay(100);
    digitalWrite(DISRESET_OUT, HIGH);
    Serial.println(F("SD card not used."));
  }
#endif
  
  if (MA735SPIEnabled) {
    SPI.begin();
    pinMode(MA735_CS, OUTPUT);
    digitalWrite(MA735_CS, HIGH);
    Ne_deg = readMA735SPI();
  }
  
  if (AFREnabled) {
    // AFR初期化
  }
  
  if (EncoderEnabled) {
    attachInterrupt(digitalPinToInterrupt(NE_A_IN), ReadNe_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(NE_Z_IN), NE_Z_RISE_ISR, RISING);
  }
  attachInterrupt(digitalPinToInterrupt(WH_IN), WH_PULSE_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(G_IN), G_PULSE_ISR, CHANGE);
  
  AGTimer.init(ROUTINE_CYCLE_US, Routine);
  AGTimer.start();
  
  xTaskCreate(statusTask, "StatusTask", 128, NULL, 2, NULL);
  
  vTaskStartScheduler();
}

void loop() {
  delay(1);
}