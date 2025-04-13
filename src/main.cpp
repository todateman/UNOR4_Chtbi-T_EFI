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
void IRAM_ATTR G_PULSE_ISR();

//-----------------------------------------------------------------------------
// 定数・設定
//-----------------------------------------------------------------------------

#define ROUTINE_CYCLE_US     24
#define STATUS_TASK_DELAY_MS 500

#define PERIMETER_MM         1548UL   // [mm]
#define TACHO_RPM_MAX        6000
#define IGNITION_HOLD_US     5000

const uint8_t NE_A_IN      = 2;
const uint8_t NE_B_IN      = 8;
const uint8_t NE_Z_IN      = 9;
const uint8_t WH_IN        = 3;
const uint8_t G_IN         = 5;
const uint8_t STR_IN       = 6;
const uint8_t ENGOFF_IN    = 7;
const uint8_t MA735_CS     = 10;
const uint8_t INJ_OUT      = A0;
const uint8_t IGN_OUT      = A1;
const uint8_t STR_OUT      = A2;
const uint8_t DISRESET_OUT = A3;

bool EncoderEnabled   = false;
bool MA735SPIEnabled  = true;
bool AFREnabled       = false;
bool Increase_Fuel    = false;
bool SDMapEnabled     = false;
bool SerialUSBEnabled = true;
bool Serial1Enabled   = true;

SPISettings ma735Settings(12000000, MSBFIRST, SPI_MODE0);

//-----------------------------------------------------------------------------
// グローバル変数（volatile指定）
//-----------------------------------------------------------------------------
volatile unsigned long tachoBefore  = 0;
volatile unsigned long tachoAfter   = 0;
volatile unsigned long tachoWidth   = 0;
volatile uint16_t      tachoRpm     = 0;
volatile float         Ne_deg       = 0.0;
volatile int16_t       Ne_rev       = 0;

volatile unsigned long speedBefore  = 0;
volatile unsigned long speedAfter   = 0;
volatile unsigned long speedWidth   = 0;
volatile unsigned long distancemm   = 0;
volatile uint16_t      distance     = 0;
volatile unsigned long speed        = 0;

volatile uint8_t  calculatedINJ_time = 0;
volatile int16_t  calculatedIGN_CA   = 0;
volatile uint8_t  INJ_Status         = 1;
volatile uint8_t  IGN_Status         = 1;

volatile unsigned long timeNow_INJ_ON  = 0;
volatile unsigned long timeNow_INJ_OFF = 0;
volatile unsigned long timeNow_IGN_ON  = 0;
volatile unsigned long timeNow_IGN_OFF = 0;

volatile bool INJ_His = false;
volatile bool IGN_His = false;

volatile bool G_Pulse      = false;
volatile bool G_Pulse_Flag = false;
volatile bool CycleReset   = false;

bool Launch = false;
volatile float gasml       = 0.0;
volatile float INJ_timems  = 0.0;
volatile float dispergas   = 0.0;
unsigned long starttime    = 0;
volatile uint16_t worktime = 0;

volatile float usecperdig = 1.0;

//-----------------------------------------------------------------------------
// MAPテーブル（SD未使用の場合のデフォルトMAP）
//-----------------------------------------------------------------------------
struct MapEntry {
  uint16_t rpm;
  uint8_t inj_time;
  int8_t ign_ca;
};

const MapEntry defaultMap[] = {
  {400,  90,   0},
  {800,  90,   5},
  {1200, 90,   7},
  {1600, 90,   9},
  {2000, 90,  11},
  {2400, 90,  13},
  {2800, 90,  15},
  {3200, 90,  17},
  {3600, 90,  17},
  {4000, 90,  17},
  {4400, 90,  20},
  {4800, 90,  20},
  {5200, 90,  20},
  {5600, 90,  20},
  {6000, 90,  20}
};
const uint8_t defaultMapSize = sizeof(defaultMap) / sizeof(defaultMap[0]);

//-----------------------------------------------------------------------------
// 関数宣言（詳細実装は下部）
//-----------------------------------------------------------------------------
void updateEngineMap();
void cycleReset();
void Routine();
float readMA735SPI();
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
  speed = (3600UL * PERIMETER_MM) / (speedWidth ? speedWidth : 1);
  speedBefore = now;
  distancemm += PERIMETER_MM;
  distance = distancemm / 1000;
}

// ReadNe_ISR：クランク角更新割込み（NE_B_INによる処理）
void IRAM_ATTR ReadNe_ISR() {
  if (!fastestdigitalRead(NE_B_IN))
    Ne_deg += 1.0;
  else
    Ne_deg -= 1.0;

  if (fastestdigitalRead(NE_Z_IN)) {
    unsigned long now = micros();
    tachoWidth = now - tachoBefore;
    tachoBefore = now;
    tachoRpm = (uint16_t)(60000000.0 / tachoWidth);
    if (Ne_deg > 360.0 && G_Pulse_Flag) {
      Ne_deg = 0.0;
      G_Pulse_Flag = false;
      CycleReset = true;
    }
  }
}

// G_PULSE_ISR：カム角センサ割込み（G_IN）
void IRAM_ATTR G_PULSE_ISR() {
  if (!G_Pulse) {
    G_Pulse = true;
    G_Pulse_Flag = true;
  }
  // リリース状態は Routine 内で処理
}

//-----------------------------------------------------------------------------
// MA735 SPIによる角度取得
float readMA735SPI() {
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
  float angle = ((float)rd / 65535.0) * 360.0 + 360.0 * Ne_rev;
  return angle;
}

//-----------------------------------------------------------------------------
// エンジンMAP更新
//-----------------------------------------------------------------------------
void updateEngineMap() {
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
  }
}

//-----------------------------------------------------------------------------
// サイクルリセット
//-----------------------------------------------------------------------------
void cycleReset() {
  updateEngineMap();
  if (tachoRpm > TACHO_RPM_MAX) {
    INJ_Status = 0;
    IGN_Status = 0;
    timeNow_INJ_ON  = 0;
    timeNow_INJ_OFF = 0;
  } else {
    INJ_Status = 1;
    IGN_Status = 1;
  }
  INJ_His = false;
  IGN_His = false;
}

//-----------------------------------------------------------------------------
// Routine(): AGTimerより周期実行されるリアルタイム処理
//-----------------------------------------------------------------------------
void Routine() {
  if (AFREnabled) {
    updateAFR();
  }
  
  if (MA735SPIEnabled) {
    Ne_deg = readMA735SPI();
  }
  
  if (!EncoderEnabled && !MA735SPIEnabled) {
    if (usecperdig > 1e-3)
      Ne_deg += ((float)ROUTINE_CYCLE_US) / usecperdig;
  }
  
  if (fastestdigitalRead(G_IN) == LOW) {
    if (!G_Pulse) {
      G_Pulse = true;
      G_Pulse_Flag = true;
    }
  } else {
    if (G_Pulse)
      G_Pulse = false;
  }
  
  if (CycleReset) {
    cycleReset();
    CycleReset = false;
  }
  
  if (INJ_Status == 1 && !INJ_His) {
    if (Ne_deg >= calculatedIGN_CA) {
      timeNow_INJ_ON = micros();
      INJ_His = true;
      fastestdigitalWrite(INJ_OUT, LOW);
      INJ_Status = 2;
    }
  }
  
  if (INJ_Status == 2) {
    unsigned long injDuration = calculatedINJ_time * 100UL;
    if (Increase_Fuel) {
      // AFRによる補正（実装必要なら）
    }
    if (micros() - timeNow_INJ_ON >= injDuration) {
      timeNow_INJ_OFF = micros();
      fastestdigitalWrite(INJ_OUT, HIGH);
      INJ_Status = 1;
      gasml += (((timeNow_INJ_OFF - timeNow_INJ_ON) * 0.0000007) + 0.0015) / 1.5073;
    }
  }
  
  if (IGN_Status == 1 && !IGN_His) {
    if (Ne_deg >= (360 - calculatedIGN_CA)) {
      timeNow_IGN_ON = micros();
      IGN_His = true;
      fastestdigitalWrite(IGN_OUT, LOW);
      IGN_Status = 2;
    }
  }
  if (IGN_Status == 2) {
    if (micros() - timeNow_IGN_ON >= IGNITION_HOLD_US) {
      timeNow_IGN_OFF = micros();
      fastestdigitalWrite(IGN_OUT, HIGH);
      IGN_Status = 1;
    }
  }
  
  if (fastestdigitalRead(ENGOFF_IN) == LOW) {
    if (fastestdigitalRead(STR_IN) == LOW) {
      fastestdigitalWrite(STR_OUT, LOW);
      Launch = true;
      if (starttime == 0)
        starttime = millis();
    } else {
      fastestdigitalWrite(STR_OUT, HIGH);
    }
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
    }
    INJ_timems = calculatedINJ_time * 0.1;
    if (gasml > 0.0)
      dispergas = (float)distance / gasml;
    else
      dispergas = 0.0;
    
    if (SerialUSBEnabled) {
      Serial.print(tachoRpm);
      Serial.print("\t");
      Serial.print(INJ_timems, 1);
      Serial.print("\t");
      Serial.print(calculatedIGN_CA);
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
      Serial.print(Ne_deg, 1);
      Serial.println();
    }
    
    if (Serial1Enabled) {
      Serial1.print(tachoRpm);
      Serial1.print(",");
      Serial1.print(INJ_timems, 1);
      Serial1.print(",");
      Serial1.print(calculatedIGN_CA);
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
      Serial1.println();
    }
    
    if (micros() - tachoBefore >= 1200000UL) {
      tachoRpm = 0;
      usecperdig = 1.0;
      calculatedINJ_time = 0;
      calculatedIGN_CA = 0;
    }
    if (micros() - speedBefore > (3600UL * PERIMETER_MM)) {
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
  pinMode(NE_A_IN, INPUT);
  pinMode(NE_B_IN, INPUT);
  pinMode(NE_Z_IN, INPUT);
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
