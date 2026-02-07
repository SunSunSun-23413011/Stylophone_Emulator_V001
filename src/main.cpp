//***************************************************************
//  Stylophone Emulator 0.0.1 メインプログラム 
//                                          2026/02/06 T. Kawabata
//
//  2026/02/05  Sample. 01 ：書き込み確認,スピーカー動作確認
//  2026/02/05  Sample. 02 ：ピン確認
//  2026/02/05  Sample. 03 ：ADC,シリアル追加
//  2026/02/06  Sample. 04 ：ADCスイッチ追加
//  2026/02/06  Sample. 05 ：ADCスイッチ鍵盤化
//
//***************************************************************
#include <Arduino.h>

//-------------------------------------------------------------------------
//  ポート定義
//-------------------------------------------------------------------------
#define SW_ADC_PIN 0   // GPIO0 = ADC1_CH0
#define BUZZER_PIN 7
#define DOS5_PIN   21
#define RE5_PIN    22
#define MI5_PIN    23
static const uint8_t adcPins[7] = {0, 1, 2, 3, 4, 5, 6};

//-------------------------------------------------------------------------
//  マクロ定義
//-------------------------------------------------------------------------
int melody[] = {175,185,195,208,220,233,247,262,277,294,311,330,349,370,392,415,440,466,494,523,554,587,622,659}; // F3?E5
static const int MV_NONE = 3340;  // 期待値(mV)
static const int MV_SW2  = 320;   // 期待値(mV)
static const int MV_SW1  = 1280;  // 期待値(mV)
static const int MV_SW0  = 2368;  // 期待値(mV)
static const int TOL_MV = 120;       // 許容幅(mV) (抵抗誤差/ノイズ分。必要に応じて調整)
int chMax = 7;
static const char* const chSw[7][3] = {
  { "RE4" , "RES4" , "MI4" },
  { "FA4" , "FAS4" , "SO4" },
  { "SHI4" , "DO5" , "DOS5" },
  { "SOS4" , "RA4" , "RAS4" },
  { "FA3" , "FAS3" , "SO3" },
  { "SOS3" , "RA3" , "RAS3"},
  { "SHI3" , "DO4" , "DOS4"}
};
static const int SW_STABLE_COUNT = 2;   // 2回連続で同じ判定なら確定(100ms周期なら200ms)
static const uint8_t adcNoteIdx[7][3] = {
  {  9, 10, 11 }, // ch0: RE4, RES4, MI4
  { 12, 13, 14 }, // ch1: FA4, FAS4, SO4
  { 18, 19, 20 }, // ch2: SHI4, DO5, DOS5
  { 15, 16, 17 }, // ch3: SOS4, RA4, RAS4
  {  0,  1,  2 }, // ch4: FA3, FAS3, SO3
  {  3,  4,  5 }, // ch5: SOS3, RA3, RAS3
  {  6,  7,  8 }  // ch6: SHI3, DO4, DOS4
};


//---------------------------------------------------------------
//  グローバル変数定義
//---------------------------------------------------------------
const uint32_t DEBOUNCE_MS = 10;
int currentFreq = -1;
// デジタル3ボタン用デバウンス
uint8_t lastRaw = 0xFF;
uint8_t stableRaw = 0xFF;
uint32_t lastChangeMs = 0;
// タイマ/セマフォ
hw_timer_t *timer0 = nullptr;
volatile SemaphoreHandle_t sampleSem;
// ADC鍵盤の安定判定
static int swStable[7]     = {3, 3, 3, 3, 3, 3, 3};   // 確定状態(0..2, 3, -1)
static int swCandidate[7]  = {3, 3, 3, 3, 3, 3, 3};   // 候補
static uint8_t swCnt[7]    = {0, 0, 0, 0, 0, 0, 0};   // 連続一致回数
// ADC鍵盤で鳴らす周波数(デジタル未押下時に使用)
volatile int adcFreq = 0;
volatile int adcActiveCh = -1; // 押下中のch(最後に確定した押下)
volatile int adcActiveSw = -1; // 押下中のsw(0..2)
//---------------------------------------------------------------
//  関数プロトタイプ宣言
//---------------------------------------------------------------
int freqFromButtons( uint8_t raw );
void ARDUINO_ISR_ATTR onTimer( void );
void updateBuzzerNonBlocking( void );
int classifySwitchMv(int mv);
void printSwitchPressedOnce( int ch, int sw );

//---------------------------------------------------------------
//  ESP32-C6初期化
//---------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // LEDC
  ledcAttach(BUZZER_PIN, 2000, 8);

  pinMode(DOS5_PIN, INPUT_PULLUP);
  pinMode(RE5_PIN,  INPUT_PULLUP);
  pinMode(MI5_PIN,  INPUT_PULLUP);

  // ADC
  analogReadResolution(12);            // 0..4095 :contentReference[oaicite:5]{index=5}
  analogSetAttenuation(ADC_11db);      // 入力レンジ拡大(目安) :contentReference[oaicite:6]{index=6}

  // Timer(例: 100ms周期でADC表示)
  sampleSem = xSemaphoreCreateBinary();
  timer0 = timerBegin(1000000);                 // 1MHz tick :contentReference[oaicite:7]{index=7}
  timerAttachInterrupt(timer0, &onTimer);       // ISR attach :contentReference[oaicite:8]{index=8}
  timerAlarm(timer0, 20000, true, 0);          // 100000us=100ms, autoreload :contentReference[oaicite:9]{index=9}
}

//---------------------------------------------------------------
//  ループ関数
//---------------------------------------------------------------
void loop() {
  // 音は常に更新(軽い処理)
  updateBuzzerNonBlocking();

  // タイマで起床したらADCを読んで表示
  if (xSemaphoreTake(sampleSem, 0) == pdTRUE) {
  for (int ch = 0; ch < chMax; ch++) {
    int mv = (int)analogReadMilliVolts(adcPins[ch]);
    int swRaw = classifySwitchMv(mv); // 0..2, 3(NONE), -1(UNKNOWN)
    if (swRaw == swCandidate[ch]) {// 候補の連続一致カウント
      if (swCnt[ch] < 255) swCnt[ch]++;
    } else {
      swCandidate[ch] = swRaw;
      swCnt[ch] = 1;
    }

    // 安定回数に達したら確定状態を更新
    if (swCnt[ch] >= SW_STABLE_COUNT) {
      int prev = swStable[ch];
      int now  = swCandidate[ch];

      if (now != prev) {
        swStable[ch] = now;

        // 出力ルール:
        // - NONE(3)になった(離した)の出力はしない
        // - 0..2への変化は出力(押し直し/スライド両対応)
        // - UNKNOWN(-1)は"WTF"だけ(1回だけ)
        if (now == 3) {
          if (adcActiveCh == ch) {
            adcActiveCh = -1;
            adcActiveSw = -1;
            adcFreq = 0;
          }
        } else if (now == -1) {
          Serial.println("WTF");
        } else {
          printSwitchPressedOnce(ch, now); // CHと音名を表示
          adcActiveCh = ch;
          adcActiveSw = now;
          adcFreq = melody[ adcNoteIdx[ch][now] ];
        }
      }
    }
  }
}
}

//---------------------------------------------------------------
//  デジタル3ボタン監視(返値=周波数)
//---------------------------------------------------------------
int freqFromButtons( uint8_t raw ) {
  // INPUT_PULLUPなので押下=0
  if ((raw & (1 << 0)) == 0) return melody[21]; // Do#5
  if ((raw & (1 << 1)) == 0) return melody[22]; // Re5
  if ((raw & (1 << 2)) == 0) return melody[23]; // Mi5
  return 0; // none
}

//---------------------------------------------------------------
//  タイマ割り込み
//---------------------------------------------------------------
void ARDUINO_ISR_ATTR onTimer( void ) {
  xSemaphoreGiveFromISR(sampleSem, nullptr);
}

//---------------------------------------------------------------
//  音階更新
//---------------------------------------------------------------
void updateBuzzerNonBlocking( void ) {
  uint8_t raw =
    ((digitalRead(DOS5_PIN) ? 1 : 0) << 0) |
    ((digitalRead(RE5_PIN)  ? 1 : 0) << 1) |
    ((digitalRead(MI5_PIN)  ? 1 : 0) << 2);

  uint32_t now = millis();
  if (raw != lastRaw) {
    lastRaw = raw;
    lastChangeMs = now;
  }
  if (now - lastChangeMs >= DEBOUNCE_MS) {
    stableRaw = raw;
  }

  int targetFreq = freqFromButtons(stableRaw);

  // 変化時だけ更新(途切れにくい)
  if (targetFreq == 0) {
    targetFreq = adcFreq;
  }
  if (targetFreq != currentFreq) {
    currentFreq = targetFreq;
    if (currentFreq == 0) {
      ledcWriteTone(BUZZER_PIN, 0);
      ledcWrite(BUZZER_PIN, 0);
    } else {
      ledcWriteTone(BUZZER_PIN, currentFreq);
      ledcWrite(BUZZER_PIN, 128); // 8bit duty 0..255
    }
  }
}

//---------------------------------------------------------------
//  ADC入力判定
//---------------------------------------------------------------
int classifySwitchMv( int mv ) {
  auto near = [&](int target) -> bool {
    return (mv >= target - TOL_MV) && (mv <= target + TOL_MV);
  };

  if (near(MV_NONE)) return 3;
  if (near(MV_SW0))  return 0;
  if (near(MV_SW1))  return 1;
  if (near(MV_SW2))  return 2;
  return -1;
}

//---------------------------------------------------------------
//  押した瞬間だけ表示
//---------------------------------------------------------------
void printSwitchPressedOnce(int ch, int sw) {
  if (sw == 3) return;
  Serial.print("CH");
  Serial.print(ch);
  Serial.print(' ');

  if (ch < 0 || ch >= chMax) {
    Serial.println("WTF");
    return;
  }

  if (sw >= 0 && sw <= 2) {
    Serial.println(chSw[ch][sw]);
  } else {
    Serial.println("N/A");
  }
}