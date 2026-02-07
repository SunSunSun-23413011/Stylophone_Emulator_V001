//***************************************************************
//  Stylophone Emulator 0.0.1 メインプログラム
//                                          2026/02/07 T. Kawabata
//
//  2026/02/05  Sample. 01 ：書き込み確認,スピーカー動作確認
//  2026/02/05  Sample. 02 ：ピン確認
//  2026/02/05  Sample. 03 ：ADC,シリアル追加
//  2026/02/06  Sample. 04 ：ADCスイッチ追加
//  2026/02/06  Sample. 05 ：ADCスイッチ鍵盤化
//  2026/02/07  Sample. 06 ：ビブラート追加
//  2026/02/07  Sample. 07 ：MODE_PIN=LOWでSPIFFS MIDI再生
//
//***************************************************************
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef MIDI_PLAYBACK_SPEED_PERMILLE
#define MIDI_PLAYBACK_SPEED_PERMILLE 1000
#endif

//-------------------------------------------------------------------------
//  ポート定義
//-------------------------------------------------------------------------
#define SW_ADC_PIN 0   // GPIO0 = ADC1_CH0
#define BUZZER_PIN 7
#define VIB_PIN    10
#define MODE_PIN   11
#define SW_UP      16
#define SW_DOWN    17
#define DOS5_PIN   21
#define RE5_PIN    22
#define MI5_PIN    23
static const uint8_t adcPins[7] = {0, 1, 2, 3, 4, 5, 6};

//-------------------------------------------------------------------------
//  マクロ定義
//-------------------------------------------------------------------------
int melody[] = {175,185,195,208,220,233,247,262,277,294,311,330,349,370,392,415,440,466,494,523,554,587,622,659}; // F3~E5
static const int MV_NONE = 3340;  // 期待値(mV)
static const int MV_SW2  = 320;   // 期待値(mV)
static const int MV_SW1  = 1280;  // 期待値(mV)
static const int MV_SW0  = 2368;  // 期待値(mV)
static const int TOL_MV = 120;    // 許容幅(mV)
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
static const int SW_STABLE_COUNT = 2;   // 2回連続で同じ判定なら確定(20ms周期なら40ms)
static const uint8_t adcNoteIdx[7][3] = {
  {  9, 10, 11 }, // ch0: RE4, RES4, MI4
  { 12, 13, 14 }, // ch1: FA4, FAS4, SO4
  { 18, 19, 20 }, // ch2: SHI4, DO5, DOS5
  { 15, 16, 17 }, // ch3: SOS4, RA4, RAS4
  {  0,  1,  2 }, // ch4: FA3, FAS3, SO3
  {  3,  4,  5 }, // ch5: SOS3, RA3, RAS3
  {  6,  7,  8 }  // ch6: SHI3, DO4, DOS4
};
static const uint8_t DIGITAL_NOTE_BASE = 21; // DOS5/RE5/MI5
static const uint32_t DEFAULT_TEMPO_US_PER_QN = 500000UL;
static const size_t MAX_SONGS = 32;
static const uint8_t MIDI_DRUM_CHANNEL = 9;
static const uint8_t MIDI_CHANNEL_COUNT = 16;
static const uint16_t MIDI_SPEED_MIN_PERMILLE = 250;   // 0.25x
static const uint16_t MIDI_SPEED_MAX_PERMILLE = 4000;  // 4.00x
static const uint16_t MIDI_SPEED_STEP_PERMILLE = 100;  // 0.10x step
static const char* const MIDI_SPEED_STORE_PATH = "/midi_speed.cfg";

//---------------------------------------------------------------
//  MIDI用構造体
//---------------------------------------------------------------
enum MidiEventType : uint8_t {
  MIDI_NOTE_OFF = 0,
  MIDI_NOTE_ON  = 1
};

struct MidiSongInfo {
  char path[96];
  char name[64];
};

struct MidiEvent {
  uint32_t tick;
  uint32_t timeUs;
  uint8_t type;     // MIDI_NOTE_ON / MIDI_NOTE_OFF
  uint8_t channel;  // 0..15
  uint8_t note;     // 0..127
  uint8_t velocity; // 0..127
};

struct TempoEvent {
  uint32_t tick;
  uint32_t usPerQuarter;
};

//---------------------------------------------------------------
//  グローバル変数定義
//---------------------------------------------------------------
const uint32_t DEBOUNCE_MS = 10;
int currentFreq = -1;
static bool lastVibEnabled = false;
static bool songMode = false;

// デジタル3ボタン用デバウンス
uint8_t lastRaw = 0xFF;
uint8_t stableRaw = 0xFF;
uint8_t prevStableRaw = 0xFF;
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

// 曲モードの鍵盤イベント
static int pendingKeyPress = -1;

// Vibrato
static const uint32_t VIBRATO_PERIOD_MS = 167; // about 6Hz
static const int VIBRATO_DEPTH_PERMILLE = 25;  // +/-2.5%

// MIDI曲管理
static MidiSongInfo midiSongs[MAX_SONGS];
static size_t midiSongCount = 0;
static bool spiffsReady = false;
static int selectedSongIndex = -1;
static uint16_t midiSongSpeedPermille[MAX_SONGS] = {0};
static uint16_t activeMidiSpeedPermille = 1000;

// MIDIデータ(現在読み込んだ1曲分)
static MidiEvent* midiEvents = nullptr;
static size_t midiEventCount = 0;
static size_t midiEventCap = 0;

static TempoEvent* tempoEvents = nullptr;
static size_t tempoEventCount = 0;
static size_t tempoEventCap = 0;

static uint16_t midiDivision = 480;
static int loadedSongIndex = -1;
static int selectedMidiChannel = -1;
static uint8_t midiChannelPriority[MIDI_CHANNEL_COUNT] = {0};
static size_t midiChannelPriorityCount = 0;

// MIDI再生状態
static bool midiIsPlaying = false;
static size_t midiPlayPos = 0;
static uint32_t midiPlayLastRealUs = 0;
static uint64_t midiPlayElapsedSongUs = 0;
static uint8_t midiActiveNotes[MIDI_CHANNEL_COUNT][128] = {{0}};
static int midiCurrentNoteByChannel[MIDI_CHANNEL_COUNT] = {0};
static int currentMidiNote = -1;
static volatile int midiPlaybackFreq = 0;
static int midiFreqTable[128] = {0};

// SW_UP / SW_DOWN debouncing
static uint8_t speedLastRaw = 0x03;
static uint8_t speedStableRaw = 0x03;
static uint8_t speedPrevStableRaw = 0x03;
static uint32_t speedLastChangeMs = 0;

//---------------------------------------------------------------
//  関数プロトタイプ宣言
//---------------------------------------------------------------
int freqFromButtons(uint8_t raw);
void ARDUINO_ISR_ATTR onTimer(void);
void processAdcKeys(void);
void updateBuzzerNonBlocking(void);
void handleModeChange(bool nextSongMode);
void processSongModeLogic(void);
void enqueueKeyPress(int noteIdx);
int consumeKeyPress(void);

int classifySwitchMv(int mv);
void printSwitchPressedOnce(int ch, int sw);
int applyVibrato(int baseFreq, uint32_t nowMs);

void buildMidiFreqTable(void);
bool initSpiffsAndSongList(void);
void scanMidiSongs(void);
void printSongList(void);
uint16_t clampMidiSpeedPermille(uint32_t value);
void initSongSpeedTable(void);
void loadSongSpeedTableFromSpiffs(void);
void saveSongSpeedTableToSpiffs(void);
void setActiveSongSpeedByIndex(int songIndex);
void changeSelectedSongSpeed(int deltaSteps);
void updateSongSpeedButtons(uint32_t nowMs);
bool pathHasMidExt(const char* path);
const char* basenameFromPath(const char* path);
void normalizeSpiffsPath(char* dst, size_t dstSize, const char* folder, const char* rawName);

void resetMidiBuffers(void);
bool reserveMidiEvents(size_t needCount);
bool reserveTempoEvents(size_t needCount);
bool appendMidiEvent(uint32_t tick, uint8_t type, uint8_t channel, uint8_t note, uint8_t velocity);
bool appendTempoEvent(uint32_t tick, uint32_t usPerQuarter);
bool readFileIntoBuffer(const char* path, uint8_t** outBuf, size_t* outLen);
bool readU16BE(const uint8_t* data, size_t len, size_t* pos, uint16_t* out);
bool readU32BE(const uint8_t* data, size_t len, size_t* pos, uint32_t* out);
bool readVLQ(const uint8_t* data, size_t end, size_t* pos, uint32_t* out);
bool parseMidiBuffer(const uint8_t* data, size_t len);
void computeEventTimesFromTempo(void);
int cmpMidiEvent(const void* a, const void* b);
int cmpTempoEvent(const void* a, const void* b);
bool loadMidiSongByIndex(size_t songIndex);

void resetMidiChannelState(void);
void startMidiPlayback(void);
void stopMidiPlayback(void);
void updateMidiPlayback(void);
void handleMidiEvent(const MidiEvent* ev);

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
  pinMode(SW_UP,    INPUT_PULLUP);
  pinMode(SW_DOWN,  INPUT_PULLUP);
  pinMode(VIB_PIN,  INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);

  // ADC
  analogReadResolution(12);       // 0..4095
  analogSetAttenuation(ADC_11db); // 入力レンジ拡大(目安)

  // Timer(20ms周期でADC鍵盤判定)
  sampleSem = xSemaphoreCreateBinary();
  timer0 = timerBegin(1000000);           // 1MHz tick
  timerAttachInterrupt(timer0, &onTimer); // ISR attach
  timerAlarm(timer0, 20000, true, 0);     // 20000us=20ms, autoreload

  activeMidiSpeedPermille = clampMidiSpeedPermille((uint32_t)MIDI_PLAYBACK_SPEED_PERMILLE);
  buildMidiFreqTable();
  spiffsReady = initSpiffsAndSongList();

  songMode = (digitalRead(MODE_PIN) == LOW);
  Serial.print("Boot Mode: ");
  Serial.println(songMode ? "SONG" : "PLAY");
  Serial.print("MIDI Speed Default(per mille): ");
  Serial.println((int)activeMidiSpeedPermille);
}

//---------------------------------------------------------------
//  ループ関数
//---------------------------------------------------------------
void loop() {
  bool nextSongMode = (digitalRead(MODE_PIN) == LOW);
  if (nextSongMode != songMode) {
    handleModeChange(nextSongMode);
  }

  if (xSemaphoreTake(sampleSem, 0) == pdTRUE) {
    processAdcKeys();
  }

  if (songMode) {
    processSongModeLogic();
  } else {
    midiPlaybackFreq = 0;
  }

  updateBuzzerNonBlocking();
}

//---------------------------------------------------------------
//  モード切替
//---------------------------------------------------------------
void handleModeChange(bool nextSongMode) {
  songMode = nextSongMode;
  pendingKeyPress = -1;

  if (songMode) {
    Serial.println("[MODE] SONG");
    adcFreq = 0;
    setActiveSongSpeedByIndex(selectedSongIndex);
  } else {
    Serial.println("[MODE] PLAY");
    stopMidiPlayback();
    selectedSongIndex = -1;
    setActiveSongSpeedByIndex(-1);
  }
  currentFreq = -1;
}

//---------------------------------------------------------------
//  ADC鍵盤スキャン
//---------------------------------------------------------------
void processAdcKeys(void) {
  for (int ch = 0; ch < chMax; ch++) {
    int mv = (int)analogReadMilliVolts(adcPins[ch]);
    int swRaw = classifySwitchMv(mv); // 0..2, 3(NONE), -1(UNKNOWN)

    if (swRaw == swCandidate[ch]) {
      if (swCnt[ch] < 255) swCnt[ch]++;
    } else {
      swCandidate[ch] = swRaw;
      swCnt[ch] = 1;
    }

    if (swCnt[ch] >= SW_STABLE_COUNT) {
      int prev = swStable[ch];
      int now = swCandidate[ch];

      if (now != prev) {
        swStable[ch] = now;

        if (now == 3) {
          if (adcActiveCh == ch) {
            adcActiveCh = -1;
            adcActiveSw = -1;
            adcFreq = 0;
          }
        } else if (now == -1) {
          Serial.println("WTF");
        } else {
          printSwitchPressedOnce(ch, now);
          adcActiveCh = ch;
          adcActiveSw = now;
          adcFreq = melody[adcNoteIdx[ch][now]];
          enqueueKeyPress(adcNoteIdx[ch][now]);
        }
      }
    }
  }
}

//---------------------------------------------------------------
//  曲モード処理
//---------------------------------------------------------------
void processSongModeLogic(void) {
  int key = consumeKeyPress();
  if (key >= 0) {
    if (!spiffsReady) {
      Serial.println("SPIFFS not ready.");
    } else if (midiSongCount == 0) {
      Serial.println("No MIDI files in /m or /midi");
    } else {
      size_t songIndex = (size_t)key % midiSongCount;
      if (loadMidiSongByIndex(songIndex)) {
        selectedSongIndex = (int)songIndex;
        setActiveSongSpeedByIndex(selectedSongIndex);
        startMidiPlayback();
        Serial.print("[PLAY] ");
        Serial.print(midiSongs[songIndex].name);
        Serial.print("  (key=");
        Serial.print(key);
        Serial.print(", idx=");
        Serial.print(songIndex);
        Serial.print(", speed=");
        Serial.print((int)activeMidiSpeedPermille);
        Serial.println(")");
      }
    }
  }

  updateMidiPlayback();
}

//---------------------------------------------------------------
//  鍵盤押下イベント登録
//---------------------------------------------------------------
void enqueueKeyPress(int noteIdx) {
  if (noteIdx < 0) return;
  pendingKeyPress = noteIdx;
}

//---------------------------------------------------------------
//  鍵盤押下イベント取得
//---------------------------------------------------------------
int consumeKeyPress(void) {
  int key = pendingKeyPress;
  pendingKeyPress = -1;
  return key;
}

//---------------------------------------------------------------
//  デジタル3ボタン監視(返値=周波数)
//---------------------------------------------------------------
int freqFromButtons(uint8_t raw) {
  // INPUT_PULLUPなので押下=0
  if ((raw & (1 << 0)) == 0) return melody[21]; // Do#5
  if ((raw & (1 << 1)) == 0) return melody[22]; // Re5
  if ((raw & (1 << 2)) == 0) return melody[23]; // Mi5
  return 0;
}

//---------------------------------------------------------------
//  タイマ割り込み
//---------------------------------------------------------------
void ARDUINO_ISR_ATTR onTimer(void) {
  xSemaphoreGiveFromISR(sampleSem, nullptr);
}

//---------------------------------------------------------------
//  出音更新
//---------------------------------------------------------------
void updateBuzzerNonBlocking(void) {
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

  updateSongSpeedButtons(now);

  // デジタルボタンの押下エッジを曲選択用に登録
  if (stableRaw != prevStableRaw) {
    for (uint8_t bit = 0; bit < 3; bit++) {
      bool wasReleased = ((prevStableRaw & (1 << bit)) != 0);
      bool isPressed = ((stableRaw & (1 << bit)) == 0);
      if (wasReleased && isPressed) {
        enqueueKeyPress((int)DIGITAL_NOTE_BASE + (int)bit);
        break;
      }
    }
    prevStableRaw = stableRaw;
  }

  int targetFreq = 0;
  if (songMode) {
    targetFreq = midiPlaybackFreq;
  } else {
    targetFreq = freqFromButtons(stableRaw);
    if (targetFreq == 0) {
      targetFreq = adcFreq;
    }
  }

  bool vibEnabled = (digitalRead(VIB_PIN) == LOW);
  int outputFreq = applyVibrato(targetFreq, now);

  if (vibEnabled != lastVibEnabled) {
    lastVibEnabled = vibEnabled;
    currentFreq = -1;
  }

  if (outputFreq != currentFreq) {
    currentFreq = outputFreq;
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
//  Vibrato
//---------------------------------------------------------------
int applyVibrato(int baseFreq, uint32_t nowMs) {
  if (baseFreq <= 0) return 0;
  if (digitalRead(VIB_PIN) != LOW) return baseFreq;

  uint32_t t = nowMs % VIBRATO_PERIOD_MS;
  uint32_t half = VIBRATO_PERIOD_MS / 2;
  int32_t lfoPermille = 0;

  if (t < half) {
    lfoPermille = -1000 + (2000 * (int32_t)t) / (int32_t)half;
  } else {
    lfoPermille = 1000 - (2000 * (int32_t)(t - half)) / (int32_t)half;
  }

  int32_t delta =
    ((int32_t)baseFreq * VIBRATO_DEPTH_PERMILLE * lfoPermille) / 1000000;
  return baseFreq + (int)delta;
}

//---------------------------------------------------------------
//  ADC入力判定
//---------------------------------------------------------------
int classifySwitchMv(int mv) {
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

//---------------------------------------------------------------
//  MIDI周波数テーブル作成
//---------------------------------------------------------------
void buildMidiFreqTable(void) {
  for (int note = 0; note < 128; note++) {
    float ratio = powf(2.0f, ((float)note - 69.0f) / 12.0f);
    float freq = 440.0f * ratio;
    midiFreqTable[note] = (int)(freq + 0.5f);
  }
}

//---------------------------------------------------------------
//  SPIFFS初期化と曲一覧取得
//---------------------------------------------------------------
bool initSpiffsAndSongList(void) {
  if (!SPIFFS.begin(false)) {
    Serial.println("SPIFFS mount failed.");
    return false;
  }
  scanMidiSongs();
  initSongSpeedTable();
  loadSongSpeedTableFromSpiffs();
  printSongList();
  return true;
}

//---------------------------------------------------------------
//  /midi 配下の .mid 列挙
//---------------------------------------------------------------
void scanMidiSongs(void) {
  midiSongCount = 0;
  const char* folder = "/m";
  File dir = SPIFFS.open(folder);
  if (!dir || !dir.isDirectory()) {
    folder = "/midi";
    dir = SPIFFS.open(folder);
  }
  if (!dir || !dir.isDirectory()) {
    Serial.println("Directory '/m' or '/midi' not found.");
    return;
  }

  File file = dir.openNextFile();
  while (file && midiSongCount < MAX_SONGS) {
    if (!file.isDirectory()) {
      const char* path = file.name();
      if (path != nullptr && pathHasMidExt(path)) {
        normalizeSpiffsPath(
          midiSongs[midiSongCount].path,
          sizeof(midiSongs[midiSongCount].path),
          folder,
          path
        );

        const char* base = basenameFromPath(path);
        strncpy(midiSongs[midiSongCount].name, base, sizeof(midiSongs[midiSongCount].name) - 1);
        midiSongs[midiSongCount].name[sizeof(midiSongs[midiSongCount].name) - 1] = '\0';
        midiSongCount++;
      }
    }
    file.close();
    file = dir.openNextFile();
  }
  dir.close();
}

//---------------------------------------------------------------
//  曲一覧表示
//---------------------------------------------------------------
void printSongList(void) {
  Serial.print("MIDI files: ");
  Serial.println((int)midiSongCount);
  for (size_t i = 0; i < midiSongCount; i++) {
    Serial.print("  [");
    Serial.print((int)i);
    Serial.print("] ");
    Serial.print(midiSongs[i].name);
    Serial.print("  speed=");
    Serial.println((int)midiSongSpeedPermille[i]);
  }
}

//---------------------------------------------------------------
//  MIDI再生速度 clamp
//---------------------------------------------------------------
uint16_t clampMidiSpeedPermille(uint32_t value) {
  if (value < (uint32_t)MIDI_SPEED_MIN_PERMILLE) return MIDI_SPEED_MIN_PERMILLE;
  if (value > (uint32_t)MIDI_SPEED_MAX_PERMILLE) return MIDI_SPEED_MAX_PERMILLE;
  return (uint16_t)value;
}

//---------------------------------------------------------------
//  曲別速度テーブル初期化
//---------------------------------------------------------------
void initSongSpeedTable(void) {
  uint16_t defaultSpeed = clampMidiSpeedPermille((uint32_t)MIDI_PLAYBACK_SPEED_PERMILLE);
  for (size_t i = 0; i < MAX_SONGS; i++) {
    midiSongSpeedPermille[i] = defaultSpeed;
  }
  activeMidiSpeedPermille = defaultSpeed;
}

//---------------------------------------------------------------
//  曲別速度テーブル読み込み
//---------------------------------------------------------------
void loadSongSpeedTableFromSpiffs(void) {
  if (midiSongCount == 0) return;

  File file = SPIFFS.open(MIDI_SPEED_STORE_PATH, "r");
  if (!file) {
    Serial.println("MIDI speed table not found. Use defaults.");
    return;
  }

  char line[192];
  while (file.available()) {
    size_t n = file.readBytesUntil('\n', line, sizeof(line) - 1);
    line[n] = '\0';

    while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == ' ' || line[n - 1] == '\t')) {
      n--;
      line[n] = '\0';
    }
    if (n == 0 || line[0] == '#') continue;

    char* sep = strchr(line, '|');
    if (sep == nullptr) continue;
    *sep = '\0';

    char* path = line;
    char* speedText = sep + 1;
    while (*speedText == ' ' || *speedText == '\t') speedText++;

    char* endPtr = nullptr;
    unsigned long raw = strtoul(speedText, &endPtr, 10);
    if (endPtr == speedText) continue;
    uint16_t speed = clampMidiSpeedPermille((uint32_t)raw);

    for (size_t i = 0; i < midiSongCount; i++) {
      if (strcmp(midiSongs[i].path, path) == 0) {
        midiSongSpeedPermille[i] = speed;
        break;
      }
    }
  }

  file.close();
}

//---------------------------------------------------------------
//  曲別速度テーブル保存
//---------------------------------------------------------------
void saveSongSpeedTableToSpiffs(void) {
  if (!spiffsReady) return;

  File file = SPIFFS.open(MIDI_SPEED_STORE_PATH, "w");
  if (!file) {
    Serial.println("Failed to save MIDI speed table.");
    return;
  }

  for (size_t i = 0; i < midiSongCount; i++) {
    file.print(midiSongs[i].path);
    file.print('|');
    file.println((int)midiSongSpeedPermille[i]);
  }

  file.close();
}

//---------------------------------------------------------------
//  選択曲の速度を現在値へ反映
//---------------------------------------------------------------
void setActiveSongSpeedByIndex(int songIndex) {
  if (songIndex >= 0 && songIndex < (int)midiSongCount) {
    activeMidiSpeedPermille = midiSongSpeedPermille[songIndex];
  } else {
    activeMidiSpeedPermille = clampMidiSpeedPermille((uint32_t)MIDI_PLAYBACK_SPEED_PERMILLE);
  }
}

//---------------------------------------------------------------
//  選択曲の速度変更
//---------------------------------------------------------------
void changeSelectedSongSpeed(int deltaSteps) {
  if (deltaSteps == 0) return;
  if (selectedSongIndex < 0 || selectedSongIndex >= (int)midiSongCount) {
    Serial.println("[SPEED] Select song first.");
    return;
  }

  int32_t next = (int32_t)midiSongSpeedPermille[selectedSongIndex] +
                 (int32_t)deltaSteps * (int32_t)MIDI_SPEED_STEP_PERMILLE;
  if (next < 0) next = 0;
  uint16_t clamped = clampMidiSpeedPermille((uint32_t)next);
  if (clamped == midiSongSpeedPermille[selectedSongIndex]) return;

  midiSongSpeedPermille[selectedSongIndex] = clamped;
  activeMidiSpeedPermille = clamped;
  saveSongSpeedTableToSpiffs();

  Serial.print("[SPEED] ");
  Serial.print(midiSongs[selectedSongIndex].name);
  Serial.print(" = ");
  Serial.print((int)clamped);
  Serial.println(" permille");
}

//---------------------------------------------------------------
//  SW_UP / SW_DOWN 監視
//---------------------------------------------------------------
void updateSongSpeedButtons(uint32_t nowMs) {
  uint8_t raw =
    ((digitalRead(SW_UP)   ? 1 : 0) << 0) |
    ((digitalRead(SW_DOWN) ? 1 : 0) << 1);

  if (raw != speedLastRaw) {
    speedLastRaw = raw;
    speedLastChangeMs = nowMs;
  }
  if (nowMs - speedLastChangeMs >= DEBOUNCE_MS) {
    speedStableRaw = raw;
  }

  if (speedStableRaw != speedPrevStableRaw) {
    bool upWasReleased = ((speedPrevStableRaw & (1 << 0)) != 0);
    bool upIsPressed = ((speedStableRaw & (1 << 0)) == 0);
    bool downWasReleased = ((speedPrevStableRaw & (1 << 1)) != 0);
    bool downIsPressed = ((speedStableRaw & (1 << 1)) == 0);

    if (songMode && upWasReleased && upIsPressed) {
      changeSelectedSongSpeed(+1);
    } else if (songMode && downWasReleased && downIsPressed) {
      changeSelectedSongSpeed(-1);
    }

    speedPrevStableRaw = speedStableRaw;
  }
}

//---------------------------------------------------------------
//  拡張子判定(.mid)
//---------------------------------------------------------------
bool pathHasMidExt(const char* path) {
  size_t n = strlen(path);
  if (n < 4) return false;
  const char* ext = path + (n - 4);
  return (ext[0] == '.') &&
         ((ext[1] == 'm') || (ext[1] == 'M')) &&
         ((ext[2] == 'i') || (ext[2] == 'I')) &&
         ((ext[3] == 'd') || (ext[3] == 'D'));
}

//---------------------------------------------------------------
//  パスからファイル名抽出
//---------------------------------------------------------------
const char* basenameFromPath(const char* path) {
  const char* slash = strrchr(path, '/');
  if (slash == nullptr) return path;
  return slash + 1;
}

//---------------------------------------------------------------
//  SPIFFSパス正規化
//---------------------------------------------------------------
void normalizeSpiffsPath(char* dst, size_t dstSize, const char* folder, const char* rawName) {
  if (dst == nullptr || dstSize == 0) return;
  if (rawName == nullptr || rawName[0] == '\0') {
    dst[0] = '\0';
    return;
  }

  if (rawName[0] == '/') {
    strncpy(dst, rawName, dstSize - 1);
    dst[dstSize - 1] = '\0';
    return;
  }

  if (folder == nullptr || folder[0] == '\0') {
    folder = "/";
  }

  size_t n = 0;
  if (folder[0] != '/') {
    if (n < dstSize - 1) dst[n++] = '/';
  }

  for (size_t i = 0; folder[i] != '\0' && n < dstSize - 1; i++) {
    dst[n++] = folder[i];
  }

  if (n > 0 && dst[n - 1] != '/' && n < dstSize - 1) {
    dst[n++] = '/';
  }

  for (size_t i = 0; rawName[i] != '\0' && n < dstSize - 1; i++) {
    dst[n++] = rawName[i];
  }
  dst[n] = '\0';
}

//---------------------------------------------------------------
//  MIDIバッファ解放
//---------------------------------------------------------------
void resetMidiBuffers(void) {
  if (midiEvents != nullptr) {
    free(midiEvents);
    midiEvents = nullptr;
  }
  if (tempoEvents != nullptr) {
    free(tempoEvents);
    tempoEvents = nullptr;
  }
  midiEventCount = 0;
  midiEventCap = 0;
  tempoEventCount = 0;
  tempoEventCap = 0;
  selectedMidiChannel = -1;
  midiChannelPriorityCount = 0;
}

//---------------------------------------------------------------
//  MIDIイベント領域確保
//---------------------------------------------------------------
bool reserveMidiEvents(size_t needCount) {
  if (needCount <= midiEventCap) return true;
  size_t newCap = (midiEventCap == 0) ? 256 : midiEventCap;
  while (newCap < needCount) {
    newCap *= 2;
  }
  MidiEvent* p = (MidiEvent*)realloc(midiEvents, newCap * sizeof(MidiEvent));
  if (p == nullptr) return false;
  midiEvents = p;
  midiEventCap = newCap;
  return true;
}

//---------------------------------------------------------------
//  テンポイベント領域確保
//---------------------------------------------------------------
bool reserveTempoEvents(size_t needCount) {
  if (needCount <= tempoEventCap) return true;
  size_t newCap = (tempoEventCap == 0) ? 16 : tempoEventCap;
  while (newCap < needCount) {
    newCap *= 2;
  }
  TempoEvent* p = (TempoEvent*)realloc(tempoEvents, newCap * sizeof(TempoEvent));
  if (p == nullptr) return false;
  tempoEvents = p;
  tempoEventCap = newCap;
  return true;
}

//---------------------------------------------------------------
//  MIDIイベント追加
//---------------------------------------------------------------
bool appendMidiEvent(uint32_t tick, uint8_t type, uint8_t channel, uint8_t note, uint8_t velocity) {
  if (!reserveMidiEvents(midiEventCount + 1)) return false;
  midiEvents[midiEventCount].tick = tick;
  midiEvents[midiEventCount].timeUs = 0;
  midiEvents[midiEventCount].type = type;
  midiEvents[midiEventCount].channel = channel;
  midiEvents[midiEventCount].note = note;
  midiEvents[midiEventCount].velocity = velocity;
  midiEventCount++;
  return true;
}

//---------------------------------------------------------------
//  テンポイベント追加
//---------------------------------------------------------------
bool appendTempoEvent(uint32_t tick, uint32_t usPerQuarter) {
  if (!reserveTempoEvents(tempoEventCount + 1)) return false;
  tempoEvents[tempoEventCount].tick = tick;
  tempoEvents[tempoEventCount].usPerQuarter = usPerQuarter;
  tempoEventCount++;
  return true;
}

//---------------------------------------------------------------
//  ファイル読み込み
//---------------------------------------------------------------
bool readFileIntoBuffer(const char* path, uint8_t** outBuf, size_t* outLen) {
  *outBuf = nullptr;
  *outLen = 0;

  char fixedPath[128];
  normalizeSpiffsPath(fixedPath, sizeof(fixedPath), "/", path);

  File file = SPIFFS.open(fixedPath, "r");
  if (!file) {
    Serial.print("Failed to open: ");
    Serial.println(fixedPath);
    return false;
  }

  size_t size = file.size();
  if (size == 0) {
    file.close();
    Serial.println("MIDI file is empty.");
    return false;
  }

  uint8_t* buf = (uint8_t*)malloc(size);
  if (buf == nullptr) {
    file.close();
    Serial.println("malloc failed.");
    return false;
  }

  size_t readLen = file.read(buf, size);
  file.close();
  if (readLen != size) {
    free(buf);
    Serial.println("read failed.");
    return false;
  }

  *outBuf = buf;
  *outLen = size;
  return true;
}

//---------------------------------------------------------------
//  16bit BE読み出し
//---------------------------------------------------------------
bool readU16BE(const uint8_t* data, size_t len, size_t* pos, uint16_t* out) {
  if (*pos + 2 > len) return false;
  *out = ((uint16_t)data[*pos] << 8) | (uint16_t)data[*pos + 1];
  *pos += 2;
  return true;
}

//---------------------------------------------------------------
//  32bit BE読み出し
//---------------------------------------------------------------
bool readU32BE(const uint8_t* data, size_t len, size_t* pos, uint32_t* out) {
  if (*pos + 4 > len) return false;
  *out = ((uint32_t)data[*pos] << 24) |
         ((uint32_t)data[*pos + 1] << 16) |
         ((uint32_t)data[*pos + 2] << 8) |
         (uint32_t)data[*pos + 3];
  *pos += 4;
  return true;
}

//---------------------------------------------------------------
//  VLQ読み出し
//---------------------------------------------------------------
bool readVLQ(const uint8_t* data, size_t end, size_t* pos, uint32_t* out) {
  uint32_t value = 0;
  for (int i = 0; i < 4; i++) {
    if (*pos >= end) return false;
    uint8_t b = data[*pos];
    (*pos)++;
    value = (value << 7) | (uint32_t)(b & 0x7F);
    if ((b & 0x80) == 0) {
      *out = value;
      return true;
    }
  }
  return false;
}

//---------------------------------------------------------------
//  MIDI解析
//---------------------------------------------------------------
bool parseMidiBuffer(const uint8_t* data, size_t len) {
  if (len < 14) return false;
  selectedMidiChannel = -1;
  midiChannelPriorityCount = 0;

  size_t pos = 0;
  if (memcmp(data + pos, "MThd", 4) != 0) return false;
  pos += 4;

  uint32_t headerLen = 0;
  if (!readU32BE(data, len, &pos, &headerLen)) return false;
  if (headerLen < 6) return false;
  if (pos + headerLen > len) return false;

  size_t headerPos = pos;
  uint16_t formatType = 0;
  uint16_t trackCount = 0;
  uint16_t division = 0;
  if (!readU16BE(data, len, &headerPos, &formatType)) return false;
  if (!readU16BE(data, len, &headerPos, &trackCount)) return false;
  if (!readU16BE(data, len, &headerPos, &division)) return false;
  if (division == 0 || (division & 0x8000) != 0) {
    Serial.println("Unsupported MIDI division.");
    return false;
  }
  midiDivision = division;
  pos += headerLen;

  (void)formatType;
  uint32_t chNoteOnCount[MIDI_CHANNEL_COUNT] = {0};
  uint32_t chNoteSum[MIDI_CHANNEL_COUNT] = {0};

  if (!appendTempoEvent(0, DEFAULT_TEMPO_US_PER_QN)) {
    return false;
  }

  for (uint16_t tr = 0; tr < trackCount; tr++) {
    if (pos + 8 > len) return false;
    if (memcmp(data + pos, "MTrk", 4) != 0) return false;
    pos += 4;

    uint32_t trLen = 0;
    if (!readU32BE(data, len, &pos, &trLen)) return false;
    if (pos + trLen > len) return false;

    size_t trPos = pos;
    size_t trEnd = pos + trLen;
    uint32_t tick = 0;
    uint8_t runningStatus = 0;

    while (trPos < trEnd) {
      uint32_t delta = 0;
      if (!readVLQ(data, trEnd, &trPos, &delta)) return false;
      tick += delta;
      if (trPos >= trEnd) break;

      uint8_t status = data[trPos++];
      if (status < 0x80) {
        if (runningStatus == 0) return false;
        trPos--;
        status = runningStatus;
      } else if (status < 0xF0) {
        runningStatus = status;
      } else {
        runningStatus = 0;
      }

      if (status == 0xFF) {
        if (trPos >= trEnd) return false;
        uint8_t metaType = data[trPos++];
        uint32_t metaLen = 0;
        if (!readVLQ(data, trEnd, &trPos, &metaLen)) return false;
        if (trPos + metaLen > trEnd) return false;

        if (metaType == 0x51 && metaLen == 3) {
          uint32_t usPerQuarter =
            ((uint32_t)data[trPos] << 16) |
            ((uint32_t)data[trPos + 1] << 8) |
            (uint32_t)data[trPos + 2];
          if (usPerQuarter > 0) {
            if (!appendTempoEvent(tick, usPerQuarter)) return false;
          }
        }
        trPos += metaLen;
        continue;
      }

      if (status == 0xF0 || status == 0xF7) {
        uint32_t syxLen = 0;
        if (!readVLQ(data, trEnd, &trPos, &syxLen)) return false;
        if (trPos + syxLen > trEnd) return false;
        trPos += syxLen;
        continue;
      }

      uint8_t kind = status & 0xF0;
      uint8_t channel = status & 0x0F;
      if (kind == 0xC0 || kind == 0xD0) {
        if (trPos + 1 > trEnd) return false;
        trPos += 1;
      } else {
        if (trPos + 2 > trEnd) return false;
        uint8_t d1 = data[trPos++];
        uint8_t d2 = data[trPos++];

        if (channel == MIDI_DRUM_CHANNEL) {
          continue;
        }

        if (kind == 0x80) {
          if (!appendMidiEvent(tick, MIDI_NOTE_OFF, channel, d1, d2)) return false;
        } else if (kind == 0x90) {
          if (d2 == 0) {
            if (!appendMidiEvent(tick, MIDI_NOTE_OFF, channel, d1, d2)) return false;
          } else {
            if (!appendMidiEvent(tick, MIDI_NOTE_ON, channel, d1, d2)) return false;
            chNoteOnCount[channel]++;
            chNoteSum[channel] += d1;
          }
        }
      }
    }

    pos = trEnd;
  }

  uint64_t chAvgTimes100[MIDI_CHANNEL_COUNT] = {0};
  for (uint8_t ch = 0; ch < MIDI_CHANNEL_COUNT; ch++) {
    if (ch == MIDI_DRUM_CHANNEL) continue;
    uint32_t count = chNoteOnCount[ch];
    if (count == 0) continue;
    chAvgTimes100[ch] = ((uint64_t)chNoteSum[ch] * 100ULL + (uint64_t)(count / 2U)) / (uint64_t)count;
    midiChannelPriority[midiChannelPriorityCount++] = ch;
  }

  if (midiChannelPriorityCount == 0) {
    Serial.println("No melody channel found.");
    return false;
  }

  for (size_t i = 0; i < midiChannelPriorityCount; i++) {
    for (size_t j = i + 1; j < midiChannelPriorityCount; j++) {
      uint8_t a = midiChannelPriority[i];
      uint8_t b = midiChannelPriority[j];
      bool bIsHigher =
        (chAvgTimes100[b] > chAvgTimes100[a]) ||
        (chAvgTimes100[b] == chAvgTimes100[a] && chNoteOnCount[b] > chNoteOnCount[a]);
      if (bIsHigher) {
        uint8_t tmp = midiChannelPriority[i];
        midiChannelPriority[i] = midiChannelPriority[j];
        midiChannelPriority[j] = tmp;
      }
    }
  }
  selectedMidiChannel = (int)midiChannelPriority[0];

  if (midiEventCount == 0) {
    Serial.println("No note events in MIDI.");
    return false;
  }

  computeEventTimesFromTempo();
  return true;
}

//---------------------------------------------------------------
//  MIDIイベント比較
//---------------------------------------------------------------
int cmpMidiEvent(const void* a, const void* b) {
  const MidiEvent* ea = (const MidiEvent*)a;
  const MidiEvent* eb = (const MidiEvent*)b;
  if (ea->tick < eb->tick) return -1;
  if (ea->tick > eb->tick) return 1;
  if (ea->type < eb->type) return -1; // OFFを先に
  if (ea->type > eb->type) return 1;
  if (ea->note < eb->note) return -1;
  if (ea->note > eb->note) return 1;
  return 0;
}

//---------------------------------------------------------------
//  テンポイベント比較
//---------------------------------------------------------------
int cmpTempoEvent(const void* a, const void* b) {
  const TempoEvent* ta = (const TempoEvent*)a;
  const TempoEvent* tb = (const TempoEvent*)b;
  if (ta->tick < tb->tick) return -1;
  if (ta->tick > tb->tick) return 1;
  if (ta->usPerQuarter < tb->usPerQuarter) return -1;
  if (ta->usPerQuarter > tb->usPerQuarter) return 1;
  return 0;
}

//---------------------------------------------------------------
//  tick -> 再生時刻(us) 変換
//---------------------------------------------------------------
void computeEventTimesFromTempo(void) {
  qsort(midiEvents, midiEventCount, sizeof(MidiEvent), cmpMidiEvent);
  qsort(tempoEvents, tempoEventCount, sizeof(TempoEvent), cmpTempoEvent);

  size_t tempoPos = 0;
  uint32_t currentTempo = tempoEvents[0].usPerQuarter;
  uint32_t segmentTick = tempoEvents[0].tick;
  uint64_t segmentUs = 0;

  for (size_t i = 0; i < midiEventCount; i++) {
    uint32_t eventTick = midiEvents[i].tick;

    while ((tempoPos + 1) < tempoEventCount && tempoEvents[tempoPos + 1].tick <= eventTick) {
      uint32_t nextTick = tempoEvents[tempoPos + 1].tick;
      if (nextTick > segmentTick) {
        segmentUs += ((uint64_t)(nextTick - segmentTick) * (uint64_t)currentTempo) / (uint64_t)midiDivision;
      }
      segmentTick = nextTick;
      tempoPos++;
      currentTempo = tempoEvents[tempoPos].usPerQuarter;
    }

    uint64_t eventUs = segmentUs;
    if (eventTick > segmentTick) {
      eventUs += ((uint64_t)(eventTick - segmentTick) * (uint64_t)currentTempo) / (uint64_t)midiDivision;
    }

    midiEvents[i].timeUs = (eventUs > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)eventUs;
  }
}

//---------------------------------------------------------------
//  曲ロード
//---------------------------------------------------------------
bool loadMidiSongByIndex(size_t songIndex) {
  if (songIndex >= midiSongCount) return false;

  if ((int)songIndex == loadedSongIndex && midiEventCount > 0) {
    return true;
  }

  uint8_t* buf = nullptr;
  size_t len = 0;
  if (!readFileIntoBuffer(midiSongs[songIndex].path, &buf, &len)) {
    return false;
  }

  stopMidiPlayback();
  resetMidiBuffers();

  bool ok = parseMidiBuffer(buf, len);
  free(buf);

  if (!ok) {
    resetMidiBuffers();
    loadedSongIndex = -1;
    return false;
  }

  loadedSongIndex = (int)songIndex;
  Serial.print("Loaded MIDI: events=");
  Serial.print((int)midiEventCount);
  Serial.print(" melody_ch=");
  Serial.print(selectedMidiChannel);
  Serial.print(" fallback_ch_count=");
  Serial.print((int)midiChannelPriorityCount);
  Serial.print(" tempo=");
  Serial.println((int)tempoEventCount);
  return true;
}

//---------------------------------------------------------------
//  チャンネル状態初期化
//---------------------------------------------------------------
void resetMidiChannelState(void) {
  memset(midiActiveNotes, 0, sizeof(midiActiveNotes));
  for (uint8_t ch = 0; ch < MIDI_CHANNEL_COUNT; ch++) {
    midiCurrentNoteByChannel[ch] = -1;
  }
  currentMidiNote = -1;
}

//---------------------------------------------------------------
//  再生開始
//---------------------------------------------------------------
void startMidiPlayback(void) {
  resetMidiChannelState();
  midiPlayPos = 0;
  midiPlaybackFreq = 0;
  midiPlayElapsedSongUs = 0;
  midiPlayLastRealUs = micros();
  midiIsPlaying = (midiEventCount > 0);
}

//---------------------------------------------------------------
//  再生停止
//---------------------------------------------------------------
void stopMidiPlayback(void) {
  midiIsPlaying = false;
  midiPlayPos = 0;
  midiPlaybackFreq = 0;
  midiPlayElapsedSongUs = 0;
  midiPlayLastRealUs = 0;
  resetMidiChannelState();
}

//---------------------------------------------------------------
//  再生進行
//---------------------------------------------------------------
void updateMidiPlayback(void) {
  if (!midiIsPlaying || midiEventCount == 0) {
    midiPlaybackFreq = 0;
    return;
  }

  uint32_t nowUs = micros();
  uint32_t realDeltaUs = (uint32_t)(nowUs - midiPlayLastRealUs);
  midiPlayLastRealUs = nowUs;

  uint64_t scaledDeltaUs =
    ((uint64_t)realDeltaUs * (uint64_t)activeMidiSpeedPermille + 500ULL) / 1000ULL;
  midiPlayElapsedSongUs += scaledDeltaUs;

  uint32_t elapsedUs = (midiPlayElapsedSongUs > 0xFFFFFFFFULL)
                         ? 0xFFFFFFFFUL
                         : (uint32_t)midiPlayElapsedSongUs;
  while (midiPlayPos < midiEventCount && midiEvents[midiPlayPos].timeUs <= elapsedUs) {
    handleMidiEvent(&midiEvents[midiPlayPos]);
    midiPlayPos++;
  }

  currentMidiNote = -1;
  for (size_t i = 0; i < midiChannelPriorityCount; i++) {
    uint8_t ch = midiChannelPriority[i];
    int note = midiCurrentNoteByChannel[ch];
    if (note >= 0 && note < 128) {
      currentMidiNote = note;
      break;
    }
  }

  if (currentMidiNote >= 0 && currentMidiNote < 128) {
    midiPlaybackFreq = midiFreqTable[currentMidiNote];
  } else {
    midiPlaybackFreq = 0;
  }

  if (midiPlayPos >= midiEventCount && currentMidiNote < 0) {
    midiIsPlaying = false;
  }
}

//---------------------------------------------------------------
//  MIDIイベント反映
//---------------------------------------------------------------
void handleMidiEvent(const MidiEvent* ev) {
  if (ev == nullptr) return;
  if (ev->channel >= MIDI_CHANNEL_COUNT) return;
  if (ev->note >= 128) return;
  uint8_t ch = ev->channel;

  if (ev->type == MIDI_NOTE_ON) {
    if (midiActiveNotes[ch][ev->note] < 255) {
      midiActiveNotes[ch][ev->note]++;
    }
    midiCurrentNoteByChannel[ch] = ev->note;
  } else {
    if (midiActiveNotes[ch][ev->note] > 0) {
      midiActiveNotes[ch][ev->note]--;
    }
    if (midiCurrentNoteByChannel[ch] == ev->note && midiActiveNotes[ch][ev->note] == 0) {
      midiCurrentNoteByChannel[ch] = -1;
    }
  }
}
