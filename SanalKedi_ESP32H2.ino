/*
 * ============================================================================
 *  SANAL KEDİ — ESP32-H2 Super Mini + SSD1306 için Tamagotchi tarzı evcil kedi
 * ============================================================================
 *
 *  DONANIM
 *    - ESP32-H2 Super Mini
 *    - 0.96" SSD1306 OLED 128x64 (I2C, adres genelde 0x3C)
 *    - TTP223B dokunmatik sensör  -> okşama (dijital çıkış, dokununca HIGH)
 *    - Buton 1 (OYUN)             -> pin ile GND arasına, INPUT_PULLUP
 *    - Buton 2 (BESLEME)          -> pin ile GND arasına, INPUT_PULLUP
 *    - Titreşim motoru MODÜLÜ     -> dijital çıkış (transistörlü modül).
 *        Çıplak motor kullanıyorsan: GPIO -> 1k direnç -> NPN/MOSFET,
 *        motora paralel flyback diyot şart. GPIO motoru doğrudan süremez.
 *    - TP4056 + 3.7V LiPo -> Super Mini'nin 5V/VBAT girişine
 *        (OLED, TTP223B ve titreşim modülü 3V3 hattından beslenebilir)
 *
 *  BAĞLANTI ÖZETİ (çalıştığı doğrulanmış kablolama)
 *    OLED SDA    -> GP10        OLED SCL      -> GP11
 *    Oyun btn    -> GP12        Besleme btn   -> GP13
 *    TTP223B I/O -> GP14        Titreşim      -> GP1
 *    Butonlar: bir bacak pine, çapraz bacak GND'ye (INPUT_PULLUP, basınca LOW)
 *
 *  YAZILIM
 *    - Kart paketi : "esp32" by Espressif, sürüm 3.x  ->  "ESP32H2 Dev Module"
 *      (Serial Monitor için: Tools > USB CDC On Boot > Enabled)
 *    - Kütüphaneler: Adafruit GFX Library, Adafruit SSD1306
 *
 *  KURAL: Hiçbir yerde delay() yok. Tüm zamanlama millis() ile non-blocking.
 *
 *  DURUM MAKİNESİ
 *    STATE_WALKING  -> rastgele hedefe yürür
 *    STATE_HAPPY    -> durur, etrafa bakar, mutlu ifade
 *    STATE_BORED    -> uzun süre etkileşim yoksa köşeye gidip oturur, uyuklar
 *    STATE_PLAYING  -> Buton 1: ip yumağı + zıplama/pati animasyonu
 *    STATE_EATING   -> Buton 2: mama kabı + balık, çiğneme, ritmik titreşim
 *    STATE_LOVE     -> Dokunma: kalp gözler, uçuşan kalpler, mırlama titreşimi
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ============================================================================
//  1) PİN TANIMLARI  (kartına göre buradan değiştir)
// ============================================================================
#define I2C_SDA_PIN          10
#define I2C_SCL_PIN          11
#define BTN_PLAY_PIN         12     // Buton 1: oyun (ip yumağı)
#define BTN_FEED_PIN         13     // Buton 2: besleme
#define TOUCH_PIN            14     // TTP223B çıkışı
#define VIBRATION_PIN        1      // titreşim modülü

#define TOUCH_ACTIVE_HIGH    1      // TTP223B varsayılan: dokununca HIGH

// ---- TEŞHİS AYARLARI ----
#define INPUT_DEBUG          1      // 1: üst barda 3 kutu (T/O/B) girişlerin ham durumunu gösterir
                                    //    + Serial'e her değişimi yazar
#define PIN_TEST_MODE        0      // 1: kedi yerine canlı pin test ekranı (kablolama kontrolü)

// ============================================================================
//  2) EKRAN AYARLARI
// ============================================================================
#define SCREEN_WIDTH         128
#define SCREEN_HEIGHT        64
#define OLED_RESET_PIN       -1     // Reset pini yok (modül dahili reset)
#define OLED_I2C_ADDR        0x3C   // Bazı modüllerde 0x3D

// ============================================================================
//  3) TİTREŞİM AYARLARI
// ============================================================================
#define VIB_ACTIVE_LOW       0      // 0: IN HIGH olunca titrer (senin modülün)
                                    // 1: IN LOW olunca titreyen modüller
#define VIB_HUNGER_ALERT     0      // 1: acıkınca çift titreşim. 0: sadece touch/buton titreştirir
#define VIB_USE_PWM          0      // 0: saf dijital (HIGH/LOW)
                                    // 1: analogWrite ile daha yumuşak titreşim
#define VIB_PWM_LEVEL        160    // VIB_USE_PWM=1 iken güç seviyesi (0-255)

// ============================================================================
//  4) ZAMANLAMA AYARLARI  (test değerleri; gerçek kullanım değerleri yorumda)
// ============================================================================
#define SPLASH_MS            1500UL    // Açılış ekranı süresi
#define FRAME_INTERVAL_MS    40UL      // ~25 FPS ekran yenileme
#define DEBOUNCE_MS          30UL      // Buton/touch titreşim süzme

#define BORED_TIMEOUT_MS     3600000UL   // TEST: 1.5 dk   | GERÇEK: 3600000UL (1 saat)
#define HUNGER_TICK_MS       36000UL    // TEST: 0->100 ~2.5 dk | GERÇEK: 36000UL (~1 saat)
#define HUNGRY_THRESHOLD     60        // Açlık bu değeri geçince kedi "aç" sayılır
#define START_HUNGER         30        // Açılıştaki açlık
#define PLAY_HUNGER_COST     5         // Oyun sonrası açlık artışı

#define PLAY_DURATION_MS     6000UL    // Oyun süresi
#define EAT_DURATION_MS      5000UL    // Yemek süresi
#define LOVE_LINGER_MS       1000UL    // Dokunma bitince sevgi modunda kalma süresi
#define JOY_AFTER_MS         2500UL    // Etkileşim sonrası ^^ gözlü neşe süresi

#define WALK_STEP_MS         55UL      // Normal yürüme hızı (1 piksel / adım)
#define WALK_STEP_HUNGRY_MS  100UL     // Açken daha yavaş
#define WALK_STEP_BORED_MS   140UL     // Sıkılınca köşeye ağır ağır gider

#define JUMP_CYCLE_MS        700UL     // Oyunda bir zıplama döngüsü
#define JUMP_AIR_MS          420UL     // Döngünün havada geçen kısmı
#define JUMP_HEIGHT_PX       10        // Zıplama yüksekliği

// ============================================================================
//  5) YERLEŞİM (piksel)
// ============================================================================
#define STATUS_BAR_H         10        // Üst bilgi çubuğu yüksekliği
#define GROUND_Y             62        // Kedinin ayaklarının bastığı referans çizgi
#define CAT_MIN_X            22        // Yürürken kedi merkezinin sınırları
#define CAT_MAX_X            105
#define CORNER_LEFT_X        16        // Sıkılınca oturacağı köşeler
#define CORNER_RIGHT_X       108

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

// ============================================================================
//  PROGMEM BITMAP'LER
// ============================================================================

// İp yumağı 12x12 (satır başına 2 bayt, MSB solda)
static const uint8_t YARN_BMP[] PROGMEM = {
  0x0F, 0x00,   // ....XXXX....
  0x30, 0xC0,   // ..XX....XX..
  0x4C, 0x20,   // .X..XX....X.
  0x43, 0x20,   // .X....XX..X.
  0xB0, 0x90,   // X.XX....X..X
  0x8C, 0x50,   // X...XX...X.X
  0xA3, 0x30,   // X.X...XX..XX
  0x98, 0x90,   // X..XX...X..X
  0x46, 0x60,   // .X...XX..XX.
  0x51, 0xA0,   // .X.X...XX.X.
  0x30, 0xC0,   // ..XX....XX..
  0x0F, 0x00    // ....XXXX....
};

// Balık 16x8 (mama kabında ve "acıktım" balonunda kullanılır)
static const uint8_t FISH_BMP[] PROGMEM = {
  0x03, 0xC0,   // ......XXXX......
  0x0F, 0xE2,   // ....XXXXXXX...X.
  0x3F, 0xF6,   // ..XXXXXXXXXX.XX.
  0x6F, 0xFE,   // .XX.XXXXXXXXXXX.   (göz boşluğu)
  0xFF, 0xFE,   // XXXXXXXXXXXXXXX.
  0x7F, 0xF6,   // .XXXXXXXXXXX.XX.
  0x1F, 0xE2,   // ...XXXXXXXX...X.
  0x07, 0xC0    // .....XXXXX......
};

// Kalp 7x6 (uçuşan kalpler)
static const uint8_t HEART_BMP[] PROGMEM = {
  0x6C,   // .XX.XX.
  0xFE,   // XXXXXXX
  0xFE,   // XXXXXXX
  0x7C,   // .XXXXX.
  0x38,   // ..XXX..
  0x10    // ...X...
};

// Kalp göz 5x4 (beyaz kafanın üstüne SİYAH çizilir, 5 bitlik satırlar)
static const uint8_t HEART_EYE[4] = { 0b01010, 0b11111, 0b01110, 0b00100 };

// ============================================================================
//  TİPLER  (Arduino'nun otomatik prototip üretimi için fonksiyonlardan önce)
// ============================================================================
enum CatState : uint8_t {
  STATE_WALKING = 0,
  STATE_HAPPY,
  STATE_BORED,
  STATE_PLAYING,
  STATE_EATING,
  STATE_LOVE
};

static const char *STATE_NAMES[] = { "WALKING", "HAPPY", "BORED", "PLAYING", "EATING", "LOVE" };

enum EyeType   : uint8_t { EYE_OPEN, EYE_BLINK, EYE_HAPPY, EYE_SLEEPY, EYE_HEART };
enum MouthType : uint8_t { MOUTH_SMILE, MOUTH_OPEN, MOUTH_FLAT, MOUTH_SAD };

// Yandan görünen kedinin çizim parametreleri
struct SideCatParams {
  EyeType   eyes;
  MouthType mouth;
  bool      legsMoving;   // yürüme bacak animasyonu
  uint8_t   legFrame;     // 0/1 bacak karesi
  bool      pawUp;        // ön pati havada (oyun)
  int8_t    headDx;       // kafa kaydırma (yemek yerken eğilme)
  int8_t    headDy;
  int8_t    tailWave;     // kuyruk ucu salınımı (piksel)
};

// Debounce'lu dijital giriş
struct DebouncedInput {
  const char *name;
  uint8_t  pin;
  bool     activeHigh;
  bool     stable;        // süzülmüş durum (true = aktif/basılı)
  bool     lastRaw;
  uint32_t lastChangeMs;
  bool     pressed;       // bu döngüde basıldı olayı
  bool     released;      // bu döngüde bırakıldı olayı
};

// Titreşim deseni: steps[] = AÇIK, KAPALI, AÇIK, KAPALI ... (ms)
struct VibPattern {
  const uint16_t *steps;
  uint8_t         len;
  bool            loop;
};

// ============================================================================
//  TİTREŞİM DESENLERİ
// ============================================================================
// Not: Titreşim motoru (ERM) dönmeye başlamak için ~60-80 ms'ye ihtiyaç duyar.
// Bundan kısa AÇIK adımlar motoru hiç döndürmez, bu yüzden adımlar uzun tutuldu.
static const uint16_t VIB_STEPS_PURR[]   = { 180, 70, 180, 70, 180, 350 };      // mırr-mırr-mırr ... nefes
static const uint16_t VIB_STEPS_EAT[]    = { 90, 410 };                         // çiğneme ritmi
static const uint16_t VIB_STEPS_TAP[]    = { 80 };                              // tek kısa dokunuş
static const uint16_t VIB_STEPS_HUNGRY[] = { 120, 120, 120 };                   // çift titreşim: "acıktım"
static const uint16_t VIB_STEPS_ERROR[]  = { 400, 600 };                        // ekran bulunamadı

static const VibPattern VIB_PURR   = { VIB_STEPS_PURR,   ARRAY_LEN(VIB_STEPS_PURR),   true  };
static const VibPattern VIB_EAT    = { VIB_STEPS_EAT,    ARRAY_LEN(VIB_STEPS_EAT),    true  };
static const VibPattern VIB_TAP    = { VIB_STEPS_TAP,    ARRAY_LEN(VIB_STEPS_TAP),    false };
static const VibPattern VIB_HUNGRY = { VIB_STEPS_HUNGRY, ARRAY_LEN(VIB_STEPS_HUNGRY), false };
static const VibPattern VIB_ERROR  = { VIB_STEPS_ERROR,  ARRAY_LEN(VIB_STEPS_ERROR),  true  };

// ============================================================================
//  GLOBAL DEĞİŞKENLER
// ============================================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
bool displayOk  = false;
bool splashDone = false;

DebouncedInput touchIn, playBtn, feedBtn;
bool touchHeld = false;

// Kedi durumu
CatState state        = STATE_WALKING;
uint32_t stateStartMs = 0;
int16_t  catX         = 64;     // kedi gövde merkezi (x)
bool     catFaceRight = true;

// Yürüme / bekleme
int16_t  targetX         = 64;
uint32_t lastWalkStepMs  = 0;
uint16_t stepCount       = 0;
uint8_t  legFrame        = 0;
uint32_t idleDurationMs  = 3000;
uint32_t nextLookMs      = 0;
uint32_t happyFlashUntil = 0;

// Göz kırpma
uint32_t nextBlinkMs  = 0;
uint32_t blinkUntilMs = 0;

// İhtiyaçlar
int16_t  hunger           = START_HUNGER;   // 0 = tok, 100 = çok aç
bool     wasHungry        = false;
uint32_t lastHungerTickMs = 0;
uint32_t lastInteractionMs = 0;
uint32_t joyUntilMs       = 0;

// Duruma özel
int16_t  boredTargetX    = CORNER_LEFT_X;
bool     boredArrived    = false;
uint32_t boredSitMs      = 0;
int32_t  lastLandedJump  = -1;
int16_t  eatStartHunger  = 0;
uint32_t touchReleasedMs = 0;

// Ekran
uint32_t lastFrameMs = 0;

// Yandan kedi çizimi için ayna bağlamı
static int16_t g_cx = 0, g_gy = 0;
static bool    g_right = true;

// ============================================================================
//  ZAMAN YARDIMCISI  (millis() taşmasına dayanıklı karşılaştırma)
// ============================================================================
inline bool timeReached(uint32_t now, uint32_t t) {
  return (int32_t)(now - t) >= 0;
}

// ============================================================================
//  TİTREŞİM MOTORU — non-blocking desen oynatıcı
// ============================================================================
const VibPattern *vibPattern = nullptr;
uint8_t  vibIdx       = 0;
uint32_t vibStepStart = 0;

void vibWrite(bool on) {
#if VIB_USE_PWM
  uint8_t duty = on ? VIB_PWM_LEVEL : 0;
  analogWrite(VIBRATION_PIN, VIB_ACTIVE_LOW ? (255 - duty) : duty);
#else
  bool level = VIB_ACTIVE_LOW ? !on : on;     // aktif-LOW modülde mantık ters
  digitalWrite(VIBRATION_PIN, level ? HIGH : LOW);
#endif
}

void vibPlay(const VibPattern &p) {
  vibPattern   = &p;
  vibIdx       = 0;
  vibStepStart = millis();
  vibWrite(true);                 // desenler her zaman AÇIK adımla başlar
}

void vibStop() {
  vibPattern = nullptr;
  vibWrite(false);
}

void vibUpdate(uint32_t now) {
  if (!vibPattern) return;
  // Geçen süre mevcut adımı aştıkça sonraki adıma geç
  while ((now - vibStepStart) >= vibPattern->steps[vibIdx]) {
    vibStepStart += vibPattern->steps[vibIdx];
    vibIdx++;
    if (vibIdx >= vibPattern->len) {
      if (vibPattern->loop) vibIdx = 0;
      else { vibStop(); return; }
    }
  }
  vibWrite((vibIdx % 2) == 0);    // çift indeks = AÇIK, tek indeks = KAPALI
}

// ============================================================================
//  GİRİŞLER — debounce + basma/bırakma olayları
// ============================================================================
void inputBegin(DebouncedInput &in, const char *name, uint8_t pin, uint8_t mode, bool activeHigh) {
  pinMode(pin, mode);
  in.name         = name;
  in.pin          = pin;
  in.activeHigh   = activeHigh;
  bool raw        = (digitalRead(pin) == (activeHigh ? HIGH : LOW));
  in.stable       = raw;
  in.lastRaw      = raw;
  in.lastChangeMs = 0;
  in.pressed      = false;
  in.released     = false;
}

void inputUpdate(DebouncedInput &in, uint32_t now) {
  in.pressed  = false;
  in.released = false;
  bool raw = (digitalRead(in.pin) == (in.activeHigh ? HIGH : LOW));
  if (raw != in.lastRaw) {        // ham değer değişti -> sayacı sıfırla
    in.lastRaw      = raw;
    in.lastChangeMs = now;
  }
  // Değer DEBOUNCE_MS boyunca sabit kaldıysa kabul et
  if ((now - in.lastChangeMs) >= DEBOUNCE_MS && raw != in.stable) {
    in.stable = raw;
    if (raw) in.pressed = true;
    else     in.released = true;
#if INPUT_DEBUG
    Serial.printf("[GIRIS] %-7s (GP%d) -> %s | pin seviyesi: %s\n",
                  in.name, in.pin, raw ? "AKTIF" : "pasif",
                  digitalRead(in.pin) ? "HIGH" : "LOW");
#endif
  }
}

// Açılışta boştaki seviyeyi raporla, şüpheli durumları uyar
void reportIdleLevel(const DebouncedInput &in, bool expectHighWhenIdle) {
  int lvl = digitalRead(in.pin);
  Serial.printf("[GIRIS] %-7s (GP%d) bosta: %s\n", in.name, in.pin, lvl ? "HIGH" : "LOW");
  if (expectHighWhenIdle && lvl == LOW) {
    Serial.printf("  UYARI: %s bosta LOW okuyor! Buton surekli basili gibi -> "
                  "yanlis bacak (ayni taraftaki bacaklar iceriden bagli) ya da kisa devre.\n", in.name);
  }
}

// ============================================================================
//  ÇİZİM YARDIMCILARI — ayna (sağa/sola bakış) dönüşümleri
//  Yandan kedi "yerel" koordinatlarla tanımlanır: +dx = kafa yönü, dy<0 = yukarı
// ============================================================================
inline int16_t MX(int16_t dx) { return g_right ? g_cx + dx : g_cx - dx; }
inline int16_t MY(int16_t dy) { return g_gy + dy; }

void mFillRect(int16_t dx, int16_t dy, int16_t w, int16_t h, uint16_t c) {
  int16_t x = g_right ? g_cx + dx : g_cx - dx - w + 1;
  display.fillRect(x, g_gy + dy, w, h, c);
}

void mFillRoundRect(int16_t dx, int16_t dy, int16_t w, int16_t h, int16_t r, uint16_t c) {
  int16_t x = g_right ? g_cx + dx : g_cx - dx - w + 1;
  display.fillRoundRect(x, g_gy + dy, w, h, r, c);
}

void mLine(int16_t dx1, int16_t dy1, int16_t dx2, int16_t dy2, uint16_t c) {
  display.drawLine(MX(dx1), MY(dy1), MX(dx2), MY(dy2), c);
}

void mTri(int16_t ax, int16_t ay, int16_t bx, int16_t by, int16_t cx, int16_t cy, uint16_t c) {
  display.fillTriangle(MX(ax), MY(ay), MX(bx), MY(by), MX(cx), MY(cy), c);
}

void mCircle(int16_t dx, int16_t dy, int16_t r, uint16_t c) {
  display.fillCircle(MX(dx), MY(dy), r, c);
}

// ============================================================================
//  YÜZ PARÇALARI  (beyaz kafa üstüne SİYAH çizilir)
// ============================================================================
void drawEye(int16_t x, int16_t y, EyeType t) {
  switch (t) {
    case EYE_OPEN:                                   // dik oval göz
      display.fillRect(x - 1, y - 1, 2, 3, SSD1306_BLACK);
      break;
    case EYE_BLINK:                                  // kapalı göz: düz çizgi
      display.drawFastHLine(x - 1, y + 1, 3, SSD1306_BLACK);
      break;
    case EYE_HAPPY:                                  // ^ şeklinde mutlu göz
      display.drawPixel(x - 1, y + 1, SSD1306_BLACK);
      display.drawPixel(x,     y,     SSD1306_BLACK);
      display.drawPixel(x + 1, y + 1, SSD1306_BLACK);
      break;
    case EYE_SLEEPY:                                 // yarı kapalı göz kapağı
      display.drawFastHLine(x - 1, y, 3, SSD1306_BLACK);
      display.drawPixel(x, y + 1, SSD1306_BLACK);
      break;
    case EYE_HEART:                                  // 5x4 kalp göz
      for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 5; c++) {
          if (HEART_EYE[r] & (1 << (4 - c))) {
            display.drawPixel(x - 2 + c, y - 1 + r, SSD1306_BLACK);
          }
        }
      }
      break;
  }
}

void drawMouth(int16_t x, int16_t y, MouthType t) {
  // Burun (küçük ters üçgen)
  display.drawFastHLine(x - 1, y - 2, 3, SSD1306_BLACK);
  display.drawPixel(x, y - 1, SSD1306_BLACK);

  switch (t) {
    case MOUTH_SMILE:                                // kedi ağzı "w"
      display.drawPixel(x - 2, y,     SSD1306_BLACK);
      display.drawPixel(x - 1, y + 1, SSD1306_BLACK);
      display.drawPixel(x,     y,     SSD1306_BLACK);
      display.drawPixel(x + 1, y + 1, SSD1306_BLACK);
      display.drawPixel(x + 2, y,     SSD1306_BLACK);
      break;
    case MOUTH_OPEN:                                 // açık ağız (çiğneme/esneme)
      display.fillRect(x - 1, y, 3, 3, SSD1306_BLACK);
      break;
    case MOUTH_FLAT:                                 // düz/sıkkın
      display.drawFastHLine(x - 1, y + 1, 3, SSD1306_BLACK);
      break;
    case MOUTH_SAD:                                  // ters "u": üzgün/aç
      display.drawPixel(x - 2, y + 2, SSD1306_BLACK);
      display.drawFastHLine(x - 1, y + 1, 3, SSD1306_BLACK);
      display.drawPixel(x + 2, y + 2, SSD1306_BLACK);
      break;
  }
}

// ============================================================================
//  KEDİ — YANDAN GÖRÜNÜM (yürüme, mutlu, oyun, yemek)
//  Referans: (cx, gy) = gövde merkezinin altı / zemin. Yükseklik ~28 px.
// ============================================================================
void drawCatSide(int16_t cx, int16_t gy, bool faceRight, const SideCatParams &p) {
  g_cx = cx; g_gy = gy; g_right = faceRight;
  const uint16_t W = SSD1306_WHITE;

  // --- Kuyruk (gövdenin arkasından yukarı kıvrılır, 2 px kalınlık) ---
  int16_t w = p.tailWave;
  mLine(-11, -11, -16, -15, W);
  mLine(-12, -11, -17, -15, W);
  mLine(-16, -15, -17 + w, -22, W);
  mLine(-17, -15, -18 + w, -22, W);

  // --- Bacaklar: çapraz çiftler sırayla kalkar ---
  const int8_t legX[4] = { -10, -6, 2, 6 };
  for (uint8_t i = 0; i < 4; i++) {
    if (p.pawUp && i == 3) continue;               // ön pati havadaysa çizme
    bool lifted = p.legsMoving && (((i + p.legFrame) % 2) == 0);
    mFillRect(legX[i], -5, 2, lifted ? 3 : 5, W);
  }

  // --- Havadaki ön pati (oyun: yumağa pati atma) ---
  if (p.pawUp) {
    mLine(6, -6, 11, -12, W);
    mLine(7, -6, 12, -12, W);
    mFillRect(11, -14, 3, 3, W);
  }

  // --- Gövde ---
  mFillRoundRect(-12, -14, 21, 10, 4, W);

  // --- Kafa ---
  int16_t hx = 10 + p.headDx;
  int16_t hy = -17 + p.headDy;
  mTri(hx - 6, hy - 3, hx - 4, hy - 11, hx,     hy - 6, W);   // arka kulak
  mTri(hx + 1, hy - 6, hx + 4, hy - 11, hx + 6, hy - 3, W);   // ön kulak
  mCircle(hx, hy, 7, W);

  // --- Yüz ---
  drawEye(MX(hx - 2), MY(hy - 1), p.eyes);
  drawEye(MX(hx + 4), MY(hy - 1), p.eyes);
  drawMouth(MX(hx + 1), MY(hy + 3), p.mouth);

  // --- Bıyıklar (kafanın dışına taşan kısım görünür) ---
  mLine(hx + 6, hy + 2, hx + 11, hy + 1, W);
  mLine(hx + 6, hy + 4, hx + 11, hy + 5, W);
}

// ============================================================================
//  KEDİ — ÖNDEN OTURAN GÖRÜNÜM (sıkılma, sevgi, açılış)
//  Yükseklik ~32 px, genişlik ~17 px + sağda kuyruk
// ============================================================================
void drawCatSitting(int16_t cx, int16_t gy, EyeType eyes, MouthType mouth,
                    int8_t tailWave, bool blush) {
  const uint16_t W = SSD1306_WHITE;
  const uint16_t B = SSD1306_BLACK;

  // Kuyruk: sağ yanda yerde kıvrılır
  display.drawLine(cx + 7,  gy - 2, cx + 13, gy - 4, W);
  display.drawLine(cx + 7,  gy - 3, cx + 13, gy - 5, W);
  display.drawLine(cx + 13, gy - 4, cx + 14 + tailWave, gy - 11, W);
  display.drawLine(cx + 14, gy - 4, cx + 15 + tailWave, gy - 11, W);

  // Gövde + ön patiler arasındaki ayrım çizgileri
  display.fillRoundRect(cx - 8, gy - 15, 17, 15, 6, W);
  display.drawFastVLine(cx - 3, gy - 4, 4, B);
  display.drawFastVLine(cx + 3, gy - 4, 4, B);

  // Kulaklar + kafa
  display.fillTriangle(cx - 8, gy - 23, cx - 6, gy - 32, cx - 1, gy - 27, W);
  display.fillTriangle(cx + 8, gy - 23, cx + 6, gy - 32, cx + 1, gy - 27, W);
  display.fillCircle(cx, gy - 21, 8, W);

  // Yüz
  drawEye(cx - 3, gy - 22, eyes);
  drawEye(cx + 3, gy - 22, eyes);
  drawMouth(cx, gy - 17, mouth);

  // Allık (sevgi modunda yanaklarda küçük çizgiler)
  if (blush) {
    display.drawFastHLine(cx - 7, gy - 19, 2, B);
    display.drawFastHLine(cx + 6, gy - 19, 2, B);
  }

  // Bıyıklar (her iki yana)
  display.drawLine(cx - 8, gy - 19, cx - 13, gy - 20, W);
  display.drawLine(cx - 8, gy - 17, cx - 13, gy - 16, W);
  display.drawLine(cx + 8, gy - 19, cx + 13, gy - 20, W);
  display.drawLine(cx + 8, gy - 17, cx + 13, gy - 16, W);
}

// ============================================================================
//  EFEKTLER
// ============================================================================

// "Acıktım" düşünce balonu (içinde balık)
void drawHungerBubble(int16_t headX, int16_t headTopY, bool toRight) {
  int16_t dir = toRight ? 1 : -1;
  display.fillCircle(headX + dir * 4, headTopY - 2, 1, SSD1306_WHITE);
  display.fillCircle(headX + dir * 8, headTopY - 5, 1, SSD1306_WHITE);

  int16_t bx = headX + dir * 16;
  if (bx < 13)  bx = 13;
  if (bx > 114) bx = 114;
  int16_t by = headTopY - 12;
  if (by < STATUS_BAR_H + 6) by = STATUS_BAR_H + 6;

  display.fillRoundRect(bx - 12, by - 6, 24, 12, 4, SSD1306_WHITE);
  display.drawBitmap(bx - 8, by - 4, FISH_BMP, 16, 8, SSD1306_BLACK);
}

// Yukarı süzülen kalpler
void drawFloatingHearts(int16_t cx, int16_t baseY, uint32_t elapsed) {
  for (uint8_t i = 0; i < 3; i++) {
    uint32_t t = (elapsed + i * 450UL) % 1350UL;
    int16_t y = baseY - (int16_t)(t * 16UL / 1350UL);
    int16_t x = cx - 13 + i * 11 + (int16_t)(sinf((elapsed + i * 300UL) / 180.0f) * 2.0f);
    if (y >= STATUS_BAR_H) display.drawBitmap(x, y, HEART_BMP, 7, 6, SSD1306_WHITE);
  }
}

// Uyku "z z Z" animasyonu
void drawZzz(int16_t cx, int16_t baseY, uint32_t elapsed) {
  int16_t dir = (cx < SCREEN_WIDTH / 2) ? 1 : -1;
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  for (uint8_t i = 0; i < 3; i++) {
    uint32_t t = (elapsed + i * 800UL) % 2400UL;
    int16_t y = baseY - (int16_t)(t * 12UL / 2400UL);
    int16_t off = 10 + i * 5 + (int16_t)(t * 4UL / 2400UL);
    int16_t x = (dir > 0) ? cx + off : cx - off - 6;
    display.setCursor(x, y);
    display.print(i == 2 ? 'Z' : 'z');
  }
}

// İp yumağı + arkasından sarkan ip
void drawYarn(int16_t bx, int16_t by, int16_t dirAwayFromCat, uint32_t now) {
  display.drawBitmap(bx - 6, by, YARN_BMP, 12, 12, SSD1306_WHITE);
  int16_t wig = (int16_t)(sinf(now / 120.0f) * 2.0f);
  int16_t x0 = bx + dirAwayFromCat * 5, y0 = by + 9;
  int16_t x1 = x0 + dirAwayFromCat * 6, y1 = GROUND_Y - 1;
  int16_t x2 = x1 + dirAwayFromCat * 6, y2 = GROUND_Y - 2 + wig;
  display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
  display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
}

// Mama kabı + yenen balık (progress 0..1)
void drawBowlWithFish(int16_t bowlX, float progress, bool catOnLeft) {
  int16_t fishX = bowlX;
  int16_t fishY = GROUND_Y - 13;
  display.drawBitmap(fishX, fishY, FISH_BMP, 16, 8, SSD1306_WHITE);

  // Yenen kısmı kedi tarafından sil
  int16_t eatenW = (int16_t)(progress * 16.0f);
  if (eatenW > 0) {
    if (catOnLeft) display.fillRect(fishX, fishY, eatenW, 8, SSD1306_BLACK);
    else           display.fillRect(fishX + 16 - eatenW, fishY, eatenW, 8, SSD1306_BLACK);
  }

  // Kap
  display.fillRoundRect(bowlX, GROUND_Y - 6, 15, 6, 2, SSD1306_WHITE);
  display.drawFastHLine(bowlX - 1, GROUND_Y - 6, 17, SSD1306_WHITE);   // kenar
  display.drawFastHLine(bowlX + 2, GROUND_Y - 3, 11, SSD1306_BLACK);   // süs şeridi
}

// ============================================================================
//  OYUN FİZİĞİ — zıplama yüksekliği ve pati durumu
// ============================================================================
void getPlayJump(uint32_t elapsed, int16_t &jumpH, bool &pawUp, uint32_t &jumpIdx, uint32_t &phase) {
  jumpIdx = elapsed / JUMP_CYCLE_MS;
  phase   = elapsed % JUMP_CYCLE_MS;
  if (phase < JUMP_AIR_MS) {
    float f = (float)phase / (float)JUMP_AIR_MS;
    jumpH = (int16_t)(sinf(f * PI) * JUMP_HEIGHT_PX);
    pawUp = (phase > 80);
  } else {
    jumpH = 0;
    pawUp = false;
  }
}

// ============================================================================
//  DURUM YARDIMCILARI
// ============================================================================
bool isHungry() { return hunger >= HUNGRY_THRESHOLD; }

EyeType idleEyes(uint32_t now) {
  if (!timeReached(now, blinkUntilMs)) return EYE_BLINK;
  if (!timeReached(now, joyUntilMs) || !timeReached(now, happyFlashUntil)) return EYE_HAPPY;
  return EYE_OPEN;
}

SideCatParams makeSideParams() {
  SideCatParams p;
  p.eyes       = EYE_OPEN;
  p.mouth      = MOUTH_SMILE;
  p.legsMoving = false;
  p.legFrame   = 0;
  p.pawUp      = false;
  p.headDx     = 0;
  p.headDy     = 0;
  p.tailWave   = 0;
  return p;
}

// Yürüme adımı (hedefe 1 piksel yaklaş)
void stepToward(int16_t tx) {
  if (catX < tx)      { catX++; catFaceRight = true;  }
  else if (catX > tx) { catX--; catFaceRight = false; }
  stepCount++;
  legFrame = (stepCount / 3) % 2;     // her 3 pikselde bacak karesi değişir
}

// ============================================================================
//  DURUM GEÇİŞİ — her durumun giriş işlemleri burada
// ============================================================================
void enterState(CatState s, uint32_t now) {
  // Döngüsel titreşimleri (mırlama/çiğneme) durum değişince kes
  if (vibPattern && vibPattern->loop) vibStop();

  state        = s;
  stateStartMs = now;

  switch (s) {
    case STATE_WALKING: {
      // Mevcut konumdan en az 15 px uzak rastgele hedef seç
      do { targetX = (int16_t)random(CAT_MIN_X, CAT_MAX_X + 1); }
      while (abs(targetX - catX) < 15);
      catFaceRight   = (targetX > catX);
      lastWalkStepMs = now;
      break;
    }
    case STATE_HAPPY:
      idleDurationMs = (uint32_t)random(2000, 4500);
      nextLookMs     = now + (uint32_t)random(600, 1400);
      break;

    case STATE_BORED:
      boredArrived   = false;
      boredTargetX   = (catX < SCREEN_WIDTH / 2) ? CORNER_LEFT_X : CORNER_RIGHT_X; // yakın köşe
      lastWalkStepMs = now;
      break;

    case STATE_PLAYING:
      // Yumağın ekrana sığacağı yöne dön
      if (catFaceRight && catX + 42 > SCREEN_WIDTH - 1) catFaceRight = false;
      else if (!catFaceRight && catX - 42 < 0)           catFaceRight = true;
      lastLandedJump = -1;
      break;

    case STATE_EATING:
      // Mama kabının sığacağı yöne dön
      if (catFaceRight && catX + 35 > SCREEN_WIDTH - 1) catFaceRight = false;
      else if (!catFaceRight && catX - 35 < 0)           catFaceRight = true;
      eatStartHunger = hunger;
      vibPlay(VIB_EAT);
      break;

    case STATE_LOVE:
      vibPlay(VIB_PURR);
      break;
  }

  Serial.printf("[KEDI] Durum -> %s (aclik=%d)\n", STATE_NAMES[s], hunger);
}

// Etkileşimsiz geçen süre aşıldıysa sıkıl
bool checkBoredom(uint32_t now) {
  if ((now - lastInteractionMs) >= BORED_TIMEOUT_MS) {
    enterState(STATE_BORED, now);
    return true;
  }
  return false;
}

// ============================================================================
//  ZAMANLA DEĞİŞEN İHTİYAÇLAR
// ============================================================================
void updateHunger(uint32_t now) {
  if ((now - lastHungerTickMs) < HUNGER_TICK_MS) return;
  lastHungerTickMs += HUNGER_TICK_MS;

  if (state != STATE_EATING && hunger < 100) hunger++;

  // Eşik ilk kez aşıldığında bildirim titreşimi
  if (!wasHungry && isHungry()) {
    wasHungry = true;
#if VIB_HUNGER_ALERT
    if (!vibPattern || !vibPattern->loop) vibPlay(VIB_HUNGRY);
#endif
    Serial.println("[KEDI] Acikti!");
  } else if (!isHungry()) {
    wasHungry = false;
  }
}

void updateBlink(uint32_t now) {
  if (timeReached(now, nextBlinkMs)) {
    blinkUntilMs = now + 120;
    nextBlinkMs  = now + (uint32_t)random(2000, 5000);
  }
}

// ============================================================================
//  KULLANICI GİRDİLERİ
// ============================================================================
void handleInputs(uint32_t now) {
  touchHeld = touchIn.stable;

  // --- Dokunma: okşama ---
  if (touchIn.pressed) {
    lastInteractionMs = now;
    if (state != STATE_EATING) enterState(STATE_LOVE, now);
  }
  if (touchIn.released) touchReleasedMs = now;

  // --- Buton 1: oyun ---
  if (playBtn.pressed) {
    lastInteractionMs = now;
    if (state != STATE_EATING && state != STATE_PLAYING && !touchHeld) {
      enterState(STATE_PLAYING, now);
      vibPlay(VIB_TAP);
    }
  }

  // --- Buton 2: besleme ---
  if (feedBtn.pressed) {
    lastInteractionMs = now;
    if (state != STATE_EATING) enterState(STATE_EATING, now);
  }
}

// ============================================================================
//  DURUM MAKİNESİ — mantık güncellemesi
// ============================================================================
void updateState(uint32_t now) {
  uint32_t elapsed = now - stateStartMs;

  switch (state) {

    // ---------------- DOLAŞMA ----------------
    case STATE_WALKING: {
      if (checkBoredom(now)) break;
      uint32_t stepMs = isHungry() ? WALK_STEP_HUNGRY_MS : WALK_STEP_MS;
      if ((now - lastWalkStepMs) >= stepMs) {
        lastWalkStepMs = now;
        stepToward(targetX);
      }
      if (catX == targetX) enterState(STATE_HAPPY, now);
      break;
    }

    // ---------------- DURUP ETRAFA BAKMA ----------------
    case STATE_HAPPY: {
      if (checkBoredom(now)) break;
      if (timeReached(now, nextLookMs)) {
        catFaceRight = !catFaceRight;                       // bir sağa bir sola bak
        nextLookMs   = now + (uint32_t)random(700, 1500);
        if (random(0, 100) < 35) happyFlashUntil = now + 600; // ara sıra ^^
      }
      if (elapsed >= idleDurationMs) enterState(STATE_WALKING, now);
      break;
    }

    // ---------------- SIKILMA ----------------
    case STATE_BORED: {
      if (!boredArrived) {
        if ((now - lastWalkStepMs) >= WALK_STEP_BORED_MS) {
          lastWalkStepMs = now;
          stepToward(boredTargetX);
        }
        if (catX == boredTargetX) {
          boredArrived = true;
          boredSitMs   = now;
        }
      }
      // Bu durumdan sadece kullanıcı etkileşimiyle çıkılır
      break;
    }

    // ---------------- İP YUMAĞI ----------------
    case STATE_PLAYING: {
      int16_t jumpH; bool pawUp; uint32_t jumpIdx, phase;
      getPlayJump(elapsed, jumpH, pawUp, jumpIdx, phase);
      // Her inişte kısa haptik "tık"
      if (phase >= JUMP_AIR_MS && (int32_t)jumpIdx != lastLandedJump) {
        lastLandedJump = (int32_t)jumpIdx;
        vibPlay(VIB_TAP);
      }
      if (elapsed >= PLAY_DURATION_MS) {
        hunger += PLAY_HUNGER_COST;
        if (hunger > 100) hunger = 100;
        joyUntilMs = now + JOY_AFTER_MS;
        enterState(STATE_HAPPY, now);
      }
      break;
    }

    // ---------------- YEMEK ----------------
    case STATE_EATING: {
      float p = (float)elapsed / (float)EAT_DURATION_MS;
      if (p > 1.0f) p = 1.0f;
      hunger = (int16_t)(eatStartHunger * (1.0f - p));   // yedikçe açlık düşer
      if (elapsed >= EAT_DURATION_MS) {
        hunger     = 0;
        wasHungry  = false;
        joyUntilMs = now + JOY_AFTER_MS;
        enterState(STATE_HAPPY, now);
      }
      break;
    }

    // ---------------- SEVGİ ----------------
    case STATE_LOVE: {
      if (touchHeld) {
        lastInteractionMs = now;                           // okşandıkça sıkılmaz
      } else {
        if (vibPattern == &VIB_PURR) vibStop();            // dokunma bitti: mırlama durur
        if ((now - touchReleasedMs) >= LOVE_LINGER_MS) {
          joyUntilMs = now + JOY_AFTER_MS;
          enterState(STATE_HAPPY, now);
        }
      }
      break;
    }
  }
}

// ============================================================================
//  EKRAN — sabit parçalar
// ============================================================================
const char *moodLabel(uint32_t now) {
  (void)now;
  // Açlık sabit etiketle gösterilir (eski sürümdeki yanıp sönen değişim kaldırıldı)
  switch (state) {
    case STATE_WALKING: return isHungry() ? "ACIKTIM" : "GEZIYOR";
    case STATE_HAPPY:   return isHungry() ? "ACIKTIM" : "MUTLU";
    case STATE_BORED:   return isHungry() ? "ACIKTIM" : "SIKILDIM";
    case STATE_PLAYING: return "OYUN!";
    case STATE_EATING:  return "NYAM NYAM";
    case STATE_LOVE:    return "MIRR <3";
  }
  return "";
}

void drawStatusBar(uint32_t now) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Tokluk çubuğu (100 - açlık)
  display.setCursor(0, 1);
  display.print("TOK");
  display.drawRect(20, 1, 32, 7, SSD1306_WHITE);
  int16_t fill = (int16_t)((100 - hunger) * 28 / 100);
  bool blinkOff = isHungry() && ((now / 400) % 2 == 0);
  if (fill > 0 && !blinkOff) display.fillRect(22, 3, fill, 3, SSD1306_WHITE);

#if INPUT_DEBUG
  // Giriş göstergeleri: [T]ouch [O]yun [B]esleme — dolu kutu = o an aktif okunuyor
  const DebouncedInput *ins[3] = { &touchIn, &playBtn, &feedBtn };
  for (uint8_t i = 0; i < 3; i++) {
    int16_t x = 55 + i * 6;
    if (ins[i]->lastRaw) display.fillRect(x, 1, 5, 7, SSD1306_WHITE);
    else                 display.drawRect(x, 1, 5, 7, SSD1306_WHITE);
  }
#endif

  // Ruh hali etiketi (sağa yaslı)
  const char *label = moodLabel(now);
  int16_t w = (int16_t)strlen(label) * 6;
  display.setCursor(SCREEN_WIDTH - w, 1);
  display.print(label);

  display.drawFastHLine(0, STATUS_BAR_H - 1, SCREEN_WIDTH, SSD1306_WHITE);
}

void drawGround() {
  for (int16_t x = 0; x < SCREEN_WIDTH; x += 3) display.drawPixel(x, 63, SSD1306_WHITE);
}

// ============================================================================
//  EKRAN — durumlara göre sahne çizimi
// ============================================================================
void drawWalkingOrHappy(uint32_t now) {
  bool walking = (state == STATE_WALKING);

  SideCatParams p = makeSideParams();
  p.eyes       = idleEyes(now);
  p.mouth      = isHungry() ? MOUTH_SAD : MOUTH_SMILE;
  p.legsMoving = walking;
  p.legFrame   = legFrame;
  p.headDy     = (walking && legFrame) ? 1 : 0;           // yürürken hafif kafa sallama
  p.tailWave   = (int8_t)(sinf(now / (walking ? 160.0f : 260.0f)) * 2.0f);
  drawCatSide(catX, GROUND_Y, catFaceRight, p);

  int16_t headX   = catFaceRight ? catX + 10 : catX - 10;
  int16_t headTop = GROUND_Y - 28;

  if (isHungry() && ((now / 1000) % 3 != 2)) {
    drawHungerBubble(headX, headTop, catFaceRight);        // 2 sn göster, 1 sn gizle
  } else if (!timeReached(now, joyUntilMs)) {
    int16_t bob = ((now / 150) % 4 < 2) ? 0 : 1;
    display.drawBitmap(headX - 3, headTop - 9 - bob, HEART_BMP, 7, 6, SSD1306_WHITE);
  }
}

void drawBored(uint32_t now) {
  if (!boredArrived) {
    // Köşeye doğru ağır ve sıkkın yürüyüş
    SideCatParams p = makeSideParams();
    p.eyes       = EYE_SLEEPY;
    p.mouth      = MOUTH_FLAT;
    p.legsMoving = true;
    p.legFrame   = legFrame;
    p.headDy     = 1;                                      // kafa hafif düşük
    p.tailWave   = (int8_t)(sinf(now / 400.0f) * 1.0f);
    drawCatSide(catX, GROUND_Y, catFaceRight, p);
    return;
  }

  // Köşede oturuyor: uyuklama + periyodik esneme
  uint32_t sitT = now - boredSitMs;
  uint32_t yawnPhase = sitT % 7000UL;
  EyeType   eyes;
  MouthType mouth;
  if (yawnPhase < 900) {                       // esneme
    eyes  = EYE_BLINK;
    mouth = MOUTH_OPEN;
  } else {
    eyes  = ((sitT % 3000UL) < 1200) ? EYE_BLINK : EYE_SLEEPY;   // gözler kapanıp açılır
    mouth = MOUTH_FLAT;
  }
  int8_t wave = (int8_t)(sinf(now / 600.0f) * 1.5f);
  drawCatSitting(catX, GROUND_Y, eyes, mouth, wave, false);

  bool toRight = (catX < SCREEN_WIDTH / 2);
  if (isHungry() && ((now / 1000) % 3 != 2)) drawHungerBubble(catX, GROUND_Y - 32, toRight);
  else                                       drawZzz(catX, GROUND_Y - 36, sitT);
}

void drawPlaying(uint32_t now) {
  uint32_t elapsed = now - stateStartMs;
  int16_t jumpH; bool pawUp; uint32_t jumpIdx, phase;
  getPlayJump(elapsed, jumpH, pawUp, jumpIdx, phase);

  // Yumak: kediden uzaklaşıp yaklaşır ve zıplar
  int16_t dir      = catFaceRight ? 1 : -1;
  int16_t ballDist = 24 + (int16_t)(fabsf(sinf(elapsed / 600.0f)) * 10.0f);
  int16_t ballX    = catX + dir * ballDist;
  int16_t ballY    = GROUND_Y - 12 - (int16_t)(fabsf(sinf(elapsed / 260.0f)) * 7.0f);
  drawYarn(ballX, ballY, dir, now);

  SideCatParams p = makeSideParams();
  bool airborne = (jumpH > 0);
  p.eyes       = airborne ? EYE_HAPPY : EYE_OPEN;
  p.mouth      = airborne ? MOUTH_OPEN : MOUTH_SMILE;
  p.pawUp      = pawUp;
  p.tailWave   = (int8_t)(sinf(now / 90.0f) * 3.0f);       // heyecanlı kuyruk
  drawCatSide(catX, GROUND_Y - jumpH, catFaceRight, p);
}

void drawEating(uint32_t now) {
  uint32_t elapsed = now - stateStartMs;
  float p = (float)elapsed / (float)EAT_DURATION_MS;
  if (p > 1.0f) p = 1.0f;
  bool chew = ((elapsed / 220) % 2) == 0;

  SideCatParams cp = makeSideParams();
  cp.eyes     = EYE_HAPPY;
  cp.mouth    = chew ? MOUTH_OPEN : MOUTH_FLAT;
  cp.headDx   = 2;                                         // kafa kaba doğru eğik
  cp.headDy   = 7 + (chew ? 0 : 1);                        // çiğnerken hafif sallanma
  cp.tailWave = (int8_t)(sinf(now / 300.0f) * 2.0f);
  drawCatSide(catX, GROUND_Y, catFaceRight, cp);

  int16_t bowlX = catFaceRight ? catX + 19 : catX - 33;
  drawBowlWithFish(bowlX, p, catFaceRight);
}

void drawLove(uint32_t now) {
  uint32_t elapsed = now - stateStartMs;
  int8_t wave = (int8_t)(sinf(now / 120.0f) * 2.0f);      // mutlu kuyruk
  drawCatSitting(catX, GROUND_Y, EYE_HEART, MOUTH_SMILE, wave, true);
  drawFloatingHearts(catX, GROUND_Y - 36, elapsed);
}

// Canlı pin test ekranı (PIN_TEST_MODE 1 iken)
void drawPinTest() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("PIN TEST  (seviye)");
  display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);

  const DebouncedInput *ins[3] = { &touchIn, &playBtn, &feedBtn };
  bool any = false;
  for (uint8_t i = 0; i < 3; i++) {
    int16_t y = 14 + i * 12;
    display.setCursor(0, y);
    display.printf("%-7s GP%-2d %-4s", ins[i]->name, ins[i]->pin,
                   digitalRead(ins[i]->pin) ? "HIGH" : "LOW");
    if (ins[i]->stable) {
      display.fillRect(104, y - 1, 24, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(107, y);
      display.print("BAS");
      display.setTextColor(SSD1306_WHITE);
      any = true;
    }
  }
  display.setCursor(0, 54);
  display.print("Basinca BAS yazmali");
  display.display();
  vibWrite(any);                  // herhangi bir giriş aktifken motor da çalışır
}

void drawSplash(uint32_t now) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(28, 6);
  display.print("MIYAV!");
  int8_t wave = (int8_t)(sinf(now / 150.0f) * 2.0f);
  drawCatSitting(64, GROUND_Y, EYE_HAPPY, MOUTH_SMILE, wave, true);
  drawGround();
  display.display();
}

void renderScene(uint32_t now) {
  display.clearDisplay();
  drawStatusBar(now);
  drawGround();

  switch (state) {
    case STATE_WALKING:
    case STATE_HAPPY:   drawWalkingOrHappy(now); break;
    case STATE_BORED:   drawBored(now);          break;
    case STATE_PLAYING: drawPlaying(now);        break;
    case STATE_EATING:  drawEating(now);         break;
    case STATE_LOVE:    drawLove(now);           break;
  }

  display.display();
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Titreşim çıkışı (önce kapalı konuma çek)
  pinMode(VIBRATION_PIN, OUTPUT);
  vibWrite(false);

  // Girişler
  inputBegin(touchIn, "TOUCH",   TOUCH_PIN,    INPUT,        TOUCH_ACTIVE_HIGH ? true : false);
  inputBegin(playBtn, "OYUN",    BTN_PLAY_PIN, INPUT_PULLUP, false);   // GND'ye bas -> LOW
  inputBegin(feedBtn, "BESLEME", BTN_FEED_PIN, INPUT_PULLUP, false);   // GND'ye bas -> LOW

  reportIdleLevel(touchIn, false);
  reportIdleLevel(playBtn, true);
  reportIdleLevel(feedBtn, true);

  // I2C + OLED
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000UL);
  // periphBegin=false: Wire'ı yukarıda özel pinlerle zaten başlattık
  displayOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR, true, false);

  if (!displayOk) {
    Serial.println("[HATA] SSD1306 bulunamadi! Adres/kablolamayi kontrol et.");
    vibPlay(VIB_ERROR);           // ekran yoksa motor hata deseni titreşir
    return;
  }

  display.clearDisplay();
  display.display();

  uint32_t now = millis();
  catX         = 64;
  nextBlinkMs  = now + 2000;
  Serial.println("[KEDI] Basladi.");
}

// ============================================================================
//  LOOP — tamamen non-blocking
// ============================================================================
void loop() {
  uint32_t now = millis();

  vibUpdate(now);                 // titreşim deseni her zaman ilerler
  if (!displayOk) return;

  inputUpdate(touchIn, now);
  inputUpdate(playBtn, now);
  inputUpdate(feedBtn, now);

#if PIN_TEST_MODE
  if ((now - lastFrameMs) >= 50) { lastFrameMs = now; drawPinTest(); }
  return;
#endif

  // Açılış ekranı
  if (!splashDone) {
    if (now < SPLASH_MS) {
      if ((now - lastFrameMs) >= FRAME_INTERVAL_MS) {
        lastFrameMs = now;
        drawSplash(now);
      }
      return;
    }
    splashDone        = true;
    lastInteractionMs = now;
    lastHungerTickMs  = now;
    enterState(STATE_WALKING, now);
  }

  updateHunger(now);
  handleInputs(now);
  updateState(now);
  updateBlink(now);

  if ((now - lastFrameMs) >= FRAME_INTERVAL_MS) {
    lastFrameMs = now;
    renderScene(now);
  }
}
