#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MPU_addr 0x68
#define BUZZER_PIN 9
#define RESET_PIN 6

#define SAMPLES 64
#define SAMPLING_FREQUENCY 100

bool alarmTetiklendi = false;
unsigned long alarmStartTime = 0;
const unsigned long alarmSuresi = 300000; // 5 dakika

ArduinoFFT<float> FFT = ArduinoFFT<float>();
float vReal[SAMPLES];
float vImag[SAMPLES];

float fftMagnitude = 0;
float fftThreshold = 10000.0; // Bu değeri sarsıntı hassasiyetine göre ayarla

class KalmanFilter {
  public:
    KalmanFilter() {
      Q_angle = 0.001;
      Q_bias = 0.003;
      R_measure = 0.03;
      angle = bias = rate = 0.0;
      P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0;
    }

    float update(float newAngle, float newRate, float dt) {
      if (dt <= 0) return angle;

      rate = newRate - bias;
      angle += dt * rate;

      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      float S = P[0][0] + R_measure;
      float K[2] = { P[0][0] / S, P[1][0] / S };

      float y = newAngle - angle;
      angle += K[0] * y;
      bias += K[1] * y;

      float P00_temp = P[0][0], P01_temp = P[0][1];
      P[0][0] -= K[0] * P00_temp;
      P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp;
      P[1][1] -= K[1] * P01_temp;

      return angle;
    }

  private:
    float Q_angle, Q_bias, R_measure;
    float angle, bias, rate;
    float P[2][2];
};

KalmanFilter kalmanRoll, kalmanPitch;

int16_t xa, ya, za;
float roll = 0, pitch = 0;
unsigned long lastTime;
float rollThreshold = 7.0;
float pitchThreshold = 5.0;
bool isOledPresent = false;

void alarmMode() {
  for (int i = 0; i < 3; i++) {
    for (int f = 1500; f <= 3000; f += 200) {
      tone(BUZZER_PIN, f);
      delay(40);
    }
    for (int f = 3000; f >= 1500; f -= 200) {
      tone(BUZZER_PIN, f);
      delay(40);
    }
    delay(100);
  }
  noTone(BUZZER_PIN);
  delay(500);
}

float gurultuToplam = 0;
const int gurultuOrnekSayisi = 20;
bool kalibreEdildi = false;

void kalibrasyonYap() {
  for (int i = 0; i < gurultuOrnekSayisi; i++) {
    doFFT();
    gurultuToplam += fftMagnitude;
  }
  float ortgurultu = gurultuToplam / gurultuOrnekSayisi;
  fftThreshold = ortgurultu * 1.5; // güvenlik çarpanı
  kalibreEdildi = true;
  Serial.print("Kalibrasyon Tamam. Eşik: ");
  Serial.println(fftThreshold);
}



void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    isOledPresent = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Deprem Sistemi");
    display.println("Baslatiliyor...");
    display.display();
  } else {
    Serial.println(F("OLED bulunamadi! Seri ekran kullanilacak."));
  }

  kalibrasyonYap();

  lastTime = millis();
}

void readMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6);

  xa = (Wire.read() << 8) | Wire.read();
  ya = (Wire.read() << 8) | Wire.read();
  za = (Wire.read() << 8) | Wire.read();

  float rawRoll = atan2(ya, za) * 180.0 / PI;
  float rawPitch = atan2(-xa, sqrt(ya * ya + za * za)) * 180.0 / PI;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  roll = kalmanRoll.update(rawRoll, 0, dt);
  pitch = kalmanPitch.update(rawPitch, 0, dt);
}

void doFFT() {
  for (int i = 0; i < SAMPLES; i++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // X akselerometre verisi
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 2);
    int16_t x = (Wire.read() << 8) | Wire.read();
    vReal[i] = x;
    vImag[i] = 0;
    delay(1000 / SAMPLING_FREQUENCY);
  }

  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  double peak = 0;
  int index = 0;
  for (int i = 1; i < SAMPLES / 2; i++) {
    if (vReal[i] > peak) {
      peak = vReal[i];
      index = i;
    }
  }

  fftMagnitude = peak; // git-gel şiddeti
  double freq = index * SAMPLING_FREQUENCY / (double)SAMPLES;

  Serial.print("Peak Freq: ");
  Serial.print(freq, 1);
  Serial.print(" Hz, Mag: ");
  Serial.println(fftMagnitude, 1);

  if (isOledPresent) {
  display.setCursor(0, 32);
  if (fftMagnitude > fftThreshold)
    display.print("ALARM: FFT Yüksek!");
  else if (abs(roll) > rollThreshold)
    display.print("ALARM: Roll!");
  else if (abs(pitch) > pitchThreshold)
    display.print("ALARM: Pitch!");
}
}

void loop() {
  readMPU();

  if (digitalRead(RESET_PIN) == LOW) {
    alarmTetiklendi = false;
    noTone(BUZZER_PIN);
  }

  if (isOledPresent) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Roll: ");
    display.print(roll, 1);
    display.print(" deg");

    display.setCursor(0, 16);
    display.print("Pitch: ");
    display.print(pitch, 1);
    display.print(" deg");
  } else {
    Serial.print("Roll: ");
    Serial.print(roll, 1);
    Serial.print(" deg, Pitch: ");
    Serial.print(pitch, 1);
    Serial.println(" deg");
  }

  doFFT(); // önce frekansı oku

  if (alarmTetiklendi) {
    if (millis() - alarmStartTime < alarmSuresi) {
      alarmMode();
    } else {
      noTone(BUZZER_PIN);
    }

    if (isOledPresent) {
      display.setCursor(0, 32);
      display.print("!!! ALARM AKTIF !!!");
    } else {
      Serial.println("!!! ALARM AKTIF !!!");
    }
  } else {
    if (abs(roll) > rollThreshold || abs(pitch) > pitchThreshold || fftMagnitude > fftThreshold) {
      alarmTetiklendi = true;
      alarmStartTime = millis();
      alarmMode();
    }
  }

  if (isOledPresent) display.display();
}
