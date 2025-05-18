#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define MPU_addr 0x68
#define buzzerPin 9
#define buttonSet 2   // interrupt destekli pin
#define buttonUp 3
#define buttonDown 4

#define EEPROM_ADDR_ROLL 0
#define EEPROM_ADDR_PITCH 10

volatile bool settingMode = false;
float xa, ya, za, roll, pitch;
float thresholdRoll = 1.0;
float thresholdPitch = 4.5;
bool alarmState = false;
int settingIndex = 0;
unsigned long lastDebounce = 0;

void loadThresholdsFromEEPROM() {
  EEPROM.get(EEPROM_ADDR_ROLL, thresholdRoll);
  EEPROM.get(EEPROM_ADDR_PITCH, thresholdPitch);
  if (isnan(thresholdRoll) || isnan(thresholdPitch)) {
    thresholdRoll = 1.0;
    thresholdPitch = 4.5;
  }
}

void saveThresholdsToEEPROM() {
  EEPROM.put(EEPROM_ADDR_ROLL, thresholdRoll);
  EEPROM.put(EEPROM_ADDR_PITCH, thresholdPitch);
}

void alarmMode() {
  for (int i = 0; i < 3; i++) {
    for (int f = 1500; f <= 3000; f += 200) {
      tone(buzzerPin, f);
      delay(40);
    }
    for (int f = 3000; f >= 1500; f -= 200) {
      tone(buzzerPin, f);
      delay(40);
    }
    delay(100);
  }
  noTone(buzzerPin);
  delay(500);
}

void readMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6);

  int t = Wire.read(); xa = (t << 8) | Wire.read();
  t = Wire.read(); ya = (t << 8) | Wire.read();
  t = Wire.read(); za = (t << 8) | Wire.read();

  roll = atan2(ya , za) * 180.0 / PI;
  pitch = atan2(-xa , sqrt(ya * ya + za * za)) * 180.0 / PI;
}

void handleSettingMode() {
  if (digitalRead(buttonUp) == LOW) {
    if (settingIndex == 0) thresholdRoll += 0.1;
    else thresholdPitch += 0.1;
    saveThresholdsToEEPROM();
    delay(150);
  }

  if (digitalRead(buttonDown) == LOW) {
    if (settingIndex == 0) thresholdRoll -= 0.1;
    else thresholdPitch -= 0.1;
    saveThresholdsToEEPROM();
    delay(150);
  }

  if (digitalRead(buttonSet) == LOW && millis() - lastDebounce > 300) {
    settingIndex = (settingIndex + 1) % 2;
    lastDebounce = millis();
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("== Ayar Modu ==");
  display.setCursor(0, 20);
  display.print("Roll Threshold: ");
  display.println(thresholdRoll, 1);
  display.print("Pitch Threshold: ");
  display.println(thresholdPitch, 1);
  display.setCursor(0, 50);
  display.print("Degistir: ");
  display.println(settingIndex == 0 ? "Roll" : "Pitch");
  display.display();
}

// Interrupt fonksiyonu
void toggleSettingMode() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastInterruptTime > 500) {
    settingMode = !settingMode;
    lastInterruptTime = currentTime;
  }
}


void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonSet, INPUT_PULLUP);
  pinMode(buttonUp, INPUT_PULLUP);
  pinMode(buttonDown, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(buttonSet), toggleSettingMode, FALLING);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  loadThresholdsFromEEPROM();
}

void loop() {
  if (settingMode) {
    handleSettingMode();
    return;
  }

  readMPU();

  if (abs(roll) > thresholdRoll || abs(pitch) > thresholdPitch) {
    alarmState = true;
  }

  if (alarmState) {
    alarmMode();
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Roll: "); display.println(roll, 1);
  display.print("Pitch: "); display.println(pitch, 1);
  display.print("T-Roll: "); display.println(thresholdRoll, 1);
  display.print("T-Pitch: "); display.println(thresholdPitch, 1);
  display.display();

  delay(50);
}
