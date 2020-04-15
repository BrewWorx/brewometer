#include <Arduino.h>
#include <AnalogPHMeter.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define LCDWidth          u8g2.getDisplayWidth()
#define ALIGN_CENTER(t)   ((LCDWidth - (u8g2.getUTF8Width(t))) / 2)
#define ALIGN_RIGHT(t)    (LCDWidth -  u8g2.getUTF8Width(t))
#define ALIGN_LEFT        0

OneWire oneWire(2);
DallasTemperature sensors(&oneWire);
AnalogPHMeter pHSensor(A0);

unsigned int rawLowAddress = 0;
unsigned int rawHighAddress = 4;
unsigned int pHCalibrationValueAddress = 8;

PHCalibrationValue pHCalibrationValue;
float RawLow, RawHigh;
float ReferenceHigh = 99.6; // Boiling point at 110m above sea level
float ReferenceLow = 0;
float RawRange = RawHigh - RawLow;
float ReferenceRange = ReferenceHigh - ReferenceLow;

int mode = 0;
int calibrationStep = 0;

struct Button {
  int state = LOW;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;
};

Button btn1;  // Mode
Button btn2;  // Decrease (-)
Button btn3;  // Increase (+)
Button btn4;  // Alarm / Calibration

void drawDisplay(char* t) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso38_tn);
  u8g2.drawStr(ALIGN_CENTER(t), 10, t);
  u8g2.sendBuffer();
}

void readButtonStates() {
  int reading = digitalRead(3);
  if (reading != btn1.state) {
    btn1.lastDebounceTime = millis();
  }

  reading = digitalRead(4);
  if (reading != btn2.state) {
    btn2.lastDebounceTime = millis();
  }

  reading = digitalRead(5);
  if (reading != btn4.state) {
    btn3.lastDebounceTime = millis();
  }

  reading = digitalRead(6);
  if (reading != btn4.state) {
    btn4.lastDebounceTime = millis();
  }
}

void setup() {
  EEPROM.get(pHCalibrationValueAddress, pHCalibrationValue);
  EEPROM.get(rawLowAddress, RawLow);
  EEPROM.get(rawHighAddress, RawHigh);

  pHSensor.initialize(pHCalibrationValue);
  sensors.begin();
  u8g2.begin();
}

void loop() {
  readButtonStates();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso38_tn);

  // Temperature mode
  if(mode == 0) {
    sensors.requestTemperatures();

    float RawValue = sensors.getTempCByIndex(0);

    switch (calibrationStep) {
      // Get "triple-point" bath temperature
      case 1:
        RawLow = RawValue;
        break;

      // Get boiling water temperature
      case 2:
        RawHigh = RawValue;
        break;

      // Save settings
      case 3:
        EEPROM.put(rawLowAddress, RawLow);
        EEPROM.put(rawHighAddress, RawHigh);
        break;
    }

    if(calibrationStep > 0) {
      u8g2.print(RawValue);
    } else {
      float tempC = (((RawValue - RawLow) * ReferenceRange) / RawRange) + ReferenceLow;
      u8g2.print(tempC);
    }

  } else {
    pHSensor.singleReading().getpH();

    switch (calibrationStep) {

      // Set mid calibration point
      case 1:
        pHSensor.calibrationMid(6.860f);
        break;

      // Set low calibration point
      case 2:
        pHSensor.calibrationLow(4.010f);
        break;

      // Set high calibration point
      case 3:
        pHSensor.calibrationHigh(9.180f);
        break;

      // Save settings
      case 4:
        EEPROM.put(pHCalibrationValueAddress, pHSensor.getCalibrationValue());
        break;
    }

    u8g2.print(pHSensor.getpH());
  }

  u8g2.sendBuffer();
}