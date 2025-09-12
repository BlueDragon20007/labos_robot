#pragma once

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
//#define SCREEN_ADDRESS 0x3C

extern unsigned long currentTime;

class SSD1306 {
private:
  uint8_t _address;
  Adafruit_SSD1306 _display;
  unsigned long _lastTime = 0;
  int _rate = 100;
  int _sunHeight;
  int _sunPosition = 105;
  float* _distance;
  int _minDist = 10;
  int _maxDist = 40;
  int _sunSize = 8;

public:
  SSD1306(uint8_t address, float* distance);

  void begin();

  void drawBaseDrawing();

  void updateSun();

  void update();
};