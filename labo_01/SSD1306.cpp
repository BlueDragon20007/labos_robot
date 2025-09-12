#include "SSD1306.h"

SSD1306::SSD1306(uint8_t address, float* distance)
  : _address(address), _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET), _distance(distance) {}

void SSD1306::begin() {
  if (!_display.begin(SSD1306_SWITCHCAPVCC, _address)) {
    while (true);
  }

  _display.clearDisplay();
  _display.display();
  drawBaseDrawing();
}

void SSD1306::drawBaseDrawing() {
  _display.fillRect(10, 30, 25, 25, 1);
  _display.fillRect(20, 45, 5, 10, 0);
  _display.fillTriangle(7, 30, 22, 20, 37, 30, 1);

  _display.setTextSize(2);
  _display.setTextColor(1);
  _display.setCursor(0, 0);
  _display.print("Doyon");

  _display.display();
}

void SSD1306::updateSun(){
  _sunHeight = map(constrain(*_distance, _minDist, _maxDist), _minDist, _maxDist, 0, 64);
  _display.fillRect(64, 0, 64, 128, 0); //Effacer moitie droite de l'ecran
  _display.fillCircle(_sunPosition, _sunHeight, _sunSize, 1);
  _display.display();
}

void SSD1306::update() {
  if (currentTime - _lastTime >= _rate) {
    _lastTime = currentTime;
    updateSun();
  }
}