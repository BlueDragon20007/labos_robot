#include "SSD1306.h"
#include <LCD_I2C.h>
#include <HCSR04.h>

#define TRIGGER_PIN 9
#define ECHO_PIN 10
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

unsigned long currentTime;
float distance = 30;
float newDistance = 0;
int distanceDelay = 100;

#define SCREEN_ADDRESS 0x3C
SSD1306 display(SCREEN_ADDRESS, &distance);

void setup() {
  Serial.begin(115200);
  display.begin();
}

void loop() {
  currentTime = millis();

  getDistance();

  display.update();
}

void getDistance() {
  static unsigned long lastTime = 0;
  if (currentTime - lastTime >= distanceDelay) {
    lastTime = currentTime;
    newDistance = hc.dist();
    if (newDistance > 0 && newDistance < 400) {
      distance = newDistance;
    }
    Serial.print("Distance:");
    Serial.print(distance);
    Serial.println("cm");
  }
}