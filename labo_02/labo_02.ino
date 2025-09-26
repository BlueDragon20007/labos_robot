// Liam Doyon - 6307713

#include <MeAuriga.h>

#define LEDNUM  12
#define LEDPIN  44
#define RINGALLLEDS 0

MeRGBLed led( PORT0, LEDNUM );

MeUltrasonicSensor ultraSensor(PORT_8);

enum DangerState {
  STOP,
  REVERSE,
  PIVOT
};

enum State {
  NORMAL,
  RALENTI, 
  DANGER, 
  RONDE
};

State state = NORMAL;

int frontLeds[7] = {1, 2, 3, 4, 5, 6, 12};
int backLeds[7] = {6, 7, 8, 9, 10, 11, 12};

float dist;
int distRalenti = 80;
int distDanger = 40;
int delayRonde = 10000;
int blinkSpeed = 125;
bool blinkState = false;
int maxPwm = 255;
int normalPwm = 0.7 * maxPwm;
int slowPwm = 0.5 * normalPwm;
int ledIntensity = 50;

// Moteur gauche
const int m1_pwm = 11;
const int m1_in1 = 48;
const int m1_in2 = 49;

// Moteur droit
const int m2_pwm = 10;
const int m2_in1 = 46;
const int m2_in2 = 47;

unsigned long currentTime = 0;

void setup() {
  Serial.begin(115200);

  led.setpin(LEDPIN);

  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);

  dist = ultraSensor.distanceCm();
}

void loop() {
  currentTime = millis();

  getDistance();

  showDistance();

  manageState();

  ledTask();

  driveManager();
}

void driveManager() {
  switch(state) {
    case NORMAL:
      normalTask();
    break;
    case RALENTI:
      ralentiTask();
    break;
    case DANGER:
      dangerTask();
    break;
    case RONDE:
      rondeTask();
    break;
  }
}

void normalTask() {
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, normalPwm);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, normalPwm);
}

void ralentiTask() {
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, slowPwm);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, slowPwm);
}

void dangerTask() {
  static DangerState dangerState = STOP;
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  static int stopDelay = 500;
  static int reverseDelay = 1000;
  static int pivotDelay = 600;

  if (firstTime) {
    firstTime = false;
    lastTime = currentTime;
  }

  switch(dangerState) {
    case STOP:
      if (currentTime - lastTime < stopDelay) {
        analogWrite(m1_pwm, 0);
        analogWrite(m2_pwm, 0);
      } else {
        lastTime = currentTime;
        dangerState = REVERSE;
      } 
    break;
    case REVERSE:
    if (currentTime - lastTime < reverseDelay) {
      digitalWrite(m1_in2, HIGH);
      digitalWrite(m1_in1, LOW);
      analogWrite(m1_pwm, slowPwm);

      digitalWrite(m2_in2, LOW);
      digitalWrite(m2_in1, HIGH);
      analogWrite(m2_pwm, slowPwm);
    } else {
      lastTime = currentTime;
      dangerState = PIVOT;
    }
    break;
    case PIVOT:
      if (currentTime - lastTime < pivotDelay) {
        digitalWrite(m1_in2, LOW);
        digitalWrite(m1_in1, HIGH);
        analogWrite(m1_pwm, normalPwm);

        digitalWrite(m2_in2, LOW);
        digitalWrite(m2_in1, HIGH);
        analogWrite(m2_pwm, normalPwm);
      } else {
        state = NORMAL;
        dangerState = STOP;
        firstTime = true;
      }
    break;
  }
}

void rondeTask() {
  static unsigned long lastTime = 0;
  static int rondeDelay = 2000;
  static bool firstTime = true;

  if (firstTime) {
    lastTime = currentTime;
    analogWrite(m1_pwm, 0);
    analogWrite(m2_pwm, 0);
    firstTime = false;
  }

  if (currentTime - lastTime >= rondeDelay) {
    state = NORMAL;
    firstTime = true;
  }
}

void ledTask() {
  led.setColor(0, 0, 0);
  switch(state) {
    case NORMAL:
      for (int l : backLeds) {
        led.setColor(l, 0, ledIntensity, 0);
      }
    break;
    case RALENTI:
      for (int l : frontLeds) {
        led.setColor(l, 0, 0, ledIntensity);
      }
    break;
    case DANGER:
      led.setColor(ledIntensity, 0, 0);
    break;
    case RONDE:
      static unsigned long lastBlink = 0;

      if (currentTime - lastBlink >= blinkSpeed) {
        blinkState = !blinkState;
        lastBlink = currentTime;
      }

      if (blinkState) {
        led.setColor(0, ledIntensity, 0); 
      } else {
        led.setColor(0, 0, 0);
      }
    break;
  }
  led.show();
}

void manageState() {
  static unsigned long lastDetectTime = 0;
  static bool firstNormal = true;

  if (state == DANGER || state == RONDE) return;

  if (currentTime - lastDetectTime >= delayRonde) {
    lastDetectTime = currentTime;
    state = RONDE;
    firstNormal = true;
  } else if (dist < distDanger) {
    state = DANGER;
    lastDetectTime = currentTime;
  } else if (dist < distRalenti) {
    state = RALENTI;
    lastDetectTime = currentTime;
  } else {
    state = NORMAL;
    if (firstNormal) {
      lastDetectTime = currentTime;
      firstNormal = false;
    }
  }
} 

void showDistance() {
  static unsigned long lastTime = 0;
  static int delay = 250;

  if (currentTime - lastTime < delay) return;

  lastTime = currentTime;

  Serial.print("Distance : ");
  Serial.print(dist);
  Serial.println("cm"); 
  // Serial.println(state);
}

void getDistance() {
  static unsigned long lastTime = 0;
  static int delay = 100;

  if (currentTime - lastTime < delay) return; 

  lastTime = currentTime;

  dist = ultraSensor.distanceCm();
}