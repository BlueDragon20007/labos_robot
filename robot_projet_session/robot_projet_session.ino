#include <Adafruit_seesaw.h>
#include <MeAuriga.h>
#include <ArduinoJson.h>

//École: {"device_name": "Makeblock_LE001b10672e24"}
//Maison: {"device_name": "Makeblock_LE001b1063e3be"}

#define NB_IR 5

struct Capteur {
  int valMin = 1023;
  int valMax = 0;
  int val;
  int normal;
  float seuil;
  bool onLine;
};

Capteur capteurs[5];

// bool onLine[NB_IR] = {false, false, false, false, false};

Adafruit_seesaw ss;

int sensorValues[NB_IR];

#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6

#define BUZZER_PIN 45
#define LED_RING_PIN 44

#define AURIGARINGLEDNUM 12
#define RINGALLLEDS 0

MeBuzzer buzzer;

MeRGBLed led_ring(0, AURIGARINGLEDNUM);

int topLeftLeds[4] = {12, 1, 2, 3};
int topRightLeds[4] = {3, 4, 5, 6};
int backLeds[7] = {6, 7, 8, 9, 10, 11, 12};
int frontLeds[5] = {1, 2, 3, 4, 5};

MeGyro gyro(0, 0x69);

MeUltrasonicSensor ultraSensor(PORT_8);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

unsigned long currentTime = 0;
unsigned long chronoStart = 0;
unsigned long chronoEnd = 0;
unsigned long missionBeginning = 0;
bool debugMode = false;

short speed = 200;
short lineSpeed = 110; // 115 - 125
float distance;

bool straightFirstRun = true;

float distToGo = 0;
float startPosAuto = 0;
bool autoRunning = false;

String currentCommand;

bool isCalibrating = false;
bool firstTurn = true;
bool firstLine;
bool firstSpin = true;
bool raceEnded = false;
bool beep = false;

short checkPoint = 0;

bool possessPackage = false;
bool arrived = false;

enum State {
  AUTO,
  MANUAL,
  LINE,
  CALIBRATION,
  MISSION,
  END
};

State state = MANUAL;

enum LineState {
  FOLLOW,
  CAREFUL,
  INTERSECTION
};

LineState lineState = FOLLOW;

enum IntState {
  ADVANCE,
  SPIN,
  FINAL,
  PARKING
};

IntState intState = ADVANCE;

enum MissionState {
  START,
  PACKAGE,
  DELIVER,
  RACE
};

MissionState missionState = START;

enum LedState {
  OFF,
  THREE,
  TWO,
  ONE,
  GO
};

LedState ledState = THREE;

enum FindLineState {
  FIND,
  CENTER,
  DONE
};

FindLineState findLineState = FIND;

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();;
  }
}

void leftEncoderInterrupt(void) {
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
    encoderLeft.pulsePosPlus();
  }
}

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  
  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);
  
  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);
  
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
  
  // DÉBUT : Ne pas modifier ce code!
  // Configuration de la fréquence du PWM
  // Copier-coller ce code si on désire
  // travailler avec les encodeurs
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // FIN : Ne pas modifier ce code!
}

// ************* DÉBUT ************

void setup() {
  Serial.begin(115200);
  ss.begin();
  buzzer.setpin(BUZZER_PIN);
  led_ring.setpin(LED_RING_PIN);
  encoderConfig();
  gyro.begin();
  commandStop();
  jsonTask();
}

bool turn(int angle) {
  static double zAngleGoal = 0.0;
  static float speed = 60;
    
  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;
  
  const double kp = 1.1;
  //const double ki = 1.0;
  const double kd = 1.1;
  
  if (firstTurn) {
    firstTurn = false;
    zAngleGoal = gyro.getAngleZ() + angle;
    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(speed);
  }

  float angleNormal = gyro.getAngleZ() - zAngleGoal;

  if (angleNormal > 180) angleNormal -= 360;
  if (angleNormal < -180) angleNormal += 360;
    
  // error = gyro.getAngleZ() - zAngleGoal;

  // if (error > 180) error -= 360;
  // if (error < 180) error += 360;
    
  // output = kp * error + kd * (error - previousError);
    
  // previousError = error;        
    
  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(speed - output);

  if (fabs(angleNormal) < 1.0) {
    firstTurn = true;
    return true;
  }
  return false;
}


void calibration() {
  static bool done = false;
  for (int i = 0; i < NB_IR; i++) {
    int val = ss.analogRead(i);
    if (val < capteurs[i].valMin) capteurs[i].valMin = val;
    if (val > capteurs[i].valMax) capteurs[i].valMax = val;
  }
  done = turn(358);
  if (done) {
    done = false;
    state = MANUAL;
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);

    for (int i = 0; i < NB_IR; i++) {
      capteurs[i].seuil = (capteurs[i].valMin + capteurs[i].valMax) / 2.0;
    }

    for (int i = 0; i < NB_IR; i++) {
      Serial.print(i);
      Serial.print(F(" - "));
      Serial.println(capteurs[i].seuil);
      // Serial.print(capteurs[i].valMin);
      // Serial.print(F(" - "));
      // Serial.println(capteurs[i].valMax);
    }
  } 
}

void loop() {
  currentTime = millis();
  //stateManager(currentTime);
  //communicationTask();

  jsonTask();

  ledCheckpoint();

  if (!raceEnded) {
    chronoEnd = currentTime;
  }

  switch(state) {
    case LINE:
      lineTask();
    break;
    case CALIBRATION:
      calibration();
    break;
    case MISSION:
      missionTask();
      break;
    case END:
      finishRace();
      break;
  }
  
  debugTask();

  moveAuto();

  gyroTask(currentTime);
  encodersTask(currentTime);
}

void gyroTask(unsigned long ct) {
  gyro.update();
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}

#pragma region manuel
// Tâche servant à communiquer des données
void communicationTask() {
}

// Événement qui se déclenche lorsqu'il y a réception de données via le port série
void serialEvent() {
  static String receivedData = "";

  if (!Serial.available()) return;

  receivedData = Serial.readStringUntil('\n');
  parseData(receivedData);
}

/**
  Fonction servant à analyser les données reçues.
  "parse" veut dire analyser
*/
void parseData(String& receivedData) {
  bool isFromBLE = false;  // Indicateur de source des données

  if (receivedData.length() >= 2) {
    // Vérifier si les deux premiers octets sont 0xFF55 (BLE)
    if ((uint8_t)receivedData[0] == 0xFF && (uint8_t)receivedData[1] == 0x55) {
      isFromBLE = true;
      // Supprimer les deux premiers octets
      receivedData.remove(0, 2);
    }
    // Vérifier si les deux premiers caractères sont "!!" (Moniteur Série)
    else if (receivedData.startsWith("!!")) {
      // Supprimer les deux premiers caractères
      receivedData.remove(0, 2);
    } else {
      // En-tête non reconnue
      Serial.print(F("Données non reconnues : "));
      Serial.println(receivedData);
      return;
    }
  } else {
    Serial.print(F("Données trop courtes : "));
    Serial.println(receivedData);
    return;
  }

  // Afficher les données reçues si le mode débogage est activé
  if (debugMode) {
    Serial.print(F("Reçu : "));
    Serial.println(receivedData);
    Serial.print(F("Source : "));
    Serial.println(isFromBLE ? F("BLE") : F("Moniteur Série"));
  }

  // Découpage de la commande et des paramètres
  int firstComma = receivedData.indexOf(',');

  if (firstComma == -1) {
    // Pas de virgule, donc c'est une commande sans paramètres
    handleCommand(receivedData);
  } else {
    // Il y a des paramètres
    String command = receivedData.substring(0, firstComma);
    String params = receivedData.substring(firstComma + 1);
    handleCommandWithParams(command, params);
  }
}

// Fonction pour gérer une commande sans paramètres
void handleCommand(String command) {
  // Utilisation d'un switch pour les commandes sans paramètres
  char cmd = command[0];
  if (cmd != 'K') analogWrite(BUZZER_PIN, 0);
  // if (cmd != 'X' && cmd != 'C' && cmd != 'V' && cmd != 'H' && cmd != 'N') {
  //   encoderLeft.setMotorPwm(0);
  //   encoderRight.setMotorPwm(0);
  // }asd
  switch (cmd) {
    case 'F':
      commandForward();
      //goStraight(speed);
      break;

    case 'L':
      commandLeft();
      //spin(spinSpeed, -1);
      break;

    case 'R':
      commandRight();
      //spin(spinSpeed, 1);
      break;

    case 'B':
      commandBack();
      //goStraight(-speed);
      break;

    case 'K':  // Commande "BEEP"
      //Serial.println(F("Commande BEEP reçue - exécuter le bip"));
      commandBeep();
      break;

    case 'X':
      encoderLeft.setMotorPwm(100);
      encoderRight.setMotorPwm(-200);
      straightFirstRun = true;
      break;
    case 'C':
      encoderLeft.setMotorPwm(150);
      encoderRight.setMotorPwm(-200);
      straightFirstRun = true;
      break;
    case 'V':
      //commandForward();
      goStraight(160);
      break;
    case 'H':
      encoderLeft.setMotorPwm(200);
      encoderRight.setMotorPwm(-150);
      straightFirstRun = true;
      break;
    case 'N':
      encoderLeft.setMotorPwm(200);
      encoderRight.setMotorPwm(-100);
      straightFirstRun = true;
      break;

    case 'i':
      state = LINE;
      firstLine = true;
      straightFirstRun = true;
      break;

    case 'M':
      state = MANUAL;
      encoderLeft.setMotorPwm(0);
      encoderRight.setMotorPwm(0);
      break;

    case 'o':
      state = CALIBRATION;
      break;

    case 'G':
      state = MISSION;
      encoderLeft.setMotorPwm(0);
      encoderRight.setMotorPwm(0);
      break;

    case 'W':
      state = END;
      encoderLeft.setMotorPwm(0);
      encoderRight.setMotorPwm(0);
      break;

    case 'S':
      commandStop();
      // encoderLeft.setMotorPwm(0);
      // encoderRight.setMotorPwm(0);
      // straightFirstRun = true;
      // analogWrite(BUZZER_PIN, 0);
      //buzzer.noTone();
      break;

    case 'd':  // Commande pour basculer le mode débogage
      debugMode = !debugMode;
      Serial.print(F("Mode débogage : "));
      Serial.println(debugMode ? F("activé") : F("désactivé"));
      break;

    default:
      Serial.print(F("Commande inconnue sans paramètres : "));
      Serial.println(command);
      break;
  }
}

// Fonction pour gérer une commande avec paramètres
void handleCommandWithParams(String command, String params) {
  char cmd = command[0];
  switch (cmd) {
    case 'l':  // Commande "LIGHT" pour définir la couleur de l'anneau LED
      commandLight(params);
      break;

    case 'A':
      commandAuto(params.toInt());
      break;

    case 'p':
      commandSetSpeed(params.toInt());
      
      break;

    default:
      Serial.print(F("Commande inconnue avec paramètres : "));
      Serial.print(command);
      Serial.print(F(", "));
      Serial.println(params);
      break;
  }
}

void stateManager(unsigned long ct) {
}

void debugTask() {
  static unsigned long lastTime = 0;
  static int delay = 200;
  static String lastCommand;

  // if (lastCommand != currentCommand) {
  //   Serial.print(F("Commande reçue : "));
  // }

  if (!debugMode || currentTime - lastTime < delay) return;

  lastTime = currentTime;

  if (state == AUTO) {
    Serial.print(F("Distance restante : "));
    Serial.println(distToGo - (getPosition() - startPosAuto), 2);
  }
    
}

void commandSetSpeed(int params) {
  static int max = 255;
  static int min = 50;
  int value = params;
  if (value >= min && value <= max) {
    speed = value;
    lineSpeed = value;
  } else {
    Serial.println(F("La valeur de la vitesse doit être entre 50 et 255"));
  }
}

void goStraight(short speed) {
    static double zAngleGoal = 0.0;
    
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    // PD Controller
    // Change les valeurs selon tes besoins
    // higher kp = plus réactive, peu osciller
    // lowewr kp = sluggish, moins d'oscillation
    // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
    const double kp = 3;
    //const double ki = 1.0;
    const double kd = 3;
    
    if (straightFirstRun) {
      straightFirstRun = false;
      
      zAngleGoal = gyro.getAngleZ();
      
      encoderLeft.setMotorPwm(speed);
      encoderRight.setMotorPwm(-speed);
      
      return;
    }
    
    error = gyro.getAngleZ() - zAngleGoal;
    
    // Google : ELI5 PID
    // Astuce web : ELI5 = Explain Like I'm 5
    output = kp * error + kd * (error - previousError);
    
    previousError = error;        
    
    encoderLeft.setMotorPwm(speed - output);
    encoderRight.setMotorPwm(-speed - output);
}

void spin(short speed, short direction) {
  encoderLeft.setMotorPwm(speed * direction);
  encoderRight.setMotorPwm(speed * direction);
  straightFirstRun = true;
}

void commandBeep() {
  static int beepIntensity = 127;
  analogWrite(BUZZER_PIN, beepIntensity);
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  // if (beep) {
    
  // } else {
  //   analogWrite(BUZZER_PIN, 0);
  // }
}

#pragma region COMMANDES

void ledAction(int r, int g, int b) {
  led_ring.setColor(r, g, b);
  led_ring.show();   
}

void ledAction(int idx, int  r, int g, int b) {
  led_ring.setColor(idx, r, g, b);
  led_ring.show(); 
}

void commandLight(String params) {
  int commaCount = countCharOccurrences(params, ',');
  
  // Vérifie le nombre de paramètres en comptant les virgules
  if (commaCount == 2) {
    // Trois paramètres (r, g, b) pour définir toute la couleur de l'anneau
    int r = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    int g = params.substring(0, params.indexOf(',')).toInt();
    int b = params.substring(params.indexOf(',') + 1).toInt();
    
    ledAction(r, g, b);  // Appel pour affecter l'ensemble de l'anneau
  } 
  else if (commaCount == 3) {
    // Quatre paramètres (idx, r, g, b) pour définir une LED spécifique
    int idx = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    int r = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    int g = params.substring(0, params.indexOf(',')).toInt();
    int b = params.substring(params.indexOf(',') + 1).toInt();
    
    ledAction(idx, r, g, b);  // Appel pour affecter une LED spécifique
  } 
  else {
    Serial.println(F("Commande lumière invalide"));
  }
}

void commandForward() {
  goStraight(speed);
}

void commandBack() {
  goStraight(-speed);
}

void commandLeft() {
  spin(speed, -1);
}

void commandRight() {
  spin(speed, 1);
}

void commandStop() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

void commandAuto(float distance) {
  distToGo = distance;
  straightFirstRun = true;
  autoRunning = true;
  startPosAuto = getPosition();
  state = AUTO;
}

void moveAuto() {
  static unsigned long lastTime = 0;
  static bool ledState = false;
  static int ledSpeed = 400;
  static int ledIntensity = 20;

  if (currentTime - lastTime >= ledSpeed) {
    lastTime = currentTime;
    ledState = !ledState;
  }

  if (state == AUTO) ledState ? ledAction(ledIntensity, ledIntensity, 0) : ledAction(0, 0, 0);

  if (autoRunning) {
    float position = getPosition();
    float traveled = position - startPosAuto;
    
    if (traveled < distToGo) {
      goStraight(speed);
    } else {
      distToGo = 0;
      autoRunning = false;
      encoderLeft.setMotorPwm(0);
      encoderRight.setMotorPwm(0);
      state = MANUAL;
      ledAction(0, 0, 0);
    }
  }
}

float getPosition() {
  float leftDeg = encoderLeft.getCurPos();
  float rightDeg = encoderRight.getCurPos();

  // conversion en tours
  float leftRot = leftDeg / 360.0;
  float rightRot = -rightDeg / 360.0;

  // circonférence en cm
  float circWheel = DIA_WHEEL * 3.14159 / 10.0; // mm → cm

  // distance moyenne
  float dist = ((leftRot + rightRot) / 2.0) * circWheel;

  return dist;
}

#pragma endregion

#pragma region HELPERS
int countCharOccurrences(const String &str, char ch) {
  int count = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str[i] == ch) {
      count++;
    }
  }
  return count;
}
#pragma endregion
#pragma endregion

#pragma region LINE

float computePID(float position, float consigne = 0.0f) {
    // Ajuster les coefficients selon vos besoins
    static float kp = 0.5; // Coefficient proportionnel
    static float ki = 0.0; // Coefficient intégral
    static float kd = 0.0; // Coefficient dérivé

    static float integral = 0;
    static float derivative = 0;
    static float lastError = 0;

    float error = position - consigne;

    integral += error;

    // Adapter cette valeur selon les besoins de votre application
    const float integralLimit = 1000;
    
    // Limiter l'intégrale pour éviter l'emballement intégral
    integral = constrain(integral, -integralLimit, integralLimit);

    derivative = error - lastError;
    lastError = error;
    
    float output = kp * error + ki * integral + kd * derivative;
    
    return output;
}

void suivreLigne(float adjustment) {
  //static int lineSpeed = 40;
  //Serial.println(adjustment);
  // encoderLeft.setMotorPwm(lineSpeed - adjustment);
  // encoderRight.setMotorPwm(-lineSpeed - adjustment);

  float absError = abs(adjustment);
  //float speed = 255 - constrain(absError * 3.5, 0, 200);
  static float max = 255.0;
  
  //Serial.println(adjustment);

  float leftPwm = lineSpeed - adjustment;
  float rightPwm = -lineSpeed - adjustment;

  if (leftPwm > max) {
    float excess = leftPwm - max;
    leftPwm = max;
    rightPwm += excess;
  }

  if (rightPwm < -max) {
    float excess = (-max) - rightPwm;
    rightPwm = -max;
    leftPwm -= excess;
  }

  leftPwm = constrain(leftPwm, -max, max);
  rightPwm = constrain(rightPwm, -max, max);

  encoderLeft.setMotorPwm(leftPwm);
  encoderRight.setMotorPwm(rightPwm);
}

float getTotalSignal() {
  float totalSignal = 0;
  for (int i = 0; i < NB_IR; i++) totalSignal += capteurs[i].normal;
  return totalSignal;
}

double capteurLectureNormalisee(int index) {
  return ((capteurs[index].val - capteurs[index].valMin) * 1.0) / (capteurs[index].valMax - capteurs[index].valMin) * 1000.0;
}

bool detectLine(float totalSignal) {
  static int seuilLignes = 4900;
  //Serial.println(totalSignal);
  return (totalSignal < seuilLignes) ? true : false;
  // if (totalSignal < seuilLignes) {
  //   return true;
  // } else {
  //   return false;
  // }
}

float getLinePosition() {
  float num = 0;
  float den = 0;

  for (int i = 0; i < NB_IR; i++) {
    num += capteurs[i].normal * (i - 2);
    den += capteurs[i].normal;
  }
  return (num / den) * 1000;
}

void normaliserValeurs() {
  for (int i = 0; i < NB_IR; i++) {
    capteurs[i].val = ss.analogRead(i);
    for (int i = 0; i < NB_IR; i++) {
      capteurs[i].onLine = (capteurs[i].val < capteurs[i].seuil);
    }
    // if (capteurs[i].val < capteurs[i].valMin) capteurs[i].valMin = capteurs[i].val;
    // if (capteurs[i].val > capteurs[i].valMax) capteurs[i].valMax = capteurs[i].val;
    // Serial.println(ss.analogRead(i));
    // Serial.println(capteurs[i].valMin);
    // Serial.println(capteurs[i].valMax);
    capteurs[i].normal = capteurLectureNormalisee(i);
  }
}

float getDistance() {
  static unsigned long lastTime = 0;
  static int delay = 100;
  // float distance;
  float newDistance;

  if (currentTime - lastTime < delay) return distance; 

  lastTime = currentTime;

  newDistance = ultraSensor.distanceCm();

  if (newDistance > 3) { //  && newDistance < 401
    distance = newDistance;
    // delay = 100;
  }

  return distance;
}

bool allSensorsOnLine() {
  for (int i = 0; i < NB_IR; i++) {
    if (!capteurs[i].onLine) return false;
  }
  return true;
}

bool noSensorsOnLine() {
  for (int i = 0; i < NB_IR; i++) {
    if (capteurs[i].onLine) return false;
  }
  return true;
}

void updateLineState(float distance) {
  // static int seuilDist = 40;
  static float seuilCareful = 30;
  static float seuilFollow  = 40;
  static bool isCareful = false;
  static short fastSpeed = lineSpeed;
  static short slowSpeed = 50;

  switch(lineState) {
    case FOLLOW:
      lineSpeed = fastSpeed;
      if (distance <= seuilCareful) {
        lineState = CAREFUL;
        isCareful = true;
        Serial.println(distance);
        Serial.println(F("CAREFUL"));
      }
      break;
    case CAREFUL:
      lineSpeed = slowSpeed;
      if (distance > seuilFollow) {
        lineState = FOLLOW;
        isCareful = false;
        Serial.println(distance);
        Serial.println(F("FOLLOW"));
      }
      break;
  }
}

void handleLinePresence(bool isOnLine, float adjustment) {
  static short speed = lineSpeed;

  // if (firstLine && !noSensorsOnLine() && !allSensorsOnLine()) {
  //   firstLine = false;
  // }

  if (allSensorsOnLine()) { // intersections
    lineState = INTERSECTION;
    return;
  }

  // if (allSensorsOnLine() && firstLine) { // entrer en ligne
  //   encoderLeft.setMotorPwm(speed);
  //   encoderRight.setMotorPwm(speed);
  //   return;
  // }

  // if (capteurs[0].onLine && capteurs[1].onLine && capteurs[2].onLine) { // angle droit vers la gauche
  //   encoderLeft.setMotorPwm(-speed);
  //   encoderRight.setMotorPwm(-speed);
  //   return;
  // }

  // if (capteurs[2].onLine && capteurs[3].onLine && capteurs[4].onLine) { // angle droit vers la droite
  //   encoderLeft.setMotorPwm(speed);
  //   encoderRight.setMotorPwm(speed);
  //   return;
  // }

  if (noSensorsOnLine()) {
    //Serial.println(firstLine);
    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(-speed);
    return;
  }

  // if (noSensorsOnLine() && firstLine) { // aucune ligne rencontrer
  //   //Serial.println(firstLine);
  //   goStraight(speed);
  //   return;
  // }

  suivreLigne(adjustment);
}

bool spin90(short direction = 1) {
  static double zAngleGoal = 0.0;
  static float speed = 60;
    
  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;
  
  // PD Controller
  // Change les valeurs selon tes besoins
  // higher kp = plus réactive, peu osciller
  // lowewr kp = sluggish, moins d'oscillation
  // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
  const double kp = 1.1;
  //const double ki = 1.0;
  const double kd = 1.1;
  
  if (firstSpin) {
    firstSpin = false;

    zAngleGoal = gyro.getAngleZ() + 90 * direction;
    // Serial.println ("Setting speed");
      
    encoderLeft.setMotorPwm(speed * direction);
    encoderRight.setMotorPwm(speed * direction);
      
    return false;
    
  }
    
  error = gyro.getAngleZ() - zAngleGoal;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
    
  // Google : ELI5 PID
  // Astuce web : ELI5 = Explain Like I'm 5
  // output = kp * error + kd * (error - previousError);
    
  // previousError = error;        
    
  encoderLeft.setMotorPwm(speed * direction);
  encoderRight.setMotorPwm(speed * direction);

  if (abs(error) < 2.0) {
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    firstSpin = true;
    return true;
  }

  return false;
}

void handleIntersections(float distance) {
  static unsigned long startTime;
  static bool direction;
  static short advanceDelay = 350;
  static short slow = 60;
  static short d = 1;
  static short dist = 40;

  switch (intState) {
    case ADVANCE:
      checkPoint++;
      startTime = currentTime;
      firstSpin = true;
      firstTurn = true;
      straightFirstRun = true;  
      goStraight(slow);
      intState = SPIN;
      break;

    case SPIN:
      if (currentTime - startTime < advanceDelay) return;
      if (!spin90(-1 * d)) return;
      direction = (distance < dist);  // TRUE = mur
      intState = FINAL;
      break;

    case FINAL:
      if (direction) {
        if (!turn(175 * d)) return; // tourner à droite si mur à gauche
      }
      
      if (distance < dist) {
        intState = PARKING;
      } else {
        lineState = FOLLOW;
        intState = ADVANCE;
        encoderLeft.setMotorPwm(0);
        encoderRight.setMotorPwm(0);
      }
      break;
    
    case PARKING:
      static bool spinDone = false;
    
      straightFirstRun = true;

      if (!spinDone && !spin90(1 * d)) return;
      spinDone = true;

      if (!noSensorsOnLine()) {
        goStraight(-slow);
        return;
      }

      lineState = FOLLOW;
      intState = ADVANCE;
      arrived = true;
      encoderLeft.setMotorPwm(0);
      encoderRight.setMotorPwm(0);

      break;
  }
}

bool findLine() {
  static short advanceDelay = 100;
  static short speed = 100;
  static unsigned long startTime;

  normaliserValeurs();

  switch(findLineState) {
    case FIND:
      goStraight(speed);
      if (!noSensorsOnLine()) {
        findLineState = CENTER;
        startTime = currentTime;
      }
      break;
    case CENTER:
      if (currentTime - startTime < advanceDelay) return;
      if (!turn(90)) return;
      findLineState = DONE;
      break;
    case DONE:
      findLineState = FIND;
      straightFirstRun = true;
      checkPoint++;
      return true;
      break;
  }

  // if (capteurs[0].onLine) {
  //   // ligne à gauche
  //   encoderLeft.setMotorPwm(50);
  //   encoderRight.setMotorPwm(50);
  //   return false;
  // } if (capteurs[4].onLine) {
  //   // ligne à droite
  //   encoderLeft.setMotorPwm(50);
  //   encoderRight.setMotorPwm(50);
  //   return false;
  // }

  // bool center = capteurs[1].onLine || capteurs[2].onLine || capteurs[3].onLine;
  // if (center && !allSensorsOnLine()) {
  //   straightFirstRun = true;
  //   return true;
  // }

  return false;
}

void lineTask() { /////////////////// LOOP FOR LINE ////////////////////
  static float consigne = 0.0f; // Position centrale
  // Normaliser les valeurs des capteurs
  normaliserValeurs();

  distance = getDistance();

  //Serial.println(distance);

  updateLineState(distance);

  // Calculer la position de la ligne
  float position = getLinePosition(); 

  // Calculer l'ajustement à apporter à la trajectoire
  float adjustment = computePID(position, consigne);

  // Ajuster la trajectoire du robot en fonction de l'ajustement
  // Par exemple, ajuster la vitesse des moteurs
  float totalSignal = getTotalSignal();
  // Serial.println(totalSignal);

  bool isOnLine = detectLine(totalSignal);
  //Serial.println(isOnLine);

  switch(lineState) {
    case FOLLOW:
    case CAREFUL:
      handleLinePresence(isOnLine, adjustment);
      break;

    case INTERSECTION:
      handleIntersections(distance);
      break;
  }
}

#pragma endregion

#pragma region OTHERS
void jsonTask() {
  static char output[128];
  static unsigned long lastTime = 0;
  static short delay = 1000;

  if (currentTime - lastTime >= delay) {
    static StaticJsonDocument<128> doc;
    lastTime = currentTime;
    doc["ts"] = currentTime;
    doc["chrono"] = chronoEnd - chronoStart;
    doc["etat"] = state;
    doc["gz"] = gyro.getAngleZ();
    doc["cp"] = checkPoint;

    static JsonObject pwm = doc.createNestedObject("pwm");
    pwm["l"] = encoderLeft.getCurPwm();
    pwm["r"] = encoderRight.getCurPwm();

    //JsonArray capt = doc.to<JsonArray>();
    static JsonArray capt = doc.createNestedArray("capt");
    for (int i = 0; i < NB_IR; i++) {
      capt[i] = capteurs[i].val;
    }
    serializeJson(doc, output); 
    Serial.println(output); 
  }
}
#pragma endregion
#pragma region MISSON
void missionTask() {
  // static bool firstMission = true;
  // if (firstMission) {
  //   firstMission = false;
  // }
  // ledTask();
  switch(missionState) {
    case START:
      ledTask();
      break;
    case PACKAGE:
    static bool lineFound = false;
    static unsigned long noLineTime = 0;
    static short noLineDelay = 150;
    static short ledDelay = 1000;
    static bool ledStart = true;

      if (ledStart && currentTime - missionBeginning > ledDelay) {
        ledStart = false;
        led_ring.setColor(0, 0, 0);
        led_ring.show();
      }

      if (!lineFound && findLine()) lineFound = true;

      if (lineFound) {

        if (!noSensorsOnLine()) noLineTime = currentTime;

        if (currentTime - noLineTime < noLineDelay) {
          lineTask();
        } else {
          takePackage();
        }
        // if (!noSensorsOnLine()) {
        //   lineTask();
        // } else {
        //   // takePackage();
        //   encoderLeft.setMotorPwm(0);
        //   encoderRight.setMotorPwm(0);
        // }
      }
      break;
    
    case DELIVER:
      lineTask();
      if (arrived) {
        missionState = RACE;

        for (int i = 0; i < sizeof(frontLeds)/sizeof(frontLeds[0]); i++) {
          led_ring.setColor(frontLeds[i], 0, 50, 0);
        }
        led_ring.show();
      }
      break;

    case RACE:
      state = MANUAL;
      break;
  }
  
}

void takePackage() {
  static int distToWall = 20;
  static bool returned = false;
  static int speed = 80;

  distance = getDistance();

  if (!possessPackage && distance > distToWall) {
    goStraight(speed);
    return;
  } else if (!possessPackage) {
    possessPackage = true;
    checkPoint++;

    for (int i = 0; i < sizeof(frontLeds)/sizeof(frontLeds[0]); i++) {
      led_ring.setColor(frontLeds[i], 50, 50, 0);
    }
    led_ring.show();
  }

  if (!returned) {
    normaliserValeurs();
    if (noSensorsOnLine()) {
      // encore rien vu -> continuer à reculer
      goStraight(-speed);
      return;           // IMPORTANT : sortir de la fonction ici
    } else {
      // au moins un capteur voit la ligne -> on est revenu dessus
      returned = true;
    }
  }

  // une fois returned == true, on exécute le 175° et la suite
  if (!turn(175)) return;
  missionState = DELIVER;
  firstLine = true;
  straightFirstRun = true;

  // if (noSensorsOnLine()) {
  //   goStraight(40);
  // } else {
    
  // }
}

void finishRace() {
  static short distToGo = 10;
  static short speed = 80;
  static bool firstTime = true;

  distance = getDistance();

  if (distance > distToGo) {
    goStraight(speed);
  } else {
    commandStop();
    rainbowLed();
    if (firstTime) {
      raceEnded = true;
      firstTime = false;
      checkPoint++;
    }
  }
}

void ledCheckpoint() {
  static short lastCheckPoint = 0;

  if (checkPoint != lastCheckPoint) {
    lastCheckPoint = checkPoint;

    for (int i = 0; i < checkPoint; i++) {
      led_ring.setColor(backLeds[i], 50, 0, 50);
    }
    led_ring.show();
  }
}

void ledTask() {
  static unsigned long lastTime = 0;
  static short ledInt = 50;
  switch(ledState) {
    case THREE:
        led_ring.setColor(ledInt, 0, 0);
        ledState = TWO;
        lastTime = currentTime;
        Serial.println(3);
      break;
    case TWO:
      static int delay1 = 2000;
      static bool secondTwo = true;

      if (secondTwo && currentTime - lastTime > 1000) {
        secondTwo = false;
        Serial.println(2);
      }

      if (currentTime - lastTime < delay1) return;

      lastTime = currentTime;
      led_ring.setColor(ledInt, ledInt, 0);
      ledState = ONE;
      lastTime = currentTime;
      Serial.println(1);
      break;
    case ONE:
      static int delay2 = 1000;
      if (currentTime - lastTime < delay2) return;
      lastTime = currentTime;
      led_ring.setColor(0, ledInt, 0);
      ledState = GO;
      lastTime = currentTime;
      Serial.println(F("GO!"));
      break;
    case GO:
      chronoStart = currentTime;
      missionBeginning = currentTime;
      ledState = OFF;
      missionState = PACKAGE;
      firstLine = true;
      straightFirstRun = true;
      break;
    case OFF:
      
      break;
  }
  led_ring.show();
}

void rainbowLed()
{
  static float j;
  static float f;
  static float k;
  
  for (uint8_t t = 0; t < AURIGARINGLEDNUM; t++ )
  {
    uint8_t red	= 8 * (1 + sin(t / 2.0 + j / 4.0) );
    uint8_t green = 8 * (1 + sin(t / 1.0 + f / 9.0 + 2.1) );
    uint8_t blue = 8 * (1 + sin(t / 3.0 + k / 14.0 + 4.2) );
    led_ring.setColorAt( t, red, green, blue );
  }
  led_ring.show();

  j += random(1, 6) / 6.0;
  f += random(1, 6) / 6.0;
  k += random(1, 6) / 6.0;
}
#pragma endregion