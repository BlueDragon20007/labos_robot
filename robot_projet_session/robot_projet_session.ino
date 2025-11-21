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
bool debugMode = false;

short speed = 100;
short lineSpeed = 100;

bool straightFirstRun = true;

float distToGo = 0;
float startPosAuto = 0;
bool autoRunning = false;

String currentCommand;

bool isCalibrating = false;
bool firstTurn = true;
bool firstLine;
bool firstSpin = true;

short checkPoint = 0;

enum State {
  AUTO,
  MANUAL,
  LINE,
  CALIBRATION,
  MISSION
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
  FINAL
};

IntState intState = ADVANCE;

enum MissionState {
  START,
  PACKAGE
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
  static float speed = 50;
    
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

  switch(state) {
    case LINE:
      lineTask();
    break;
    case CALIBRATION:
      calibration();
    break;
    case MISSION:
      missionTask();
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

    case 'k':
      state = CALIBRATION;
      break;

    case 'G':
      state = MISSION;
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
    static float kp = 0.4; // Coefficient proportionnel
    static float ki = 0.01; // Coefficient intégral
    static float kd = 0.01; // Coefficient dérivé

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
  static int delay = 250;
  float distance;
  float newDistance;

  if (currentTime - lastTime < delay) return distance; 

  lastTime = currentTime;

  newDistance = ultraSensor.distanceCm();

  if (newDistance > 0 && newDistance < 250) {
    distance = newDistance;
    delay = 100;
  } else {
    delay = 250;
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
  static int seuilDist = 40;
  static bool isCareful = false;
  static short fastSpeed = lineSpeed;
  static short slowSpeed = 40;

  switch(lineState) {
    case FOLLOW:
      lineSpeed = fastSpeed;
      if (distance <= seuilDist && !isCareful) {
        lineState = CAREFUL;
        isCareful = true;

        Serial.println(F("CAREFUL"));
      }
      break;
    case CAREFUL:
      lineSpeed = slowSpeed;
      if (distance > seuilDist && isCareful) {
        lineState = FOLLOW;
        isCareful = false;
         // speed est la variable globale de vitesse en manuelle
        Serial.println(F("FOLLOW"));
      }
      break;
  }
}

void handleLinePresence(bool isOnLine, float adjustment) {
  static short speed = lineSpeed;

  if (firstLine && !noSensorsOnLine()) {
    firstLine = false;
  }

  // static int fullLine = 2000;
  // if (isOnLine && getTotalSignal() > fullLine) {
  //   if (firstLine) firstLine = false;
  //   suivreLigne(adjustment);
  // } else {
  //   if (firstLine) {
  //     goStraight(speed);
  //   } else {
  //     encoderLeft.setMotorPwm(speed);
  //     encoderRight.setMotorPwm(speed);
  //   }
  // }

  if (allSensorsOnLine() && !firstLine) { // intersections
    lineState = INTERSECTION;
    return;
  }

  if (allSensorsOnLine() && firstLine) { // entrer en ligne
    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(speed);
    return;
  }

  if (capteurs[0].onLine && capteurs[1].onLine && capteurs[2].onLine) { // angle droit vers la gauche
    encoderLeft.setMotorPwm(-speed);
    encoderRight.setMotorPwm(-speed);
    return;
  }

  if (capteurs[2].onLine && capteurs[3].onLine && capteurs[4].onLine) { // angle droit vers la droite
    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(speed);
    return;
  }

  if (noSensorsOnLine() && !firstLine) { // tourne pour revenir sur ligne
    //Serial.println(firstLine);
    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(speed);
    return;
  }

  if (noSensorsOnLine() && firstLine) { // aucune ligne rencontrer
    //Serial.println(firstLine);
    goStraight(speed);
    return;
  }

  suivreLigne(adjustment);
}

bool spin90(short direction = 1) {
  static double zAngleGoal = 0.0;
  static float speed = 33;
    
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
    Serial.println ("Setting speed");
      
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
  static short advanceDelay = 500;
  static short slow = 50;

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
      if (!spin90(-1)) return;
      direction = (distance < 50);  // TRUE = mur
      intState = FINAL;
      break;

    case FINAL:
      if (direction) {
        if (!turn(175)) return; // tourner à droite si mur à gauche
      }
      // Sortie : remise à zéro des phases et retour au FOLLOW
      Serial.println(F("gauche"));
      lineState = FOLLOW;
      intState = ADVANCE;
      encoderLeft.setMotorPwm(0);
      encoderRight.setMotorPwm(0);
      break;
  }
}

void lineTask() { /////////////////// LOOP FOR LINE ////////////////////
  static float consigne = 0.0f; // Position centrale
  static float distance;
  // Normaliser les valeurs des capteurs
  normaliserValeurs();

  distance = getDistance();

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
    doc["chrono"] = currentTime - chronoStart;
    doc["etat"] = state;
    doc["gz"] = gyro.getAngleZ();

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
  static bool firstMission = true;
  if (firstMisson) {
    firstMisson = false;
  }
  ledTask();
  switch(missionState) {
    case START:
      break;
  }
  
}

void ledTask() {
  static unsigned long lastTime = 0;
  static short ledInt = 25;
  switch(ledState) {
    case THREE:
        led_ring.setColor(ledInt, 0, 0);
        ledState = TWO;
        lastTime = currentTime;
        Serial.println(3);
      break;
    case TWO:
      static int delay = 2000;
      if (currentTime - lastTime < delay) return;
      led_ring.setColor(ledInt, ledInt, 0);
      ledState = ONE;
      lastTime = currentTime;
      Serial.println(1);
      break;
    case ONE:
      static int delay = 1000;
      if (currentTime - lastTime < delay) return;
      led_ring.setColor(0, ledInt, 0);
      ledState = GO;
      lastTime = currentTime;
      Serial.println(F("GO!"));
      break;
    case GO:
        ledState = OFF;
      break;
    case OFF:
      break;
  }
}
#pragma endregion