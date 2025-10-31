#include <MeAuriga.h>

//École: {"device_name": "Makeblock_LE001b10672e24"}
//Maison: {"device_name": "Makeblock_LE001b1063e3be"}

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

MeGyro gyro(0, 0x69);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

unsigned long currentTime = 0;
bool debugMode = false;

short speed = 150;
short spinSpeed = 100;

int stopLedIntensity = 20;

bool straightFirstRun = true;

float distToGo = 0;
float startPosAuto = 0;
bool autoRunning = false;
bool firstBeep = true;

String currentCommand;

enum State {
  AUTO,
  CONTROLLED
};

State state = CONTROLLED;

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

// ************* DÉBUT ************

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

void setup() {
  Serial.begin(115200);
  buzzer.setpin(BUZZER_PIN);
  led_ring.setpin(LED_RING_PIN);
  encoderConfig();
  gyro.begin();
  commandStop();
}

void loop() {
  currentTime = millis();
  //stateManager(currentTime);
  //communicationTask();
  
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

  if (receivedData[0] != 'K' && !firstBeep) firstBeep = true;

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

    case 'LIGHT_TOGGLE':
      led_ring.setColor(0, 0, 50);
      break;

    case 'S':
      commandStop();
      // encoderLeft.setMotorPwm(0);
      // encoderRight.setMotorPwm(0);
      // straightFirstRun = true;
      // analogWrite(BUZZER_PIN, 0);
      //buzzer.noTone();
      break;

    case 'K':  // Commande "BEEP"
      //Serial.println(F("Commande BEEP reçue - exécuter le bip"));
      commandBeep();
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
  //String cmd = command;
  switch (cmd) {
    case 'F':  // Commande "FORWARD"
      // Serial.print(F("Commande FORWARD reçue avec paramètres : "));
      // Serial.println(params);

      //commandForward(params);
      break;

    case 'K':  // Commande "BEEP"
      // Serial.println(F("Commande BEEP reçue - exécuter le bip"));
      // commandBeep();
      break;

    case 'l':  // Commande "LIGHT" pour définir la couleur de l'anneau LED
      // Serial.print(F("Commande LIGHT reçue avec paramètres : "));
      // Serial.println(params);
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
    const double kp = 8;
    //const double ki = 1.0;
    const double kd = 8;
    
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

void commandBeep() {
  static int beepIntensity = 127;
  analogWrite(BUZZER_PIN, beepIntensity);
  if (firstBeep) {
    firstBeep = false;
    ledAction(0, 0, 0);
    for (int l : backLeds) {
      ledAction(l, stopLedIntensity, 0, 0);
    }
  }
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
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
  ledAction(0, 0, 0);
  analogWrite(BUZZER_PIN, 0);
}

void commandBack() {
  static unsigned long lastTime = 0;
  static bool beepState = false;
  static int beepSpeed = 400;
  static int beepIntensity = 200;

  goStraight(-speed);

  ledAction(0, 0, 0);

  if (currentTime - lastTime >= beepSpeed) {
    lastTime = currentTime;
    beepState = !beepState;
  }

  beepState ? analogWrite(BUZZER_PIN, beepIntensity) : analogWrite(BUZZER_PIN, 0);

}

void commandLeft() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  static int blinkSpeed = 300;
  static int ledIntensity = 20;

  spin(speed, -1);

  ledAction(0, 0, 0);

  straightFirstRun = true;

  if (currentTime - lastBlink >= blinkSpeed) {
    blinkState = !blinkState;
    lastBlink = currentTime;
  }

  if (blinkState) {
    for (int l : topLeftLeds) {
      ledAction(l, ledIntensity, ledIntensity, 0);
    }
  } else {
    ledAction(0, 0, 0);
  }
  
}

void commandRight() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  static int blinkSpeed = 300;
  static int ledIntensity = 20;

  spin(speed, 1);

  ledAction(0, 0, 0);

  straightFirstRun = true;

  if (currentTime - lastBlink >= blinkSpeed) {
    blinkState = !blinkState;
    lastBlink = currentTime;
  }

  if (blinkState) {
    for (int l : topRightLeds) {
      ledAction(l, ledIntensity, ledIntensity, 0);
    }
  } else {
    ledAction(0, 0, 0);
  }
}

void commandStop() {
  static int ledIntensity = 20;

  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);

  straightFirstRun = true;

  analogWrite(BUZZER_PIN, 0);

  ledAction(0, 0, 0);

  for (int l : backLeds) {
    ledAction(l, stopLedIntensity, 0, 0);
  }
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
      state = CONTROLLED;
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

// void commandForward(String params) {
//     // paramètre
//     Serial.print(F("Paramètre : "));
//     Serial.println(params);
//     // Ajouter le code pour traiter la commande FORWARD avec ses paramètres
// }

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