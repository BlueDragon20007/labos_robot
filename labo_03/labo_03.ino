// Liam Doyon 6307713

#include <MeAuriga.h>

#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6

MeGyro gyro(0, 0x69);

MeUltrasonicSensor ultraSensor(PORT_8);

#define LEDNUM  12
#define LEDPIN  44
#define RINGALLLEDS 0

MeRGBLed led( PORT0, LEDNUM );

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

unsigned long currentTime = 0;

enum State {
  SETUP,
  MOVE,
  TURN,
  MOVEUNTIL,
  SUCCESS,
  RETURN,
  DONE
};

State state;

// unsigned long movePrevious = 0;
// int moveDelay = 2000;

// int moveDir = 1;

unsigned long serialPrintPrevious = 0;
int serialPrintInterval = 500;
String msg = "";

int distSeg1 = 210;
int distSeg2;
int distWall = 50;

float dist;
float position;
int speed = 100;
int ledInt = 40;

bool straightFirstRun = true;
bool spinFirstRun = true;

// ********* INTERRUPTIONS ***********

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
  encoderConfig();
  gyro.begin();
  
  state = SETUP;

  led.setpin(LEDPIN);
  
  dist = ultraSensor.distanceCm();
}

void loop() {
  currentTime = millis();

  position = getPosition();

  switch (state) {
    case SETUP:
      setupState();
      break;
    case MOVE:
      moveState();
      break;
    case TURN:
      turnState();
      break;
    case MOVEUNTIL:
      moveUntilState();
      break;
    case SUCCESS:
      successState();
      break;
    case RETURN:
      returnState();
      break;
    case DONE:
      doneState();
      break;
  }

  ledTask();
  
  gyroTask(currentTime);
  encodersTask(currentTime);
  //serialPrintTask(currentTime);
}

void gyroTask(unsigned long ct) {
  gyro.update();
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}

void ledTask() {
  static short idx = LEDNUM;
  static unsigned long lastTime = 0;

  switch(state) {
    case MOVE: {
      int startIndex = 11;
      int nbLeds = map(position, 0, distSeg1, 0, LEDNUM + 1);

      led.setColor(0, 0, 0);

      for (int i = 0; i < nbLeds; i++) {
        int ledIndex = ((startIndex + i) % LEDNUM) + 1; 
        led.setColor(ledIndex, 0, ledInt, 0);
      }
      break;
    }

    case TURN:
      led.setColor(0, 0, 0);
      break;

    case SUCCESS:
    case DONE:
      static unsigned long lastBlink = 0;
      static bool blinkState = false;
      const int blinkSpeed = 125;


      if (currentTime - lastBlink >= blinkSpeed) {
        blinkState = !blinkState;
        lastBlink = currentTime;
      }

      blinkState ? led.setColor(0, ledInt, 0) : led.setColor(0, 0, 0);

    break;

    case RETURN: {
      int rate = 100;

      if (currentTime - lastTime < rate) break;
      lastTime = currentTime;

      led.setColor(0, 0, 0);
      led.setColor(idx, 0, 0, ledInt);

      idx = (idx <= 1) ? LEDNUM : idx - 1;
      break;
    }
  }

  led.show();
}


void setupState() {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  static unsigned long exitTime = 0;
  
  const int timeout = 3000;
  
  if (firstTime) {
    firstTime = false;
    exitTime = currentTime + timeout;
    
    Serial.println("Attente de 3 secondes avant de démarrer.");
  }
  
  if (currentTime >= exitTime) {
    firstTime = true;
    state = MOVE;
  }
}

void moveState() {
  if (position < distSeg1) {
    goStraight(speed);
  } else {
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    state = TURN;
    resetControllers();
  }
}

void turnState() {
  if (spin90()) {
    resetControllers();
    state = MOVEUNTIL;
  } 
}

void moveUntilState() {
  static bool firstTime = true;
  static float startingPos;
  if (firstTime) {
    firstTime = false;
    startingPos = position;
  }
  goStraight(speed);
  getDistance();
  if (dist <= distWall) {
    distSeg2 = position - startingPos;
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    state = SUCCESS;
    resetControllers();
  }
}

void successState() {
  static bool firstTime = true;
  static unsigned long exitTime = 0;

  const int timeout = 3000;
  
  if (firstTime) {
    firstTime = false;
    exitTime = currentTime + timeout;
  }

  if (currentTime >= exitTime) {
    firstTime = true;
    state = RETURN;
  }

}

void returnState() {
  static float startPos;
  static bool firstTime = true;

  enum ReturnPhase {
    SEGMENT2,
    TURN,
    SEGMENT1
  };

  static ReturnPhase phase = SEGMENT2;

  switch(phase) {
    case SEGMENT2:
      if (firstTime) {
        startPos = position;
        firstTime = false;
      }

      goStraight(-speed);

      if ((startPos - position) >= distSeg2) {
        encoderLeft.setMotorPwm(0);
        encoderRight.setMotorPwm(0);
        phase = TURN;
        resetControllers();
        firstTime = true;
      }

    break;
    case TURN:
      if (firstTime) {
        firstTime = false;
      }
      if (spin90()) {
        phase = SEGMENT1;
        firstTime = true;
        resetControllers();
        //delay(100);  // Sinon crash
      }
    break;
    case SEGMENT1:
      if (firstTime) {
        startPos = position;
        firstTime = false;
      }

      goStraight(speed);

      if ((position - startPos) >= distSeg1) {
        encoderLeft.setMotorPwm(0);
        encoderRight.setMotorPwm(0);
        resetControllers();
        state = DONE;
      }

    break;
  }

}

void doneState() {
  static bool firstTime = true;

  if (firstTime) {
    firstTime = false;
    Serial.println("Youpi");
  }
}

bool spin90() {
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
  
  if (spinFirstRun) {
    spinFirstRun = false;

    zAngleGoal = gyro.getAngleZ() + 90;
    Serial.println ("Setting speed");
      
    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(speed);
      
    return false;
    
  }
    
  error = gyro.getAngleZ() - zAngleGoal;
    
  // Google : ELI5 PID
  // Astuce web : ELI5 = Explain Like I'm 5
  output = kp * error + kd * (error - previousError);
    
  previousError = error;        
    
  msg = "z : ";
  msg += gyro.getAngleZ();
  msg += "\tleft : ";
  msg += encoderLeft.getCurPwm();
  msg += "\tright : ";
  msg += encoderRight.getCurPwm();
  msg += "\toutput :";
  msg += output;
    
  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(speed - output);

  if (abs(error) < 2.0) {
    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
    return true;
  }

  return false;
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
    const double kp = 30;
    //const double ki = 1.0;
    const double kd = 30;
    
    if (straightFirstRun) {
      straightFirstRun = false;
      
      zAngleGoal = gyro.getAngleZ();
      Serial.println ("Setting speed");
      
      encoderLeft.setMotorPwm(speed);
      encoderRight.setMotorPwm(-speed);
      
      return;
    }
    
    error = gyro.getAngleZ() - zAngleGoal;
    
    // Google : ELI5 PID
    // Astuce web : ELI5 = Explain Like I'm 5
    output = kp * error + kd * (error - previousError);
    
    previousError = error;        
    
    msg = "z : ";
    msg += gyro.getAngleZ();
    msg += "\tleft : ";
    msg += encoderLeft.getCurPwm();
    msg += "\tright : ";
    msg += encoderRight.getCurPwm();
    msg += "\toutput :";
    msg += output;
    
    encoderLeft.setMotorPwm(speed - output);
    encoderRight.setMotorPwm(-speed - output);
}

void serialPrintTask(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 500;
  
  if (cT - lastTime < rate) return;

  lastTime = cT;

  if (msg != "") {
    Serial.println(msg);
    msg = "";
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

void getDistance() {
  static unsigned long lastTime = 0;
  static int delay = 100;

  if (currentTime - lastTime < delay) return; 

  lastTime = currentTime;

  dist = ultraSensor.distanceCm();
}

void resetControllers() {
  straightFirstRun = true;
  spinFirstRun = true;
}