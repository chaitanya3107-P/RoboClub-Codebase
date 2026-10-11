#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// =====================================================
// PCA9685 SERVO RANGE
// =====================================================
#define SERVO_MIN 102
#define SERVO_MAX 512

// =====================================================
// ARM PCA9685 CHANNELS
// =====================================================
#define RIGHT_ELBOW 8
#define LEFT_ELBOW 10
#define LEFT_WRIST 11
#define RIGHT_WRIST 12

// =====================================================
// JAW PCA9685 CHANNELS
// =====================================================
#define JAW_SERVO1 13
#define JAW_SERVO2 14

// =====================================================
// RECEIVER
//
// CH5 / SWA -> D4 -> ARM ONLY
// CH6 / SWB -> D3 -> JAW ONLY
// =====================================================
#define SWA_PIN 4
#define SWB_PIN 3

#define SWA_THRESHOLD 1500
#define SWB_THRESHOLD 1500

unsigned long lastSWA = 1000;
unsigned long lastSWB = 1000;

// =====================================================
// ARM CURRENT POSITION
// =====================================================
int currentServo0 = 0;
int currentServo1 = 0;

bool hifiActive = false;

// =====================================================
// JAW
//
// CH13: 0 -> 55 -> 0
// CH14: 55 -> 0 -> 55
// =====================================================
int jawAngle = 0;
bool jawForward = true;

unsigned long lastJawMove = 0;

// 1 degree steps = smooth
// 2 ms = faster movement
#define JAW_STEP 1
#define JAW_DELAY 0.1

// =====================================================
// SERIAL MONITOR
// =====================================================
unsigned long lastSerialPrint = 0;

// =====================================================
// ANGLE TO PCA9685
// =====================================================
int angleToPulse(int angle) {

  angle = constrain(angle, 0, 180);

  return map(
    angle,
    0,
    180,
    SERVO_MIN,
    SERVO_MAX
  );
}

// =====================================================
// READ SWA - CH5
// ARM ONLY
// =====================================================
unsigned long readSWA() {

  unsigned long value = pulseIn(
    SWA_PIN,
    HIGH,
    25000
  );

  if (value > 0) {
    lastSWA = value;
  }

  return lastSWA;
}

// =====================================================
// READ SWB - CH6
// JAW ONLY
// =====================================================
unsigned long readSWB() {

  unsigned long value = pulseIn(
    SWB_PIN,
    HIGH,
    25000
  );

  if (value > 0) {
    lastSWB = value;
  }

  return lastSWB;
}

// =====================================================
// ARM SERVO GROUP 0
//
// CH8  = Normal
// CH10 = Reversed
// =====================================================
void setServo0(int angle) {

  angle = constrain(angle, 0, 180);

  pwm.setPWM(
    RIGHT_ELBOW,
    0,
    angleToPulse(angle)
  );

  pwm.setPWM(
    LEFT_ELBOW,
    0,
    angleToPulse(120 - angle)
  );

  currentServo0 = angle;
}

// =====================================================
// ARM SERVO GROUP 1
//
// CH12 = Reversed
// CH11 = Normal
// =====================================================
void setServo1(int angle) {

  angle = constrain(angle, 0, 180);

  pwm.setPWM(
    RIGHT_WRIST,
    0,
    angleToPulse(180 - angle)
  );

  pwm.setPWM(
    LEFT_WRIST,
    0,
    angleToPulse(angle)
  );

  currentServo1 = angle;
}

// =====================================================
// NAMASTE POSITION
// =====================================================
void homePosition() {

  setServo0(0);
  setServo1(0);

  Serial.println(">>> NAMASTE POSITION");
}

// =====================================================
// ARM RETURN TO NAMASTE
// =====================================================
void smoothHomePosition() {

  int startServo0 = currentServo0;
  int startServo1 = currentServo1;

  Serial.println(">>> RETURNING TO NAMASTE");

  int maxDistance = max(
    abs(startServo0),
    abs(startServo1)
  );

  if (maxDistance == 0) {

    homePosition();

    return;
  }

  for (int i = 0; i <= maxDistance; i++) {

    int newServo0 =
      startServo0 -
      ((startServo0 * i) / maxDistance);

    int newServo1 =
      startServo1 -
      ((startServo1 * i) / maxDistance);

    setServo0(
      constrain(newServo0, 0, 180)
    );

    setServo1(
      constrain(newServo1, 0, 180)
    );

    delay(15);
  }

    pwm.setPWM(RIGHT_ELBOW, 0, angleToPulse(0));    // CH8 = 0°
  pwm.setPWM(LEFT_ELBOW, 0, angleToPulse(140));   // CH10 = 115°
  setServo1(0);

  Serial.println(">>> NAMASTE REACHED");
}

// =====================================================
// ARM HI-FI ELBOW
//
// CH8  : 0 -> 110
// CH10 : 90 -> 10
// =====================================================
bool moveElbowsForHiFi() {

  Serial.println(">>> ELBOW MOVEMENT START");

  for (int angle = 0; angle <= 110; angle++) {

    // ONLY SWA controls ARM
    if (readSWA() < SWA_THRESHOLD) {

      Serial.println(
        ">>> SWA OFF - ARM STOP"
      );

      return false;
    }

    setServo0(angle);

    delay(15);
  }

  setServo0(110);

  Serial.println(">>> ELBOW MOVEMENT COMPLETE");

  return true;
}

// =====================================================
// ARM HI-FI WRIST
// =====================================================
bool wristHiFi() {

  // -------------------------------------------------
  // 60 -> 10
  // -------------------------------------------------
  for (int angle = 50; angle >=0 ; angle--) {

    if (readSWA() < SWA_THRESHOLD) {

      Serial.println(
        ">>> SWA OFF - WRIST STOP"
      );

      return false;
    }

    setServo1(angle);

    delay(15);
  }

  // -------------------------------------------------
  // 10 -> 60
  // -------------------------------------------------
  for (int angle = 0; angle <= 50; angle++) {

    if (readSWA() < SWA_THRESHOLD) {

      Serial.println(
        ">>> SWA OFF - WRIST STOP"
      );

      return false;
    }

    setServo1(angle);

    delay(15);
  }

  delay(250);

  Serial.println(">>> WRIST CYCLE COMPLETE");

  return true;
}

// =====================================================
// COMPLETE ARM HI-FI
// =====================================================
void robotMovement() {

  Serial.println();
  Serial.println("==============================");
  Serial.println(">>> HI-FI START");
  Serial.println("==============================");

  // ARM STEP 1
  if (!moveElbowsForHiFi()) {
    return;
  }

  delay(300);

  // ARM STEP 2
  while (readSWA() >= SWA_THRESHOLD) {

    if (!wristHiFi()) {
      return;
    }
  }
}

// =====================================================
// JAW HOME
//
// CH13 = 0°
// CH14 = 55°
// =====================================================
void jawHome() {

  pwm.setPWM(
    JAW_SERVO1,
    0,
    angleToPulse(0)
  );

  pwm.setPWM(
    JAW_SERVO2,
    0,
    angleToPulse(45)
  );

  jawAngle = 0;
  jawForward = true;

  Serial.println(">>> JAW HOME");
}

// =====================================================
// WRITE BOTH JAW SERVOS
//
// CH13: 0 -> 55
// CH14: 55 -> 0
// =====================================================
void writeJawPosition() {

  // CH13
  pwm.setPWM(
    JAW_SERVO1,
    0,
    angleToPulse(jawAngle)
  );

  // CH14 opposite
  pwm.setPWM(
    JAW_SERVO2,
    0,
    angleToPulse(45 - jawAngle)
  );
}

// =====================================================
// JAW CONTROL
//
// IMPORTANT:
// ONLY SWB / CH6 / D3 CONTROLS THE JAW.
//
// SWA IS NOT USED HERE.
// =====================================================
void updateJaw() {

  unsigned long swb = readSWB();

  // =================================================
  // SWB OFF
  // JAW STOPS
  // =================================================
  if (swb < SWB_THRESHOLD) {

    return;
  }

  // =================================================
  // JAW SPEED
  // =================================================
  if (
    millis() - lastJawMove <
    JAW_DELAY
  ) {

    return;
  }

  lastJawMove = millis();

  // =================================================
  // OPENING
  //
  // CH13: 0 -> 55
  // CH14: 55 -> 0
  // =================================================
  if (jawForward) {

    jawAngle += JAW_STEP;

    if (jawAngle >= 45) {

      jawAngle = 45;

      jawForward = false;

      Serial.println(
        ">>> JAW FULL OPEN"
      );
    }
  }

  // =================================================
  // CLOSING
  //
  // CH13: 55 -> 0
  // CH14: 0 -> 55
  // =================================================
  else {

    jawAngle -= JAW_STEP;

    if (jawAngle <= 0) {

      jawAngle = 0;

      jawForward = true;

      Serial.println(
        ">>> JAW CLOSED"
      );
    }
  }

  // =================================================
  // BOTH SERVOS MOVE TOGETHER
  // =================================================
  writeJawPosition();
}

// =====================================================
// SERIAL STATUS
// =====================================================
void printStatus() {

  if (
    millis() - lastSerialPrint <
    500
  ) {

    return;
  }

  lastSerialPrint = millis();

  unsigned long swa = readSWA();
  unsigned long swb = readSWB();

  Serial.println();
  Serial.println("========== STATUS ==========");

  // -------------------------------------------------
  // ARM SWITCH
  // -------------------------------------------------
  Serial.print("CH5 / SWA / D4 = ");
  Serial.print(swa);
  Serial.print(" us -> ");

  if (swa < SWA_THRESHOLD) {
    Serial.println("ARM NAMASTE");
  }
  else {
    Serial.println("ARM HI-FI");
  }

  // -------------------------------------------------
  // JAW SWITCH
  // -------------------------------------------------
  Serial.print("CH6 / SWB / D3 = ");
  Serial.print(swb);
  Serial.print(" us -> ");

  if (swb < SWB_THRESHOLD) {
    Serial.println("JAW OFF");
  }
  else {
    Serial.println("JAW ON");
  }

  // -------------------------------------------------
  // JAW ANGLES
  // -------------------------------------------------
  Serial.print("PCA CH13 = ");
  Serial.print(jawAngle);
  Serial.println(" deg");

  Serial.print("PCA CH14 = ");
  Serial.print(45 - jawAngle);
  Serial.println(" deg");

  // -------------------------------------------------
  // ARM
  // -------------------------------------------------
  Serial.print("Arm Servo0 = ");
  Serial.print(currentServo0);
  Serial.println(" deg");

  Serial.print("Arm Servo1 = ");
  Serial.print(currentServo1);
  Serial.println(" deg");

  Serial.println("============================");
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(9600);

  // =================================================
  // RECEIVER PINS
  // =================================================
  pinMode(SWA_PIN, INPUT);
  pinMode(SWB_PIN, INPUT);

  // =================================================
  // PCA9685
  // =================================================
  pwm.begin();

  pwm.setPWMFreq(50);

  delay(500);

  // =================================================
  // ARM HOME
  // =================================================
  homePosition();

  // =================================================
  // JAW HOME
  // =================================================
  jawHome();

  delay(1000);

  Serial.println();
  Serial.println("============================");
  Serial.println("       ROBOT READY");
  Serial.println("============================");

  Serial.println(
    "CH5/SWA -> D4 -> ARM ONLY"
  );

  Serial.println(
    "CH6/SWB -> D3 -> JAW ONLY"
  );

  Serial.println("----------------------------");

  Serial.println(
    "CH8  -> Right Elbow"
  );

  Serial.println(
    "CH10 -> Left Elbow"
  );

  Serial.println(
    "CH11 -> Left Wrist"
  );

  Serial.println(
    "CH12 -> Right Wrist"
  );

  Serial.println(
    "CH13 -> Jaw Servo 1"
  );

  Serial.println(
    "CH14 -> Jaw Servo 2"
  );

  Serial.println("----------------------------");

  Serial.println(
    "JAW: CH13 0->45->0"
  );

  Serial.println(
    "JAW: CH14 45->0->45"
  );

  Serial.println("============================");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  // =================================================
  // JAW
  //
  // ONLY SWB / D3
  // =================================================
  updateJaw();

  // =================================================
  // ARM
  //
  // ONLY SWA / D4
  // =================================================
  unsigned long swa = readSWA();

  // =================================================
  // ARM = NAMASTE
  // =================================================
  if (swa < SWA_THRESHOLD) {

    if (hifiActive) {

      smoothHomePosition();

      hifiActive = false;
    }
  }

  // =================================================
  // ARM = HI-FI
  // =================================================
  else {

    if (!hifiActive) {

      hifiActive = true;

      robotMovement();
    }
  }

  // =================================================
  // SERIAL
  // =================================================
  printStatus();

  // NO DELAY HERE
}