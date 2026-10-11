// ============================================================
//  Quadruped Robot Control
//  Stand / Reverse Bend / Walk / LF Handshake
//  Simple Calibration Style
// ============================================================
 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int PWM_FREQ = 50;

// ------------------------------------------------------------
// Servo angle → PWM mapping
// ------------------------------------------------------------
const int angleTable[] = {0,20,40,60,80,100,120,140,160,180};
const int pwmTable[] = {75,100,130,160,195,225,260,290,320,350};

// ------------------------------------------------------------
// Leg structure
// ------------------------------------------------------------
struct Leg {
  int hipChan, shChan, ankChan;
  float neutralHip, neutralSh, neutralAnk;
};

// 0=LF, 1=RF, 2=LB, 3=RB
Leg legs[4] = {
  { 0,  1,  2, 75.0, 140.0,  65.0 },   // LF
  { 3,  4,  5, 70.0, 10.0, 150.0 },    // RF
  { 6,  7,  8, 100.0, 140.0,  70.0 },   // LB
  { 9, 10, 11, 120.0, 90.0, 150.0 }    // RB
};

// ------------------------------------------------------------
// HIP OFFSET
// ------------------------------------------------------------
float hipOffset[4] = { -20, 10, -20, 11 };
float hipOffset1[4] = { -20, 10, -20, 11 };

// ============================================================
// BACK LEG BEND CONTROL (TUNE HERE)
// ============================================================

float LB_bend = 38;
float RB_bend = 28;

float LB_shBend = -15;
float RB_shBend = -25;

float LB_hipOffset = 0;
float RB_hipOffset = 0;

float LB_shOffset = 0;
float RB_shOffset = 0;

float LB_ankOffset = 35;
float RB_ankOffset = -35;

// ============================================================
// SIT CONTROL (TUNE HERE)
// ============================================================

// FRONT LEFT
float SIT_LF_shBend2  = 130;
float SIT_LF_ankBend2 = 10;

// FRONT RIGHT
float SIT_RF_shBend2  = 25;
float SIT_RF_ankBend2 = 190;

// BACK LEFT
float SIT_LB_shBend2  = 130;
float SIT_LB_ankBend2 = 10;

// BACK RIGHT
float SIT_RB_shBend2  = 115;
float SIT_RB_ankBend2 = 300;
// ============================================================
// LF HANDSHAKE CONTROL (TUNE HERE)
// ============================================================
float LB_bend1 = 35;
float RB_bend1 = 25;

float LB_shBend1 = -15;
float RB_shBend1 = -25;

float LB_hipOffset1 = 0;
float RB_hipOffset1 = 0;

float LB_shOffset1 = 0;
float RB_shOffset1 = 0;

float LB_ankOffset1 = 35;
float RB_ankOffset1 = -35;

float LF_shBend   = -30;   // shoulder lift
float LF_ankBend  = 30;   // ankle fold
float LF_shakeAmp = 5;   // shake strength

float LF_hipTrim  = 0;
float LF_shTrim   = 0;
float LF_ankTrim  = 0;

// ------------------------------------------------------------
// WALK SETTINGS
// ------------------------------------------------------------
const int SMOOTH_STEPS = 26;
float liftHeight = 40.0;
float tapBack    = 20.0;
int walkSpeed    = 7;

int shSign[4] = { 1, -1, 1, -1 };
int akSign[4] = { 1, -1, 1, -1 };

float shScale[4] = { 1.0, 0.92, 1.0, 0.92 };
float akScale[4] = { 1.0, 0.95, 1.0, 0.95 };

float frontLiftScale = 0.82;
float frontTapScale  = 0.78;
float backLiftScale  = 0.82; 
float backTapScale   = 0.78;

volatile bool walking = false;

// ------------------------------------------------------------
// Utility
// ------------------------------------------------------------
float clampf(float v,float l,float h){
  return v<l?l:(v>h?h:v);
}

uint16_t angleToTick(float a){
  a = clampf(a,0,180);
  for(int i=0;i<9;i++){
    if(a>=angleTable[i] && a<=angleTable[i+1]){
      float t=(a-angleTable[i])/(angleTable[i+1]-angleTable[i]);
      return pwmTable[i]+t*(pwmTable[i+1]-pwmTable[i]);
    }
  }
  return pwmTable[0];
}

void setS(int ch,float a){
  pwm.setPWM(ch,0,angleToTick(a));
}

// ============================================================
// STAND
// ============================================================
void doStand(){
  for(int i=0;i<4;i++){
    setS(legs[i].hipChan, legs[i].neutralHip + hipOffset[i]);
    setS(legs[i].shChan,  legs[i].neutralSh);
    setS(legs[i].ankChan, legs[i].neutralAnk);
  }
}

// ============================================================
// REVERSE BEND
// ============================================================
void doBow(){

  walking = false;

  // Front legs straight
  for(int j=0; j<2; j++){
    setS(legs[j].hipChan, legs[j].neutralHip + hipOffset[j]);
    setS(legs[j].shChan,  legs[j].neutralSh);
    setS(legs[j].ankChan, legs[j].neutralAnk);
  }

  // LB
  float lbHip = legs[2].neutralHip + hipOffset[2] + LB_hipOffset;
  float lbSh  = legs[2].neutralSh + (shSign[2] * LB_shBend) + LB_shOffset;
  float lbAk  = legs[2].neutralAnk - (akSign[2] * LB_bend) + LB_ankOffset;

  setS(legs[2].hipChan, lbHip);
  setS(legs[2].shChan,  lbSh);
  setS(legs[2].ankChan, lbAk);

  // RB
  float rbHip = legs[3].neutralHip + hipOffset[3] + RB_hipOffset;
  float rbSh  = legs[3].neutralSh + (shSign[3] * RB_shBend) + RB_shOffset;
  float rbAk  = legs[3].neutralAnk - (akSign[3] * RB_bend) + RB_ankOffset;

  setS(legs[3].hipChan, rbHip);
  setS(legs[3].shChan,  rbSh);
  setS(legs[3].ankChan, rbAk);
}
void doBow1(){

  walking = false;

  // Front legs straight
  for(int j=0; j<2; j++){
    setS(legs[j].hipChan, legs[j].neutralHip + hipOffset1[j]);
    setS(legs[j].shChan,  legs[j].neutralSh);
    setS(legs[j].ankChan, legs[j].neutralAnk);
  }

  // LB
  float lbHip = legs[2].neutralHip + hipOffset1[2] + LB_hipOffset;
  float lbSh  = legs[2].neutralSh + (shSign[2] * LB_shBend1) + LB_shOffset1;
  float lbAk  = legs[2].neutralAnk - (akSign[2] * LB_bend1) + LB_ankOffset1;

  setS(legs[2].hipChan, lbHip);
  setS(legs[2].shChan,  lbSh);
  setS(legs[2].ankChan, lbAk);

  // RB
  float rbHip = legs[3].neutralHip + hipOffset1[3] + RB_hipOffset1;
  float rbSh  = legs[3].neutralSh + (shSign[3] * RB_shBend1) + RB_shOffset1;
  float rbAk  = legs[3].neutralAnk - (akSign[3] * RB_bend1) + RB_ankOffset1;

  setS(legs[3].hipChan, rbHip);
  setS(legs[3].shChan,  rbSh);
  setS(legs[3].ankChan, rbAk);
}

void doSit2(){

  walking = false;

  // ================= FRONT LEGS =================
  setS(legs[0].hipChan, legs[0].neutralHip + hipOffset[0]);
  setS(legs[0].shChan,  SIT_LF_shBend2);
  setS(legs[0].ankChan, SIT_LF_ankBend2);

  setS(legs[1].hipChan, legs[1].neutralHip + hipOffset[1]);
  setS(legs[1].shChan,  SIT_RF_shBend2);
  setS(legs[1].ankChan, SIT_RF_ankBend2);

  // ================= BACK LEGS =================
  setS(legs[2].hipChan, legs[2].neutralHip + hipOffset[2]);
  setS(legs[2].shChan,  SIT_LB_shBend2);
  setS(legs[2].ankChan, SIT_LB_ankBend2);

  setS(legs[3].hipChan, legs[3].neutralHip + hipOffset[3]);
  setS(legs[3].shChan,  SIT_RB_shBend2);
  setS(legs[3].ankChan, SIT_RB_ankBend2);
  delay(200);
  doStand();
}
// ============================================================
// LF HANDSHAKE
// ============================================================
void doHandshake(){

  walking = false;

  doBow1();
  delay(500);

  int lf = 0;

  float hip = legs[lf].neutralHip + hipOffset[lf] + LF_hipTrim;

  float sh  = legs[lf].neutralSh
              + (shSign[lf] * LF_shBend)
              + LF_shTrim;

  float ak  = legs[lf].neutralAnk
              - (akSign[lf] * LF_ankBend)
              + LF_ankTrim;

  setS(legs[lf].hipChan, hip);
  setS(legs[lf].shChan,  sh);
  setS(legs[lf].ankChan, ak);

  delay(300);

  for(int i=0;i<3;i++){
    setS(legs[lf].shChan, sh + (shSign[lf] * LF_shakeAmp));
    delay(200);
    setS(legs[lf].shChan, sh - (shSign[lf] * LF_shakeAmp));
    delay(200);
  }

  delay(200);
  doStand();

}
// ============================================================
// WALK
// ============================================================
void diagonalWalk(){

  for(int phase=0; phase<2 && walking; phase++){
    for(int i=0;i<=SMOOTH_STEPS && walking;i++){

      float e = sin(i * PI / SMOOTH_STEPS);

      for(int j=0;j<4;j++){

        float hip = legs[j].neutralHip + hipOffset[j];
        float sh  = legs[j].neutralSh;
        float ak  = legs[j].neutralAnk;

        bool active =
          (phase==0 && (j==0 || j==3)) ||
          (phase==1 && (j==1 || j==2));

        if(active){
          bool isFront = (j==0 || j==1);

          float liftS = isFront ? frontLiftScale : backLiftScale;
          float tapS  = isFront ? frontTapScale  : backTapScale;

          sh += shSign[j] * shScale[j] *
                (-liftHeight * liftS * e);

          ak += akSign[j] * akScale[j] *
                ( tapBack * tapS * e );
        }

        setS(legs[j].hipChan, hip);
        setS(legs[j].shChan, sh);
        setS(legs[j].ankChan, ak);
      }

      delay(walkSpeed);
    }
  }
}

// ============================================================
// SETUP
// ============================================================
void setup(){
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  doStand();

  Serial.println("1 = Stand");
  Serial.println("2 = Reverse Bend");
  Serial.println("3 = Walk");
  Serial.println("4 = Stop");
  Serial.println("5 = LF Handshake");
   Serial.println("6 = sit");
}

// ============================================================
// LOOP
// ============================================================
void loop(){

  if(Serial.available()){
    char c = Serial.read();

    if(c=='1'){ walking=false; doStand(); }
    if(c=='2'){ doBow(); }
    if(c=='3'){ walking=true; }
    if(c=='4'){ walking=false; doStand(); }
    if(c=='5'){ doHandshake(); }
    if(c=='6'){ doSit2(); }
  }

  if(walking){
    diagonalWalk();
  }
}