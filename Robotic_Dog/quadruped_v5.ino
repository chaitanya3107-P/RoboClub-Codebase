// ============================================================
//  QUADRUPED ROBOT CONTROL v5  -  SMOOTH MOTION
//
//  12x 20kg servos, PCA9685, Arduino Mega
//
//  CHANNEL MAP:      FRONT
//                LF          RF
//               0 1 2       3 4 5
//                LB          RB
//               6 7 8       9 10 11
//                 BACK
//
//  CALIBRATED STAND (hip, shoulder, ankle):
//    LF = 45, 120, 65     RF = 80, 10, 130
//    LB = 75, 120, 70     RB = 120, 90, 130
//
//  SERIAL COMMANDS:
//    1 Stand      2 Reverse Bend   3 Walk fwd   7 Walk back
//    8 Turn left  9 Turn right     4 Stop
//    5 Handshake  6 Sit
//    d Dance  w Wave  g Wiggle  b Bounce
//    + / -  Walk faster / slower
//    j  Joint test (moves every joint +JOINT_TEST_DEG and back)
//
//  WHAT CHANGED vs v4 (all for smoothness):
//   1. Walking is a continuous, non-blocking gait (elliptical foot
//      path). Legs never stop between half-steps.
//   2. Walk start/stop/direction/speed changes are blended, so
//      there are no pose jumps when you send a command mid-walk.
//   3. Repeating animations (bounce, wiggle, wave, paw lifts,
//      handshake) are one continuous cosine motion instead of a
//      chain of stop-and-go glides.
//   4. Servo is only written when its PWM tick actually changes,
//      and frames run at 50 Hz (the servo refresh rate).
//   5. Per-joint min/max limits (jMin/jMax) protect the linkages.
//   6. Legs power up one at a time to avoid a current spike.
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int PWM_FREQ = 50;
const int NUM_CH = 12;

enum LegID { LF = 0, RF = 1, LB = 2, RB = 3 };


// ============================================================
// ANGLE -> PWM CALIBRATION (unchanged)
// ============================================================

const int angleTable[] = { 0, 20, 40, 60, 80, 100, 120, 140, 160, 180 };
const int pwmTable[]   = { 75, 100, 130, 160, 195, 225, 260, 290, 320, 350 };
const int TABLE_SIZE = sizeof(angleTable) / sizeof(angleTable[0]);


// ============================================================
// LEGS + MASTER STAND CALIBRATION (unchanged)
// ============================================================

struct Leg {
  int hipChan, shChan, ankChan;
  float neutralHip, neutralSh, neutralAnk;
};

Leg legs[4] = {
  /* LF */ { 0,  1,  2,   45.0, 120.0,  65.0 },
  /* RF */ { 3,  4,  5,   80.0,  10.0, 130.0 },
  /* LB */ { 6,  7,  8,   75.0, 120.0,  70.0 },
  /* RB */ { 9, 10, 11,  120.0,  90.0, 130.0 }
};


// ============================================================
// JOINT LIMITS  (channel order: hip, shoulder, ankle per leg)
// ============================================================
// Every write is clamped to these. Run the 'j' joint test, note
// where a link starts to hit the body or another link, and
// tighten the numbers here.
//
//              LF             RF             LB             RB
//           hip  sh  ank   hip  sh  ank   hip  sh  ank   hip  sh  ank
float jMin[NUM_CH] = {  0,  0,  0,    0,  0,  0,    0,  0,  0,    0,  0,  0 };
float jMax[NUM_CH] = {180,180,180,  180,180,180,  180,180,180,  180,180,180 };

const float JOINT_TEST_DEG = 15.0;


// ============================================================
// DIRECTION / SCALE (unchanged)
// ============================================================

int shSign[4] = {  1, -1,  1, -1 };
int akSign[4] = {  1, -1,  1, -1 };

float shScale[4] = { 1.00, 0.92, 1.00, 0.92 };
float akScale[4] = { 1.00, 0.95, 1.00, 0.95 };


// ============================================================
// REVERSE BEND / BOW (unchanged)
// ============================================================

float LB_bend = 38;     float RB_bend = 28;
float LB_shBend = -15;  float RB_shBend = -25;
float LB_hipOffset = 0; float RB_hipOffset = 0;
float LB_shOffset = 0;  float RB_shOffset = 0;
float LB_ankOffset = 35; float RB_ankOffset = -35;

// SIT (unchanged)
float SIT_LF_shBend2 = 130; float SIT_LF_ankBend2 = 10;
float SIT_RF_shBend2 = 25;  float SIT_RF_ankBend2 = 170;
float SIT_LB_shBend2 = 130; float SIT_LB_ankBend2 = 10;
float SIT_RB_shBend2 = 115; float SIT_RB_ankBend2 = 170;
const unsigned long SIT_HOLD_MS = 200;

// HANDSHAKE / WAVE PRE-BOW (unchanged)
float LB_bend1 = 35;     float RB_bend1 = 25;
float LB_shBend1 = -15;  float RB_shBend1 = -25;
float LB_hipOffset1 = 0; float RB_hipOffset1 = 0;
float LB_shOffset1 = 0;  float RB_shOffset1 = 0;
float LB_ankOffset1 = 35; float RB_ankOffset1 = -35;

// LF HANDSHAKE / WAVE (unchanged)
float LF_shBend = -30;
float LF_ankBend = 30;
float LF_shakeAmp = 5;
float LF_hipTrim = 0, LF_shTrim = 0, LF_ankTrim = 0;
float WAVE_HIP_AMP = 15;


// ============================================================
// WALK TUNING
// ============================================================
// liftHeight : shoulder lift of the swinging foot (degrees).
// tapBack    : HALF stride, in ankle degrees. The foot now swings
//              +/- this around the stand pose (v4 only went 0..+).
//              So 14 here is about the same total travel as v4's 20
//              before the front/back scales. Raise for longer steps.
// ============================================================

float liftHeight = 40.0;
float tapBack = 14.0;

float frontLiftScale = 0.82;
float frontTapScale  = 0.78;
float backLiftScale  = 0.82;
float backTapScale   = 0.78;

// Time one diagonal pair spends in the air. Full cycle = 2x.
unsigned long WALK_PHASE_MS = 900;
const unsigned long WALK_MIN_MS = 350;
const unsigned long WALK_MAX_MS = 2000;

// How long walking takes to fade in / out (ms)
const unsigned long GAIT_RAMP_MS = 900;
// How fast direction changes blend (1.0 = full change in 1 s)
const float CMD_BLEND_PER_S = 2.0;

float TURN_HIP_AMP = 12;
int hipTurnSign[4] = { -1, 1, -1, 1 };

float WIGGLE_AMP = 10;


// ============================================================
// MOTION TIMING
// ============================================================

float MOVE_SPEED_DPS = 60.0;
const unsigned long MIN_MOVE_MS = 300;

// 20 ms = 50 Hz. Hobby servos only read a new pulse every 20 ms,
// so updating faster adds I2C load but no extra smoothness.
const unsigned long UPDATE_MS = 20;

const unsigned long SHAKE_HALF_MS = 250;


// ============================================================
// STATE
// ============================================================

float curAng[16];
float tgtAng[16];
uint16_t lastTick[16];

bool  walking = false;       // user wants to walk
float strideCmd = 0, yawCmd = 0;   // commanded direction
float strideCur = 0, yawCur = 0;   // smoothed direction
float gaitPhase = 0;         // 0..1 through one full cycle
float gaitAmp = 0;           // 0 = standing, 1 = full walk
float periodCur = 1800;      // smoothed full-cycle time (ms)


// ============================================================
// BASIC HELPERS
// ============================================================

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool cmdPending() {
  while (Serial.available() &&
         (Serial.peek() == '\n' || Serial.peek() == '\r')) {
    Serial.read();
  }
  return Serial.available() > 0;
}

uint16_t angleToTick(float a) {
  a = clampf(a, 0, 180);
  for (int i = 0; i < TABLE_SIZE - 1; i++) {
    if (a >= angleTable[i] && a <= angleTable[i + 1]) {
      float t = (a - angleTable[i]) /
                (float)(angleTable[i + 1] - angleTable[i]);
      return (uint16_t)(pwmTable[i] +
                        t * (pwmTable[i + 1] - pwmTable[i]) + 0.5f);
    }
  }
  return pwmTable[0];
}

// Clamp to joint limits, and only talk to the PCA9685 when the
// PWM tick really changed (less I2C traffic = steadier timing).
void writeCh(int ch, float a) {
  a = clampf(a, jMin[ch], jMax[ch]);
  curAng[ch] = a;
  uint16_t t = angleToTick(a);
  if (t != lastTick[ch]) {
    lastTick[ch] = t;
    pwm.setPWM(ch, 0, t);
  }
}

void writeLeg(int j, float hip, float sh, float ak) {
  writeCh(legs[j].hipChan, hip);
  writeCh(legs[j].shChan, sh);
  writeCh(legs[j].ankChan, ak);
}

void setTarget(int ch, float aRaw) {
  float a = clampf(aRaw, jMin[ch], jMax[ch]);
  if (a != aRaw) {
    Serial.print(F("[LIMIT] CH "));
    Serial.print(ch);
    Serial.print(F(" "));
    Serial.print(aRaw);
    Serial.print(F(" -> "));
    Serial.println(a);
  }
  tgtAng[ch] = a;
}

void setLegPose(LegID leg, float hip, float sh, float ank) {
  setTarget(legs[leg].hipChan, hip);
  setTarget(legs[leg].shChan, sh);
  setTarget(legs[leg].ankChan, ank);
}

void copyTargets(float *dst) {
  for (int ch = 0; ch < NUM_CH; ch++) dst[ch] = tgtAng[ch];
}


// ============================================================
// GLIDE: smooth point-to-point move (smootherstep)
// ============================================================

bool glide(bool interruptible, unsigned long forceMs) {

  float start[NUM_CH];
  float maxD = 0;

  for (int ch = 0; ch < NUM_CH; ch++) {
    start[ch] = curAng[ch];
    float d = fabs(tgtAng[ch] - curAng[ch]);
    if (d > maxD) maxD = d;
  }

  if (maxD < 0.3f) {
    for (int ch = 0; ch < NUM_CH; ch++) writeCh(ch, tgtAng[ch]);
    return true;
  }

  unsigned long dur = forceMs;
  if (dur == 0) {
    dur = (unsigned long)(maxD / MOVE_SPEED_DPS * 1000.0f);
    if (dur < MIN_MOVE_MS) dur = MIN_MOVE_MS;
  }

  unsigned long t0 = millis();

  while (true) {
    unsigned long el = millis() - t0;
    if (el >= dur) break;
    if (interruptible && cmdPending()) return false;

    float u = el / (float)dur;
    float s = u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);

    for (int ch = 0; ch < NUM_CH; ch++) {
      if (start[ch] != tgtAng[ch]) {
        writeCh(ch, start[ch] + (tgtAng[ch] - start[ch]) * s);
      }
    }
    delay(UPDATE_MS);
  }

  for (int ch = 0; ch < NUM_CH; ch++) writeCh(ch, tgtAng[ch]);
  return true;
}

// Glide to a saved pose
bool moveTo(const float *P, unsigned long ms) {
  for (int ch = 0; ch < NUM_CH; ch++) tgtAng[ch] = P[ch];
  return glide(true, ms);
}

// Continuous back-and-forth between two poses A and B.
// Moves along a cosine, so it never pauses at A or B mid-way and
// starts/ends at A with zero speed. Robot must already be at A.
bool oscillate(const float *A, const float *B,
               int cycles, unsigned long periodMs) {

  unsigned long total = (unsigned long)cycles * periodMs;
  unsigned long t0 = millis();

  while (true) {
    unsigned long el = millis() - t0;
    if (el >= total) break;
    if (cmdPending()) return false;

    float ph = (float)(el % periodMs) / (float)periodMs;
    float s = 0.5f * (1.0f - cos(TWO_PI * ph));

    for (int ch = 0; ch < NUM_CH; ch++) {
      if (A[ch] != B[ch]) writeCh(ch, A[ch] + (B[ch] - A[ch]) * s);
    }
    delay(UPDATE_MS);
  }

  for (int ch = 0; ch < NUM_CH; ch++) writeCh(ch, A[ch]);
  return true;
}


// ============================================================
// POSE BUILDERS
// ============================================================

void buildStand() {
  for (int i = 0; i < 4; i++) {
    setLegPose((LegID)i, legs[i].neutralHip,
               legs[i].neutralSh, legs[i].neutralAnk);
  }
}

void bowLeg(LegID j, float bend, float shBend,
            float hipO, float shO, float akO) {
  setLegPose(j,
    legs[j].neutralHip + hipO,
    legs[j].neutralSh + shSign[j] * shBend + shO,
    legs[j].neutralAnk - akSign[j] * bend + akO);
}

void buildBow() {
  setLegPose(LF, legs[LF].neutralHip, legs[LF].neutralSh, legs[LF].neutralAnk);
  setLegPose(RF, legs[RF].neutralHip, legs[RF].neutralSh, legs[RF].neutralAnk);
  bowLeg(LB, LB_bend, LB_shBend, LB_hipOffset, LB_shOffset, LB_ankOffset);
  bowLeg(RB, RB_bend, RB_shBend, RB_hipOffset, RB_shOffset, RB_ankOffset);
}

void buildBow1() {
  setLegPose(LF, legs[LF].neutralHip, legs[LF].neutralSh, legs[LF].neutralAnk);
  setLegPose(RF, legs[RF].neutralHip, legs[RF].neutralSh, legs[RF].neutralAnk);
  bowLeg(LB, LB_bend1, LB_shBend1, LB_hipOffset1, LB_shOffset1, LB_ankOffset1);
  bowLeg(RB, RB_bend1, RB_shBend1, RB_hipOffset1, RB_shOffset1, RB_ankOffset1);
}

void buildSit() {
  setLegPose(LF, legs[LF].neutralHip, SIT_LF_shBend2, SIT_LF_ankBend2);
  setLegPose(RF, legs[RF].neutralHip, SIT_RF_shBend2, SIT_RF_ankBend2);
  setLegPose(LB, legs[LB].neutralHip, SIT_LB_shBend2, SIT_LB_ankBend2);
  setLegPose(RB, legs[RB].neutralHip, SIT_RB_shBend2, SIT_RB_ankBend2);
}

void buildWiggle(int dir) {
  for (int j = 0; j < 4; j++) {
    setLegPose((LegID)j,
      legs[j].neutralHip + dir * WIGGLE_AMP * hipTurnSign[j],
      legs[j].neutralSh,
      legs[j].neutralAnk);
  }
}

// Shared leg maths for walking and paw lifts.
//  lift   0..1   shoulder lift
//  stride -1..1  ankle sweep (+ = forward)
//  yaw    -1..1  hip sweep for turning
void legPose(int j, float lift, float stride, float yaw,
             float &hip, float &sh, float &ak) {

  bool isFront = (j == LF || j == RF);
  float liftS = isFront ? frontLiftScale : backLiftScale;
  float tapS  = isFront ? frontTapScale  : backTapScale;

  hip = legs[j].neutralHip + TURN_HIP_AMP * yaw * hipTurnSign[j];

  sh = legs[j].neutralSh +
       shSign[j] * shScale[j] * (-liftHeight * liftS * lift);

  ak = legs[j].neutralAnk +
       akSign[j] * akScale[j] * (tapBack * tapS * stride);
}

void raiseLF(float &hip, float &sh, float &ak) {
  hip = legs[LF].neutralHip + LF_hipTrim;
  sh  = legs[LF].neutralSh + shSign[LF] * LF_shBend + LF_shTrim;
  ak  = legs[LF].neutralAnk - akSign[LF] * LF_ankBend + LF_ankTrim;
  setLegPose(LF, hip, sh, ak);
}


// ============================================================
// CONTINUOUS GAIT (trot: LF+RB swing together, RF+LB opposite)
// ============================================================
// Each foot follows an ellipse over one cycle:
//   swing  (first half) : foot lifts and moves rear -> front
//   stance (second half): foot is down and moves front -> rear
// Position and speed are continuous everywhere, so there is no
// stop-and-go. gaitAmp fades the whole motion in and out, so the
// robot eases out of and back into the calibrated stand pose.
// ============================================================

bool gaitRunning() {
  return walking || gaitAmp > 0.0f;
}

void updateGait(float dt) {

  // Blend commanded direction
  float step = CMD_BLEND_PER_S * dt;
  strideCur += clampf(strideCmd - strideCur, -step, step);
  yawCur    += clampf(yawCmd - yawCur, -step, step);

  // Fade in / out
  float ramp = dt * 1000.0f / (float)GAIT_RAMP_MS;
  gaitAmp = clampf(gaitAmp + (walking ? ramp : -ramp), 0.0f, 1.0f);

  // Smooth speed changes
  float target = 2.0f * (float)WALK_PHASE_MS;
  periodCur += (target - periodCur) * clampf(dt * 4.0f, 0.0f, 1.0f);

  // Phase integrates time, so speed changes never jump the legs
  gaitPhase += dt * 1000.0f / periodCur;
  if (gaitPhase >= 1.0f) gaitPhase -= 1.0f;
  if (gaitAmp <= 0.0f && !walking) gaitPhase = 0.0f;

  float a = gaitAmp * gaitAmp * (3.0f - 2.0f * gaitAmp);   // smoothstep

  for (int j = 0; j < 4; j++) {

    float p = gaitPhase + ((j == RF || j == LB) ? 0.5f : 0.0f);
    if (p >= 1.0f) p -= 1.0f;

    float th = TWO_PI * p;
    float x = -cos(th);                 // -1 rear ... +1 front

    float lift = 0.0f;
    if (p < 0.5f) {                     // swing half only
      float s = sin(th);
      lift = s * s;                     // smooth up/down, no kink
    }

    float hip, sh, ak;
    legPose(j, lift * a, x * strideCur * a, x * yawCur * a, hip, sh, ak);
    writeLeg(j, hip, sh, ak);
  }
}

// Non-blocking: called every loop()
void runGait() {
  static unsigned long last = 0;

  if (!gaitRunning()) {
    last = millis();
    return;
  }

  unsigned long now = millis();
  if (now - last < UPDATE_MS) return;

  float dt = (now - last) / 1000.0f;
  if (dt > 0.1f) dt = 0.1f;
  last = now;

  updateGait(dt);
}

// Blocking: ease the gait out and wait until the legs are at
// the stand pose. Called before every animation.
void settleGait() {
  walking = false;
  unsigned long last = millis();
  while (gaitAmp > 0.0f) {
    unsigned long now = millis();
    if (now - last >= UPDATE_MS) {
      updateGait((now - last) / 1000.0f);
      last = now;
    }
  }
}

void startGait(float stride, float yaw) {
  if (!gaitRunning()) {
    // Begin from the exact calibrated stand
    buildStand();
    glide(false, 0);
    strideCur = stride;
    yawCur = yaw;
    gaitPhase = 0;
  }
  strideCmd = stride;
  yawCmd = yaw;
  walking = true;
}


// ============================================================
// BASIC MOVES
// ============================================================

void doStand() {
  settleGait();
  buildStand();
  glide(false, 0);
}

void doBow() {
  settleGait();
  buildBow();
  glide(false, 0);
}

void doSit2() {
  settleGait();
  buildSit();
  glide(false, 0);
  delay(SIT_HOLD_MS);
  doStand();
}


// ============================================================
// ANIMATIONS (continuous)
// ============================================================

// ms = time for one direction (stand -> bow), as in v4
bool bounce(int n, unsigned long ms) {
  float A[NUM_CH], B[NUM_CH];
  buildStand(); copyTargets(A);
  buildBow();   copyTargets(B);
  if (!moveTo(A, 0)) return false;
  return oscillate(A, B, n, 2 * ms);
}

bool wiggle(int n, unsigned long ms) {
  float S[NUM_CH], L[NUM_CH], R[NUM_CH];
  buildStand();   copyTargets(S);
  buildWiggle(1); copyTargets(L);
  buildWiggle(-1); copyTargets(R);
  if (!moveTo(S, 0)) return false;
  if (!moveTo(L, ms)) return false;
  if (!oscillate(L, R, n, 2 * ms)) return false;
  return moveTo(S, ms);
}

bool pawLifts(int n, unsigned long ms) {
  float S[NUM_CH], P[2][NUM_CH];

  buildStand(); copyTargets(S);

  for (int pair = 0; pair < 2; pair++) {
    buildStand();
    for (int j = 0; j < 4; j++) {
      bool up = (pair == 0 && (j == LF || j == RB)) ||
                (pair == 1 && (j == RF || j == LB));
      float hip, sh, ak;
      legPose(j, up ? 1.0f : 0.0f, 0, 0, hip, sh, ak);
      setLegPose((LegID)j, hip, sh, ak);
    }
    copyTargets(P[pair]);
  }

  if (!moveTo(S, 0)) return false;

  for (int k = 0; k < n; k++) {
    if (!oscillate(S, P[0], 1, 2 * ms)) return false;
    if (!oscillate(S, P[1], 1, 2 * ms)) return false;
  }
  return true;
}

bool wave(int n) {
  buildBow1();
  if (!glide(true, 0)) return false;

  float hip, sh, ak;
  raiseLF(hip, sh, ak);
  if (!glide(true, 0)) return false;

  setTarget(legs[LF].hipChan, hip + WAVE_HIP_AMP);
  if (!glide(true, 300)) return false;

  float A[NUM_CH], B[NUM_CH];
  copyTargets(A);
  setTarget(legs[LF].hipChan, hip - WAVE_HIP_AMP);
  copyTargets(B);

  return oscillate(A, B, n, 600);
}

void doBounce() {
  settleGait();
  if (!bounce(3, 600)) return;
  doStand();
}

void doWiggle() {
  settleGait();
  if (!wiggle(4, 300)) return;
  doStand();
}

void doWave() {
  settleGait();
  if (!wave(4)) return;
  doStand();
}

void doDance() {
  settleGait();
  if (!bounce(2, 450))   return;
  if (!wiggle(3, 260))   return;
  if (!pawLifts(2, 450)) return;
  if (!bounce(2, 380))   return;
  if (!wiggle(3, 200))   return;
  if (!wave(3))          return;
  doStand();
}

void doHandshake() {
  settleGait();

  buildBow1();
  glide(false, 0);
  delay(500);

  float hip, sh, ak;
  raiseLF(hip, sh, ak);
  glide(false, 0);
  delay(300);

  // Shoulder shake as one continuous motion
  setTarget(legs[LF].shChan, sh + shSign[LF] * LF_shakeAmp);
  glide(false, SHAKE_HALF_MS);

  float A[NUM_CH], B[NUM_CH];
  copyTargets(A);
  setTarget(legs[LF].shChan, sh - shSign[LF] * LF_shakeAmp);
  copyTargets(B);

  if (!oscillate(A, B, 3, 2 * SHAKE_HALF_MS)) return;

  delay(200);
  doStand();
}


// ============================================================
// JOINT TEST
// ============================================================
// Moves each joint +JOINT_TEST_DEG from stand and back, one at a
// time, slowly. Use it to check that every joint moves, moves the
// expected way (mirrored on the right side), and doesn't bind.
// ============================================================

void jointTest() {
  settleGait();

  const char *legName[4] = { "LF", "RF", "LB", "RB" };
  const char *jName[3]   = { "hip", "shoulder", "ankle" };

  float S[NUM_CH], B[NUM_CH];
  buildStand(); copyTargets(S);
  if (!moveTo(S, 0)) return;

  for (int leg = 0; leg < 4; leg++) {
    for (int jt = 0; jt < 3; jt++) {

      for (int ch = 0; ch < NUM_CH; ch++) B[ch] = S[ch];
      int ch = legs[leg].hipChan + jt;
      B[ch] = clampf(S[ch] + JOINT_TEST_DEG, jMin[ch], jMax[ch]);

      Serial.print(legName[leg]);
      Serial.print(F(" "));
      Serial.print(jName[jt]);
      Serial.print(F("  (ch "));
      Serial.print(ch);
      Serial.print(F(")  "));
      Serial.print(S[ch]);
      Serial.print(F(" -> "));
      Serial.println(B[ch]);

      if (!oscillate(S, B, 1, 2000)) {
        Serial.println(F("Joint test interrupted"));
        return;
      }
      delay(300);
    }
  }
  Serial.println(F("Joint test done"));
}


// ============================================================
// SERIAL
// ============================================================

void printMenu() {
  Serial.println(F("------------------------------------"));
  Serial.println(F("1 Stand       2 Reverse Bend"));
  Serial.println(F("3 Walk fwd    7 Walk back"));
  Serial.println(F("8 Turn left   9 Turn right"));
  Serial.println(F("4 Stop"));
  Serial.println(F("5 Handshake   6 Sit"));
  Serial.println(F("d Dance       w Wave"));
  Serial.println(F("g Wiggle      b Bounce"));
  Serial.println(F("+ / - Walk speed"));
  Serial.println(F("j Joint test"));
  Serial.println(F("------------------------------------"));
}

void handleSerial() {

  if (!cmdPending()) return;

  char c = Serial.read();
  if (c >= 'A' && c <= 'Z') c += 32;

  switch (c) {

    case '1': doStand();   Serial.println(F("-> Stand")); break;
    case '2': doBow();     Serial.println(F("-> Reverse Bend")); break;

    case '3': startGait( 1,  0); Serial.println(F("-> Walk forward"));  break;
    case '7': startGait(-1,  0); Serial.println(F("-> Walk backward")); break;
    case '8': startGait( 0,  1); Serial.println(F("-> Turn left"));     break;
    case '9': startGait( 0, -1); Serial.println(F("-> Turn right"));    break;

    case '4':
      walking = false;            // gait eases itself back to stand
      Serial.println(F("-> Stop"));
      break;

    case '5': doHandshake(); Serial.println(F("-> Handshake")); break;
    case '6': doSit2();      Serial.println(F("-> Sit"));       break;
    case 'd': doDance();     Serial.println(F("-> Dance"));     break;
    case 'w': doWave();      Serial.println(F("-> Wave"));      break;
    case 'g': doWiggle();    Serial.println(F("-> Wiggle"));    break;
    case 'b': doBounce();    Serial.println(F("-> Bounce"));    break;
    case 'j': jointTest();   break;

    case '+':
      WALK_PHASE_MS = max(WALK_MIN_MS, WALK_PHASE_MS - 100);
      Serial.print(F("Step time ms: "));
      Serial.println(WALK_PHASE_MS);
      break;

    case '-':
      WALK_PHASE_MS = min(WALK_MAX_MS, WALK_PHASE_MS + 100);
      Serial.print(F("Step time ms: "));
      Serial.println(WALK_PHASE_MS);
      break;

    default:
      Serial.print(F("[?] Unknown command: "));
      Serial.println(c);
      printMenu();
      break;
  }
}


// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  delay(500);

  // Go straight to the calibrated stand, one leg at a time so
  // 12 servos don't all slam to position (and spike current) at
  // once. We can't know where the legs were at power-up.
  buildStand();
  for (int i = 0; i < 4; i++) {
    writeCh(legs[i].shChan,  tgtAng[legs[i].shChan]);
    writeCh(legs[i].ankChan, tgtAng[legs[i].ankChan]);
    writeCh(legs[i].hipChan, tgtAng[legs[i].hipChan]);
    delay(250);
  }

  Serial.println();
  Serial.println(F("===================================="));
  Serial.println(F(" QUADRUPED ROBOT v5 - SMOOTH"));
  Serial.println(F("===================================="));
  Serial.println();
  printMenu();
}

void loop() {
  handleSerial();
  runGait();
}
