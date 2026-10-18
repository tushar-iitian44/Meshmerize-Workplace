/*
  Meshmerize - 6 Sensor PID Line Follower  (junction / turn / recovery version)
  Board        : Arduino Uno R3
  Motor driver : L298N
  Sensors      : 6 analog IR sensors

  Sensors physically ordered LEFT to RIGHT:  A5, A4, A3, A2, A1, A0
  Sensor_Pins[0] = A5 (leftmost) ... Sensor_Pins[5] = A0 (rightmost)
  pattern bit i = Sensor index i  (bit0 = A5 leftmost, bit5 = A0 rightmost)
  Patterns are PRINTED and WRITTEN as binary literals in A0 -> A5 order,
  exactly like your original sketch:

    011110 = CENTRE / STRAIGHT
    111100 = LINE TO RIGHT   (A0..A3 active)
    001111 = LINE TO LEFT    (A2..A5 active)
    111111 = WIDE LINE / INTERSECTION
    000000 = LINE LOST

  POLARITY (taken from your code): analogRead() < 200  ==>  sensor is ON the line
  (flip LINE_IS_BELOW_THRESHOLD if you ever swap track colours).

  MOTOR MAPPING (taken from your code, preserved):
    ENA/IN1/IN2 = physical RIGHT motor
    ENB/IN3/IN4 = physical LEFT  motor
    Forward = INx HIGH, INy LOW on both (as in your sketch).
    Reverse is only used for pivots (ALLOW_REVERSE).

  INTEGRATED IMPROVEMENTS
   - One-time, independent startup breakaway pulse for each motor.
   - More assertive startup alignment; avoids needless stop when a detected line
     is nearly centred but the full centred sensor pattern is not stable yet.
   - 1.5x line-loss / recovery timing and repeated recovery rather than permanent
     recovery-failed lockout. Serial 'x' emergency stop remains available.
   - Direction evidence is collected from side-specific patterns while crossing
     a confirmed junction; wide bars alone do not automatically prove both sides.
   - Full L > S > R > B decision path; B executes a U-turn when no exit is found.
   - Junction lockout ends only after its minimum time AND the sensor row has
     cleared the wide pattern for several samples.
   - Normal PID following maintains a forward minimum on both wheels; pivot and
     recovery states may still reverse wheels intentionally.

  PID gains were changed with your permission (see section 4). Live tuning:
    'p'/'o' = Kp up/down   'd'/'c' = Kd up/down   'i'/'u' = Ki up/down
    'g' = print gains      'v' = print raw sensor values

  Junction priority: LEFT > STRAIGHT > RIGHT > U-TURN (only if nothing else).

  Serial commands (115200): 'x' = emergency stop (latched), 'r' = restart after emergency stop.

  NOT competition-ready until compiled on your board and tuned on the track.
*/


// ==================================================
// 0. DEBUG SWITCH
// ==================================================

#define DEBUG_SERIAL 1   // set to 0 after tuning: removes all debug prints

#if DEBUG_SERIAL
  #define DBG_PRINT(x)   Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif

const unsigned long DEBUG_INTERVAL_MS = 100;


// ==================================================
// 1. SENSOR SETTINGS
// ==================================================

const int Num_Sensor = 6;

const int Sensor_Pins[Num_Sensor] = {
  A5, A4, A3, A2, A1, A0
};

int Sensor_Value[Num_Sensor];

const int  SENSOR_THRESHOLD         = 200;
const bool LINE_IS_BELOW_THRESHOLD  = true;   // true = reading < 200 means "on line"

// Analog position interpolation (smooth error instead of 0.5-step jumps).
// raw <= ANALOG_FULL counts as fully on the line, raw >= ANALOG_ZERO as off it.
// Use the 'v' serial command to read your real values and adjust these.
const bool  USE_ANALOG_POSITION = true;
const int   ANALOG_FULL = 150;
const int   ANALOG_ZERO = 450;

// Physical sensor positions: left negative, right positive.
const float Position[Num_Sensor] = {
  -2.5, -1.5, -0.5, 0.5, 1.5, 2.5
};

// Bit masks (bit i = Sensor index i, bit0 = A5 = leftmost)
const byte MASK_LEFT_EDGE   = 0b000001;   // A5
const byte MASK_RIGHT_EDGE  = 0b100000;   // A0
const byte MASK_LEFT_OUTER  = 0b000011;   // A5, A4
const byte MASK_RIGHT_OUTER = 0b110000;   // A1, A0
const byte MASK_CENTRE_BOTH = 0b001100;   // A3, A2


// ==================================================
// 2. L298N MOTOR CONNECTIONS (unchanged from original)
// ==================================================

const int ENA = 5;  // Physical RIGHT motor PWM
const int IN1 = 7;
const int IN2 = 8;

const int IN3 = 9;
const int IN4 = 10;
const int ENB = 6;  // Physical LEFT motor PWM

// If one motor spins backwards when commanded "forward", flip its flag.
// (With your current wiring both are false: IN HIGH/LOW = forward.)
const bool RIGHT_MOTOR_INVERT = false;
const bool LEFT_MOTOR_INVERT  = false;

// Pivots (one wheel reversed) need the L298N to run a motor backwards.
// Set false to make pivots into one-wheel arcs instead.
const bool ALLOW_REVERSE = true;

// Optional hardware emergency stop: connect a switch between this pin and
// GND (pin uses INPUT_PULLUP, LOW = stop).  -1 = not used.
const int ESTOP_PIN = -1;


// ==================================================
// 3. PWM SETTINGS  (all tunable)
// ==================================================

const int BASE_SPEED        = 120;   // cruise ceiling on straights (original value)
const int ALIGN_SPEED       = 80;    // original value (start of run)
const int MAX_CORRECTION    = 90;    // original value
const int SEARCH_PWM        = 80;    // original value (line-loss search)

const int MAX_PWM           = 220;   // hard limit for any motor command
const int MOTOR_MIN_PWM     = 40;    // smallest non-zero command (friction); 0 disables
const int FOLLOW_MIN_FORWARD_PWM = 50; // normal PID keeps both wheels moving forward

// One-time breakaway boost, tracked separately for the physical left/right motors.
// Tune these two values to the actual motor/gearbox/battery combination.
const int           POWER_ON_KICK_PWM = 110;
const unsigned long POWER_ON_KICK_MS  = 180;
const int           STARTUP_CREEP_PWM = 80;

const int APPROACH_SPEED    = 80;    // speed while a junction is being confirmed
const int CREEP_SPEED       = 70;    // speed while advancing axle onto the junction
const int REACQUIRE_SPEED   = 75;    // speed after a turn / recovery
const int MIN_CRUISE_SPEED  = 70;    // lowest cruise speed produced by speed scheduling

// Speed scheduling
const float SPEED_REDUCE_PER_ERROR = 14.0;  // PWM removed per sensor-unit of |error|
const float SPEED_RAMP_UP_PER_CYCLE = 1.5;  // PWM added per 10 ms when speeding up

// Pivot / turn power (negative = wheel runs in reverse)
const int TURN_FAST_OUTER   = 140;
const int TURN_FAST_INNER   = -110;
const int TURN_SLOW_OUTER   = 95;
const int TURN_SLOW_INNER   = -75;
const int TURN_BRAKE_PWM    = 90;    // counter-rotation magnitude

// Startup pivot
const int   ALIGN_PIVOT_MIN  = 80;
const int   ALIGN_PIVOT_MAX  = 110;
const float ALIGN_PIVOT_GAIN = 10.0;   // PWM per sensor-unit of |error|

// Recovery
const int SEARCH_WIDEN_PWM   = 15;   // extra PWM on the wider second sweep
const int GAP_CREEP_PWM      = 70;


// ==================================================
// 4. PID SETTINGS  (Kp / Ki / Kd UNCHANGED)
// ==================================================

// Starting gains (error is in sensor-spacing units, 100 Hz loop).
// Original were 25.0 / 0.0 / 0.50.  Tune live with the Serial keys.
float Kp = 24.0;
float Ki = 0.8;
float Kd = 1.20;

float previousError       = 0.0;
float integral            = 0.0;
float filteredDerivative  = 0.0;
float lastCorrection      = 0.0;

const float INTEGRAL_LIMIT = 8.0;
const float ERROR_DEADBAND = 0.05;

// Derivative handling
const float DERIVATIVE_FILTER = 0.35;   // weight of the newest derivative sample
const float DERIVATIVE_LIMIT  = 60.0;   // clamp, sensor-units per second
// Integral only works near the centre; it decays when the error is large.
const float INTEGRAL_ACTIVE_ERROR = 1.0;
const float INTEGRAL_DECAY        = 0.95;

// How the PID output is APPLIED (the gains themselves are not touched):
// beyond this |error| the output is multiplied to pull the line back faster.
const float OUTER_ERROR_THRESHOLD  = 1.8;
const float OUTER_CORRECTION_BOOST = 1.35;


// ==================================================
// 5. TIMING / CONFIRMATION PARAMETERS (all tunable)
// ==================================================

const unsigned long CONTROL_INTERVAL_MS = 10;

// ---- startup alignment
const unsigned long STARTUP_TIMEOUT_MS      = 4000;
const int           STARTUP_CENTRE_SAMPLES  = 8;      // 8 x 10 ms = 80 ms stable
const float         STARTUP_CENTRE_ERROR    = 0.30;
const float         STARTUP_PIVOT_DEADZONE  = 0.30;

// ---- junction detection
const int           JUNCTION_MIN_COUNT      = 5;      // >= this many sensors = wider than the line
const bool          USE_EDGE_PATTERN_CANDIDATES = true;   // permit side-only T-junctions, but require persistence
const unsigned long EDGE_PERSIST_MS         = 60;    // longer than a brief PID drift sample
const int           JUNCTION_MIN_EDGE_SAMPLES = 3;    // repeated edge-pattern samples during confirmation
const unsigned long JUNCTION_CONFIRM_MS     = 25;     // candidate must last this long
const unsigned long JUNCTION_DROP_MS        = 10;     // allows ONE missing sample while confirming
const int           JUNCTION_MIN_WIDE_SAMPLES = 3;    // a real bar stays wide for several samples; noise does not
const unsigned long JUNCTION_CREEP_MS       = 130;    // time to move sensor-array -> axle distance
const unsigned long STRAIGHT_PROBE_MS       = 60;     // last part of creep used to look straight ahead
const int           EVIDENCE_MIN_SAMPLES    = 3;      // persistent side-pattern samples needed to accept a branch
const unsigned long JUNCTION_LOCKOUT_MS     = 150;    // minimum time to leave the handled junction
const unsigned long POST_REACQ_LOCKOUT_MS   = 150;    // minimum time after a turn / recovery
const int           JUNCTION_CLEAR_SAMPLES  = 4;      // also require this many non-wide sensor samples

// ---- turns
const unsigned long TURN_MIN_LEAVE_MS       = 30;     // minimum fast rotation before searching
const unsigned long U_TURN_MIN_LEAVE_MS     = 180;
const unsigned long TURN_TIMEOUT_MS         = 1050;   // allow time for a slower real chassis to find the new line
const unsigned long U_TURN_TIMEOUT_MS       = 3750;   // 1.5x recovery-related U-turn timeout
const int           TURN_CONFIRM_SAMPLES    = 2;      // centred samples needed to end a turn
const int           TURN_ALIGN_MAX_COUNT    = 4;      // line must look like a line, not a bar
const unsigned long TURN_BRAKE_MS           = 25;     // 0 = no brake pulse
const int           U_TURN_DIRECTION        = -1;     // -1 = left, +1 = right

// ---- line loss / recovery
const unsigned long LOSS_CONFIRM_MS         = 30;     // 1.5x: line left through the side
const unsigned long CENTRED_LOSS_CONFIRM_MS = 90;     // 1.5x: line vanished while centred (gap / dead end)
const bool          ENABLE_DEAD_END_UTURN   = true;   // U-turn only after creep + both sweeps found nothing
const float         SHARP_LOSS_ERROR        = 1.2;    // |lastError| >= this => line left via the side
const unsigned long GAP_CREEP_MS            = 150;    // 1.5x
const unsigned long SWEEP1_MS               = 450;    // 1.5x
const unsigned long SWEEP2_MS               = 1350;   // 1.5x
const unsigned long RECOVERY_TIMEOUT_MS     = 3750;   // 1.5x
const int           UTURN_AFTER_FAILED_SEARCHES = 2;  // escalate to a U-turn after repeated failed sweeps
const unsigned long STABLE_RESET_MS         = 500;    // clean tracking this long clears attempt counter
const int           REACQ_CONFIRM_SAMPLES   = 3;
const unsigned long REACQUIRE_MS            = 375;    // 1.5x
const float         REACQUIRE_ALIGN_ERROR   = 1.0;

// ---- safety
const unsigned long MAX_STATE_MS            = 9000;   // 1.5x watchdog; timeout restarts a bounded recovery cycle


// ==================================================
// 6. STATES
// ==================================================

enum RobotStates {
  ST_STARTUP_ALIGN = 0,
  ST_NORMAL_FOLLOWING,
  ST_APPROACHING_INTERSECTION,
  ST_CONFIRMING_INTERSECTION,
  ST_TURNING_LEFT,
  ST_TURNING_RIGHT,
  ST_TURNING_AROUND,
  ST_SEARCHING_FOR_LINE,
  ST_REACQUIRING_LINE,
  ST_ESTOP
};

enum TurnPhases   { TP_LEAVE = 0, TP_FAST, TP_SLOW, TP_BRAKE };
enum RecoveryPhases { REC_CREEP = 0, REC_SWEEP1, REC_SWEEP2 };

// Junction decisions
const byte DIR_LEFT     = 0;
const byte DIR_STRAIGHT = 1;
const byte DIR_RIGHT    = 2;
const byte DIR_UTURN    = 3;

byte robotState = ST_STARTUP_ALIGN;
unsigned long stateEnterTime = 0;


// ==================================================
// 7. GLOBAL RUNTIME VARIABLES
// ==================================================

// ---- sensors
float weightedPosition = 0.0;
float analogWeighted   = 0.0;
float analogWeightSum  = 0.0;
byte  activeCount      = 0;
byte  pattern          = 0;
byte  previousPattern  = 0;
float currentError     = 0.0;

// ---- last valid line information (narrow patterns only)
float lastValidError       = 0.0;
byte  lastNonZeroPattern   = 0b011110;
int   lastTurnDirection    = 1;      // +1 = line was last to the RIGHT, -1 = LEFT

// ---- motor bookkeeping
int   lastLeftCmd  = 0;
int   lastRightCmd = 0;
float cruiseSpeed  = ALIGN_SPEED;

// Independent one-time startup boost state for the physical left/right motors.
// Do not re-arm on PID zero commands; restartRobot() re-arms after an E-stop reset.
bool          leftPowerOnKickStarted = false;
bool          rightPowerOnKickStarted = false;
unsigned long leftPowerOnKickStart = 0;
unsigned long rightPowerOnKickStart = 0;

// ---- timing
unsigned long lastControlTime = 0;
unsigned long lastDebugTime   = 0;

// ---- startup
unsigned long startupStart = 0;
int           startupCentreSamples = 0;

// ---- junction
bool          edgeTiming = false;
unsigned long edgeStart  = 0;
unsigned long approachStart = 0;
unsigned long lastCandidateTime = 0;
unsigned long creepStart = 0;
unsigned long junctionLockoutUntil = 0;
bool junctionClearRequired = false;
int  junctionClearSamples = 0;
int  leftHits = 0, rightHits = 0;
int  approachWideCount = 0;
int  approachEdgeCount = 0;
int  probeSamples = 0, probeHits = 0;
bool availLeft = false, availStraight = false, availRight = false;

// ---- turning
int           turnDir = -1;          // +1 = rotate right, -1 = rotate left
byte          turnPhase = TP_LEAVE;
unsigned long turnStart = 0, turnPhaseStart = 0, turnMinLeaveMs = 0;
int           turnClearSamples = 0, turnAlignSamples = 0;

// ---- loss / recovery
bool          lossTiming = false;
unsigned long lossStart = 0;
byte          recPhase = REC_CREEP;
unsigned long recStart = 0, recPhaseStart = 0;
int           searchDir = 1;
bool          uTurnOnFail = false;
int           recoveryAttempts = 0;
int           reacqSamples = 0;
bool          stableTiming = false;
unsigned long stableStart = 0;

// ---- reacquire
unsigned long reacqStart = 0;

// ---- failure printing
unsigned long lastFailPrint = 0;


// ==================================================
// 8. FUNCTION DECLARATIONS
// ==================================================

void readSensors();
float calculateLinePosition();
void resetPIDState(float error);
float calculatePID(float error, float dt);
int updateCruiseSpeed(float error, int ceilingSpeed);
void followLine(float error, float dt, int ceilingSpeed);
void driveMotor(int pwmPin, int inA, int inB, int cmd, bool invert);
int shapePWM(int cmd);
void setMotorSpeed(int leftCmd, int rightCmd);
void stopMotors();
void applyTurnDrive(int dir, int outer, int inner);
bool isWide();
bool isJunctionCandidateSample();
bool detectIntersection(unsigned long now);
void accumulateJunctionEvidence();
void classifyAvailableDirections();
byte selectTurnDirection(bool l, bool s, bool r);
void armJunctionLockout(unsigned long now, unsigned long minimumMs);
void updateJunctionLockout(unsigned long now);
void beginApproach(unsigned long now);
void runApproaching(unsigned long now, float dt);
void beginConfirming(unsigned long now);
void runConfirming(unsigned long now, float dt);
void executeJunctionDecision(unsigned long now);
void beginTurn(byte stateId, int dir, unsigned long now);
void updateTurn(unsigned long now);
void handleLineLoss(unsigned long now);
void beginLineRecovery(unsigned long now, int dir, bool gapCreepFirst, bool allowUTurn);
void updateLineRecovery(unsigned long now);
bool checkLineReacquisition();
void beginReacquire(unsigned long now);
void runReacquiring(unsigned long now, float dt);
void runNormalFollowing(unsigned long now, float dt);
void performStartupAlignment(unsigned long now);
void retryRecovery(const __FlashStringHelper* why);
void emergencyStop();
void restartRobot(unsigned long now);
void pollSerialCommands();
void changeState(byte s);
const __FlashStringHelper* stateName(byte s);
void printDebug();
void printGains();
void printRaw();


// ==================================================
// 9. SENSORS
// ==================================================

void readSensors() {

  weightedPosition = 0.0;
  analogWeighted = 0.0;
  analogWeightSum = 0.0;
  activeCount = 0;
  pattern = 0;

  for (int i = 0; i < Num_Sensor; i++) {

    Sensor_Value[i] = analogRead(Sensor_Pins[i]);

    bool onLine = LINE_IS_BELOW_THRESHOLD
                    ? (Sensor_Value[i] < SENSOR_THRESHOLD)
                    : (Sensor_Value[i] > SENSOR_THRESHOLD);

    if (onLine) {
      activeCount++;
      weightedPosition += Position[i];
      pattern |= (1U << i);       // bit0 = A5
    }

    // Analog weight 0..1 for position interpolation.
    float w = 0.0;
    if (LINE_IS_BELOW_THRESHOLD) {
      w = (float)(ANALOG_ZERO - Sensor_Value[i]) / (float)(ANALOG_ZERO - ANALOG_FULL);
    } else {
      w = (float)(Sensor_Value[i] - (1023 - ANALOG_ZERO)) / (float)(ANALOG_ZERO - ANALOG_FULL);
    }
    if (w < 0.0) w = 0.0;
    if (w > 1.0) w = 1.0;

    analogWeighted  += w * Position[i];
    analogWeightSum += w;
  }
}

float calculateLinePosition() {
  if (activeCount == 0) {
    return lastValidError;
  }
  if (USE_ANALOG_POSITION && analogWeightSum > 0.3) {
    return analogWeighted / analogWeightSum;
  }
  return weightedPosition / activeCount;
}

bool isWide() {
  return activeCount >= JUNCTION_MIN_COUNT;
}


// ==================================================
// 10. PID  (constants untouched)
// ==================================================

void resetPIDState(float error) {
  integral = 0.0;
  filteredDerivative = 0.0;
  previousError = error;
  lastCorrection = 0.0;
}

float calculatePID(float error, float dt) {

  if (dt < 0.001) dt = 0.001;

  float P = error;

  // Integral: only near the centre, never while saturated, decays elsewhere.
  if (fabs(error) <= INTEGRAL_ACTIVE_ERROR) {
    if (fabs(lastCorrection) < MAX_CORRECTION) {
      integral += error * dt;
    }
  } else {
    integral *= INTEGRAL_DECAY;
  }
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Derivative: clamped, then low-pass filtered.
  float rawDerivative = (error - previousError) / dt;
  rawDerivative = constrain(rawDerivative, -DERIVATIVE_LIMIT, DERIVATIVE_LIMIT);

  filteredDerivative = (1.0 - DERIVATIVE_FILTER) * filteredDerivative
                     + DERIVATIVE_FILTER * rawDerivative;

  float correction = Kp * P + Ki * integral + Kd * filteredDerivative;

  // Stronger pull-back when the line is near an outer sensor.
  if (fabs(error) > OUTER_ERROR_THRESHOLD) {
    correction *= OUTER_CORRECTION_BOOST;
  }

  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  previousError  = error;
  lastCorrection = correction;
  return correction;
}

// Slows down quickly when the error grows, speeds up gradually afterwards.
int updateCruiseSpeed(float error, int ceilingSpeed) {

  float target = ceilingSpeed - SPEED_REDUCE_PER_ERROR * fabs(error);
  if (target < MIN_CRUISE_SPEED) target = MIN_CRUISE_SPEED;
  if (target > ceilingSpeed)     target = ceilingSpeed;

  if (target < cruiseSpeed) {
    cruiseSpeed = target;                       // brake immediately
  } else {
    cruiseSpeed += SPEED_RAMP_UP_PER_CYCLE;     // recover smoothly
    if (cruiseSpeed > target) cruiseSpeed = target;
  }
  return (int)cruiseSpeed;
}

// One PID line-following step.
// Positive error = line to the RIGHT -> left wheel faster, right wheel slower.
void followLine(float error, float dt, int ceilingSpeed) {

  if (error > -ERROR_DEADBAND && error < ERROR_DEADBAND) {
    error = 0.0;
  }

  float correction = calculatePID(error, dt);
  int base = updateCruiseSpeed(error, ceilingSpeed);

  int leftSpeed  = (int)(base + correction);
  int rightSpeed = (int)(base - correction);

  // Normal line following is forward-only: do not let a large PID correction
  // silently stop the inner wheel. Explicit turn/recovery states handle pivots.
  leftSpeed  = constrain(leftSpeed,  FOLLOW_MIN_FORWARD_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, FOLLOW_MIN_FORWARD_PWM, MAX_PWM);

  setMotorSpeed(leftSpeed, rightSpeed);
}


// ==================================================
// 11. MOTOR CONTROL
// ==================================================

void driveMotor(int pwmPin, int inA, int inB, int cmd, bool invert) {

  if (cmd == 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(pwmPin, 0);
    return;
  }

  bool forward = (cmd > 0);
  if (invert) forward = !forward;

  digitalWrite(inA, forward ? HIGH : LOW);
  digitalWrite(inB, forward ? LOW  : HIGH);
  analogWrite(pwmPin, abs(cmd));
}

// Applies the minimum power (friction) and the maximum power limit.
int shapePWM(int cmd) {

  if (cmd == 0) return 0;

  int mag = abs(cmd);
  if (mag < MOTOR_MIN_PWM) mag = MOTOR_MIN_PWM;
  if (mag > MAX_PWM)       mag = MAX_PWM;

  return (cmd > 0) ? mag : -mag;
}

// Signed commands: +forward / -reverse, range -255..255 before shaping.
void setMotorSpeed(int leftCmd, int rightCmd) {

  if (!ALLOW_REVERSE) {
    if (leftCmd  < 0) leftCmd  = 0;
    if (rightCmd < 0) rightCmd = 0;
  }

  // Apply a brief, independent breakaway boost when each physical motor first
  // receives a non-zero command after boot/restart. Keep the sign, so pivots
  // still turn in the requested direction. This is one-time, not every stop/start.
  unsigned long now = millis();

  if (leftCmd != 0 && !leftPowerOnKickStarted) {
    leftPowerOnKickStarted = true;
    leftPowerOnKickStart = now;
  }
  if (leftCmd != 0 && (now - leftPowerOnKickStart < POWER_ON_KICK_MS)) {
    int mag = abs(leftCmd);
    if (mag < POWER_ON_KICK_PWM) mag = POWER_ON_KICK_PWM;
    if (mag > MAX_PWM) mag = MAX_PWM;
    leftCmd = (leftCmd > 0) ? mag : -mag;
  }

  if (rightCmd != 0 && !rightPowerOnKickStarted) {
    rightPowerOnKickStarted = true;
    rightPowerOnKickStart = now;
  }
  if (rightCmd != 0 && (now - rightPowerOnKickStart < POWER_ON_KICK_MS)) {
    int mag = abs(rightCmd);
    if (mag < POWER_ON_KICK_PWM) mag = POWER_ON_KICK_PWM;
    if (mag > MAX_PWM) mag = MAX_PWM;
    rightCmd = (rightCmd > 0) ? mag : -mag;
  }

  leftCmd  = shapePWM(leftCmd);
  rightCmd = shapePWM(rightCmd);

  lastLeftCmd  = leftCmd;
  lastRightCmd = rightCmd;

  // Physical mapping preserved: ENA/IN1/IN2 = RIGHT, ENB/IN3/IN4 = LEFT.
  driveMotor(ENA, IN1, IN2, rightCmd, RIGHT_MOTOR_INVERT);
  driveMotor(ENB, IN3, IN4, leftCmd,  LEFT_MOTOR_INVERT);
}

void stopMotors() {
  setMotorSpeed(0, 0);
}

// dir = +1 rotate right (left wheel outer/forward, right wheel inner)
// dir = -1 rotate left  (right wheel outer/forward, left wheel inner)
void applyTurnDrive(int dir, int outer, int inner) {
  if (dir > 0) {
    setMotorSpeed(outer, inner);
  } else {
    setMotorSpeed(inner, outer);
  }
}


// ==================================================
// 12. STATE HELPERS / SAFETY
// ==================================================

const __FlashStringHelper* stateName(byte s) {
  switch (s) {
    case ST_STARTUP_ALIGN:            return F("STARTUP_ALIGN");
    case ST_NORMAL_FOLLOWING:         return F("NORMAL");
    case ST_APPROACHING_INTERSECTION: return F("APPROACHING");
    case ST_CONFIRMING_INTERSECTION:  return F("CONFIRMING");
    case ST_TURNING_LEFT:             return F("TURNING_LEFT");
    case ST_TURNING_RIGHT:            return F("TURNING_RIGHT");
    case ST_TURNING_AROUND:           return F("TURNING_AROUND");
    case ST_SEARCHING_FOR_LINE:       return F("SEARCHING");
    case ST_REACQUIRING_LINE:         return F("REACQUIRING");
    case ST_ESTOP:                    return F("ESTOP");
  }
  return F("?");
}

void changeState(byte s) {
  if (s == robotState) return;
  robotState = s;
  stateEnterTime = millis();
  DBG_PRINT(F(">>> STATE -> "));
  DBG_PRINTLN(stateName(s));
}

// Recovery failure is non-terminal. Start another bounded search cycle and
// alternate sweep direction. After repeated failed searches, permit a U-turn
// even if the first loss looked like an edge loss, since the line may be behind.
void retryRecovery(const __FlashStringHelper* why) {
  DBG_PRINT(F("!!! RECOVERY RETRY: "));
  DBG_PRINTLN(why);

  int dir = -lastTurnDirection;
  bool gapCreepFirst = false;
  bool allowUTurn = false;

  if (robotState == ST_SEARCHING_FOR_LINE) {
    dir = -searchDir;
    allowUTurn = uTurnOnFail;
    if (!allowUTurn && recoveryAttempts >= UTURN_AFTER_FAILED_SEARCHES) {
      allowUTurn = ENABLE_DEAD_END_UTURN;
      if (allowUTurn) DBG_PRINTLN(F("Recovery escalation: U-turn enabled after repeated failed searches"));
    }
  } else if (robotState == ST_TURNING_AROUND) {
    dir = -turnDir;
    allowUTurn = ENABLE_DEAD_END_UTURN;
  } else if (robotState == ST_TURNING_LEFT || robotState == ST_TURNING_RIGHT) {
    dir = -turnDir;
    allowUTurn = false;
  } else if (robotState == ST_REACQUIRING_LINE) {
    dir = (lastTurnDirection == 0) ? -1 : lastTurnDirection;
    allowUTurn = uTurnOnFail;
  } else if (robotState == ST_STARTUP_ALIGN) {
    dir = lastTurnDirection;
    allowUTurn = false;
  }

  beginLineRecovery(millis(), dir, gapCreepFirst, allowUTurn);
}

void emergencyStop() {
  stopMotors();
  changeState(ST_ESTOP);
}

void restartRobot(unsigned long now) {
  stopMotors();
  resetPIDState(0.0);
  cruiseSpeed = ALIGN_SPEED;
  recoveryAttempts = 0;
  lossTiming = false;
  edgeTiming = false;
  stableTiming = false;
  junctionLockoutUntil = 0;
  junctionClearRequired = false;
  junctionClearSamples = 0;
  leftPowerOnKickStarted = false;
  rightPowerOnKickStarted = false;
  leftPowerOnKickStart = 0;
  rightPowerOnKickStart = 0;
  startupStart = now;
  startupCentreSamples = 0;
  changeState(ST_STARTUP_ALIGN);
  stateEnterTime = now;
}

void printGains() {
  DBG_PRINT(F("Kp="));
  DBG_PRINT(Kp);
  DBG_PRINT(F("  Ki="));
  DBG_PRINT(Ki);
  DBG_PRINT(F("  Kd="));
  DBG_PRINTLN(Kd);
}

void printRaw() {
#if DEBUG_SERIAL
  Serial.print(F("RAW A5..A0: "));
  for (int i = 0; i < Num_Sensor; i++) {
    Serial.print(Sensor_Value[i]);
    Serial.print(F(" "));
  }
  Serial.println();
#endif
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') {
      emergencyStop();
    } else if (c == 'r' || c == 'R') {
      if (robotState == ST_ESTOP) {
        restartRobot(millis());
      }
    } else if (c == 'p') { Kp += 2.0;  printGains();
    } else if (c == 'o') { Kp -= 2.0;  if (Kp < 0.0) Kp = 0.0; printGains();
    } else if (c == 'd') { Kd += 0.2;  printGains();
    } else if (c == 'c') { Kd -= 0.2;  if (Kd < 0.0) Kd = 0.0; printGains();
    } else if (c == 'i') { Ki += 0.1;  printGains();
    } else if (c == 'u') { Ki -= 0.1;  if (Ki < 0.0) Ki = 0.0; printGains();
    } else if (c == 'g') { printGains();
    } else if (c == 'v') { printRaw();
    }
  }
}


// ==================================================
// 13. STARTUP SELF-ALIGNMENT
// ==================================================
//
// Robot stays still, reads the line, and pivots slowly towards it.
// Normal PID tracking begins only after the centre sensors have seen the line
// centred for STARTUP_CENTRE_SAMPLES consecutive samples.
// If no line is seen, it remains stationary rather than driving blindly.

void performStartupAlignment(unsigned long now) {

  if (now - startupStart > STARTUP_TIMEOUT_MS) {
    if (activeCount == 0) {
      // No line detected at all: remain stationary rather than driving blind.
      // Recheck periodically; the serial monitor will show the current pattern.
      stopMotors();
      startupCentreSamples = 0;
      startupStart = now;
      DBG_PRINTLN(F("Startup: no line detected; waiting safely for sensor detection"));
      return;
    }
    retryRecovery(F("startup alignment timeout"));
    return;
  }

  if (activeCount == 0) {
    // Cannot know where the line is: do not guess, do not move.
    stopMotors();
    startupCentreSamples = 0;
    return;
  }

  float error = calculateLinePosition();

  bool centred =
      ((pattern & MASK_CENTRE_BOTH) == MASK_CENTRE_BOTH) &&
      (fabs(error) <= STARTUP_CENTRE_ERROR);

  if (centred) {

    stopMotors();
    startupCentreSamples++;

    if (startupCentreSamples >= STARTUP_CENTRE_SAMPLES) {
      resetPIDState(error);
      cruiseSpeed = ALIGN_SPEED;
      recoveryAttempts = 0;
      // Do not apply a post-reacquisition lockout at initial startup: the first
      // real junction must be detectable immediately after the start line.
      junctionLockoutUntil = 0;
      junctionClearRequired = false;
      junctionClearSamples = 0;
      changeState(ST_NORMAL_FOLLOWING);
    }
    return;
  }

  startupCentreSamples = 0;

  int p = (int)(ALIGN_PIVOT_MIN + ALIGN_PIVOT_GAIN * fabs(error));
  p = constrain(p, ALIGN_PIVOT_MIN, ALIGN_PIVOT_MAX);

  if (error > STARTUP_PIVOT_DEADZONE) {
    applyTurnDrive(+1, p, -p);        // line on the right: rotate right
  } else if (error < -STARTUP_PIVOT_DEADZONE) {
    applyTurnDrive(-1, p, -p);        // line on the left: rotate left
  } else {
    // Line is detected and nearly centred, but the stable two-centre-sensor
    // pattern is not ready. Creep rather than requiring a manual push.
    setMotorSpeed(STARTUP_CREEP_PWM, STARTUP_CREEP_PWM);
  }
}


// ==================================================
// 14. INTERSECTION DETECTION AND CLASSIFICATION
// ==================================================

// A single sample that LOOKS like part of a junction:
//  - wider than the line (>= JUNCTION_MIN_COUNT sensors), or
//  - an edge-shifted pattern (111100 / 001111) when enabled.
bool isJunctionCandidateSample() {
  if (activeCount >= JUNCTION_MIN_COUNT) return true;
  if (USE_EDGE_PATTERN_CANDIDATES &&
      (pattern == 0b111100 || pattern == 0b001111)) return true;
  return false;
}

// Used from NORMAL_FOLLOWING.  Wide patterns trigger immediately (the
// confirmation happens in APPROACHING).  Edge-shifted patterns must persist,
// because ordinary PID drift produces them too.
bool detectIntersection(unsigned long now) {

  if (activeCount >= JUNCTION_MIN_COUNT) {
    edgeTiming = false;
    return true;
  }

  if (USE_EDGE_PATTERN_CANDIDATES &&
      (pattern == 0b111100 || pattern == 0b001111)) {

    if (!edgeTiming) {
      edgeTiming = true;
      edgeStart = now;
    }
    return (now - edgeStart >= EDGE_PERSIST_MS);
  }

  edgeTiming = false;
  return false;
}

// Collect evidence only while traversing a candidate. Exact four-sensor
// patterns 001111/111100 can indicate a one-sided T-junction; they are accepted
// only after EDGE_PERSIST_MS and repeated samples. A confirmed wide bar during
// the actual creep is treated as lateral-branch evidence, because the candidate
// has already persisted and the robot is now crossing the junction.
void accumulateJunctionEvidence() {
  if (activeCount >= JUNCTION_MIN_COUNT) {
    approachWideCount++;
    if (robotState == ST_CONFIRMING_INTERSECTION) {
      if (pattern & MASK_LEFT_OUTER)  leftHits++;
      if (pattern & MASK_RIGHT_OUTER) rightHits++;
    }
    return;
  }

  if (pattern == 0b001111 || pattern == 0b111100) {
    approachEdgeCount++;
  }

  if (activeCount < 4) return;

  bool centreSeen = (pattern & MASK_CENTRE_BOTH) != 0;
  bool leftOuterSeen = (pattern & MASK_LEFT_OUTER) != 0;
  bool rightOuterSeen = (pattern & MASK_RIGHT_OUTER) != 0;

  if (!centreSeen) return;

  if (leftOuterSeen && !rightOuterSeen) {
    leftHits++;
  } else if (rightOuterSeen && !leftOuterSeen) {
    rightHits++;
  }
}

void classifyAvailableDirections() {
  availLeft     = (leftHits >= EVIDENCE_MIN_SAMPLES);
  availRight    = (rightHits >= EVIDENCE_MIN_SAMPLES);
  availStraight = (probeSamples >= 3) && (probeHits * 2 > probeSamples);
}

// Priority: LEFT, then STRAIGHT, then RIGHT, U-turn only if nothing else.
byte selectTurnDirection(bool l, bool s, bool r) {
  if (l) return DIR_LEFT;
  if (s) return DIR_STRAIGHT;
  if (r) return DIR_RIGHT;
  return DIR_UTURN;
}

void beginApproach(unsigned long now) {
  leftHits = 0;
  rightHits = 0;
  approachWideCount = 0;
  approachEdgeCount = 0;
  probeSamples = 0;
  probeHits = 0;
  approachStart = now;
  lastCandidateTime = now;
  accumulateJunctionEvidence();
  changeState(ST_APPROACHING_INTERSECTION);
}

void runApproaching(unsigned long now, float dt) {

  bool candidate = isJunctionCandidateSample();
  if (candidate) lastCandidateTime = now;

  // Keep collecting side-specific evidence for a short transition after the
  // wide bar; branch patterns often appear as the array enters/leaves it.
  accumulateJunctionEvidence();

  bool enoughWideEvidence = (approachWideCount >= JUNCTION_MIN_WIDE_SAMPLES);
  bool enoughEdgeEvidence = (approachEdgeCount >= JUNCTION_MIN_EDGE_SAMPLES);
  bool enoughCandidateEvidence = enoughWideEvidence || enoughEdgeEvidence;

  if (!enoughCandidateEvidence && now - lastCandidateTime > JUNCTION_DROP_MS) {
    // Neither a persistent wide bar nor a persistent side-only pattern: noise/drift.
    changeState(ST_NORMAL_FOLLOWING);
    return;
  }

  if (now - approachStart >= JUNCTION_CONFIRM_MS) {
    if (!enoughCandidateEvidence) {
      changeState(ST_NORMAL_FOLLOWING);
      return;
    }
    beginConfirming(now);
    return;
  }

  // Slow down and keep rolling across the candidate.
  if (activeCount == 0 || isWide()) {
    setMotorSpeed(APPROACH_SPEED, APPROACH_SPEED);
  } else {
    followLine(currentError, dt, APPROACH_SPEED);
  }
}

void beginConfirming(unsigned long now) {
  creepStart = now;
  probeSamples = 0;
  probeHits = 0;
  changeState(ST_CONFIRMING_INTERSECTION);
}

// Advance straight (no steering) until the wheel axle is over the junction,
// remembering which branches the sensor array saw, then look straight ahead.
void runConfirming(unsigned long now, float dt) {
  (void)dt; // this phase intentionally uses equal motor commands, not PID

  // Advance straight while classifying. Steering here can pull the robot away
  // from the branch being measured and can make it pass an exit before a
  // decision is made. The normal PID state resumes after classification.
  setMotorSpeed(CREEP_SPEED, CREEP_SPEED);

  // Keep sampling side patterns while travelling over the junction.
  accumulateJunctionEvidence();

  unsigned long t = now - creepStart;

  if (t + STRAIGHT_PROBE_MS >= JUNCTION_CREEP_MS) {
    probeSamples++;
    bool lineAhead = ((pattern & MASK_CENTRE_BOTH) == MASK_CENTRE_BOTH) &&
                     (activeCount < JUNCTION_MIN_COUNT);
    if (lineAhead) probeHits++;
  }

  if (t >= JUNCTION_CREEP_MS) {
    executeJunctionDecision(now);
  }
}

void armJunctionLockout(unsigned long now, unsigned long minimumMs) {
  junctionLockoutUntil = now + minimumMs;
  junctionClearRequired = true;
  junctionClearSamples = 0;
}

// A time-only lockout can hide a second junction or keep the robot driving over
// the first one. Release it only after both the minimum time and four non-wide
// sensor samples have elapsed.
void updateJunctionLockout(unsigned long now) {
  if (!junctionClearRequired) return;

  if (activeCount < JUNCTION_MIN_COUNT) {
    if (junctionClearSamples < JUNCTION_CLEAR_SAMPLES) junctionClearSamples++;
  } else {
    junctionClearSamples = 0;
  }

  bool minimumTimePassed = ((long)(now - junctionLockoutUntil) >= 0);
  if (minimumTimePassed && junctionClearSamples >= JUNCTION_CLEAR_SAMPLES) {
    junctionClearRequired = false;
    junctionClearSamples = 0;
  }
}

void executeJunctionDecision(unsigned long now) {

  classifyAvailableDirections();
  byte choice = selectTurnDirection(availLeft, availStraight, availRight);

  DBG_PRINT(F("JUNCTION L/S/R = "));
  DBG_PRINT(availLeft);
  DBG_PRINT(availStraight);
  DBG_PRINT(availRight);
  DBG_PRINT(F("  choice = "));
  DBG_PRINTLN(choice);          // 0 left, 1 straight, 2 right, 3 U-turn

  if (choice == DIR_LEFT) {
    beginTurn(ST_TURNING_LEFT, -1, now);
  } else if (choice == DIR_STRAIGHT) {
    // Straight is selected only after L has been ruled out by the evidence.
    resetPIDState(0.0);
    armJunctionLockout(now, JUNCTION_LOCKOUT_MS);
    lossTiming = false;
    edgeTiming = false;
    changeState(ST_NORMAL_FOLLOWING);
  } else if (choice == DIR_RIGHT) {
    beginTurn(ST_TURNING_RIGHT, +1, now);
  } else {
    // L/S/R were not detected during the confirmed junction traversal: apply
    // the B priority and execute a U-turn rather than silently continuing.
    DBG_PRINTLN(F("LSRB: no L/S/R exit proven -> B / U-turn"));
    beginTurn(ST_TURNING_AROUND, U_TURN_DIRECTION, now);
  }
}


// ==================================================
// 15. SENSOR-GUIDED TURNING (LEFT / RIGHT / U-TURN)
// ==================================================
//
// Phases:
//   LEAVE : fast pivot until the OLD line has left the centre sensors
//   FAST  : fast pivot until the NEW line shows on the turn-side outer pair
//   SLOW  : slow pivot until both centre sensors are on the new line
//   BRAKE : short counter-rotation so the robot does not coast past the line
// then REACQUIRING_LINE (slow PID).  A timeout hands over to line recovery.

void beginTurn(byte stateId, int dir, unsigned long now) {

  turnDir = dir;
  turnPhase = TP_LEAVE;
  turnStart = now;
  turnPhaseStart = now;
  turnClearSamples = 0;
  turnAlignSamples = 0;
  turnMinLeaveMs = (stateId == ST_TURNING_AROUND) ? U_TURN_MIN_LEAVE_MS
                                                  : TURN_MIN_LEAVE_MS;
  resetPIDState(0.0);
  changeState(stateId);
}

void updateTurn(unsigned long now) {

  unsigned long timeout = (robotState == ST_TURNING_AROUND) ? U_TURN_TIMEOUT_MS
                                                            : TURN_TIMEOUT_MS;

  if (now - turnStart > timeout) {
    DBG_PRINTLN(F("TURN TIMEOUT -> recovery"));
    bool timedOutDuringUTurn = (robotState == ST_TURNING_AROUND);
    // If a U-turn itself times out, preserve the ability to retry it after
    // another bounded sweep. A failed 90-degree turn does not immediately imply B.
    beginLineRecovery(now, -turnDir, false,
                      timedOutDuringUTurn && ENABLE_DEAD_END_UTURN);
    return;
  }

  byte sideMask = (turnDir > 0) ? MASK_RIGHT_OUTER : MASK_LEFT_OUTER;

  bool centred =
      ((pattern & MASK_CENTRE_BOTH) == MASK_CENTRE_BOTH) &&
      (activeCount <= TURN_ALIGN_MAX_COUNT);

  if (turnPhase == TP_LEAVE) {

    applyTurnDrive(turnDir, TURN_FAST_OUTER, TURN_FAST_INNER);

    if (!centred) turnClearSamples++; else turnClearSamples = 0;

    if (now - turnPhaseStart >= turnMinLeaveMs && turnClearSamples >= 2) {
      turnPhase = TP_FAST;
      turnPhaseStart = now;
      turnAlignSamples = 0;
    }
    return;
  }

  if (turnPhase == TP_FAST || turnPhase == TP_SLOW) {

    if (turnPhase == TP_FAST) {
      applyTurnDrive(turnDir, TURN_FAST_OUTER, TURN_FAST_INNER);
    } else {
      applyTurnDrive(turnDir, TURN_SLOW_OUTER, TURN_SLOW_INNER);
    }

    if (centred) turnAlignSamples++; else turnAlignSamples = 0;

    if (turnAlignSamples >= TURN_CONFIRM_SAMPLES) {
      if (TURN_BRAKE_MS > 0) {
        turnPhase = TP_BRAKE;
        turnPhaseStart = now;
      } else {
        beginReacquire(now);
      }
      return;
    }

    if (turnPhase == TP_FAST && (pattern & sideMask)) {
      turnPhase = TP_SLOW;           // new line is close: slow down
      turnPhaseStart = now;
    }
    return;
  }

  // TP_BRAKE
  applyTurnDrive(-turnDir, TURN_BRAKE_PWM, -TURN_BRAKE_PWM);
  if (now - turnPhaseStart >= TURN_BRAKE_MS) {
    beginReacquire(now);
  }
}


// ==================================================
// 16. LINE LOSS RECOVERY
// ==================================================

// Called whenever all sensors read zero in a state that expects a line.
void handleLineLoss(unsigned long now) {

  // Decide HOW the line was lost from the last valid readings.
  bool leftEdge  = (lastNonZeroPattern & MASK_LEFT_EDGE)  != 0;
  bool rightEdge = (lastNonZeroPattern & MASK_RIGHT_EDGE) != 0;

  bool lostAtEdge = (fabs(lastValidError) >= SHARP_LOSS_ERROR) ||
                    (leftEdge != rightEdge);

  if (!lossTiming) {
    lossTiming = true;
    lossStart = now;
  }

  unsigned long needed = lostAtEdge ? LOSS_CONFIRM_MS : CENTRED_LOSS_CONFIRM_MS;

  if (now - lossStart < needed) {
    // Short drop-out: keep the previous command, do not react yet.
    setMotorSpeed(lastLeftCmd, lastRightCmd);
    return;
  }

  int dir;
  if (rightEdge && !leftEdge)       dir = +1;
  else if (leftEdge && !rightEdge)  dir = -1;
  else if (lastValidError > 0.0)    dir = +1;
  else if (lastValidError < 0.0)    dir = -1;
  else                              dir = lastTurnDirection;

  if (lostAtEdge) {
    // Line slid off the side: bounded pivot search, no U-turn.
    beginLineRecovery(now, dir, false, false);
  } else {
    // Line vanished while centred: gap or dead end.
    beginLineRecovery(now, dir, true, ENABLE_DEAD_END_UTURN);
  }
}

void beginLineRecovery(unsigned long now, int dir, bool gapCreepFirst, bool allowUTurn) {

  // Attempts intentionally have no maximum. Saturate to avoid AVR int overflow.
  if (recoveryAttempts < 30000) recoveryAttempts++;

  searchDir = (dir >= 0) ? +1 : -1;
  uTurnOnFail = allowUTurn;
  recStart = now;
  recPhaseStart = now;
  recPhase = gapCreepFirst ? REC_CREEP : REC_SWEEP1;
  reacqSamples = 0;
  lossTiming = false;
  stableTiming = false;
  resetPIDState(0.0);

  changeState(ST_SEARCHING_FOR_LINE);
  // changeState() is a no-op if already searching; refresh the watchdog anyway.
  stateEnterTime = now;
}

// Any non-zero pattern can be a valid line during a recovery search, including
// a wide intersection (all six sensors). Requiring <=5 sensors would make the
// robot sweep past some real intersections indefinitely.
bool checkLineReacquisition() {
  if (activeCount > 0) {
    reacqSamples++;
  } else {
    reacqSamples = 0;
  }
  return (reacqSamples >= REACQ_CONFIRM_SAMPLES);
}

void updateLineRecovery(unsigned long now) {

  if (checkLineReacquisition()) {
    beginReacquire(now);
    return;
  }

  if (now - recStart > RECOVERY_TIMEOUT_MS) {
    retryRecovery(F("search timeout"));
    return;
  }

  if (recPhase == REC_CREEP) {

    setMotorSpeed(GAP_CREEP_PWM, GAP_CREEP_PWM);

    if (now - recPhaseStart >= GAP_CREEP_MS) {
      recPhase = REC_SWEEP1;
      recPhaseStart = now;
    }
    return;
  }

  if (recPhase == REC_SWEEP1) {

    applyTurnDrive(searchDir, SEARCH_PWM, -SEARCH_PWM);

    if (now - recPhaseStart >= SWEEP1_MS) {
      recPhase = REC_SWEEP2;
      recPhaseStart = now;
    }
    return;
  }

  // REC_SWEEP2: wider sweep in the opposite direction
  applyTurnDrive(-searchDir, SEARCH_PWM + SEARCH_WIDEN_PWM,
                             -(SEARCH_PWM + SEARCH_WIDEN_PWM));

  if (now - recPhaseStart >= SWEEP2_MS) {

    if (uTurnOnFail) {
      int retryUTurnDir = (recoveryAttempts % 2 == 0) ? +1 : -1;
      DBG_PRINTLN(F("No line after bounded sweep: attempting U-turn"));
      beginTurn(ST_TURNING_AROUND, retryUTurnDir, now);
    } else {
      retryRecovery(F("line not found"));
    }
  }
}

// Smooth hand-over from a turn or recovery back to PID.
void beginReacquire(unsigned long now) {
  reacqStart = now;
  lossTiming = false;
  cruiseSpeed = REACQUIRE_SPEED;
  resetPIDState(currentError);       // no derivative spike
  changeState(ST_REACQUIRING_LINE);
}

void runReacquiring(unsigned long now, float dt) {

  if (activeCount == 0) {
    handleLineLoss(now);
    return;
  }
  lossTiming = false;

  if (isWide()) {
    setMotorSpeed(REACQUIRE_SPEED, REACQUIRE_SPEED);
  } else {
    followLine(currentError, dt, REACQUIRE_SPEED);
  }

  if (now - reacqStart >= REACQUIRE_MS &&
      !isWide() &&
      fabs(currentError) <= REACQUIRE_ALIGN_ERROR) {

    armJunctionLockout(now, POST_REACQ_LOCKOUT_MS);
    edgeTiming = false;
    changeState(ST_NORMAL_FOLLOWING);
  }
}


// ==================================================
// 17. NORMAL LINE FOLLOWING
// ==================================================

void runNormalFollowing(unsigned long now, float dt) {

  if (activeCount == 0) {
    handleLineLoss(now);
    return;
  }
  lossTiming = false;

  // Clean tracking for a while clears the recovery attempt counter.
  if (activeCount < JUNCTION_MIN_COUNT) {
    if (!stableTiming) {
      stableTiming = true;
      stableStart = now;
    } else if (now - stableStart >= STABLE_RESET_MS) {
      recoveryAttempts = 0;
    }
  } else {
    stableTiming = false;
  }

  bool lockedOut = junctionClearRequired || ((long)(now - junctionLockoutUntil) < 0);

  if (!lockedOut) {
    if (detectIntersection(now)) {
      beginApproach(now);
      return;
    }
  } else {
    edgeTiming = false;
  }

  // A wide pattern (inside the lockout window) has no usable line position.
  if (isWide()) {
    int s = (int)cruiseSpeed;
    setMotorSpeed(s, s);
    return;
  }

  followLine(currentError, dt, BASE_SPEED);
}


// ==================================================
// 18. SERIAL DEBUGGING
// ==================================================

void printDebug() {

#if DEBUG_SERIAL
  unsigned long now = millis();

  if (now - lastDebugTime < DEBUG_INTERVAL_MS) {
    return;
  }
  lastDebugTime = now;

  Serial.print(stateName(robotState));
  Serial.print(F(" | Pattern A0->A5: "));

  for (int bit = 5; bit >= 0; bit--) {
    Serial.print((pattern & (1U << bit)) ? '1' : '0');
  }

  Serial.print(F(" | Error: "));
  Serial.print(currentError, 2);

  Serial.print(F(" | PID: "));
  Serial.print(lastCorrection, 1);

  Serial.print(F(" | PWM L/R: "));
  Serial.print(lastLeftCmd);
  Serial.print(F("/"));
  Serial.print(lastRightCmd);

  Serial.print(F(" | Active: "));
  Serial.println(activeCount);
#endif
}


// ==================================================
// 19. SETUP
// ==================================================

void setup() {

  for (int i = 0; i < Num_Sensor; i++) {
    pinMode(Sensor_Pins[i], INPUT);
  }

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  if (ESTOP_PIN >= 0) {
    pinMode(ESTOP_PIN, INPUT_PULLUP);
  }

  Serial.begin(115200);

  stopMotors();

  unsigned long now = millis();
  lastControlTime = now;
  lastDebugTime = now;
  startupStart = now;
  stateEnterTime = now;
  robotState = ST_STARTUP_ALIGN;

  DBG_PRINTLN(F("Meshmerize junction PID ready"));
  DBG_PRINTLN(F("Startup alignment active  ('x' = e-stop, 'r' = restart after e-stop)"));
}


// ==================================================
// 20. MAIN LOOP
// ==================================================

void loop() {

  unsigned long now = millis();

  pollSerialCommands();

  if (ESTOP_PIN >= 0 && digitalRead(ESTOP_PIN) == LOW) {
    emergencyStop();
  }

  // Emergency stop always wins; nothing below can override it.
  if (robotState == ST_ESTOP) {
    if (lastLeftCmd != 0 || lastRightCmd != 0) stopMotors();
    return;
  }

  unsigned long elapsed = now - lastControlTime;

  // Consistent PID sampling interval (about 10 ms).
  if (elapsed < CONTROL_INTERVAL_MS) {
    return;
  }

  float dt = elapsed / 1000.0;
  lastControlTime = now;

  // ---- read sensors and update line memory ----
  readSensors();

  currentError = calculateLinePosition();
  updateJunctionLockout(now);

  // Watchdog restarts a bounded search instead of permanently disabling motors.
  // Startup alignment has its own timeout and remains stationary if no sensor
  // sees the line at all, avoiding blind movement from an unknown start pose.
  if (robotState != ST_NORMAL_FOLLOWING && robotState != ST_STARTUP_ALIGN &&
      (now - stateEnterTime) > MAX_STATE_MS) {
    retryRecovery(F("state watchdog"));
    printDebug();
    return;
  }


  if (activeCount > 0 && activeCount < JUNCTION_MIN_COUNT) {
    lastValidError = currentError;
    lastNonZeroPattern = pattern;

    if (currentError > 0.15) {
      lastTurnDirection = 1;
    } else if (currentError < -0.15) {
      lastTurnDirection = -1;
    }
  }

  // ---- state machine ----
  switch (robotState) {

    case ST_STARTUP_ALIGN:
      performStartupAlignment(now);
      break;

    case ST_NORMAL_FOLLOWING:
      runNormalFollowing(now, dt);
      break;

    case ST_APPROACHING_INTERSECTION:
      runApproaching(now, dt);
      break;

    case ST_CONFIRMING_INTERSECTION:
      runConfirming(now, dt);
      break;

    case ST_TURNING_LEFT:
    case ST_TURNING_RIGHT:
    case ST_TURNING_AROUND:
      updateTurn(now);
      break;

    case ST_SEARCHING_FOR_LINE:
      updateLineRecovery(now);
      break;

    case ST_REACQUIRING_LINE:
      runReacquiring(now, dt);
      break;

    default:
      stopMotors();
      break;
  }

  previousPattern = pattern;

  printDebug();
}
