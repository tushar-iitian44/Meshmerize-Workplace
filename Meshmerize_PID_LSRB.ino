/*
  Meshmerize - 6 sensor PID white-line maze follower
  Board: Arduino Uno R3 | Driver: L298N

  Sensor order on the robot, LEFT to RIGHT:
    A5, A4, A3, A2, A1, A0

  The Serial pattern is printed A0 -> A5, therefore it is the reverse of
  the physical left-to-right order.  For example, 011110 is centred.

  At every junction this sketch uses the LSR rule:
    Left -> Straight -> Right
  It NEVER makes a 180 degree/U-turn.  A persistent 000000 pattern is treated
  as a lost line and safely stops both motors instead.

  IMPORTANT: This is for a WHITE line on a dark floor.  It assumes a white
  surface reads BELOW SENSOR_THRESHOLD.  Tune that one value on the actual
  track before increasing speed.
*/

// ==================================================
// 1. SENSOR SETTINGS
// ==================================================

const byte NUM_SENSORS = 6;

// Physical order: robot LEFT -> RIGHT.
const byte SENSOR_PINS[NUM_SENSORS] = {A5, A4, A3, A2, A1, A0};
const float SENSOR_POSITION[NUM_SENSORS] = {-2.5f, -1.5f, -0.5f,
                                             0.5f,  1.5f,  2.5f};
const int SENSOR_THRESHOLD = 200;

// ==================================================
// 2. L298N CONNECTIONS
// ==================================================
// ENA/IN1/IN2 drive the physical RIGHT motor.
// ENB/IN3/IN4 drive the physical LEFT motor.

const byte ENA = 5;  // right motor PWM
const byte IN1 = 7;
const byte IN2 = 8;
const byte IN3 = 9;
const byte IN4 = 10;
const byte ENB = 6;  // left motor PWM

// Keep these true if the original forward-only program drove forward with
// IN1=HIGH, IN2=LOW and IN3=HIGH, IN4=LOW.  Change only the motor that is
// physically reversed after lifting the robot and testing it.
const bool RIGHT_FORWARD_IN1_HIGH = true;
const bool LEFT_FORWARD_IN3_HIGH = true;

// ==================================================
// 3. SPEED AND TIME SETTINGS
// ==================================================

const int BASE_SPEED = 120;
const int ALIGN_SPEED = 80;
const int TURN_PWM = 105;
const int JUNCTION_ADVANCE_PWM = 80;
const int MAX_CORRECTION = 90;

const unsigned long CONTROL_INTERVAL_MS = 10;
const unsigned long DEBUG_INTERVAL_MS = 100;

// A junction must remain visible for this long before the robot commits.
// This filters a single noisy outer-sensor reading.
const unsigned long JUNCTION_CONFIRM_MS = 35;
const unsigned long JUNCTION_COOLDOWN_MS = 250;

// When no left route exists, the robot gently moves past the junction before
// deciding between Straight and Right.  This lets it distinguish a real
// straight exit from a right-only 90 degree corner.  Tune for the width of
// the painted junction and your wheel speed.
const unsigned long JUNCTION_ADVANCE_MS = 90;

// Turning finishes only after the old line was left and the new centred line
// was found.  Increase TURN_MIN_MS if the chassis turns too little.
const unsigned long TURN_MIN_MS = 105;
const unsigned long TURN_TIMEOUT_MS = 1500;

// A 000000 pattern must persist for this long before the robot stops.  This
// filters a single bad reading.  It does NOT cause a U-turn.
const unsigned long LINE_LOST_CONFIRM_MS = 70;

const unsigned long CENTER_STABLE_MS = 250;

// ==================================================
// 4. PID SETTINGS (kept from the working follower)
// ==================================================

float Kp = 25.0f;
float Ki = 0.0f;
float Kd = 0.50f;

float previousError = 0.0f;
float integral = 0.0f;
float filteredDerivative = 0.0f;

const float INTEGRAL_LIMIT = 8.0f;
const float ERROR_DEADBAND = 0.08f;

// ==================================================
// 5. STATE
// ==================================================

enum RobotState {
  FOLLOW_LINE,
  CONFIRM_JUNCTION,
  ADVANCE_JUNCTION,
  TURNING,
  CONFIRM_LINE_LOST,
  FAULT_STOP
};

struct SensorFrame {
  int raw[NUM_SENSORS];
  bool active[NUM_SENSORS];
  byte pattern;          // bit 0=A5 (left), bit 5=A0 (right)
  byte activeCount;
  float error;
  bool leftChoice;
  bool straightChoice;
  bool rightChoice;
};

RobotState state = FOLLOW_LINE;
SensorFrame sensors;

bool startupAlign = true;
bool centerTimerActive = false;
unsigned long centerStartTime = 0;

unsigned long lastControlTime = 0;
unsigned long lastDebugTime = 0;
unsigned long stateStartTime = 0;
unsigned long lastJunctionTime = 0;
float controlDt = CONTROL_INTERVAL_MS / 1000.0f;

// -1 = left, +1 = right.  No other turn direction is allowed.
int requestedTurn = 0;
bool turnLeftOldLine = false;
bool junctionSawLeft = false;
bool junctionSawRight = false;

// ==================================================
// 6. MOTOR CONTROL
// ==================================================
// Speeds are signed: positive = forward, negative = reverse.

void driveRightMotor(int speed) {
  bool forward = speed >= 0;
  int pwm = speed >= 0 ? speed : -speed;
  pwm = constrain(pwm, 0, 255);

  if (pwm == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    return;
  }

  bool in1High = forward ? RIGHT_FORWARD_IN1_HIGH : !RIGHT_FORWARD_IN1_HIGH;
  digitalWrite(IN1, in1High ? HIGH : LOW);
  digitalWrite(IN2, in1High ? LOW : HIGH);
  analogWrite(ENA, pwm);
}

void driveLeftMotor(int speed) {
  bool forward = speed >= 0;
  int pwm = speed >= 0 ? speed : -speed;
  pwm = constrain(pwm, 0, 255);

  if (pwm == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
    return;
  }

  bool in3High = forward ? LEFT_FORWARD_IN3_HIGH : !LEFT_FORWARD_IN3_HIGH;
  digitalWrite(IN3, in3High ? HIGH : LOW);
  digitalWrite(IN4, in3High ? LOW : HIGH);
  analogWrite(ENB, pwm);
}

void setMotors(int leftSpeed, int rightSpeed) {
  driveLeftMotor(leftSpeed);
  driveRightMotor(rightSpeed);
}

void stopMotors() {
  setMotors(0, 0);
}

// ==================================================
// 7. SENSORS, PATTERNS, AND JUNCTIONS
// ==================================================

void readSensors(SensorFrame &sample) {
  float weightedPosition = 0.0f;
  sample.pattern = 0;
  sample.activeCount = 0;

  for (byte i = 0; i < NUM_SENSORS; i++) {
    sample.raw[i] = analogRead(SENSOR_PINS[i]);
    sample.active[i] = sample.raw[i] < SENSOR_THRESHOLD;

    if (sample.active[i]) {
      sample.activeCount++;
      weightedPosition += SENSOR_POSITION[i];
      sample.pattern |= (1U << i);
    }
  }

  sample.error = sample.activeCount > 0
                 ? weightedPosition / sample.activeCount
                 : 0.0f;

  // An outer physical sensor seeing white is the branch evidence.  The four
  // inner sensors alone (011110) are the normal centred-line pattern.
  sample.leftChoice = sample.active[0] && sample.activeCount >= 4;
  sample.rightChoice = sample.active[5] && sample.activeCount >= 4;
  sample.straightChoice = sample.active[2] || sample.active[3];
}

bool lineVisible(const SensorFrame &sample) {
  return sample.activeCount > 0;
}

bool centredLine(const SensorFrame &sample) {
  // The desired line must be in the central sensors, no longer on either
  // outer sensor, and must not be a wide intersection.
  return sample.straightChoice && !sample.active[0] && !sample.active[5]
         && sample.activeCount <= 4;
}

bool looksLikeJunction(const SensorFrame &sample) {
  return sample.leftChoice || sample.rightChoice;
}

char chooseLSR(bool leftAvailable, bool straightAvailable,
               bool rightAvailable) {
  // Fixed L -> S -> R priority.  There is deliberately no Back/U-turn case.
  if (leftAvailable) return 'L';
  if (straightAvailable) return 'S';
  if (rightAvailable) return 'R';
  return 'N';  // no usable L/S/R route
}

// ==================================================
// 8. SERIAL DEBUGGING
// ==================================================

const char *stateName() {
  switch (state) {
    case FOLLOW_LINE:      return startupAlign ? "STARTUP ALIGNING" : "PID TRACKING";
    case CONFIRM_JUNCTION: return "JUNCTION CONFIRM";
    case ADVANCE_JUNCTION: return "JUNCTION CHECK S/R";
    case TURNING:          return requestedTurn == -1 ? "TURN LEFT" : "TURN RIGHT";
    case CONFIRM_LINE_LOST:return "LINE LOST CONFIRM";
    case FAULT_STOP:       return "FAULT STOP";
    default:               return "UNKNOWN";
  }
}

void printDebug(float correction, int leftSpeed, int rightSpeed) {
  unsigned long now = millis();
  if (now - lastDebugTime < DEBUG_INTERVAL_MS) return;
  lastDebugTime = now;

  Serial.print(stateName());
  Serial.print(F(" | Pattern A0->A5: "));
  for (int bit = 5; bit >= 0; bit--) {
    Serial.print((sensors.pattern & (1U << bit)) ? '1' : '0');
  }
  Serial.print(F(" | Error: "));
  Serial.print(sensors.error, 2);
  Serial.print(F(" | PID: "));
  Serial.print(correction, 1);
  Serial.print(F(" | PWM L/R: "));
  Serial.print(leftSpeed);
  Serial.print('/');
  Serial.print(rightSpeed);
  Serial.print(F(" | Active: "));
  Serial.println(sensors.activeCount);
}

// ==================================================
// 9. STATE CHANGES
// ==================================================

void resetPid(float newError) {
  previousError = newError;
  integral = 0.0f;
  filteredDerivative = 0.0f;
}

void beginTurn(int direction) {
  requestedTurn = direction;
  turnLeftOldLine = false;
  stateStartTime = millis();
  resetPid(sensors.error);
  state = TURNING;
}

void enterFaultStop(const __FlashStringHelper *reason) {
  stopMotors();
  state = FAULT_STOP;
  Serial.print(F("FAULT: "));
  Serial.println(reason);
}

void resolveJunction() {
  // The candidate may have been a momentary wide reading.  Re-check before
  // turning, rather than committing to an old sensor frame.
  if (!looksLikeJunction(sensors)) {
    state = FOLLOW_LINE;
    return;
  }

  lastJunctionTime = millis();
  junctionSawLeft = junctionSawLeft || sensors.leftChoice;
  junctionSawRight = junctionSawRight || sensors.rightChoice;

  // The first part of LSR is unambiguous: if the left branch is present,
  // take it immediately.  For S-versus-R, the robot must move beyond the
  // wide paint before it can know whether "straight" really exits.
  if (chooseLSR(junctionSawLeft, false, junctionSawRight) == 'L') {
    Serial.println(F("LSR choice: L"));
    beginTurn(-1);
  } else {
    stateStartTime = millis();
    state = ADVANCE_JUNCTION;
  }
}

// ==================================================
// 10. PID FOLLOWING
// ==================================================

void runPid() {
  float error = sensors.error;

  if (error > -ERROR_DEADBAND && error < ERROR_DEADBAND) error = 0.0f;

  if (startupAlign) {
    if (sensors.pattern == 0b011110 && error > -0.2f && error < 0.2f) {
      if (!centerTimerActive) {
        centerTimerActive = true;
        centerStartTime = millis();
      }
      if (millis() - centerStartTime >= CENTER_STABLE_MS) {
        startupAlign = false;
        centerTimerActive = false;
        Serial.println(F("STARTUP ALIGNMENT COMPLETE"));
      }
    } else {
      centerTimerActive = false;
    }
  }

  float dt = controlDt;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float rawDerivative = (error - previousError) / dt;
  filteredDerivative = 0.75f * filteredDerivative + 0.25f * rawDerivative;

  float correction = Kp * error + Ki * integral + Kd * filteredDerivative;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  int currentBaseSpeed = startupAlign ? ALIGN_SPEED : BASE_SPEED;
  int leftSpeed = constrain((int)(currentBaseSpeed + correction), 0, 255);
  int rightSpeed = constrain((int)(currentBaseSpeed - correction), 0, 255);

  setMotors(leftSpeed, rightSpeed);
  previousError = error;
  printDebug(correction, leftSpeed, rightSpeed);
}

// ==================================================
// 11. TURN AND DEAD-END HANDLING
// ==================================================

void runTurn() {
  unsigned long elapsed = millis() - stateStartTime;

  // Pivot turns use reverse on one wheel and forward on the other.  Only a
  // 90 degree left or right pivot can be requested; U-turns are not present.
  if (requestedTurn == -1) {
    setMotors(-TURN_PWM, TURN_PWM);       // pivot left
  } else if (requestedTurn == 1) {
    setMotors(TURN_PWM, -TURN_PWM);       // pivot right
  } else {
    enterFaultStop(F("invalid turn request"));
    return;
  }

  // At a junction the central sensors begin on white.  Require them to leave
  // that original line before accepting a new centred line as turn-complete.
  if (!sensors.straightChoice) turnLeftOldLine = true;

  if (elapsed >= TURN_MIN_MS && turnLeftOldLine && centredLine(sensors)) {
    stopMotors();
    resetPid(sensors.error);
    lastJunctionTime = millis();
    state = FOLLOW_LINE;
    Serial.println(F("Turn complete: line reacquired"));
    return;
  }

  if (elapsed >= TURN_TIMEOUT_MS) {
    enterFaultStop(F("turn timeout; check motor direction, sensor order, and TURN_PWM"));
  }

  printDebug(0.0f, requestedTurn == -1 ? -TURN_PWM : TURN_PWM,
                    requestedTurn == -1 ? TURN_PWM : -TURN_PWM);
}

void runLineLostConfirm() {
  stopMotors();

  if (lineVisible(sensors)) {
    // It was a brief dropout, not a lost line.
    resetPid(sensors.error);
    state = FOLLOW_LINE;
    return;
  }

  if (millis() - stateStartTime >= LINE_LOST_CONFIRM_MS) {
    // Intentionally stop.  A sensor array alone cannot reliably distinguish
    // a true dead end from a missing/dirty section of line, so a U-turn here
    // would make the robot behave unpredictably.
    enterFaultStop(F("line lost: motors stopped; no U-turn is permitted"));
    return;
  }

  printDebug(0.0f, 0, 0);
}

void runJunctionAdvance() {
  // A right-only 90 degree corner can still illuminate the middle sensors
  // while the array is over the intersection.  Crawl forward briefly, record
  // every side branch seen, then test for a centred exit.
  setMotors(JUNCTION_ADVANCE_PWM, JUNCTION_ADVANCE_PWM);
  junctionSawLeft = junctionSawLeft || sensors.leftChoice;
  junctionSawRight = junctionSawRight || sensors.rightChoice;

  if (millis() - stateStartTime < JUNCTION_ADVANCE_MS) {
    printDebug(0.0f, JUNCTION_ADVANCE_PWM, JUNCTION_ADVANCE_PWM);
    return;
  }

  char choice = chooseLSR(junctionSawLeft, centredLine(sensors),
                          junctionSawRight);
  if (choice == 'L') {
    Serial.println(F("LSR choice: L"));
    beginTurn(-1);
  } else if (choice == 'S') {
    Serial.println(F("LSR choice: S"));
    resetPid(sensors.error);
    state = FOLLOW_LINE;
  } else if (choice == 'R') {
    Serial.println(F("LSR choice: R"));
    beginTurn(1);
  } else {
    enterFaultStop(F("junction has no L/S/R exit"));
  }
}

// ==================================================
// 12. ARDUINO SETUP AND LOOP
// ==================================================

void setup() {
  for (byte i = 0; i < NUM_SENSORS; i++) pinMode(SENSOR_PINS[i], INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();
  Serial.begin(115200);
  lastControlTime = millis();
  lastDebugTime = millis();

  Serial.println(F("Meshmerize PID LSR maze follower ready"));
  Serial.println(F("Priority: Left -> Straight -> Right (no U-turn)"));
}

void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastControlTime;
  if (elapsed < CONTROL_INTERVAL_MS) return;

  // Read exactly one fresh frame for every state-machine step.
  controlDt = elapsed / 1000.0f;
  lastControlTime = now;
  readSensors(sensors);

  if (state == FAULT_STOP) {
    stopMotors();
    printDebug(0.0f, 0, 0);
    return;
  }

  if (state == TURNING) {
    runTurn();
    return;
  }

  if (state == CONFIRM_LINE_LOST) {
    runLineLostConfirm();
    return;
  }

  if (state == ADVANCE_JUNCTION) {
    runJunctionAdvance();
    return;
  }

  if (!lineVisible(sensors)) {
    stateStartTime = now;
    state = CONFIRM_LINE_LOST;
    stopMotors();
    printDebug(0.0f, 0, 0);
    return;
  }

  if (state == CONFIRM_JUNCTION) {
    stopMotors();
    if (!looksLikeJunction(sensors)) {
      state = FOLLOW_LINE;
    } else if (now - stateStartTime >= JUNCTION_CONFIRM_MS) {
      resolveJunction();
    }
    printDebug(0.0f, 0, 0);
    return;
  }

  // Start a junction confirmation only when outside the cooldown period.  The
  // cooldown prevents the still-visible wide line from retriggering a turn.
  if (looksLikeJunction(sensors) &&
      now - lastJunctionTime >= JUNCTION_COOLDOWN_MS) {
    junctionSawLeft = sensors.leftChoice;
    junctionSawRight = sensors.rightChoice;
    stateStartTime = now;
    state = CONFIRM_JUNCTION;
    stopMotors();
    printDebug(0.0f, 0, 0);
    return;
  }

  runPid();
}
