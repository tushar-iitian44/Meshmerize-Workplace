/*
  Meshmerize physical Arduino Uno R3 firmware.

  This sketch physically controls an L298N. Every setMotors(left, right) call:
  - writes L298N IN1/IN2 and IN3/IN4 direction signals
  - applies PWM to ENA and ENB
  It is not a movement simulation.

  Wire exactly as follows before testing:
    RLS-06 VCC -> Uno 5V, RLS-06 GND -> Uno GND
    RLS-06 analogue A0,A1,A2,A3,A4,A5 -> Uno A0,A1,A2,A3,A4,A5
    L298N ENA (remove jumper) -> D5, IN1 -> D7, IN2 -> D8
    L298N ENB (remove jumper) -> D6, IN3 -> D9, IN4 -> D10
    L298N OUT1/OUT2 -> left motor, OUT3/OUT4 -> right motor
    L298N GND, Uno GND, sensor GND, battery negative are all common.
    External red LED: D13 -> 220 ohm resistor -> LED anode; LED cathode -> GND.

  Keep the Uno on USB for initial testing. Use a verified motor battery only
  on the L298N motor-supply terminals. Do not put an unverified four-cell
  battery directly on Uno VIN or 5V.

  Serial Monitor: 115200 baud, No line ending
    c calibrate, d diagnostics, 1..4 individual motor tests, l learn,
    r replay learned RAM route, x immediate stop, h help.
*/

const byte SENSOR_COUNT = 6;
const byte sensorPins[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5};
const int sensorWeight[SENSOR_COUNT] = {-2500, -1500, -500, 500, 1500, 2500};

const byte PIN_EN_LEFT = 5;   // L298N ENA, PWM
const byte PIN_IN1 = 7;
const byte PIN_IN2 = 8;
const byte PIN_EN_RIGHT = 6;  // L298N ENB, PWM
const byte PIN_IN3 = 9;
const byte PIN_IN4 = 10;
const byte PIN_RED_LED = 13;

// Change one of these only when that motor test proves forward is reversed.
const bool LEFT_FORWARD_IS_IN1_HIGH = true;
const bool RIGHT_FORWARD_IS_IN3_HIGH = true;

// RLS-06 analogue voltage is normally lower over the more reflective white.
const bool WHITE_LINE_IS_LOW = true;

const unsigned long CONTROL_PERIOD_MS = 8;
const unsigned long DEBUG_PERIOD_MS = 150;
const unsigned long CALIBRATION_TIME_MS = 8000;
const unsigned long MOTOR_TEST_TIME_MS = 800;
const unsigned long JUNCTION_CONFIRM_MS = 90;
const unsigned long JUNCTION_COOLDOWN_MS = 350;
const unsigned long TURN_MIN_MS = 120;
const unsigned long TURN_TIMEOUT_MS = 1500;
const unsigned long RECOVERY_TIMEOUT_MS = 2400;
const unsigned long FINISH_CONFIRM_MS = 350;

const int MIN_CALIBRATION_SPAN = 100;
const int LINE_ACTIVE_LEVEL = 430;  // calibrated strength 0 through 1000
const int BASE_PWM = 135;
const int MIN_FOLLOW_PWM = 45;
const int MAX_FOLLOW_PWM = 210;
const int TURN_PWM = 145;
const int SEEK_PWM = 115;
const int TEST_PWM = 115;

// Tune KP first on the real track. KD damps rapid changes in line error.
const float KP = 0.050f;
const float KD = 0.12f;

// The exact Techfest finish-zone signature still needs confirmation. This
// provisional detector requires all six sensors to see white for 350 ms.
const bool ENABLE_PROVISIONAL_FINISH_DETECTOR = true;

const byte MAX_ROUTE_LENGTH = 64;
char route[MAX_ROUTE_LENGTH];
byte routeLength = 0;
byte replayIndex = 0;
bool replayMode = false;

enum RobotState {
  SAFE_IDLE, SENSOR_DIAGNOSTIC, MOTOR_TEST, FOLLOWING, JUNCTION_CONFIRM,
  TURNING, RECOVERING, FINISHED, FAULT
};

struct SensorFrame {
  int raw[SENSOR_COUNT];
  int strength[SENSOR_COUNT];  // 0 floor, 1000 white line
  bool active[SENSOR_COUNT];
  byte activeCount;
  long position;               // -2500 left to +2500 right
  bool leftBranch;
  bool centerLine;
  bool rightBranch;
};

RobotState state = SAFE_IDLE;
SensorFrame frame;
int calibrationMin[SENSOR_COUNT];
int calibrationMax[SENSOR_COUNT];
bool calibrationReady = false;

long previousError = 0;
long lastNonZeroError = 0;
unsigned long lastControlAt = 0;
unsigned long lastDebugAt = 0;
unsigned long lastJunctionAt = 0;
unsigned long allLineSince = 0;
unsigned long stateStartedAt = 0;
unsigned long motorTestUntil = 0;
int turnDirection = 0;          // -1 left, +1 right, 2 U turn
bool turnSawCenterDisappear = false;
int recoveryFirstDirection = 1;

const char *stateName(RobotState value) {
  switch (value) {
    case SAFE_IDLE: return "SAFE_IDLE";
    case SENSOR_DIAGNOSTIC: return "SENSOR_DIAGNOSTIC";
    case MOTOR_TEST: return "MOTOR_TEST";
    case FOLLOWING: return "FOLLOWING";
    case JUNCTION_CONFIRM: return "JUNCTION_CONFIRM";
    case TURNING: return "TURNING";
    case RECOVERING: return "RECOVERING";
    case FINISHED: return "FINISHED";
    case FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

void writeOneMotor(byte enablePin, byte inA, byte inB, int signedPwm,
                   bool forwardIsInAHigh) {
  int pwm = signedPwm < 0 ? -signedPwm : signedPwm;
  pwm = constrain(pwm, 0, 255);

  // Disable PWM before every direction update to make reversals safer.
  analogWrite(enablePin, 0);
  if (pwm == 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    return;
  }

  bool forward = signedPwm > 0;
  bool inAHigh = forward ? forwardIsInAHigh : !forwardIsInAHigh;
  digitalWrite(inA, inAHigh ? HIGH : LOW);
  digitalWrite(inB, inAHigh ? LOW : HIGH);
  analogWrite(enablePin, pwm);
}

// Positive means physical robot-forward. This is the real L298N command path.
void setMotors(int leftPwm, int rightPwm) {
  writeOneMotor(PIN_EN_LEFT, PIN_IN1, PIN_IN2, leftPwm,
                LEFT_FORWARD_IS_IN1_HIGH);
  writeOneMotor(PIN_EN_RIGHT, PIN_IN3, PIN_IN4, rightPwm,
                RIGHT_FORWARD_IS_IN3_HIGH);
}

void stopMotors() {
  setMotors(0, 0);  // ENA and ENB PWM become zero; all direction pins low.
}

void initialiseCalibration() {
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    calibrationMin[i] = 1023;
    calibrationMax[i] = 0;
  }
}

void readSensors(SensorFrame &sample) {
  long weightedSum = 0;
  long strengthSum = 0;
  sample.activeCount = 0;

  for (byte i = 0; i < SENSOR_COUNT; i++) {
    sample.raw[i] = analogRead(sensorPins[i]);
    sample.strength[i] = 0;
    sample.active[i] = false;

    if (calibrationReady) {
      int span = calibrationMax[i] - calibrationMin[i];
      long normalized = ((long)(sample.raw[i] - calibrationMin[i]) * 1000L) / span;
      normalized = constrain(normalized, 0L, 1000L);
      sample.strength[i] = WHITE_LINE_IS_LOW ? 1000 - normalized : normalized;
      sample.active[i] = sample.strength[i] >= LINE_ACTIVE_LEVEL;

      if (sample.active[i]) {
        sample.activeCount++;
        weightedSum += (long)sample.strength[i] * sensorWeight[i];
        strengthSum += sample.strength[i];
      }
    }
  }

  sample.position = strengthSum > 0 ? weightedSum / strengthSum : 0;
  sample.leftBranch = sample.active[0] || sample.active[1];
  sample.centerLine = sample.active[2] || sample.active[3];
  sample.rightBranch = sample.active[4] || sample.active[5];
}

bool lineVisible(const SensorFrame &sample) {
  return sample.activeCount > 0;
}

bool looksLikeJunction(const SensorFrame &sample) {
  if (sample.activeCount >= 4) return true;
  if (sample.leftBranch && sample.rightBranch) return true;
  return sample.activeCount >= 3 && sample.centerLine &&
         (sample.leftBranch || sample.rightBranch);
}

void printFrame(const SensorFrame &sample) {
  Serial.print(F("state="));
  Serial.print(stateName(state));
  Serial.print(F(" raw="));
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(sample.raw[i]);
    if (i < SENSOR_COUNT - 1) Serial.print(',');
  }
  if (calibrationReady) {
    Serial.print(F(" strength="));
    for (byte i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(sample.strength[i]);
      if (i < SENSOR_COUNT - 1) Serial.print(',');
    }
    Serial.print(F(" active="));
    for (byte i = 0; i < SENSOR_COUNT; i++) Serial.print(sample.active[i] ? '1' : '0');
    Serial.print(F(" position="));
    Serial.print(sample.position);
    Serial.print(F(" error="));
    Serial.print(sample.position);
  }
  Serial.println();
}

void printRoute() {
  Serial.print(F("route["));
  Serial.print(routeLength);
  Serial.print(F("]="));
  for (byte i = 0; i < routeLength; i++) Serial.print(route[i]);
  Serial.println();
}

int turnValue(char choice) {
  if (choice == 'L') return -90;
  if (choice == 'S') return 0;
  if (choice == 'R') return 90;
  return 180;
}

char turnFromValue(int degrees) {
  while (degrees > 180) degrees -= 360;
  while (degrees <= -180) degrees += 360;
  if (degrees == -90) return 'L';
  if (degrees == 0) return 'S';
  if (degrees == 90) return 'R';
  return 'B';
}

void simplifyLatestRoute() {
  // A-B-C describes entering and returning from a dead end. Replace it with
  // one equivalent decision to retain the shortest known tree-maze path.
  if (routeLength < 3 || route[routeLength - 2] != 'B') return;
  int combined = turnValue(route[routeLength - 3]) +
                 turnValue(route[routeLength - 2]) +
                 turnValue(route[routeLength - 1]);
  route[routeLength - 3] = turnFromValue(combined);
  routeLength -= 2;
}

void enterSafeIdle() {
  stopMotors();
  digitalWrite(PIN_RED_LED, LOW);
  allLineSince = 0;
  state = SAFE_IDLE;
}

void enterFault(const __FlashStringHelper *reason) {
  stopMotors();
  Serial.print(F("FAULT: "));
  Serial.println(reason);
  state = FAULT;
}

void recordDecision(char choice) {
  if (replayMode) return;
  if (routeLength >= MAX_ROUTE_LENGTH) {
    enterFault(F("route memory full"));
    return;
  }
  route[routeLength++] = choice;
  simplifyLatestRoute();
  Serial.print(F("decision="));
  Serial.println(choice);
  printRoute();
}

void startTurn(int direction) {
  turnDirection = direction;
  turnSawCenterDisappear = false;
  stateStartedAt = millis();
  stopMotors();
  state = TURNING;
}

void startRecovery() {
  // Negative error was left of centre, positive was right of centre.
  recoveryFirstDirection = lastNonZeroError < 0 ? -1 : 1;
  stateStartedAt = millis();
  state = RECOVERING;
}

void calibrateSensors() {
  enterSafeIdle();
  initialiseCalibration();
  Serial.println(F("Calibration: motors stopped. Sweep every sensor over white and dark for 8 seconds."));
  unsigned long started = millis();
  unsigned long progressAt = started;

  while (millis() - started < CALIBRATION_TIME_MS) {
    for (byte i = 0; i < SENSOR_COUNT; i++) {
      int value = analogRead(sensorPins[i]);
      if (value < calibrationMin[i]) calibrationMin[i] = value;
      if (value > calibrationMax[i]) calibrationMax[i] = value;
    }
    if (millis() >= progressAt) {
      Serial.print('.');
      progressAt += 250;
    }
    delay(4);
  }
  Serial.println();

  calibrationReady = true;
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    int span = calibrationMax[i] - calibrationMin[i];
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(" min="));
    Serial.print(calibrationMin[i]);
    Serial.print(F(" max="));
    Serial.print(calibrationMax[i]);
    Serial.print(F(" span="));
    Serial.println(span);
    if (span < MIN_CALIBRATION_SPAN) calibrationReady = false;
  }
  if (calibrationReady) Serial.println(F("Calibration accepted. Send d, then l."));
  else Serial.println(F("Calibration rejected. Each sensor needs more white/dark contrast; send c again."));
  enterSafeIdle();
}

void startMotorTest(int leftPwm, int rightPwm, const __FlashStringHelper *label) {
  enterSafeIdle();
  Serial.print(F("Motor test: "));
  Serial.println(label);
  Serial.println(F("Lift wheels. Actual L298N command will run for 0.8 seconds, then stop."));
  setMotors(leftPwm, rightPwm);
  motorTestUntil = millis() + MOTOR_TEST_TIME_MS;
  state = MOTOR_TEST;
}

void startRun(bool useReplay) {
  if (!calibrationReady) {
    Serial.println(F("No valid calibration. Send c first."));
    enterSafeIdle();
    return;
  }
  if (useReplay && routeLength == 0) {
    Serial.println(F("No learned route in RAM."));
    enterSafeIdle();
    return;
  }
  if (!useReplay) routeLength = 0;
  replayMode = useReplay;
  replayIndex = 0;
  previousError = 0;
  lastNonZeroError = 0;
  allLineSince = 0;
  lastJunctionAt = 0;
  digitalWrite(PIN_RED_LED, LOW);
  state = FOLLOWING;
  Serial.println(useReplay ? F("REPLAY run started") : F("LEARNING run started"));
}

char chooseLearningDecision(const SensorFrame &sample) {
  // Physical exploration policy: left, then straight, then right, then back.
  if (sample.leftBranch) return 'L';
  if (sample.centerLine) return 'S';
  if (sample.rightBranch) return 'R';
  return 'B';
}

void executeDecision(char choice) {
  Serial.print(F("execute="));
  Serial.println(choice);
  lastJunctionAt = millis();
  if (choice == 'L') startTurn(-1);
  else if (choice == 'R') startTurn(1);
  else if (choice == 'B') startTurn(2);
  else if (choice == 'S') state = FOLLOWING;
  else enterFault(F("invalid route item"));
}

void resolveJunction() {
  if (!looksLikeJunction(frame)) {
    state = FOLLOWING;
    return;
  }
  char choice;
  if (replayMode) {
    if (replayIndex >= routeLength) {
      enterFault(F("route ended before finish"));
      return;
    }
    choice = route[replayIndex++];
  } else {
    choice = chooseLearningDecision(frame);
    recordDecision(choice);
    if (state == FAULT) return;
  }
  executeDecision(choice);
}

void runFollowing() {
  if (!lineVisible(frame)) {
    Serial.println(F("Line lost; beginning sensor-feedback recovery."));
    startRecovery();
    return;
  }
  if (frame.position > 150 || frame.position < -150) lastNonZeroError = frame.position;

  if (ENABLE_PROVISIONAL_FINISH_DETECTOR && frame.activeCount == SENSOR_COUNT) {
    if (allLineSince == 0) allLineSince = millis();
    if (millis() - allLineSince >= FINISH_CONFIRM_MS) {
      stopMotors();
      digitalWrite(PIN_RED_LED, HIGH);
      state = FINISHED;
      Serial.println(F("FINISHED by provisional all-sensor signature."));
      printRoute();
      return;
    }
  } else {
    allLineSince = 0;
  }

  if (looksLikeJunction(frame) &&
      millis() - lastJunctionAt > JUNCTION_COOLDOWN_MS) {
    stopMotors();
    stateStartedAt = millis();
    state = JUNCTION_CONFIRM;
    return;
  }

  long error = frame.position;
  long derivative = error - previousError;
  previousError = error;
  int correction = (int)(KP * error + KD * derivative);
  correction = constrain(correction, -110, 110);

  // Positive error means the white line is to the right. Increase left PWM
  // and reduce right PWM, which physically steers the robot right.
  int leftPwm = constrain(BASE_PWM + correction, MIN_FOLLOW_PWM, MAX_FOLLOW_PWM);
  int rightPwm = constrain(BASE_PWM - correction, MIN_FOLLOW_PWM, MAX_FOLLOW_PWM);
  setMotors(leftPwm, rightPwm);
}

void runTurning() {
  unsigned long elapsed = millis() - stateStartedAt;
  if (turnDirection == -1) {
    // L reverse, R forward: real in-place left rotation.
    setMotors(-TURN_PWM, TURN_PWM);
  } else {
    // L forward, R reverse: real right rotation (also used for U-turn).
    setMotors(TURN_PWM, -TURN_PWM);
  }

  if (!frame.centerLine) turnSawCenterDisappear = true;
  if (elapsed >= TURN_MIN_MS && turnSawCenterDisappear &&
      frame.centerLine && frame.activeCount <= 3) {
    stopMotors();
    previousError = frame.position;
    lastJunctionAt = millis();
    state = FOLLOWING;
    Serial.println(F("Turn completed by sensor reacquisition."));
    return;
  }
  if (elapsed > TURN_TIMEOUT_MS) {
    enterFault(F("turn timeout: check wheel direction and sensor order"));
  }
}

void runRecovery() {
  unsigned long elapsed = millis() - stateStartedAt;
  if (lineVisible(frame) && frame.centerLine && frame.activeCount <= 3) {
    stopMotors();
    previousError = frame.position;
    state = FOLLOWING;
    Serial.println(F("Line recovered by sensor feedback."));
    return;
  }

  int direction = elapsed < (RECOVERY_TIMEOUT_MS / 2) ?
                  recoveryFirstDirection : -recoveryFirstDirection;
  if (direction < 0) setMotors(-SEEK_PWM, SEEK_PWM);
  else setMotors(SEEK_PWM, -SEEK_PWM);

  if (elapsed > RECOVERY_TIMEOUT_MS) {
    if (!replayMode) {
      recordDecision('B');
      if (state != FAULT) startTurn(2);
    } else if (replayIndex < routeLength && route[replayIndex] == 'B') {
      replayIndex++;
      startTurn(2);
    } else {
      enterFault(F("line not recovered"));
    }
  }
}

void updateRedLed() {
  if (state == FAULT) digitalWrite(PIN_RED_LED, (millis() / 250) % 2 ? HIGH : LOW);
}

void printHelp() {
  Serial.println(F("c=calibrate d=diagnostics x=stop 1..4=motor tests l=learn r=replay h=help"));
  Serial.println(F("1 left forward, 2 left reverse, 3 right forward, 4 right reverse"));
}

void handleSerial() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '\r' || command == '\n' || command == ' ') continue;
    if (command == 'h' || command == 'H') printHelp();
    else if (command == 'c' || command == 'C') calibrateSensors();
    else if (command == 'd' || command == 'D') {
      enterSafeIdle();
      state = SENSOR_DIAGNOSTIC;
      Serial.println(F("Diagnostics active. Send x for safe idle."));
    } else if (command == 'x' || command == 'X') {
      enterSafeIdle();
      Serial.println(F("Safe idle: both motors commanded off."));
    } else if (command == '1') startMotorTest(TEST_PWM, 0, F("left forward"));
    else if (command == '2') startMotorTest(-TEST_PWM, 0, F("left reverse"));
    else if (command == '3') startMotorTest(0, TEST_PWM, F("right forward"));
    else if (command == '4') startMotorTest(0, -TEST_PWM, F("right reverse"));
    else if (command == 'l' || command == 'L') startRun(false);
    else if (command == 'r' || command == 'R') startRun(true);
    else Serial.println(F("Unknown command. Send h."));
  }
}

void setup() {
  pinMode(PIN_EN_LEFT, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_EN_RIGHT, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_RED_LED, OUTPUT);
  for (byte i = 0; i < SENSOR_COUNT; i++) pinMode(sensorPins[i], INPUT);

  stopMotors();  // safe startup: PWM zero and L298N direction outputs low
  digitalWrite(PIN_RED_LED, LOW);
  initialiseCalibration();
  Serial.begin(115200);
  Serial.println(F("Meshmerize RLS-06 plus L298N physical firmware."));
  Serial.println(F("Motors stopped. Wire hardware, then send h."));
  printHelp();
}

void loop() {
  handleSerial();
  readSensors(frame);

  if (millis() - lastControlAt >= CONTROL_PERIOD_MS) {
    lastControlAt = millis();
    if (state == MOTOR_TEST && (long)(millis() - motorTestUntil) >= 0) {
      enterSafeIdle();
      Serial.println(F("Motor test complete; motors stopped."));
    } else if (state == FOLLOWING) {
      runFollowing();
    } else if (state == JUNCTION_CONFIRM) {
      stopMotors();
      if (millis() - stateStartedAt >= JUNCTION_CONFIRM_MS) resolveJunction();
    } else if (state == TURNING) {
      runTurning();
    } else if (state == RECOVERING) {
      runRecovery();
    } else if (state == SAFE_IDLE || state == SENSOR_DIAGNOSTIC ||
               state == FINISHED || state == FAULT) {
      stopMotors();
    }
    updateRedLed();
  }

  if ((state == SENSOR_DIAGNOSTIC || state == FOLLOWING ||
       state == JUNCTION_CONFIRM || state == RECOVERING) &&
      millis() - lastDebugAt >= DEBUG_PERIOD_MS) {
    lastDebugAt = millis();
    printFrame(frame);
  }
}
