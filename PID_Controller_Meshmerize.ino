
/*
  Meshmerize - 6 Sensor PID Line Follower
  Board: Arduino Uno R3
  Motor driver: L298N
  Sensor array: 6 analog IR sensors

  Sensors physically ordered LEFT to RIGHT:
  A5, A4, A3, A2, A1, A0

  Displayed pattern order: A0 -> A5

  011110 = CENTRE / STRAIGHT
  111100 = LINE TO RIGHT
  001111 = LINE TO LEFT
  111111 = INTERSECTION / WIDE LINE
  000000 = LINE LOST

  Assumption: analogRead() < 200 detects white.

  Motor mapping:
  ENA = physical RIGHT motor
  ENB = physical LEFT motor
*/


// ==================================================
// 1. SENSOR SETTINGS
// ==================================================

const int Num_Sensor = 6;

const int Sensor_Pins[Num_Sensor] = {
  A5, A4, A3, A2, A1, A0
};

int Sensor_Value[Num_Sensor];

const int SENSOR_THRESHOLD = 200;

// Physical sensor positions:
// Left is negative, right is positive.
const float Position[Num_Sensor] = {
  -2.5, -1.5, -0.5, 0.5, 1.5, 2.5
};


// ==================================================
// 2. L298N MOTOR CONNECTIONS
// ==================================================

const int ENA = 5;  // Physical right motor PWM
const int IN1 = 7;
const int IN2 = 8;

const int IN3 = 9;
const int IN4 = 10;
const int ENB = 6;  // Physical left motor PWM


// ==================================================
// 3. PWM SETTINGS
// ==================================================

const int BASE_SPEED = 120;
const int ALIGN_SPEED = 80;
const int MAX_CORRECTION = 90;
const int SEARCH_PWM = 80;


// ==================================================
// 4. PID SETTINGS
// ==================================================

float Kp = 25.0;
float Ki = 0.0;
float Kd = 0.50;

float previousError = 0.0;
float integral = 0.0;
float filteredDerivative = 0.0;

const float INTEGRAL_LIMIT = 8.0;
const float ERROR_DEADBAND = 0.08;


// ==================================================
// 5. STARTUP ALIGNMENT
// ==================================================

bool startupAlign = true;
bool centerTimerActive = false;

const unsigned long CENTER_STABLE_MS = 250;

unsigned long centerStartTime = 0;


// ==================================================
// 6. TIMING AND LINE RECOVERY
// ==================================================

const unsigned long CONTROL_INTERVAL_MS = 10;
const unsigned long DEBUG_INTERVAL_MS = 100;
const unsigned long LOST_LINE_SEARCH_MS = 900;

unsigned long lastControlTime = 0;
unsigned long lastDebugTime = 0;

bool lineLostTiming = false;
unsigned long lineLostStart = 0;

// +1 = search RIGHT
// -1 = search LEFT
int lastTurnDirection = 1;

float lastError = 0.0;


// ==================================================
// 7. MOTOR CONTROL
// ==================================================

void setMotors(int leftSpeed, int rightSpeed) {

  // Both motors commanded forward.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Physical motor assignments are swapped.
  analogWrite(ENA, rightSpeed);
  analogWrite(ENB, leftSpeed);
}


// ==================================================
// 8. SERIAL DEBUGGING
// ==================================================

void printDebug(
  const char* state,
  byte pattern,
  float error,
  float correction,
  int leftSpeed,
  int rightSpeed,
  int activeCount
) {

  unsigned long now = millis();

  if (now - lastDebugTime < DEBUG_INTERVAL_MS) {
    return;
  }

  lastDebugTime = now;

  Serial.print(state);
  Serial.print(" | Pattern A0->A5: ");

  for (int bit = 5; bit >= 0; bit--) {
    Serial.print((pattern & (1U << bit)) ? '1' : '0');
  }

  Serial.print(" | Error: ");
  Serial.print(error, 2);

  Serial.print(" | PID: ");
  Serial.print(correction, 1);

  Serial.print(" | PWM L/R: ");
  Serial.print(leftSpeed);
  Serial.print("/");
  Serial.print(rightSpeed);

  Serial.print(" | Active: ");
  Serial.println(activeCount);
}


// ==================================================
// 9. SETUP
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

  Serial.begin(115200);

  setMotors(0, 0);

  lastControlTime = millis();
  lastDebugTime = millis();

  Serial.println("Meshmerize PID ready");
  Serial.println("Startup alignment active");
}


// ==================================================
// 10. MAIN LOOP
// ==================================================

void loop() {

  unsigned long now = millis();
  unsigned long elapsed = now - lastControlTime;

  // PID control runs approximately every 10 ms.
  if (elapsed < CONTROL_INTERVAL_MS) {
    return;
  }

  float dt = elapsed / 1000.0;
  lastControlTime = now;


  // ------------------------------------------------
  // A. READ SENSORS AND GENERATE PATTERN
  // ------------------------------------------------

  float weightedPosition = 0.0;
  int activeCount = 0;
  byte pattern = 0;

  for (int i = 0; i < Num_Sensor; i++) {

    Sensor_Value[i] = analogRead(Sensor_Pins[i]);

    // Assumption: lower reading means white.
    if (Sensor_Value[i] < SENSOR_THRESHOLD) {

      activeCount++;
      weightedPosition += Position[i];

      // Sensor 0 (A5) is bit 0.
      pattern |= (1U << i);
    }
  }


  // ------------------------------------------------
  // B. LINE LOST: SEARCH IN LAST KNOWN DIRECTION
  // ------------------------------------------------

  if (activeCount == 0) {

    if (!lineLostTiming) {
      lineLostTiming = true;
      lineLostStart = now;
    }

    integral = 0.0;
    filteredDerivative = 0.0;

    centerTimerActive = false;

    if (now - lineLostStart < LOST_LINE_SEARCH_MS) {

      if (lastTurnDirection > 0) {

        // Search right.
        setMotors(SEARCH_PWM, 0);

        printDebug(
          "LOST - SEARCH RIGHT",
          pattern, lastError, 0,
          SEARCH_PWM, 0, activeCount
        );

      } else {

        // Search left.
        setMotors(0, SEARCH_PWM);

        printDebug(
          "LOST - SEARCH LEFT",
          pattern, lastError, 0,
          0, SEARCH_PWM, activeCount
        );
      }

    } else {

      // Stop if the line is not recovered.
      setMotors(0, 0);

      printDebug(
        "LINE LOST - STOP",
        pattern, lastError, 0,
        0, 0, activeCount
      );
    }

    return;
  }


  // ------------------------------------------------
  // C. LINE REACQUIRED
  // ------------------------------------------------

  bool wasLineLost = lineLostTiming;
  lineLostTiming = false;


  // ------------------------------------------------
  // D. ALL SIX SENSORS WHITE: WIDE LINE / INTERSECTION
  // ------------------------------------------------

  if (pattern == 0b111111) {

    integral = 0.0;
    previousError = 0.0;
    filteredDerivative = 0.0;
    lastError = 0.0;

    int speedNow =
      startupAlign ? ALIGN_SPEED : BASE_SPEED;

    setMotors(speedNow, speedNow);

    printDebug(
      "INTERSECTION - STRAIGHT",
      pattern, 0, 0,
      speedNow, speedNow, activeCount
    );

    return;
  }


  // ------------------------------------------------
  // E. CALCULATE ACTUAL LINE POSITION
  // ------------------------------------------------

  float error = weightedPosition / activeCount;

  // Prevent a derivative spike when the line returns.
  if (wasLineLost) {
    previousError = error;
    integral = 0.0;
    filteredDerivative = 0.0;
  }


  // Small deadband around the centre.
  if (error > -ERROR_DEADBAND &&
      error < ERROR_DEADBAND) {
    error = 0.0;
  }


  // Remember the direction for recovery.
  if (error > 0.15) {
    lastTurnDirection = 1;
  } else if (error < -0.15) {
    lastTurnDirection = -1;
  }


  // ------------------------------------------------
  // F. STARTUP SELF-ALIGNMENT
  // ------------------------------------------------
  //
  // Move slowly while PID corrects the line position.
  // Alignment is complete after 011110 stays centred
  // for 250 ms.
  //
  // This is line-following alignment, not a compass-
  // based measurement of the robot's exact heading.

  if (startupAlign) {

    if (pattern == 0b011110 &&
        error > -0.2 && error < 0.2) {

      if (!centerTimerActive) {
        centerTimerActive = true;
        centerStartTime = millis();
      }

      if (millis() - centerStartTime >=
          CENTER_STABLE_MS) {

        startupAlign = false;
        centerTimerActive = false;

        Serial.println("STARTUP ALIGNMENT COMPLETE");
      }

    } else {

      // Restart the timer if alignment is disturbed.
      centerTimerActive = false;
    }
  }


  // Use lower speed during initial alignment.
  int currentBaseSpeed =
      startupAlign ? ALIGN_SPEED : BASE_SPEED;


  // ------------------------------------------------
  // G. PID CALCULATION
  // ------------------------------------------------

  float P = error;

  integral += error * dt;

  integral = constrain(
    integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT
  );

  float rawDerivative =
      (error - previousError) / dt;

  // Filter derivative to reduce sensor noise.
  filteredDerivative =
      0.75 * filteredDerivative
    + 0.25 * rawDerivative;

  float correction =
      Kp * P
    + Ki * integral
    + Kd * filteredDerivative;

  correction = constrain(
    correction, -MAX_CORRECTION, MAX_CORRECTION
  );


  // ------------------------------------------------
  // H. PID TO PWM CORRECTION
  // ------------------------------------------------
  //
  // Positive error: line is to the RIGHT.
  // Left wheel faster; right wheel slower.
  //
  // Negative error: line is to the LEFT.
  // Left wheel slower; right wheel faster.

  int leftSpeed =
      (int)(currentBaseSpeed + correction);

  int rightSpeed =
      (int)(currentBaseSpeed - correction);

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotors(leftSpeed, rightSpeed);


  // ------------------------------------------------
  // I. SAVE PID VALUES
  // ------------------------------------------------

  previousError = error;
  lastError = error;


  // ------------------------------------------------
  // J. DEBUGGING
  // ------------------------------------------------

  const char* state = "PID TRACKING";

  if (startupAlign) {
    state = "STARTUP ALIGNING";
  } else if (pattern == 0b011110) {
    state = "CENTRED";
  } else if (pattern == 0b111100) {
    state = "LINE RIGHT";
  } else if (pattern == 0b001111) {
    state = "LINE LEFT";
  } else if (error > 0.2) {
    state = "CORRECT RIGHT";
  } else if (error < -0.2) {
    state = "CORRECT LEFT";
  }

  printDebug(
    state,
    pattern,
    error,
    correction,
    leftSpeed,
    rightSpeed,
    activeCount
  );
}
