  junctionSawRight = junctionSawRight || sensors.rightChoice;
  if (millis() - stateStartTime < JUNCTION_ADVANCE_MS) {
    printDebug(0.0f, JUNCTION_ADVANCE_PWM, JUNCTION_ADVANCE_PWM);
    return;
  }
  char choice = chooseLSRB(junctionSawLeft, centredLine(sensors),
                           junctionSawRight);
  if (choice == 'L') {
    Serial.println(F("LSRB choice: L"));
    beginTurn(-1);
  } else if (choice == 'S') {
    Serial.println(F("LSRB choice: S"));
    resetPid(sensors.error);
    state = FOLLOW_LINE;
  } else if (choice == 'R') {
    Serial.println(F("LSRB choice: R"));
    beginTurn(1);
  } else {
    // No left was seen, the forward exit disappeared, and no right branch was
    // detected: this junction is a dead end relative to the arrival path.
    Serial.println(F("LSRB choice: B"));
    beginTurn(2);
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
  Serial.println(F("Meshmerize PID LSRB maze follower ready"));
  Serial.println(F("Priority: Left -> Straight -> Right -> Back"));
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
  if (state == CONFIRM_DEAD_END) {
    runDeadEndConfirm();
    return;
  }
  if (state == ADVANCE_JUNCTION) {
    runJunctionAdvance();
    return;
  }
  if (!lineVisible(sensors)) {
    stateStartTime = now;
    state = CONFIRM_DEAD_END;
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
