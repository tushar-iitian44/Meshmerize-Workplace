const int Num_Sensor = 6;
const int Sensor_Pins[Num_Sensor] = {A5, A4, A3, A2, A1, A0};
int Sensor_Value[Num_Sensor];

const int ENA = 5;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 6;

const int SENSOR_THRESHOLD = 200;

// Sensor positions: left is negative, right is positive
const float Position[Num_Sensor] = {
  -2.5, -1.5, -0.5, 0.5, 1.5, 2.5
};

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

  Serial.begin(9600);
  setMotors(0, 0);
}

void setMotors(int leftSpeed, int rightSpeed) {
  // Command both motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, constrain(leftSpeed, 0, 255));
  analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

float calculateError() {
  float sum = 0;
  int count = 0;

  for (int i = 0; i < Num_Sensor; i++) {
    if (Sensor_Value[i] < SENSOR_THRESHOLD) {
      // This sensor detects the white line
      sum += Position[i];
      count++;
    }
  }

  if (count == 0) {
    return 999;  // No sensor detects white
  }

  return sum / count;
}

void loop() {
  // 1. Read all six sensors
  for (int i = 0; i < Num_Sensor; i++) {
    Sensor_Value[i] = analogRead(Sensor_Pins[i]);
  }

  // 2. Calculate the line position
  float error = calculateError();

  // 3. Choose the motor commands
  if (error == 999) {
    setMotors(0, 0);
    Serial.println("LINE LOST - STOP");

  } else if (error <= -1.0) {
    setMotors(180, 100);
    Serial.println("LINE LEFT");

  } else if (error < -0.25) {
    setMotors(170, 130);
    Serial.println("SLIGHT LEFT");

  } else if (error >= 1.0) {
    setMotors(100, 180);
    Serial.println("LINE RIGHT");

  } else if (error > 0.25) {
    setMotors(130, 170);
    Serial.println("SLIGHT RIGHT");

  } else {
    setMotors(150, 150);
    Serial.println("LINE CENTRE");
  }

  // 4. Display the sensor pattern for debugging
  Serial.print("Pattern: ");
  for (int i = 0; i < Num_Sensor; i++) {
    Serial.print(
      Sensor_Value[i] < SENSOR_THRESHOLD ? '0' : '1'
    );
  }

  Serial.print(" | Error: ");
  Serial.println(error);

  delay(30);
}