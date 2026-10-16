# Meshmerize-Workplace
This repository contains the firmware and source code for an autonomous, line-following, maze-solving robot built for Meshmerize, the premier robotics competition at Techfest, IIT Bombay. 

# Meshmerize PID Line Follower

A 6-sensor line-following robot controller written in C. It computes the line position from infrared sensor readings, applies a PID controller to calculate steering correction, and drives two motors using PWM-style outputs. The code is written as a desktop simulation but includes comments for Arduino porting.

Features

· 6-sensor line position calculation
· PID control with proportional, integral, and derivative terms
· Integral anti-windup
· Derivative smoothing
· PWM output clamping from 0 to 255
· State detection for:
  · Following line
  · Lost line
  · Junction
  · Turning
  · Stopped
· Simulated motor output via printf
· Arduino-ready analogWrite hooks
· Basic left/right sensor-shifting routines

File

· Meshmerize_Code_PID.c — main source file

Sensor Layout

The robot uses 6 sensors indexed from left to right:

Index Weight / Position
0 -2.5
1 -1.5
2 -0.5
3 +0.5
4 +1.5
5 +2.5

The center of the sensor array is between index 2 and 3, i.e. position 2.5.

Position Calculation

position = (sum of (i - 2.5) for active sensors) / (number of active sensors)

· sensor[i] == 1 means the sensor detects the line.
· If no sensor detects the line, bot_position() returns 999.
· Negative position means the line is to the left of center.
· Positive position means the line is to the right of center.

PID Controller

Default gains:
float Kp = 25.0f;
float Ki = 0.5f;
float Kd = 8.0f;

PID update logic:

integral += error;     // only when error != 999
integral = clamp(integral, -100, 100);

derivative = error - previousError;
derivative = 0.7 * derivative + 0.3 * previousDeriv;

correction = Kp * error + Ki * integral + Kd * derivative;

Motor speeds are calculated as:
leftSpeed  = baseSpeed + correction;
rightSpeed = baseSpeed - correction;

Default base speed:
int baseSpeed = 150;

PWM limits:
#define PWM_MAX 255
#define PWM_MIN 0

State Machine
The Robot_State enum defines these states:

State Value
Follow_Line 0
Turning 1
Stopped 10
Lost_line 999
Junction 1111

State Logic

· Stopped: both motors are 0.
· Follow_Line: error is 0.
· Lost_line: all sensors are 0.
· Junction: three consecutive sensors detect the line.
· Turning: left and right motor speeds differ.

When a junction is detected, the robot goes straight at baseSpeed.

When the line is lost, the motors are stopped.

Main Functions

Function Purpose
calculatePID(float error) Computes PID correction
setMotorPWM(int leftSpeed, int rightSpeed) Clamps and outputs motor PWM values
driveWithPID(float error) Converts PID correction into left/right motor speeds
bot_position(int sensor[]) Computes weighted line position
Robot_state(...) Determines current robot state
callback_actual_position(...) Main control callback for position/error
left_move(int sensor[]) Shifts sensor readings toward the left
right_move(int sensor[]) Shifts sensor readings toward the right
main() Example simulation entry point

Build and Run

Compile with any standard C compiler:

bash
gcc Meshmerize_Code_PID.c -o meshmerize
./meshmerize

No external libraries are required.

Example Input

The default main() uses:
int sensor[NUM_SENSORS] = {1, 1, 0, 0, 0, 0};

This means the line is under the two leftmost sensors. The robot is therefore far to the right of the line and should steer left.

Arduino Porting

The code simulates PWM output with printf. To run on Arduino, replace setMotorPWM() with real hardware output:

void setMotorPWM(int leftSpeed, int rightSpeed) {
    if (leftSpeed  > PWM_MAX) leftSpeed  = PWM_MAX;
    if (leftSpeed  < PWM_MIN) leftSpeed  = PWM_MIN;
    if (rightSpeed > PWM_MAX) rightSpeed = PWM_MAX;
    if (rightSpeed < PWM_MIN) rightSpeed = PWM_MIN;

    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
}

You will also need to:

· Define ENA and ENB pins.
· Configure motor driver pins in setup().
· Read the 6 sensors into sensor[].
· Replace printf() with Serial.print() or remove debug output.
· Ensure the control loop runs at a stable interval.

Tuning Guide

1. Start with Ki = 0 and Kd = 0.
2. Increase Kp until the robot follows the line but starts to oscillate.
3. Add Kd to reduce oscillation and improve stability.
4. Add a small Ki only if steady-state error remains.
5. Keep baseSpeed low enough that PID correction can still change motor speeds without hitting PWM limits.
6. Adjust the integral clamp if the robot reacts too slowly or winds up on long turns.

Notes and Known Limitations

· The weights[] array is defined but not used. Position calculation uses (i - 2.5f) directly.
· left_move() and right_move() modify the input sensor[] array in place.
· callback_actual_position() calls driveWithPID() and also returns a speed value based on Kp * error. The returned value is mainly used for logging in the current code.
· Robot_state() receives motor speeds before PID correction in callback_actual_position(), so the Turning state is not triggered there.
· right_move() contains a redundant condition: position > 0 && position != 0.
· There is no fixed timing loop. PID behavior assumes a reasonably constant loop interval.
· Junction detection only looks for three consecutive active sensors. Other junction patterns may require additional logic.
· 999 and 1111 are used as sentinel values. The normal position range is about -2.5 to +2.5, so there is no conflict.
· The code is a simulation/demo and may need hardware-specific tuning before real robot use.
