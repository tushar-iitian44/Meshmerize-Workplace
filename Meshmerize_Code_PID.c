#include <stdio.h>

/* ============================================================
   PID + PWM SECTION (6-sensor version)
   ============================================================ */

#define NUM_SENSORS 6
#define PWM_MAX     255
#define PWM_MIN     0

/* PID gains — tune these */
float Kp = 25.0f;
float Ki = 0.5f;
float Kd = 8.0f;

/* PID state */
float integral      = 0.0f;
float previousError = 0.0f;
float previousDeriv = 0.0f;

/* Base speed for both motors */
int baseSpeed = 150;

/* 6 sensors, symmetric weights around center (index 2.5) */
float weights[NUM_SENSORS] = {-2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f};

/* ---------- PID core ---------- */
float calculatePID(float error) {
    float derivative, correction;

    if (error != 999.0f) {      /* only integrate when line is visible */
        integral += error;
    }

    if (integral >  100.0f) integral =  100.0f;   /* anti-windup */
    if (integral < -100.0f) integral = -100.0f;

    derivative = error - previousError;
    derivative = 0.7f * derivative + 0.3f * previousDeriv;  /* smoothing */
    previousDeriv = derivative;

    correction = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    return correction;
}

/* ---------- PWM output (simulated in C, real on Arduino) ---------- */
void setMotorPWM(int leftSpeed, int rightSpeed) {
    if (leftSpeed  > PWM_MAX) leftSpeed  = PWM_MAX;
    if (leftSpeed  < PWM_MIN) leftSpeed  = PWM_MIN;
    if (rightSpeed > PWM_MAX) rightSpeed = PWM_MAX;
    if (rightSpeed < PWM_MIN) rightSpeed = PWM_MIN;

    printf("[PWM] ENA(left)  = %3d | ENB(right) = %3d\n", leftSpeed, rightSpeed);
    /* On Arduino:
       analogWrite(ENA, leftSpeed);
       analogWrite(ENB, rightSpeed);   */
}

/* Drive both motors from a single PID error value */
void driveWithPID(float error) {
    float correction = calculatePID(error);
    int leftSpeed    = (int)(baseSpeed + correction);
    int rightSpeed   = (int)(baseSpeed - correction);

    printf("PID -> error=%.2f  integral=%.2f  correction=%.2f\n",
           error, integral, correction);

    setMotorPWM(leftSpeed, rightSpeed);
}

/* ============================================================
   ORIGINAL LOGIC (adapted for 6 sensors)
   ============================================================ */

void sensor_data(int sensor[]) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        printf(" %d", sensor[i]);
    }
    printf("\n");
}

int left_move(int sensor[]);
int right_move(int sensor[]);

/* Center is between index 2 and 3 → subtract 2.5 */
float bot_position(int sensor[]) {
    float sum   = 0;
    float count = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor[i] == 1) {
            count += 1;
            sum = sum + (i - 2.5f);
        }
    }
    if (count == 0) return 999;
    return sum / count;
}

enum Robot_State {
    Follow_Line = 0,
    Lost_line   = 999,
    Turning     = 1,
    Junction    = 1111,
    Stopped     = 10,
};

int Robot_state(float error, int left_motor, int right_motor, int sensor[]) {
    int new_sensor[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) new_sensor[i] = 0;

    if (left_motor == right_motor && left_motor == 0) return Stopped;
    if (error == 0) return Follow_Line;

    /* All zero → lost line */
    int all_zero = 1;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor[i] != 0) { all_zero = 0; break; }
    }
    if (all_zero) {
        printf("Robot Lost or Robot missed a junction (check previous data)\n");
        return Lost_line;
    }

    /* Junction: 3 consecutive 1s (safe loop bound) */
    for (int i = 0; i < NUM_SENSORS - 2; i++) {
        new_sensor[i] = sensor[i];
        if (new_sensor[i]     == 1 &&
            new_sensor[i + 1] == 1 &&
            new_sensor[i + 2] == 1) {
            printf("Warning Sensed, Maybe Turning Ahead.\n");
            return Junction;
        }
    }

    if (left_motor != right_motor) return Turning;
    return Follow_Line;
}

/* ---------- Uses PID + PWM ---------- */
int callback_actual_position(float pos, int sensor[]) {
    float error = pos;
    int left_motor  = baseSpeed;
    int right_motor = baseSpeed;
    int x = Robot_state(error, left_motor, right_motor, sensor);

    printf("Robot State: %d\n", x);

    if (error == 999.0f) {
        printf("Line lost -> motors stopped\n");
        setMotorPWM(0, 0);
        return 0;
    }

    if (x == Junction) {
        printf("Junction detected -> going straight\n");
        setMotorPWM(baseSpeed, baseSpeed);
        return baseSpeed;
    }

    driveWithPID(error);

    if (error < 0) {
        return (int)(baseSpeed - Kp * error);   /* right side */
    } else if (error > 0) {
        return (int)(baseSpeed + Kp * error);   /* left side  */
    }
    return baseSpeed;
}

/* ---------- left_move / right_move (6-sensor) ---------- */
int left_move(int sensor[]) {
    int value_sensor[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) value_sensor[i] = 0;

    for (int i = 0; i < NUM_SENSORS - 2; i++) {
        if (sensor[i + 1] == sensor[i] && sensor[i] == 1) {
            sensor[i]     = 0;
            sensor[i + 2] = 1;
            printf("Sensor Value:");
            for (int j = 0; j < NUM_SENSORS; j++) {
                value_sensor[j] = sensor[j];
                printf("%d", sensor[j]);
            }
        }
        printf(" \n");
        float position = bot_position(value_sensor);

        if (position < 0) {
            int right_motor = callback_actual_position(position, sensor);
            printf("Right motor speed: %d\n", right_motor);
        }
        if (position == 0) {
            printf("Motor speed should be: %d %d\n", baseSpeed, baseSpeed);
            printf("Follow line\n");
            setMotorPWM(baseSpeed, baseSpeed);
        }
        if (position > 0 && position != 999) {
            int left_motor = callback_actual_position(position, sensor);
            printf("Left motor speed should be: %d\n", left_motor);
        }
        /* 6-sensor center check → indices 2 and 3 */
        if (sensor[2] == 1 && sensor[3] == 1) {
            printf("Current State: Follow line\n");
            break;
        }
    }
    return 0;
}

int right_move(int sensor[]) {
    int new_sensor[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) new_sensor[i] = 0;

    for (int i = NUM_SENSORS - 1; i > 1; i--) {
        if (sensor[i] == sensor[i - 1] && sensor[i] == 1) {
            sensor[i]     = 0;
            sensor[i - 2] = 1;
            printf("Sensor Value: ");
            for (int j = 0; j < NUM_SENSORS; j++) {
                new_sensor[j] = sensor[j];
                printf("%d", sensor[j]);
            }
        }
        printf("\n");
        float position = bot_position(new_sensor);

        if (position < 0) {
            int right_motor = callback_actual_position(position, sensor);
            printf("Right motor speed: %d\n", right_motor);
        }
        if (position == 0) {
            printf("Motor speed should be: %d %d\n", baseSpeed, baseSpeed);
            printf("Follow line\n");
            setMotorPWM(baseSpeed, baseSpeed);
        }
        if (position > 0 && position != 0) {
            int left_motor = callback_actual_position(position, sensor);
            printf("Left motor speed should be: %d\n", left_motor);
        }
        if (sensor[2] == 1 && sensor[3] == 1) {
            printf("Current State: Follow line\n");
            break;
        }
    }
    return 0;
}

/* ---------- MAIN ---------- */
int main() {
    /* 6 sensors. Example: line is under sensors 0 and 1 → bot is far right */
    int sensor[NUM_SENSORS] = {1, 1, 0, 0, 0, 0};

    printf("Sensor Output: ");
    sensor_data(sensor);

    float position = bot_position(sensor);
    printf("Position: %.2f\n", position);

    if (position < 0) {
        int right_motor = callback_actual_position(position, sensor);
        printf("Right motor speed: %d\n", right_motor);
        left_move(sensor);
    }
    if (position == 0) {
        printf("Motor speed: %d %d\n", baseSpeed, baseSpeed);
        setMotorPWM(baseSpeed, baseSpeed);
    }
    if (position > 0) {
        int left_motor = callback_actual_position(position, sensor);
        printf("Left motor speed: %d\n", left_motor);
        right_move(sensor);
    }

    return 0;
}