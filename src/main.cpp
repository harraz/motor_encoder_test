#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <TrivialPID.h>
#include <Arduino.h>
#include <basicMPU6050.h>
#include <TaskScheduler.h>

// Function prototypes
void updateMotors();
void checkObstacle();
long getDistance();
void startMotor(int dir, int pwmVal, int enableMotor, int in1, int in2);
void stopMotor(int motorNumber);

// Define Scheduler
Scheduler runner;

// Define Tasks
Task taskMotors(20, TASK_FOREVER, &updateMotors);
Task taskObstacle(200, TASK_FOREVER, &checkObstacle);

#define NMOTORS 2
const int TRIG_PIN = 23; // Ultrasonic sensor trigger pin
const int ECHO_PIN = 22; // Ultrasonic sensor echo pin

TrivialPID pid[NMOTORS];

// Motor setup
const int encAPins[] = {2, 19};
const int encBPins[] = {18, 3};
const int motorPWMPins[] = {9, 8}; // PWM pins for motors
const int motorInPins[2][2] = { {44, 42}, {34, 32} }; // Direction pins

volatile int posi[] = {0, 0}; 
bool obstacleDetected = false;
long prevT = 0;

// Interrupt Service Routine (ISR) for Encoders
template <int m>
void ISR_motor() {
    int encB = digitalRead(encBPins[m]);
    posi[m] += (encB > 0) ? 1 : -1;
}

void setup() {
    Serial.begin(9600);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    for (int i = 0; i < NMOTORS; i++) {
        pinMode(motorPWMPins[i], OUTPUT);
        pinMode(motorInPins[i][0], OUTPUT);
        pinMode(motorInPins[i][1], OUTPUT);
    }

    pinMode(encAPins[0], INPUT_PULLUP);
    pinMode(encAPins[1], INPUT_PULLUP);
    pinMode(encBPins[0], INPUT_PULLUP);
    pinMode(encBPins[1], INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encAPins[0]), ISR_motor<0>, RISING);
    attachInterrupt(digitalPinToInterrupt(encAPins[1]), ISR_motor<1>, RISING);

    pid[0].setParams(0.399, 0.01, 0.0001, 140);
    pid[1].setParams(0.36, 0, 0.0001, 125);

    // Add tasks to the scheduler
    runner.addTask(taskMotors);
    runner.addTask(taskObstacle);

    // Enable tasks
    taskMotors.enable();
    taskObstacle.enable();
}

void loop() {
    runner.execute(); // Runs all scheduled tasks
}

// **Task 1: Motor Control**
void updateMotors() {
    if (obstacleDetected) {
        stopMotor(0);
        stopMotor(1);
        Serial.println("Obstacle detected! Stopping motors.");
        return;
    }

    int target[NMOTORS] = {2400, 2400};
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    prevT = currT;

    int pos[NMOTORS];

    // **Protect posi[] from ISR modifications**
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (int k = 0; k < NMOTORS; k++) {
            pos[k] = posi[k]; // Read encoder values safely
        }
    }

    for (int k = 0; k < NMOTORS; k++) {
        int pwr, dir;
        pid[k].errorValue(pos[k], target[k], deltaT, pwr, dir);
        startMotor(dir, pwr, motorPWMPins[k], motorInPins[k][0], motorInPins[k][1]);
    }
}

// **Task 2: Check for Obstacles**
void checkObstacle() {
    long distance = getDistance();
    if (distance < 20) {
        obstacleDetected = true;
        Serial.println("Obstacle detected!");
    } else {
        obstacleDetected = false;
    }
}

// **Motor Control Functions**
void startMotor(int dir, int pwmVal, int enableMotor, int in1, int in2) {
    digitalWrite(in1, dir == 1);
    digitalWrite(in2, dir == -1);
    analogWrite(enableMotor, pwmVal);
}

void stopMotor(int motorNumber) {
    digitalWrite(motorInPins[motorNumber][0], LOW);
    digitalWrite(motorInPins[motorNumber][1], LOW);
    analogWrite(motorPWMPins[motorNumber], 0);
}

// **Ultrasonic Sensor Function**
long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    return duration * 0.034 / 2; // Distance in cm
}
                  