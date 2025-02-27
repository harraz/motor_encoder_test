#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <TrivialPID.h>
#include <Arduino.h>
#include <basicMPU6050.h>
#include <TaskScheduler.h>

// Function prototypes
void maneuverLeft();
void maneuverRight();
void updateMotors();
void checkObstacle();
long getDistance();
void startMotor(int dir, int pwmVal, int enableMotor, int in1, int in2);
void stopMotor(int motorNumber);
void processSerialCommand(); // Process UDP serial input
void checkPosition();
void checkAndAvoidObstacle(); // New avoidance function (non-blocking)
void sendTelemetryData();
const char* getAvoidanceStageString();

// Define Scheduler
Scheduler runner;

// Define Tasks
Task taskMotors(50, TASK_FOREVER, &updateMotors);
Task taskObstacle(100, TASK_FOREVER, &checkObstacle);
Task taskSerial(50, TASK_FOREVER, &processSerialCommand);
Task taskCheckPos(50, TASK_FOREVER, &checkPosition);
Task taskAvoidance(50, TASK_FOREVER, &checkAndAvoidObstacle);
Task taskTelemetry(100, TASK_FOREVER, &sendTelemetryData);

// Define a new avoidance stage enum and related variables:
enum AvoidanceStage { 
  AV_NONE, 
  AV_REVERSE, 
  AV_TURN, 
  AV_DONE 
};
AvoidanceStage avoidanceStage = AV_NONE;
unsigned long avoidanceStageStartTime = 0;
const unsigned long reverseDuration = 1500; // Reverse for 1500 ms
const unsigned long turnDuration    = 1000; // Turn for 1500 ms

// Define Drive Modes
enum DriveMode { 
  DRIVE_IDLE, 
  DRIVE_FORWARD, 
  DRIVE_BACKWARD, 
  DRIVE_LEFT, 
  DRIVE_RIGHT 
};
DriveMode lastCommand = DRIVE_IDLE;

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
int pos[NMOTORS];
int prevPos[NMOTORS];
bool isMotorsRunning = false;
bool isMoving = false;

// Interrupt Service Routine (ISR) for Encoders
template <int m>
void ISR_motor() {
  int encB = digitalRead(encBPins[m]);
  posi[m] += (encB > 0) ? 1 : -1;
}

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  while (Serial3.available() > 0) {
    Serial3.read();
  }
    
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
  attachInterrupt(digitalPinToInterrupt(encAPins[0]), ISR_motor<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAPins[1]), ISR_motor<1>, CHANGE);
  // Optionally initialize PID parameters:
  // pid[0].setParams(0.399, 0.01, 0.0001, 100);
  // pid[1].setParams(0.36, 0, 0.0001, 105);
  // Add tasks to the scheduler
  runner.addTask(taskSerial);
  runner.addTask(taskMotors);
  runner.addTask(taskObstacle);
  runner.addTask(taskCheckPos);
  runner.addTask(taskAvoidance);
  runner.addTask(taskTelemetry);
    
  delay(2000);  // Sensor calibration delay.
  // Enable tasks
  taskMotors.enable();
  taskObstacle.enable();
  taskSerial.enable();
  taskAvoidance.enable();
  taskTelemetry.enable();
  taskCheckPos.enable();
}

void loop() {
  runner.execute();
}

// Task 1: Motor Control
void updateMotors() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (int k = 0; k < NMOTORS; k++) {
      pos[k] = posi[k];
    }
  }
}

// Task 2: Check Position
void checkPosition() {
  for (int k = 0; k < NMOTORS; k++) {
    if (pos[k] >= prevPos[k] + 5000) {
      stopMotor(k);
      prevPos[k] = pos[k];
    }
  }
}

// Task 3: Check for Obstacles
void checkObstacle() {
  // Skip sensor update during an avoidance maneuver.
  if (avoidanceStage != AV_NONE)
    return;
    
  long distance = getDistance();
  // Use a threshold (adjust if needed)
  if (distance < 50) {
    obstacleDetected = true;
    Serial.println("Obstacle detected!");
    // Immediately stop motors on detection.
    stopMotor(0);
    stopMotor(1);
  } else {
    obstacleDetected = false;
  }
}

// Task 4: Process Serial Commands from HUZZAH (UDP)
void processSerialCommand() {
  if (Serial3.available() > 0) {
    String command = Serial3.readStringUntil('\n');
    command.trim();
    Serial.println(command);
    char cmd = command.charAt(0);
        
    // Reset avoidance state on new command.
    avoidanceStage = AV_NONE;
        
    if (cmd == 'F') {
      Serial.println("Moving Forward");
      isMoving = true;
      lastCommand = DRIVE_FORWARD;
      taskObstacle.enable();
      startMotor(1, 65, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
      startMotor(1, 65, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
    } else if (cmd == 'B') {
      Serial.println("Moving Backward");
      isMoving = true;
      lastCommand = DRIVE_BACKWARD;
      taskObstacle.disable();
      startMotor(-1, 65, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
      startMotor(-1, 65, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
    } else if (cmd == 'L') {
      Serial.println("Turning Left");
      isMoving = true;
      lastCommand = DRIVE_LEFT;
      taskObstacle.enable();
      maneuverLeft();
    } else if (cmd == 'R') {
      Serial.println("Turning Right");
      isMoving = true;
      lastCommand = DRIVE_RIGHT;
      taskObstacle.enable();
      maneuverRight();
    } else if (cmd == 'S') {
      Serial.println("Stopping");
      isMoving = false;
      lastCommand = DRIVE_IDLE;
      taskObstacle.disable();
      obstacleDetected = false;
      stopMotor(0);
      stopMotor(1);
    } else {
      Serial.println("Invalid Command!");
    }
  }
}

// Task 5: Check and Perform Obstacle Avoidance Maneuver (non-blocking)
void checkAndAvoidObstacle() {
  // Only attempt avoidance if the robot is moving.
  if (!isMoving) return;
  
  // If not already in avoidance and an obstacle is detected, start avoidance.
  if (avoidanceStage == AV_NONE && obstacleDetected) {
    Serial.println("Starting avoidance maneuver");
    avoidanceStage = AV_REVERSE;
    avoidanceStageStartTime = millis();
    // Disable further obstacle checking during avoidance.
    taskObstacle.disable();
    // Begin by reversing.
    stopMotor(0);
    stopMotor(1);
    startMotor(-1, 65, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
    startMotor(-1, 65, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
    return;
  }
  
  // Process avoidance stages:
  if (avoidanceStage == AV_REVERSE) {
    if (millis() - avoidanceStageStartTime >= reverseDuration) {
      // End reverse; begin turning.
      stopMotor(0);
      stopMotor(1);
      avoidanceStage = AV_TURN;
      avoidanceStageStartTime = millis();
      // Randomly choose to turn left or right.
      if (random(0, 2) == 0) {
        maneuverLeft();
      } else {
        maneuverRight();
      }
      return;
    }
  }
  else if (avoidanceStage == AV_TURN) {
    if (millis() - avoidanceStageStartTime >= turnDuration) {
      // End turn.
      stopMotor(0);
      stopMotor(1);
      avoidanceStage = AV_DONE;
    }
  }
  else if (avoidanceStage == AV_DONE) {
    // Resume last command.
    Serial.println("Avoidance complete, resuming last command");
    switch (lastCommand) {
      case DRIVE_FORWARD:
        startMotor(1, 65, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
        startMotor(1, 64, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
        break;
      case DRIVE_BACKWARD:
        startMotor(-1, 65, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
        startMotor(-1, 64, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
        break;
      case DRIVE_LEFT:
        maneuverLeft();
        break;
      case DRIVE_RIGHT:
        maneuverRight();
        break;
      default:
        break;
    }
    // Reset avoidance state and sensor flag.
    avoidanceStage = AV_NONE;
    obstacleDetected = false;
    // Re-enable obstacle checking.
    taskObstacle.enable();
  }
}

// Helper function to return a string for the avoidance stage.
const char* getAvoidanceStageString() {
  switch(avoidanceStage) {
    case AV_NONE:    return "NONE";
    case AV_REVERSE: return "REVERSE";
    case AV_TURN:    return "TURN";
    case AV_DONE:    return "DONE";
    default:         return "UNKNOWN";
  }
}

// Task 6: Telemetry Data
void sendTelemetryData() {
  // Format telemetry data: T,<avoidanceStage>,<pos0>,<pos1>,<obstacle>,<distance>
  String telemetry = "T:";
  telemetry += String(getAvoidanceStageString()) + ",";
  telemetry += String(pos[0]) + ",";
  telemetry += String(pos[1]) + ",";
  telemetry += String(obstacleDetected ? 1 : 0) + ",";
  telemetry += String(getDistance());
  Serial3.println(telemetry);
}

// Motor Control Functions
void startMotor(int dir, int pwmVal, int enableMotor, int in1, int in2) {
  digitalWrite(in1, dir == 1);
  digitalWrite(in2, dir == -1);
  pwmVal = constrain(pwmVal, 0, 255);
  analogWrite(enableMotor, pwmVal);
  isMotorsRunning = true;
}

void stopMotor(int motorNumber) {
  digitalWrite(motorInPins[motorNumber][0], LOW);
  digitalWrite(motorInPins[motorNumber][1], LOW);
  analogWrite(motorPWMPins[motorNumber], 0);
  isMotorsRunning = false;
}

void maneuverLeft() {
  startMotor(-1, 45, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
  startMotor(1, 50, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
  isMotorsRunning = true;
}

void maneuverRight() {
  startMotor(1, 45, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
  startMotor(-1, 50, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
  isMotorsRunning = true;
}

long getDistance() {
  // Ultrasonic sensor code
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // ~0.034 cm/µs
  Serial.println(distance);
  return distance;
}
