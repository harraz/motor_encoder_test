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
void processSerialCommand(); // Function to handle serial input
void checkPosition();
void avoidObstacle();

// Define Scheduler
Scheduler runner;

// Define Tasks
Task taskMotors(20, TASK_FOREVER, &updateMotors);
Task taskObstacle(200, TASK_FOREVER, &checkObstacle);
Task taskSerial(50, TASK_FOREVER, &processSerialCommand);
Task taskCheckPos(20, TASK_FOREVER, &checkPosition);
Task taskAvoidObs(10,TASK_FOREVER, &avoidObstacle);

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
bool isMotorsRunning =false;

// Interrupt Service Routine (ISR) for Encoders
template <int m>
void ISR_motor() {
    int encB = digitalRead(encBPins[m]);
    posi[m] += (encB > 0) ? 1 : -1;
}

void setup() {
    Serial.begin(9600);
    Serial3.begin(9600);

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

    // pid[0].setParams(0.399, 0.01, 0.0001, 100);
    // pid[1].setParams(0.36, 0, 0.0001, 105);

    // Add tasks to the scheduler
    runner.addTask(taskMotors);
    runner.addTask(taskObstacle);
    runner.addTask(taskSerial);
    runner.addTask(taskCheckPos);
    runner.addTask(taskAvoidObs);

    // Enable tasks
    taskMotors.enable();
    taskObstacle.enable();
    taskSerial.enable();
    taskCheckPos.enable();
    taskAvoidObs.enable();
}

void loop() {

    runner.execute(); // Runs all scheduled tasks

}

// **Task 1: Motor Control**
void updateMotors() {

    // **Protect posi[] from ISR modifications**
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (int k = 0; k < NMOTORS; k++) {
            pos[k] = posi[k]; // Read encoder values safely
        }
    }

}


void avoidObstacle(){

    if (obstacleDetected) {
      taskObstacle.disable();
      Serial.println("avoiding");
  
      // Randomly choose a delay between 2000 and 4000 milliseconds
      int randomDelay = random(2000, 4000);
      delay(randomDelay);
  
      Serial.println("Moving Backward");
      startMotor(-1, 65, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
      startMotor(-1, 65, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
      delay(1000);
  
      // Randomly choose between maneuverLeft and maneuverRight
      if (random(0, 2) == 0) {
        maneuverLeft();
      } else {
        maneuverRight();
      }
      delay(1000);
  
      stopMotor(0); stopMotor(1);
  
      startMotor(1, 65, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
      startMotor(1, 65, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
  
      // Reset the obstacleDetected flag after a random delay between 1000 and 3000 milliseconds
      randomDelay = random(1000, 3000);
      delay(randomDelay);
      obstacleDetected = false;
  
      taskObstacle.enable();
    }
  }

void checkPosition() {

    for (int k = 0; k < NMOTORS; k++) {
        Serial.print(pos[k]); Serial.print("\t");
        if ((pos[k] >= prevPos[k] + 5000)) {
            stopMotor(k);
            prevPos[k]=pos[k];
        }
    }

    Serial.println();

}

// **Task 2: Check for Obstacles**
void checkObstacle() {
    long distance = getDistance();
    if (distance < 45) {
        obstacleDetected = true;
        Serial.println("Obstacle detected!");
        stopMotor(0); stopMotor(1);
    } else {
        obstacleDetected = false;
    }
}

// **Task 3: Process Serial Commands from HUZZAH**
void processSerialCommand() {
    if (Serial3.available() > 0) {
        String command = Serial3.readStringUntil('\n');
        command.trim();
        Serial.println(command);

        char cmd = command.charAt(0);
        if (cmd == 'F') {
            Serial.println("Moving Forward");
            startMotor(1, 65, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
            startMotor(1, 65, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
        } else if (cmd == 'B') {
            Serial.println("Moving Backward");
            startMotor(-1, 65, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
            startMotor(-1, 65, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
        } else if (cmd == 'L') {
            Serial.println("Turning Left");
            maneuverLeft();
        } else if (cmd == 'R') {
            Serial.println("Turning Right");
            maneuverRight();
        } else if (cmd == 'S') {
            Serial.println("Stopping");
            stopMotor(0); stopMotor(1);
        } else {
            Serial.println("Invalid Command!");
        }
    }
}

// **Motor Control Functions**
void startMotor(int dir, int pwmVal, int enableMotor, int in1, int in2) {

    digitalWrite(in1, dir == 1);
    digitalWrite(in2, dir == -1);
    
    pwmVal = constrain(pwmVal, 0, 255);

    // Serial.print(pos[0]); Serial.print("\t"); Serial.println(pos[1]);
    analogWrite(enableMotor, pwmVal);
    isMotorsRunning= true;
}

void stopMotor(int motorNumber) {
    digitalWrite(motorInPins[motorNumber][0], LOW);
    digitalWrite(motorInPins[motorNumber][1], LOW);
    analogWrite(motorPWMPins[motorNumber], 0);
    isMotorsRunning= false;

}

void maneuverLeft() {
    startMotor(-1, 45, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
    startMotor(1, 50, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
    isMotorsRunning= true;
}

void maneuverRight() {
    startMotor(1, 45, motorPWMPins[0], motorInPins[0][0], motorInPins[0][1]);
    startMotor(-1, 50, motorPWMPins[1], motorInPins[1][0], motorInPins[1][1]);
    isMotorsRunning= true;
}
long getDistance() {
  // Example ultrasonic code
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // Speed of sound = ~0.034 cm/µs

//   Serial.println(distance);
  return distance; 
}
