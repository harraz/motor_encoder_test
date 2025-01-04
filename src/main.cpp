#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <Arduino.h>

void activateMotors(int, int);
void stopMotors();
float calcDistance(int revs, float wheelDiameter = 3.4f); 

// Variables to track motor positions
volatile int motor1_position = 0;
volatile int motor2_position = 0;

volatile float distanceTravelled = 0.0;

bool motorRunning = false;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

// Motor A Pins
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// Motor B Pins
const int enB = 3;
const int in3 = 5;
const int in4 = 4;

// Interrupt Service Routines (ISRs)
void ISR_motor1_A()
{

    // int m1A = digitalRead(18);
    int m1B = digitalRead(19);
    // motor1_position += (m1A > m1B) ? -1 : 1; // Clockwise or counter-clockwise

    if (m1B > 0)
    {
        motor1_position++;
    }
    else
    {
        motor1_position--;
    }
}

void ISR_motor2_A()
{
    int m2B = digitalRead(21);

    if (m2B > 0)
    {
        motor2_position++;
    }
    else
    {
        motor2_position--;
    }
}

void setup()
{
    Serial.begin(9600);

    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // Configure pins as inputs
    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    pinMode(20, INPUT_PULLUP);
    pinMode(21, INPUT_PULLUP);

    // Attach pin change interruptsu
    attachInterrupt(digitalPinToInterrupt(18), ISR_motor1_A, RISING);
    // attachInterrupt(digitalPinToInterrupt(19), ISR_motor1_A, FALLING);
    attachInterrupt(digitalPinToInterrupt(20), ISR_motor2_A, RISING);
    // attachInterrupt(digitalPinToInterrupt(21), ISR_motor2_B, CHANGE);

    // stopMotors();
    // activateMotors(55, 1);
    delay(1200);

    Serial.println("Setup completed!");
}

void loop()
{

    // set target position
    int target = 1200;
    //   int target = 250*sin(prevT/1e6);

    // PID constants
    float kp = 1.0;
    float kd = 0.0;
    float ki = 0.0;

    // time difference
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;

    int pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pos = (motor2_position + motor2_position)/2;
        distanceTravelled = calcDistance(pos/4);
    }

    // Serial.println(pos);

    // error
    int e = pos - target;

    // derivative
    float dedt = (e - eprev) / (deltaT);

    // integral
    eintegral = eintegral + e * deltaT;

    // control signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // motor power
    float pwr = fabs(u);
    if (pwr > 255)
    {
        pwr = 255;
    } 
    // else if (pwr <= 20) {
    //     pwr = 40;
    // }

    // motor direction
    int dir = 1;
    if (u < 0)
    {
        dir = 0;
    }

    // signal the motor
    //   setMotor(dir,pwr,PWM,IN1,IN2);

    if ((e <= 0.1 ) && (motorRunning))
    {
        stopMotors();
        // Serial.println("motor stopped");
        // delay(5);
    }
    else
    {
        activateMotors(pwr, dir);
    }

    // store previous error
    eprev = e;

    // Serial.print(target);
    // Serial.print(" ");
    // Serial.print(pos);
    // Serial.print(" ");
    // Serial.print(e);
    // Serial.print(" ");
    // Serial.print(" ");
    // Serial.print(pwr);
    // Serial.print(" ");
    // Serial.print(distanceTravelled);
    // Serial.print(" ");
    // Serial.println();

    Serial.print(motor1_position);
    Serial.print(" ");
    Serial.print(motor2_position);
    Serial.print(" ");
    Serial.print(" ");
    Serial.print(distanceTravelled);
    Serial.print(" ");
    Serial.println();
    
    // delay(5);
}

void activateMotors(int speed, int direction)
{

    //   motorRunning = true; // Update motor state

    // Motor A Control
    if (direction == 1)
    { // Forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else
    { // Backward
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    analogWrite(enA, speed);

    // Motor B Control
    if (direction == 1)
    { // Forward
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }
    else
    { // Backward
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }
    analogWrite(enB, speed);

    motorRunning= true;
}

void stopMotors()
{

    //   motorRunning = false; // Update motor state

    // Stop Motor A
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);

    // Stop Motor B
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);

    motorRunning = false;
}

/// @brief function that returns distnace travelled in cm
/// @param revs number of revolution at the function call
/// @param wheelDiameter diameter of the wheel in cm
/// @return would be distnace travelled in cm
float calcDistance(int revs, float wheelDiameter = 3.4f) {

    // float wheelDiameter = 4.3f;
    float wheelCircumfetence = PI * wheelDiameter;

    return (revs * wheelCircumfetence);

    
}