#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <TrivialPID.h>
#include <Arduino.h>

void activateMotors(int, int);
void stopMotors();
// float calcDistance(int revs, float wheelDiameter = 3.4f); 
void acitvateMotor(int dir, int pwmVal, int enableMotor, int in1, int in2);
void stopMotor(int motor);

#define NMOTORS 2

// Variables to track motor positions
volatile int motor1_position = 0;
volatile int motor2_position = 0;
volatile double posi[] = {0,0}; volatile double correction =0;
volatile float distanceTravelled = 0.0;

bool motorsState [NMOTORS] = {false, false};

TrivialPID pid[NMOTORS];

bool motorRunning = false;

long prevT = 0;

const int motorPWMPins[] = {9, 3};
const int encAPins[] = {18, 20};
const int encBPins[] = {19, 21};
const int motorInPins[2][2] = {
                                {8, 7},  // Motor 1: in1[0] = 8, in2[0] = 11
                                {5, 4}  // Motor 2: in1[1] = 10, in2[1] = 12
                            };


// Motor A Pins
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// Motor B Pins
const int enB = 3;
const int in3 = 5;
const int in4 = 4;

// Interrupt Service Routines (ISRs)
template <int m>
void ISR_motor()
{

    // int m1A = digitalRead(18);
    int encB = digitalRead(encBPins[m]);
    // motor1_position += (m1A > m1B) ? -1 : 1; // Clockwise or counter-clockwise

    if (encB > 0)
    {
        posi[m]++;
    }
    else
    {
        posi[m]--;
}
}

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

    pid[0].setParams(0.95, 0.0, 0.0, 100);
    pid[1].setParams(0.95, 0.122, 0.0, 100);

    // Attach pin change interruptsu
    // attachInterrupt(digitalPinToInterrupt(18), ISR_motor1_A, RISING);
    // attachInterrupt(digitalPinToInterrupt(20), ISR_motor2_A, RISING);
    attachInterrupt(digitalPinToInterrupt(encAPins[0]), ISR_motor<0>, RISING);
    attachInterrupt(digitalPinToInterrupt(encAPins[1]), ISR_motor<1>, RISING);

    // stopMotors();
    // activateMotors(55, 1);
    delay(3000);

    Serial.println("Setup completed!");
}

void loop()
{

    // set target position
    int target[NMOTORS] = {2500, 2500};
    //   int target = 250*sin(prevT/1e6);

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // int pos = 0;
    int pos[NMOTORS];

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
    //     float averagePosition = (motor1_position + motor2_position)/2;
    //     distanceTravelled = calcDistance(averagePosition/4);
        for(int k = 0; k < NMOTORS; k++){
            pos[k] = posi[k];
            // e[k] = pos[k] - target[k];
        }
    }
    
    // Serial.print(correction); Serial.print("\t");
    // Serial.print(pos[0]); Serial.print("\t");
    // Serial.print(pos[1]); Serial.print("\t");

    // loop through the motors
    for(int k = 0; k < NMOTORS; k++) {

        if ((k==1) && (pos[1]>pos[0])) {
        
            int drift = pos[1] - pos[0];
            correction = exp(drift * 0.011470) * (0.961126);
            Serial.print(pos[0]); Serial.print("\t");
            pos[0] =+ correction;
            Serial.print(correction); Serial.print("\t");
            Serial.print(pos[0]); Serial.print("\t");

        }

        int pwr, dir;
        // evaluate the control signal
        pid[k].errorValue(pos[k],target[k],deltaT,pwr,dir);
        
        acitvateMotor(dir,pwr, motorPWMPins[k], motorInPins[k][0], motorInPins[k][1]);

    }

    // for(int k = 0; k < NMOTORS; k++){
    //     Serial.print(target[k]);
    //     Serial.print(" \t ");
    //     Serial.print(pos[k]);
    //     Serial.print(" \t ");
    // }

    Serial.println(":eol:");

}

void acitvateMotor(int dir, int pwmVal, int enableMotor, int in1, int in2){
    

    if (enableMotor == motorPWMPins[0]) {

        if(dir == 1){
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        }
        else if(dir == -1){
        digitalWrite(in1,LOW);
        digitalWrite(in2,HIGH);
        }
        else{
        digitalWrite(in1,LOW);
        digitalWrite(in2,LOW);
        }

        motorsState[0]= true;

    } else if (enableMotor == motorPWMPins[1]) {
        if(dir == 1){
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        }
        else if(dir == -1){
        digitalWrite(in1,LOW);
        digitalWrite(in2,HIGH);
        }
        else{
        digitalWrite(in1,LOW);
        digitalWrite(in2,LOW);
        }

        motorsState[1]= true;
    }

    analogWrite(enableMotor, pwmVal);

    // motorRunning = true;
}

void stopMotor(int motorNumber) {
    if (motorNumber == 0) {
        // Stop Motor A
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(enA, 0);
        motorsState[motorNumber]=false;
    } else if (motorNumber == 1) {
        // Stop Motor B
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        analogWrite(enB, 0);
        motorsState[motorNumber]=false;
    }

    // motorRunning = false;
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

/// @brief function that returns distnace travelled 
/// @param revs number of revolution at the function call
/// @param wheelDiameter diameter of the wheel 
/// @return would be distnace travelled 
float calcDistance(int revs, float wheelDiameter = 3.4f) {

    // float wheelDiameter = 4.3f;
    float wheelCircumfetence = PI * wheelDiameter;

    return (revs * wheelCircumfetence);

    
}