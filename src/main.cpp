#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <TrivialPID.h>
#include <Arduino.h>
#include <basicMPU6050.h> 

// float calcDistance(int revs, float wheelDiameter = 3.4f); 

void activateMotor(int dir, int in1, int in2);
void startMotor(int dir, int pwmVal, int enableMotor, int in1, int in2, int accelTo);
void stopMotor(int motor);
void accelerateMotorsSimultaneously(const int* motorPins, int targetPWM, \
                                    int accelStep, int delayMs);

#define NMOTORS 2

TrivialPID pid[NMOTORS];
TrivialPID driftCorrection;
// Variables to track motor positions
volatile int motor1_position = 0;
volatile int motor2_position = 0;
volatile int posi[] = {0,0}; 
volatile float distanceTravelled = 0.0;

volatile int correction[] = {0,0};

bool motorsState [NMOTORS] = {false, false};
bool motorsAccelState [NMOTORS] = {true, true};
bool motorRunning = false;

long prevT = 0;

const int encAPins[] = {18, 3};
const int encBPins[] = {2, 19};

const int motorPWMPins[] = {9, 8};
const int motorInPins[2][2] = {
                                {42, 44},  // Motor 1: in1[0] = 8, in2[0] = 11
                                {32, 34}  // Motor 2: in1[1] = 10, in2[1] = 12
                            };

// Create MPU instance
basicMPU6050<> imu;

float prevZ=0;

// Interrupt Service Routines (ISRs)
template <int m>
void ISR_motor()
{
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
void setup()
{       
    // Optional debugging
    // Serial.print("PWM: ");
    // Serial.println(currentPWM);
    Serial.begin(9600);

    // // Set registers - Always required
    // imu.setup();

    // // Initial calibration of gyro
    // imu.setBias();

    pinMode(motorPWMPins[0], OUTPUT);
    pinMode(motorInPins[0][0], OUTPUT);
    pinMode(motorInPins[0][1], OUTPUT);

    pinMode(motorPWMPins[1], OUTPUT);
    pinMode(motorInPins[1][0], OUTPUT);
    pinMode(motorInPins[1][1], OUTPUT);

    // Configure pins as inputs
    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

    // pid[0].setParams(1.75, 0.2, 0.0, 135);
    // pid[1].setParams(0.49999, 0.01, 0.0, 140);

    pid[0].setParams(0.5, 0, 0.0, 135);
    pid[1].setParams(0.499, 0.01, 0.0001, 140);

    delay(3000);

    // Attach pin change interruptsu
    attachInterrupt(digitalPinToInterrupt(encAPins[0]), ISR_motor<0>, RISING);
    attachInterrupt(digitalPinToInterrupt(encAPins[1]), ISR_motor<1>, RISING);

    Serial.println("Setup completed!");

    if (motorsAccelState[0])
    {
        for(int k = 0; k < NMOTORS; k++){
            activateMotor(1,motorInPins[k][0],motorInPins[k][1]);
            // Serial.println(k);
            // delay(2000);
        }
        accelerateMotorsSimultaneously(motorPWMPins, 90, 1, 1500);
        // delay(1000);
        // Serial.print("simultan fisnish");
        motorsAccelState[0]=false;

        posi[0] = 0; posi[1]=0;
    }

}

void loop()
{

    // // Update gyro calibration 
    // imu.updateBias();

    // prevZ -= imu.gz();
    // Serial.print(prevZ);
    // Serial.print(" \t ");

    // set target position
    int target[NMOTORS] = {2400, 2400};
    //   int target = 250*sin(prevT/1e6);

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // int pos = 0;
    int pos[NMOTORS];
    // int correction[] = {0,0};
    int drift =0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        for(int k = 0; k < NMOTORS; k++){
            pos[k] = posi[k];

            // Serial.print(pos[k]); Serial.print("\t");

            drift = (posi[0] - posi[1]) ; 

            // // // -6.790e-5 x^3 + 0.014 x^2 - 0.494 x + 8.767
            // float kernel = -6.181e-5 * pow(drift,3) + 0.0161 * pow(drift,2) \
            //                     - 0.670 * drift + 8.533;
            // if (drift > 0) {
            //     correction[1] =  kernel + drift * (pow(deltaT, 2));
            //     correction[0] =  kernel - drift / (pow(deltaT, 2));
            // }
            
            // Serial.print(kernel); Serial.print("\t");
            // Serial.print(drift); Serial.print("\t");
        }
    }


    // loop through the motors
    for(int k = 0; k < NMOTORS; k++) {

        int pwr, dir;
        // evaluate the control signal
        pid[k].errorValue(pos[k],target[k],deltaT,pwr,dir);

        // Serial.print(correction[k]); Serial.print("\t");
        // pos[k]=pos[k]+correction[k];
        // Serial.print(pwr); Serial.print("\t");

        startMotor(dir,pwr, motorPWMPins[k], motorInPins[k][0], motorInPins[k][1], 90);
        
            // Serial.print(pos[k]);
            // Serial.print(" \t ");
            // Serial.print(correction[k]);
            // Serial.print(" \t ");

    }

    for(int k = 0; k < NMOTORS; k++){
        Serial.print(target[k]);
        Serial.print(" \t ");
        Serial.print(pos[k]);
        Serial.print(" \t ");
    }

    Serial.println();

}

void startMotor(int dir, int pwmVal, int enableMotor, int in1, int in2, int accelTo) {
    // Set motor direction
    if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }

    // Apply steady-state PWM
    analogWrite(enableMotor, pwmVal);
}

void activateMotor(int dir, int in1, int in2) {
    // Set motor direction
    if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}


void stopMotor(int motorNumber) {
    if (motorNumber == 0) {
        // Stop Motor A
        digitalWrite(motorInPins[0][0], LOW);
        digitalWrite(motorInPins[0][1], LOW);
        analogWrite(motorPWMPins[0], 0);
        motorsState[motorNumber]=false;
    } else if (motorNumber == 1) {
        // Stop Motor B
        digitalWrite(motorInPins[1][0], LOW);
        digitalWrite(motorInPins[1][1], LOW);
        analogWrite(motorPWMPins[1], 0);

        motorsState[motorNumber]=false;
    }
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

void accelerateMotorsSimultaneously(const int* motorPins, int targetPWM, int accelStep, int delayMs) {
    int currentPWM = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)

    // Gradually increase PWM for both motors
{    while (currentPWM < targetPWM) {
        currentPWM += accelStep;
        if (currentPWM > targetPWM) {
            currentPWM = targetPWM;  // Prevent overshooting
        }

        // Apply the same PWM to both motors
        analogWrite(motorPins[0], currentPWM);
        analogWrite(motorPins[1], currentPWM);

        // Debugging output
        // Serial.print("Accelerating motors with PWM: ");
        // Serial.println(currentPWM);

        // Delay to control ramp-up speed
        delay(delayMs);
    }

    // Final PWM to ensure motors are running at the target speed
    analogWrite(motorPins[0], targetPWM);
    analogWrite(motorPins[1], targetPWM);
    // Serial.println("Acceleration complete!");
    }
}
