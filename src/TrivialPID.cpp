#include "TrivialPID.h"

// Constructor
TrivialPID::TrivialPID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

// Set PID parameters
void TrivialPID::setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
}

// Evaluate the control signal
void TrivialPID::errorValue(int value, int target, float deltaT, int &pwr, int &dir) {
    // Error
    int e = target - value;

    // Derivative
    float dedt = (e - eprev) / deltaT;

    // Integral
    eintegral += e * deltaT;

    // Control signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // Motor power
    pwr = (int)fabs(u);
    if (pwr > umax) {
        pwr = umax;
    }

    // Motor direction
    dir = (u < 0) ? -1 : 1;

    // Store previous error
    eprev = e;
}
