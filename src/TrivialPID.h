#ifndef TRIVIALPID_H
#define TRIVIALPID_H

#include <Arduino.h>  // Include Arduino library for types like `float`

class TrivialPID {
  private:
    float kp, kd, ki, umax;  // Parameters
    float eprev, eintegral;  // Storage

  public:
    // Constructor
    TrivialPID();

    // A function to set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn);

    // A function to compute the control signal
    void errorValue(int value, int target, float deltaT, int &pwr, int &dir);
};

#endif
