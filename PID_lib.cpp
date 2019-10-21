#include "Arduino.h"
#include "PID_lib.h"

PID::PID(int16_t kp, int16_t ki, int16_t kd,
         int16_t outMin, int16_t outMax,
         unsigned long period)
{
    this->pidOutput = 0;
    this->pidSetpoint = 0;

    this->outMin = outMin;
    this->outMax = outMax;

    this->period = period;
    
    this->k1 =  kp + ki +     kd;
    this->k2 = -kp      - 2 * kd;
    this->k3 =                kd;

    this->e1 = 0;
    this->e2 = 0;
    this->e3 = 0;

    this->prevTime = millis() - period;

    this->enabled = false;
}

int16_t PID::compute(double input)
{
    unsigned long currTime = millis();
    unsigned long timeChange = currTime - prevTime;
    if(timeChange >= this->period && this->enabled)
    {
        int16_t deltaOutput = 0;

        e3 = e2;
        e2 = e1;
        e1 = this->pidSetpoint - (int16_t)(input * INT_RESOLUTION);

        deltaOutput = (this->k1 * e1) + (this->k2 * e2) + (this->k3 * e3);

        this->pidOutput += deltaOutput;

        if (this->pidOutput > INT_RESOLUTION * this->outMax)
        {
            this->pidOutput = INT_RESOLUTION * this->outMax;
        }
        else if (this->pidOutput < INT_RESOLUTION * this->outMin)
        {
            this->pidOutput = INT_RESOLUTION * this->outMin;
        }
    }
    return this->pidOutput / INT_RESOLUTION;
}

void PID::set(double target)
{
    this->pidSetpoint = (int16_t)(target * INT_RESOLUTION);
}

void PID::start()
{
    this->e1 = 0; // reset e terms
    this->e2 = 0;
    this->e3 = 0;
    this->pidOutput = this->outMin;
    this->enabled = true;
}

void PID::stop()
{
    this->pidOutput = this->outMin; // disable the output
    this->enabled = false;
}

bool PID::isEnabled()
{
    return this->enabled;
}