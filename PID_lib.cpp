#include "Arduino.h"
#include "PID_lib.h"

PID::PID(double* input, double* output, double* setpoint,
         double initKp, double initKi, double initKd,
         double outMin, double outMax, Direction direction,
         unsigned long period)
{
    this->controllerInput = input;
    this->controllerOutput = output;
    this->controllerSetpoint = setpoint;

    this->outMin = outMin;
    this->outMax = outMax;

    this->direction = direction;

    this->period = period;
    
    PID::setGains(initKp, initKi, initKd);

    this->prevTime = millis() - period;

    this->enabled = false;
}

bool PID::compute()
{
    if (!this->enabled) return false; // Don't compute if the controller is not enabled

    unsigned long currTime = millis();
    unsigned long timeChange = currTime - prevTime;
    if(timeChange >= this->period)
    {
        double input = *(this->controllerInput);
        double error = *(this->controllerSetpoint) - input;
        double dError = error - this->prevError;

        this->errorSum = PID::clampToEndpoints(this->errorSum + 
                                               (this->ki * error)); // Keep track of accumulating
                                                                    // error for I-term

        *(this->controllerOutput) = PID::clampToEndpoints((this->kp * error) + 
                                                          this->errorSum + 
                                                          (this->kd * dError));

        this->prevError = error;
        this->prevTime = currTime;

        return true;
    }
    else
        return false;
}

void PID::start()
{
    this->errorSum = 0; // reset I-term
    this->prevError = 0; // reset variable for D-term
    this->enabled = true;
}

void PID::stop()
{
    *(this->controllerOutput) = this->outMin; // disable the output
    this->enabled = false;
}

void PID::setGains(double kp, double ki, double kd)
{
    if (kp < 0 || ki < 0 || kd < 0) return;

    double periodInSec = ((double)this->period) / 1000.0;
    this->kp = kp;
    this->ki = ki * periodInSec; // do this now to save
    this->kd = kd / periodInSec; // computations later

    this->kpDisp = kp;
    this->kiDisp = ki; // not optimized so the user can
    this->kdDisp = kd; // retrieve actual gain values

    if(this->direction == Reverse)
    {
        this->kp = 0 - this->kp;
        this->ki = 0 - this->ki;
        this->kd = 0 - this->kd;
    }
}

void PID::setPeriod(unsigned long period)
{
    double ratio  = (double)period / (double)this->period;
    this->ki *= ratio; // Adjust the I-term and D-term to
    this->kd /= ratio; // account for the new period
    this->period = period;
}

double PID::getKp()
{
    return  this->kpDisp;
}

double PID::getKi()
{
    return  this->kiDisp;
}

double PID::getKd()
{
    return  this->kdDisp;
}

Direction PID::getDirection()
{
    return this->direction;
}

bool PID::isEnabled()
{
    return this->enabled;
}

double PID::clampToEndpoints(double val)
{
    if(val > this->outMax)
        return this->outMax;
    else if(val < this->outMin)
        return this->outMin;
    return val;
}