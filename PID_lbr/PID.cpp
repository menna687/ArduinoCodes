#include "Arduino.h"
#include "PID.h"

PID::PID()
{
  iTerm=0;
  previousTime=0;
  lastPV=0;
  last_iTerm=0;
}
void PID::setTunings(double _kp, double _kd, double _ki)
{
  kp= _kp;
  ki= _ki;
  kd= _kd;
}

void PID::setLimits(double _maxOut, double _minOut)      //output limits
{
maxOut= _maxOut;
minOut= _minOut;
}

double PID::compute(double _PV, double _SP)
{
  setP = _SP;
  PV = _PV;
  currentTime=millis();
  elapsedTime=currentTime-previousTime;
  error = setP - PV;                         //proportional term
  iTerm += (error * elapsedTime);          //integral term
  dTerm = (PV - lastPV) / elapsedTime;     //differential term

  out = kp*error + ki*iTerm - kd*dTerm;    // total PID output

  if (out > maxOut)           //check for upper limit
  {
    if((error*out) <=0)        //check for clamping integral (different signs of error and output)
    {
      iTerm=last_iTerm;     //ignore recently calculated value of the integral
    }
    out= maxOut;
  }

  else if (out<minOut)      //check for lower limit
  {
    if((error*out) <=0)        //check for clamping integral (different signs of error and output)
    {
      iTerm=last_iTerm;     //ignore recently calculated value of the integral
    }
    out= minOut;
  }

   //update last PV, time and iTerm values
   lastPV = PV;
   previousTime = currentTime;
   last_iTerm=iTerm;
   
   return out;
}

