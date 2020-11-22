#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID
{
  public:
  PID();
  void setTunings(double _kp, double _kd, double _ki);
  void setLimits(double _max, double _min);
  double compute(double _PV, double _SP);

  private:
  double error, iTerm, dTerm, last_iTerm;
  double currentTime, elapsedTime, previousTime;
  double kp, kd, ki;
  double maxOut, minOut,out;
  double lastPV;
  double PV;
  double setP;
};
#endif
