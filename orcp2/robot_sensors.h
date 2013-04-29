// 
// functions for process sensors
//
//
// robocraft.ru
//

#ifndef _ROBOT_SENSORS_H_
#define _ROBOT_SENSORS_H_

#if defined(ARDUINO)
# if ARDUINO >= 100
#  include "Arduino.h"
# else
#  include "WProgram.h"
# endif

// store motor pins
typedef struct MOTOR
{
  int in;     // INVERTOR INPUT (dir)
  int enable; // ENABLE (pwm)
} MOTOR;

#endif //#if defined(ARDUINO)

#endif //#ifndef _ROBOT_SENSORS_H_
