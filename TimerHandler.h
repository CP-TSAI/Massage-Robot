#ifndef TIMER_HANDLER_H
#define TIMER_HANDLER_H

#include <windows.h>
#include <wchar.h>
#include <rtapi.h>
#include <stdlib.h>
#include <stdio.h>
//#include <conio.h>
#include <math.h>
#include <vector>
#include <utility>	//pair

#include "Control.h"
#include "System.h"

#include "Tcp6D.h"
// Robotics Library
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/math/Cubic.h>

// function prototype for periodic timer function
void RTFCNDCL TimerHandler( void * nContext );

// Interrupt handler prototype
void RTFCNDCL InterruptHandler( void * nContext ); //.cpp裡沒有實作

#endif