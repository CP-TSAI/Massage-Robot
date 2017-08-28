#ifndef CONTROL_H
#define CONTROL_H

#include <math.h>
#include "System.h"

//  Add Function prototypes Here
void Gravity_Compensator( int ARM, float Gcomp[6] );//重力補償
void Forward_Kinematic( int ARM, float end_effector_pos[3] );//運動分析


#endif