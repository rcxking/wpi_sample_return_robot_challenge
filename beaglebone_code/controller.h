#ifndef CONTROLLER_H
#define CONTROLLER_H 

#include "math.h"

// Torque constant in Newton Meters
// Value given in motor documentation
#define KT (5.7*0.00706155)

// back-emf constant in rad/(sV)
// Value given in motor documentation
#define KV (237/(60*2*M_PI))

//Motor and wheel inertia (kgm^2), found experimentally
#define JM 1

//Motor viscous friction coefficient (kgm^2/s), found experimentally
#define BM 1

// Maximum allowed linear velocity, set in competition rules
#define MAX_VEL 2

// Maximum allowed acceleration m/s^2
#define MAX_ACCEL 2

// distance between left and right wheels
#define WHEEL_BASE 1

// wheel radius in (m)
#define WHEEL_RADIUS (12.5/2*0.0254)

// mass of the robot (kg)
#define ROBOT_MASS 45.3

// inertia of the robot about z axis, not yet known
#define ROBOT_INERTIA 1



#endif /* CONTROLLER_H */
