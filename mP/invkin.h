#include <math.h>

/*
 * In case you can't program the inverse kinematics, include this header library in util.c with
 *  #include "invkin.h"
 *  and use the function "inverseKinematicsLib" within the function inverseKinematics in util.c as follows:
 *  int inverseKinematics(const double *plate_angles, double *servo_angles){
 *  return inverseKinematicsLib(plate_angles,servo_angles);
 */
int inverseKinematicsLib(const double *plate_angles, double *servo_angles);
