#include <stdio.h>
#include <math.h>
#include <time.h>
#include "invkin.h"
#include "feather_serial.h"
#define M_PI 3.14159265358979323846

/* Parameter structs: two new types are defined which
 * will be used to access calibration parameters. The
 * "extern" keyword tells the compiler that the variable
 * has been declared elsewhere, and that this variable
 * should be linked. This way, the structs bbs and servo
 * are globally available and do not need to be entered
 * as arguments to each function.
 * In this case, the variables are declared in the file
 * parameters.c. This file also includes definitions for
 * functions which load the parameter values. The
 * declarations for these functions are here in util.h,
 * since they will be called by main.c 
 */

typedef struct
{
  double plate_height;
  double R_plate_joint;
  double R_base_servo;
  double l1;
  double l2;
  double first_link_angle_bias;
  double diameter_plate;
  double base_angles[3];
  double ball_radius;
  double focal_length;
  double radial_distortion_coeff[2];
  double distortion_center[2];
  double calibration_image_scale;
  double cam_offset[3];
  int window_size;
} parameter_struct;
extern parameter_struct bbs;

typedef struct
{
  int min_angle;
  int max_angle;
  double bias_A;
  double bias_B;
  double bias_C;
} servo_struct;

extern servo_struct servo;

/*initializes and tests system */
int initBallBalancingRobot(int fd);

/*Loads constant system parameters*/
int load_parameters();

/*Loads parameters related to the servomotors used */
int load_servo();

int inverseKinematics(const double *plate_angles, double *servo_angles);

int project2worldFrame(const int x_in, const int y_in, double *x_out, double *y_out);

int servoCommand(int fd, double *servo_angles);

int readFromPixy(int fd, int *flag, int *x, int *y);
