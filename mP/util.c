#include "util.h"
#include "newton_raphson.h"

int initBallBalancingRobot(int fd)
{

  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#######################\n");
  printf("Hardware Initialization\n");
  printf("#######################\n");

  // Let feather reboot
  usleep(200);

  // Check pixy readings
  int pixy_return, flag, x, y;
  pixy_return = readFromPixy(fd, &flag, &x, &y);
  while (!pixy_return)
  {
    // retry until connection established
    pixy_return = readFromPixy(fd, &flag, &x, &y);
  }

  printf("%s, x = %d, y = %d \n","INIT: Pixy Coordinates received", x, y);

  // Do some inverse kinematics to check for errors
  double position[] = {0, 0, 130};
  double plate_angles[] = {0, 0};
  double servo_angles[] = {0, 0, 0};
  inverseKinematics(plate_angles, servo_angles);
  printf("INIT: Inverse Kinematics initialized\n");
  printf("INIT: Finished\n");
  // printf("INIT: A: %.2f, B: %.2f, C: %.2f
  // \n",servo_angles[0],servo_angles[1],servo_angles[2]);

  tcflush(fd, TCIFLUSH);

  // Send motor commands
  // servoCommand(fd,servo_angles);

  return 1;
}

int inverseKinematics(const double *plate_angles, double *servo_angles)
{
  // Load parameters R, L_1, L_2, P_z etc. from parameters file. Example: double R = bbs.R_plate_joint;
  // Then implement inverse kinematics similar to prelab

  //Ball balance system parameters
  float L1 = bbs.l1;
  float L2 = bbs.l2;
  float P_z = bbs.plate_height;
  float R = bbs.R_plate_joint;
  float beta[3];

  //convert the input angles from degrees to radians
  double phi_x = *plate_angles;
  double tetha_y = *(plate_angles+1); 
  double phi_x_rad = (phi_x/180)*M_PI;
  double tetha_y_rad = (tetha_y/180)*M_PI;

  //equations for offsets
  double del_zA = R*sin(phi_x_rad);
  double del_zB = -0.5*R*sin(phi_x_rad)+(sqrt(3.00)/2)*R*sin(tetha_y_rad);
  double del_zC = -0.5*R*sin(phi_x_rad)-(sqrt(3.00)/2)*R*sin(tetha_y_rad);
  double offsets[] = {del_zA,del_zB,del_zC};

  //loop through each robot arm and find the corresponding beta
  int i;
  for (i = 0; i < 3; i ++) {
    double La = P_z + offsets[i];
    double num = (La*La)+(L1*L1)-(L2*L2);
    double den = 2*La*L1;
    beta[i] = acos(num/den);

    if(isnan(beta[i])){
      return -1;
    }
  }

  //calculate the servo angles alpha in degrees.
  for(i = 0 ;i < 3; i++) {
    *(servo_angles+i)= (0.5*M_PI-beta[i])*(180/M_PI);
  }


  // return -1; // if invalid input angle
  return 0; // if ok
};

int project2worldFrame(const int x_in, const int y_in, double *x_out, double *y_out)
{

  // implement the code to project the coordinates in the image frame to the world frame
  // make sure to multiply the raw pixy2 coordinates with the scaling factor (ratio between
  // image fed to python for calibration and pixy2 resolution): bbs.calibration_image_scale.

  /* ********************* */
  //load camera intrinsic parameters
  double P_z = bbs.plate_height;
  double u_o = bbs.distortion_center[0];
  double v_o = bbs.distortion_center[1];
  double alpha = bbs.focal_length;
  double k1 = bbs.radial_distortion_coeff[0];
  double k2 = bbs.radial_distortion_coeff[1];
  // project the coordinates in the image frame to the world frame
  // multiply the raw pixy2 coordinates with the scaling factor (ratio between image
  // fed to python for calibration and pixy2 resolution) : bbs.calibration_image_scale.

  double u_d = bbs.calibration_image_scale * x_in;
  double v_d = bbs.calibration_image_scale * y_in;
  // normalizing the distorted coordinates
  double ud_norm = (u_d - u_o) / alpha;
  double vd_norm = (v_d - v_o) / alpha;

  // distortion radius
  double r_d = sqrt((ud_norm*ud_norm) + (vd_norm*vd_norm));

  // using the Newton-Raphson method to obtain r(undistortion radius)
  double r = newtonRaphson(r_d, k1, k2);

  // computing the normalized undistorted coordinates
  double u_norm = (1/(1 + k1*pow(r,2) + k2*pow(r,4))) * ud_norm;
  double v_norm = (1/(1 + k1*pow(r,2) + k2*pow(r,4))) * vd_norm;

  // Intrinsics obtaining Xc,Yc, Zc

  double Pc[3];
  Pc[2] = P_z + bbs.ball_radius + bbs.cam_offset[2];
  Pc[0] =  Pc[2] * u_norm;
  Pc[1] =  Pc[2] * v_norm;

  // obtaining R and T to get real world coordinates Xw, Yw and Zw
  // Pc = R*Pw + T => Pw = inv(R)*(Pc - T)
  // for nature of R we have, inv(R) == R

  double R[3][3] = {{cos(M_PI), sin(M_PI), 0}, {-sin(M_PI), cos(M_PI), 0}, {0, 0, 1}};
  double Pw[3];

  // computing matrix multiplication

  int i, j;
  for (i = 0; i < 3; i ++){
    Pw[i] = 0;
    for (j = 0; j < 3; j ++){
      Pw[i] += R[i][j]*(Pc[j] - bbs.cam_offset[j]);
    }
  }

  //store world coordinates in x_out and y_out
  *x_out = Pw[0];
  *y_out = Pw[1];
  printf("\nradius = %.0f", sqrt((Pw[0]*Pw[0])+ (Pw[1]*Pw[1])));
  
  /* ********************* */

  return 0;
};

/* Sends servo angles to serial port */
int servoCommand(int fd, double *servo_angles)
{
  // check serial
  int writeval;

  // assign values
  double angleA = servo_angles[0] + servo.bias_A;
  double angleB = servo_angles[1] + servo.bias_B;
  double angleC = servo_angles[2] + servo.bias_C;

  int min = servo.min_angle;
  int max = servo.max_angle;

  // check if values are valid
  int condition = (angleA < max && angleA > min) &&
                  (angleB < max && angleB > min) &&
                  (angleC < max && angleC > min);

  if (condition != 1)
  {
    printf("ERROR: Servo angles out of bounds.\n");
    return -1;
  }

  // assemble command
  char command[50];
  sprintf(command, "C %.2f %.2f %.2f\n", angleA, angleB, angleC);

  // Flush serial port output
  tcflush(fd, TCOFLUSH);
  // send command
  writeval = write(fd, command, strlen(command));

  return 0;
}

/* Reads pixel coordinates from Pixycam. Also returns a flag whether an object
 * was detected or not */
int readFromPixy(int fd, int *flag, int *x, int *y)
{
  char buff[20];
  const char command[] = "P\n";
  int writeval;
  char *token;
  const char delim[] = " ";

  // Flush serial port input
  tcflush(fd, TCIFLUSH);
  tcflush(fd, TCOFLUSH);

  // Write command to pixy
  writeval = serialport_write(fd, command);
  usleep(10 * 1000);

  // Read until until no more bytes available
  // If not data is availabe, retry until success
  int readval = 0;
  while (readval != 1)
  {
    readval = serialport_read_until(fd, buff, sizeof(buff), '\n', 100);
  }

  // printf("readFromPixy: after read \n");
  // printf("writeval = %d, readval = %d", writeval,readval);

  // Catch read write errors
  if (!readval)
  {
    // printf("SERIAl READ FAILED with %d \n",readval);
    return -1;
  }

  // Add terminating 0 to make string
  buff[sizeof(buff) - 1] = 0;

  // extract values using strtok
  token = strtok(buff, delim);

  // Verify initial character
  if (token[0] != 'A')
  {
    // printf("SERIAL HEADER ERROR: %.20s\n",buff);
    return -1;
  }

  token = strtok(NULL, delim);
  *flag = atoi(token);
  token = strtok(NULL, delim);
  token = strtok(NULL, delim);
  *x = atoi(token);
  token = strtok(NULL, delim);
  token = strtok(NULL, delim);
  *y = atoi(token);

  return 1;
}
