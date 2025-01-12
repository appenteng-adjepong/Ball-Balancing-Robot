#include <stdio.h>
#include "util.h"

int main()
{
  //////////////////////////////
  /////// Initialization ///////
  //////////////////////////////

// Time information

time_t rawtime;
struct tm *info;
char buffer[80];
time (&rawtime);
info = localtime(&rawtime);
printf("Formatted date and time: | %s| \n", buffer);

  // Print Welcome Message
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#####################\n");
  printf("Ball and Plate System\n");
  printf("#####################\n");
  printf("\n");
  printf("Opening serial port...\n");

  // Initialize the serial port
  const char *port = "/dev/ttyUSB0"; // vm: "/dev/ttyUSB0", mac: "/dev/cu.SLAB_USBtoUART"
  int fd = serialport_init(port, 115200);
  if (fd == -1)
   {
       printf("Could not open the port.\n");
       printf(" - Is the Arduino IDE terminal opened?\n");
       printf(" - Is the device connected to the VM?\n");
      return -1;
   }

  // Initialize robot and check
  // if messages are received
  initBallBalancingRobot(fd);

  // Make sure that serial port is relaxed
   usleep(20 * 1000);

  // Parameter loading functions
  load_parameters();
  load_servo();

  //////////////////////////////
  //////// Task Selection //////
  //////////////////////////////
  int task_selection = 0;
  printf("Select Task: ");
  scanf("%d", &task_selection);

  int u_pixy, v_pixy, flag;
  double Xw, Yw;

  //////////////////////////////
  /////////// Task 1 ///////////
  //////////////////////////////

  if (task_selection == 1)
  {
    /* Test inverse kinematics via
    terminal */
    int user =1;

    // initalize variables:
    while(user){
    double plate_angles[] = {0, 0};
    double servo_angles[] = {0, 0, 0};

    /* ********************* */
    printf("\nEnter rotation angle about x(phi_x)= ");
    scanf("%lf", plate_angles);
    printf("\nEnter the rotation angle about y(tetha_y)= ");
    scanf("%lf",plate_angles + 1);

    int output = inverseKinematics(plate_angles, servo_angles);
    if(output==0){
      //servoCommand(fd,servo_angles);
      servoCommand(fd,servo_angles);

      // printing plate angles to the terminal if they are feasible 
      printf("\nPlate angles: phi_x = %.1f, tetha_y = %.1f", plate_angles[0],plate_angles[1]);
      printf("\nServo angles: alpha_A = %.1f, alpha_B = %.1f, alpha_C = %.1f", servo_angles[0],servo_angles[1],servo_angles[2]);

    }else{
      printf("\nPlate angles not valid");
    }

    printf("\nDp you wish to try wtih different plate angles? (Enter 1 for YES or 0 for NO)");
    scanf("%d",&user);
    
    
    /* ********************* */
  }

  //////////////////////////////
  /////////// Task 2 ///////////
  //////////////////////////////
  /*Test projection from the image frame to the world frame*/
  }
  if (task_selection == 2)
  {
      
    // initalize variables:
    int u_pixy, v_pixy, flag;
    double Xw, Yw;


    /* ********************* */
    //read u,v coordinates from pixycamera
    /* ********************* */
    readFromPixy(fd, &flag, &u_pixy, &v_pixy);

    // printing read pixel coordinates from pixyCam
    if (flag == 1){
      //obtain real world coordinates
      project2worldFrame(u_pixy, v_pixy, &Xw, &Yw);

      printf("\nPixy coordinates(pixels): u = %d, v = %d", u_pixy, v_pixy);
      printf("\nCalibrated coordinates: Xw = %.2f, Yw = %.2f\n", Xw, Yw);
    }

    else{
      printf("\nNo object detected\n");
    }
  }

  return 0;
}
