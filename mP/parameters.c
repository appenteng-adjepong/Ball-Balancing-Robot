#include "util.h"

// Allocate structs
parameter_struct bbs;
servo_struct servo;

/*Parameters related to robot hardware */
int load_parameters()
{
  // System Dimensions
  // All lengths in [mm]
  // Change this if you wan't to change plate height:
  bbs.plate_height = 100;
  // DON'T CHANGE THESE:
  bbs.R_plate_joint = 113.24;
  bbs.R_base_servo = 102.92;
  bbs.l1 = 42.0; // short arms
  bbs.l2 = 100.0;
  bbs.first_link_angle_bias = 4.6; // deg
  bbs.diameter_plate = 250.0;
  bbs.base_angles[0] = 0.5 * M_PI;          // rad
  bbs.base_angles[1] = (7.0 * M_PI) / 6.0;  // rad
  bbs.base_angles[2] = (11.0 * M_PI) / 6.0; // rad
  bbs.ball_radius = 20;

  // Camera Calibration Parameters:
  // **These are dummy parameters and need to be replaced with your calibration**:
  // When pictures are taken in PixyMon, their resolution varies. This factor
  // accounts for this scaling, such that the bbs can be copied directly
  bbs.calibration_image_scale = 3.56;
  bbs.focal_length = 339.7691;
  bbs.radial_distortion_coeff[0] = -0.2117;
  bbs.radial_distortion_coeff[1] = 0.0422;
  bbs.distortion_center[0] = 587;
  bbs.distortion_center[1] = 452;

  // Adjust these if you experience a constant offset in your coordinates the world frame
  bbs.cam_offset[0] = 6.5;
  bbs.cam_offset[1] = -8.7; // 7
  bbs.cam_offset[2] = 28.1;

  return 1;
}

/*Parameters related to the servo KST 05725MG. */
int load_servo()
{
  // Set angle bounds to avoid damage
  servo.min_angle = -50;
  servo.max_angle = 70;

  // Calibration of the mounting offset. Sum of bias should be as close to 0 as possible.
  // **These are dummy parameters and need to be replaced with your calibration**:

  servo.bias_A = 0;
  servo.bias_B = 0;
  servo.bias_C = 0;

  return 1;
}
