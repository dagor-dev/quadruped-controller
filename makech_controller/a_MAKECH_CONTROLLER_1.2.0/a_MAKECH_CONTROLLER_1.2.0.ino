/* 
 *  ESP32 based controller for Makech Quadruped robot through directional ESP-NOW.
 *      - Full body Kinematics and open-loop gait generation -
 *  
 *  David Gonzalez 2023
 *  
 */

//##########################_ROBOT PARAMETERS_##########################
typedef struct Robot{
  const float femur_length = 126.0;        // Length of the femur [mm]
  const float tibia_length = 148.0;        // Length of the tibia [mm]
  const float center_to_hip = 161;         // Distance from center of the robot to center of shoulder's axis of rotation [mm]
  const float center_to_shoulder = 50;     // Distance from center of the robot to center of hip's axis of ratiation [mm]
  const float foot_hip_offset = 85;        // Offset from hip axis of rotation to foot [mm]
  const float kneeRR = 11.25;              // Reduction ratio of the knee actuators
  const float hipRR = 10.0;                // Reduction ratio of the hip actuators
  const float shoulderRR = 10.0;           // Reduction ratio of the shoulder actuators
} Robot;

Robot quad;

/*
 *  Current state of the Robot
 *
 */
typedef struct Robot_state{
  float height = 210;                      // Actual height of CV (center of volume of the torso) of the Robot [mm]
  float transl_x = 0;                      // CV translation in X axis [mm]
  float transl_y = 0;                      // CV translation in y axis [mm]
  float pitch_angle = 0;                   // Pitch angle [rads]
  float roll_angle = 0;                    // Roll angle [rads]
  float yaw_angle = 0;                     // Yaw angle [rads]
  float foot_pos_offset_x = 0;             // Offset in the Y axis, move the feet closer or further apart [mm]
  float foot_pos_offset_y = 0;             // Offset in the X axis, move the feet closer or further apart [mm]
} Robot_state;

Robot_state r_state;

//##########################_GAIT PARAMETERS_##########################
typedef struct Gait_parameters{
  float step_length_x = 45;                // Length of the step in the X axis [mm]
  float step_length_y = 25;                // Length of the step in the Y axis [mm]
  float step_length_z = 70;                // Lenght the foot is picked up on the swing phase of the gait [mm]
  float yaw_angle = 6;                     // Degrees to turn in yaw during gait [degrees]
  float pitch_angle = 14;                  // Max pitch angle during gait [degrees]
  float trot_period = 2000;
  float stance_period = 2000;                 // Time each phase of the gait takes [ms]
  float swing_period = 100;
  float sync_ratio = 1.0;                  // 0 -> completely in sync, 1 completely out of sync
  const float update_freq = 250;           // [Hz]
} Gait_parameters;

Gait_parameters gait;


//##########################_INVERSE KINEMATICS DEMO PARAMETERS_##########################
typedef struct IK_parameters{
  int orientation = 1;                       // Leg orientation
  const float max_x_movement = 50;           // Max torso movement in the X axis [mm]
  const float max_y_movement = 50;           // Max torso movement in the Y axis [mm]
  const float max_yaw_movement = PI/9;       // Max torso movement in the Y axis [rads]
  const float max_pitch_movement = PI/9;     // Max torso movement in the Y axis [rads]
  const float max_roll_movement = PI/3.5;    // Max torso movement in the Y axis [rads]
  const float update_freq = 125;             // [Hz]
} IK_parameters;

IK_parameters ik;
