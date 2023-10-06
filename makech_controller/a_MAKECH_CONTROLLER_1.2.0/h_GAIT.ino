

void walking(int stepPeriod, float constantX, float constantY, float constantYaw, float constantPitch){
  static bool stoppingFlag = false;
  static float prev_feet_offset_y = r_state.foot_pos_offset_y;
  static float positionZ = r_state.height;
  static float positionX, positionY, yawAngle, pitchAngle;
  static int stepFlag = 0;
  static unsigned long previousStepMillis = runTime;
  static float legLength1 = r_state.height;
  static float legLength2 = r_state.height;
  static float fr_x = 0;
  static float fl_x = 0;
  static float bl_x = 0;
  static float br_x = 0;
  static float fr_y = 0;
  static float fl_y = 0;
  static float bl_y = 0;
  static float br_y = 0;
  static float fr_yaw = 0;
  static float fl_yaw = 0;
  static float bl_yaw = 0;
  static float br_yaw = 0;
  
  // Z input
  positionZ = holdPositionZ(1.5);
  
  // Y input
  float a2 = ps2x.analog(2);                          // Read left analog stick from left to right
  float newY = -((a2-128)/128) * constantY;           // Scale to a value from positive constantY to negative constantY
  positionY = 0.99*positionY + 0.01*newY;               // Apply complementary filter to smooth out the input
  
  // X input
  float a3 = ps2x.analog(3);                          // Read left analog stick from up to down
  float newX = ((a3-128)/128) * constantX;
  positionX = 0.99*positionX + 0.01*newX;
  
  // Yaw input
  float a0 = ps2x.analog(0);                          // Read right analog stick from left to right
  float newYaw = ((a0-128)/128) * constantYaw;
  yawAngle = 0.99*yawAngle + 0.01*newYaw;

  // Pitch input
  float a1 = ps2x.analog(1);                          // Read left analog stick from up to right
  float newPitch = ((a1-128)/128) * constantPitch;
  pitchAngle = 0.99*pitchAngle + 0.01*newPitch; 

  holdFeetOffsetY(1);
  
  //if( a0 != 128 || a1 != 128 || a2 != 128 || a3 != 128 || stoppingFlag || (prev_feet_offset_y != r_state.foot_pos_offset_y) ){
  if( abs(positionX) > 6 || abs(positionY) > 3 || abs(yawAngle) > 0.5 || abs(pitchAngle) > 0.5 || stoppingFlag || (prev_feet_offset_y != r_state.foot_pos_offset_y) ){
    if (stoppingFlag && runTime - previousStepMillis > stepPeriod*0.8){
      stoppingFlag = false;
      if (stepFlag == 1) stepFlag = 2;
      else stepFlag = 0;
    }
    else if (stepFlag == 0 && runTime - previousStepMillis > stepPeriod) {
      legLength1 = positionZ - gait.step_length_z;
      legLength2 = positionZ; 
      fr_x = -positionX;
      fl_x = +positionX;
      bl_x = -positionX;
      br_x = +positionX;
      fr_y = -positionY + r_state.foot_pos_offset_y;
      fl_y = -positionY + r_state.foot_pos_offset_y;
      bl_y = +positionY + r_state.foot_pos_offset_y;
      br_y = +positionY + r_state.foot_pos_offset_y;
      fr_yaw = +yawAngle;
      fl_yaw = -yawAngle;
      bl_yaw = -yawAngle;
      br_yaw = +yawAngle;
      
      stepFlag = 1;              
      previousStepMillis = runTime;
      
    }
    else if (stepFlag == 1 && runTime - previousStepMillis > stepPeriod/2) {
      legLength1 = positionZ;
      legLength2 = positionZ; 
      //fr_x = -positionX;
      //fl_x = +positionX;
      //bl_x = -positionX;
      //br_x = +positionX;
      //fr_y = -positionY + r_state.foot_pos_offset_y;
      //fl_y = -positionY + r_state.foot_pos_offset_y;
      //bl_y = +positionY + r_state.foot_pos_offset_y;
      //br_y = +positionY + r_state.foot_pos_offset_y;
      //fr_yaw = +yawAngle;
      //fl_yaw = -yawAngle;
      //bl_yaw = -yawAngle;
      //br_yaw = +yawAngle;
      
      stepFlag = 2;              
      //previousStepMillis = runTime;
    }
    else if (stepFlag == 2 && runTime - previousStepMillis > stepPeriod) {
      legLength1 = positionZ;
      legLength2 = positionZ - gait.step_length_z; 
      fr_x = +positionX;
      fl_x = -positionX;
      bl_x = +positionX;
      br_x = -positionX;
      fr_y = +positionY + r_state.foot_pos_offset_y;
      fl_y = +positionY + r_state.foot_pos_offset_y;
      bl_y = -positionY + r_state.foot_pos_offset_y;
      br_y = -positionY + r_state.foot_pos_offset_y;
      fr_yaw = -yawAngle;
      fl_yaw = +yawAngle;
      bl_yaw = +yawAngle;
      br_yaw = -yawAngle;
      
      stepFlag = 3;              
      previousStepMillis = runTime;
    }
    else if (stepFlag == 3 && runTime - previousStepMillis > stepPeriod/2) {
      legLength1 = positionZ;
      legLength2 = positionZ; 
      //fr_x = +positionX;
      //fl_x = -positionX;
      //bl_x = +positionX;
      //br_x = -positionX;
      //fr_y = +positionY + r_state.foot_pos_offset_y;
      //fl_y = +positionY + r_state.foot_pos_offset_y;
      //bl_y = -positionY + r_state.foot_pos_offset_y;
      //br_y = -positionY + r_state.foot_pos_offset_y;
      //fr_yaw = -yawAngle;
      //fl_yaw = +yawAngle;
      //bl_yaw = +yawAngle;
      //br_yaw = -yawAngle;
      
      stepFlag = 0;              
      //previousStepMillis = runTime;
    }
  }
  else{

      if ( stepFlag == 1 || stepFlag == 3 ) stoppingFlag = true;
      else {
        legLength1 = positionZ;
        legLength2 = positionZ; 
        fr_x = 0;
        fl_x = 0;
        bl_x = 0;
        br_x = 0;
        fr_y = 0 + r_state.foot_pos_offset_y;
        fl_y = 0 + r_state.foot_pos_offset_y;
        bl_y = 0 + r_state.foot_pos_offset_y;
        br_y = 0 + r_state.foot_pos_offset_y;
        fr_yaw = 0;
        fl_yaw = 0;
        bl_yaw = 0;
        br_yaw = 0;
      }

  }

  prev_feet_offset_y = r_state.foot_pos_offset_y;

  gaitKinematics (0, fr_x, -fr_y, legLength1, fr_yaw, pitchAngle, 0, stepPeriod);   // front right leg
  gaitKinematics (1, fl_x, -fl_y, legLength2, fl_yaw, pitchAngle, 0, stepPeriod);   // front left leg
  gaitKinematics (2, bl_x, -bl_y, legLength2, bl_yaw, pitchAngle, 0, stepPeriod);   // back left leg
  gaitKinematics (3, br_x, -br_y, legLength1, br_yaw, pitchAngle, 0, stepPeriod);   // back right leg

  sendCalculatedAngles();
}
