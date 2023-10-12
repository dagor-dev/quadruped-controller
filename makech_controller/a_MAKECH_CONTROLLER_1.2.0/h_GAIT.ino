#define UNSIG_TIME_DIF(a,b) (a > b) ? a - b : 0;

void crawl(float ratio, float stancePeriod, float swingPeriod, float constantX, float constantY, float constantYaw, float constantPitch){
  static bool stoppingFlag = false;
  static float prev_feet_offset_y = r_state.foot_pos_offset_y;
  static float positionZ = r_state.height;
  static float positionX, positionY, yawAngle, pitchAngle;
  
  static int stepFlag_fr = 0;
  static int stepFlag_fl = 0;
  static int stepFlag_br = 0;
  static int stepFlag_bl = 0;
  static unsigned long prevStepMillis_fr = runTime;
  static unsigned long prevStepMillis_fl = runTime + ratio*(swingPeriod+stancePeriod);
  static unsigned long prevStepMillis_br = runTime;
  static unsigned long prevStepMillis_bl = runTime + ratio*(swingPeriod+stancePeriod);
  static int stepPeriod_fr = 0; //swingPeriod;
  static int stepPeriod_fl = 0; //swingPeriod;
  static int stepPeriod_br = 0; //swingPeriod;
  static int stepPeriod_bl = 0; //swingPeriod;

  static float fr_z = r_state.height;
  static float fl_z = r_state.height;
  static float br_z = r_state.height;
  static float bl_z = r_state.height;
  static float fr_x = 0;
  static float fl_x = 0;
  static float br_x = 0;
  static float bl_x = 0;
  static float fr_y = 0;
  static float fl_y = 0;
  static float br_y = 0;
  static float bl_y = 0;
  static float fr_yaw = 0;
  static float fl_yaw = 0;
  static float br_yaw = 0;
  static float bl_yaw = 0;
  
  // Z input
  positionZ = holdPositionZ(1.5);
  static float z_pushup = 0;                        // add some value to Z for the two legs that keep in contact with the ground to help keep the robot's
                                                    // torso at the same height. Only support from two legs will make the robot sink a little without this.
  
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
  
  if( abs(positionX) > 6 || abs(positionY) > 3 || abs(yawAngle) > 0.5 || abs(pitchAngle) > 0.5 || (prev_feet_offset_y != r_state.foot_pos_offset_y) ){
    // Front Right ----------------------------------------------------------------
    if (stepFlag_fr == 0 && runTime - prevStepMillis_fr > stancePeriod) {
      stepPeriod_fr = swingPeriod/3;

      fr_z = positionZ - gait.step_length_z;
    
      stepFlag_fr = 1;              
      prevStepMillis_fr = runTime;    
    }
    else if (stepFlag_fr == 1 && runTime - prevStepMillis_fr > (swingPeriod/3)) {
      stepPeriod_fr = swingPeriod/3;
    
      fr_x = -positionX;
      fr_y = -positionY + r_state.foot_pos_offset_y;
      fr_yaw = +yawAngle;
    
      stepFlag_fr = 2;         
    }
    else if (stepFlag_fr == 2 && runTime - prevStepMillis_fr > (2*swingPeriod/3)) {
      stepPeriod_fr = swingPeriod/3;

      fr_z = positionZ;
    
      stepFlag_fr = 3;            
    }
    else if (stepFlag_fr == 3 && runTime - prevStepMillis_fr > swingPeriod) {
      stepPeriod_fr = stancePeriod;

      fr_z = positionZ + z_pushup;
      fr_x = +positionX;
      fr_y = +positionY + r_state.foot_pos_offset_y;
      fr_yaw = -yawAngle;
    
      stepFlag_fr = 0;              
      prevStepMillis_fr = runTime;
    }
    
    // Back Left ----------------------------------------------------------------
    if (stepFlag_bl == 0 && runTime - prevStepMillis_bl > stancePeriod) {
      stepPeriod_bl = swingPeriod/3;

      bl_z = positionZ - gait.step_length_z;
    
      stepFlag_bl = 1;              
      prevStepMillis_bl = runTime;    
    }
    else if (stepFlag_bl == 1 && runTime - prevStepMillis_bl > (swingPeriod/3)) {
      stepPeriod_bl = swingPeriod/3;
    
      bl_x = +positionX;
      bl_y = +positionY + r_state.foot_pos_offset_y;
      bl_yaw = +yawAngle;
    
      stepFlag_bl = 2;              
    }
    else if (stepFlag_bl == 2 && runTime - prevStepMillis_bl > (2*swingPeriod/3)) {
      stepPeriod_fr = swingPeriod/3;

      bl_z = positionZ;
      
      stepFlag_bl = 3;              
    }
    else if (stepFlag_bl == 3 && runTime - prevStepMillis_bl > swingPeriod) {
      stepPeriod_bl = stancePeriod;

      bl_z = positionZ + z_pushup;
      bl_x = -positionX;
      bl_y = -positionY + r_state.foot_pos_offset_y;
      bl_yaw = -yawAngle;
      
      stepFlag_bl = 0;              
      prevStepMillis_bl = runTime;
    }

    // Front Left ----------------------------------------------------------------
    unsigned long desync_fl = UNSIG_TIME_DIF(runTime, prevStepMillis_fl);
    if (stepFlag_fl == 0 && desync_fl > stancePeriod) {
      stepPeriod_fl = swingPeriod/3;
      
      fl_z = positionZ - gait.step_length_z;
      
      stepFlag_fl = 1;              
      prevStepMillis_fl = runTime;
    }
    else if (stepFlag_fl == 1 && runTime - prevStepMillis_fl > (swingPeriod/3)) {
      stepPeriod_fl = swingPeriod/3;

      fl_x = -positionX;
      fl_y = +positionY + r_state.foot_pos_offset_y;
      fl_yaw = +yawAngle;
      
      stepFlag_fl = 2;              
    }
    else if (stepFlag_fl == 2 && runTime - prevStepMillis_fl > (2*swingPeriod/3)) {
      stepPeriod_fl = swingPeriod/3;

      fl_z = positionZ;
      
      stepFlag_fl = 3;              
    }
    else if (stepFlag_fl == 3 && runTime - prevStepMillis_fl > swingPeriod) {
      stepPeriod_fl = stancePeriod;

      fl_z = positionZ + z_pushup;
      fl_x = +positionX;
      fl_y = -positionY + r_state.foot_pos_offset_y;
      fl_yaw = -yawAngle;

      stepFlag_fl = 0;              
      prevStepMillis_fl = runTime;
    }

    // Back Right ----------------------------------------------------------------
    unsigned long desync_br = UNSIG_TIME_DIF(runTime, prevStepMillis_br);
    if (stepFlag_br == 0 && desync_br > stancePeriod) {
      stepPeriod_br = swingPeriod/3;

      br_z = positionZ - gait.step_length_z;
      
      stepFlag_br = 1;              
      prevStepMillis_br = runTime;
      
    }
    else if (stepFlag_br == 1 && runTime - prevStepMillis_br > (swingPeriod/3)) {
      stepPeriod_br = swingPeriod/3;

      br_x = +positionX;
      br_y = -positionY + r_state.foot_pos_offset_y;
      br_yaw = +yawAngle;
      
      stepFlag_br = 2;              
    }
    else if (stepFlag_br == 2 && runTime - prevStepMillis_br > (2*swingPeriod/3)) {
      stepPeriod_br = swingPeriod/3;
      
      br_z = positionZ;
      
      stepFlag_br = 3;              
    }
    else if (stepFlag_br == 3 && runTime - prevStepMillis_br > swingPeriod) {
      stepPeriod_br = stancePeriod;

      br_z = positionZ + z_pushup;
      br_x = -positionX;
      br_y = +positionY + r_state.foot_pos_offset_y;
      br_yaw = -yawAngle;
      
      stepFlag_br = 0;              
      prevStepMillis_br = runTime;
    }

  }
  else{
        stepPeriod_fr = 0;
        stepPeriod_fl = 0;
        stepPeriod_br = 0;
        stepPeriod_bl = 0;
        prevStepMillis_fr = runTime;
        prevStepMillis_fl = runTime + ratio*(swingPeriod+stancePeriod);
        prevStepMillis_br = runTime + ratio*(swingPeriod+stancePeriod);
        prevStepMillis_bl = runTime;
        stepFlag_fr = 0;
        stepFlag_fl = 0;
        stepFlag_br = 0;
        stepFlag_bl = 0; 
        fr_z = positionZ;
        fl_z = positionZ; 
        br_z = positionZ;
        bl_z = positionZ; 
        fr_x = 0;
        fl_x = 0;
        br_x = 0;
        bl_x = 0;
        fr_y = 0 + r_state.foot_pos_offset_y;
        fl_y = 0 + r_state.foot_pos_offset_y;
        br_y = 0 + r_state.foot_pos_offset_y;
        bl_y = 0 + r_state.foot_pos_offset_y;
        fr_yaw = 0;
        fl_yaw = 0;
        br_yaw = 0;
        bl_yaw = 0;
  }

  prev_feet_offset_y = r_state.foot_pos_offset_y;

  gaitKinematics (0, fr_x, -fr_y, fr_z, fr_yaw, pitchAngle, 0, stepPeriod_fr);   // front right leg
  gaitKinematics (1, fl_x, -fl_y, fl_z, fl_yaw, pitchAngle, 0, stepPeriod_fl);   // front left leg
  gaitKinematics (2, br_x, -br_y, br_z, br_yaw, pitchAngle, 0, stepPeriod_br);   // back right leg
  gaitKinematics (3, bl_x, -bl_y, bl_z, bl_yaw, pitchAngle, 0, stepPeriod_bl);   // back left leg
  

  sendCalculatedAngles();
}

/*
void trot(float trotPeriod, float constantX, float constantY, float constantYaw, float constantPitch){
  static bool stoppingFlag = false;
  static float prev_feet_offset_y = r_state.foot_pos_offset_y;
  static float positionZ = r_state.height;
  static float positionX, positionY, yawAngle, pitchAngle;
  
  static int stepFlag_1 = 0;
  static int stepFlag_2 = 0;
  static unsigned long prevStepMillis_1 = runTime;
  static unsigned long prevStepMillis_2 = runTime + trotPeriod;

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
  static float z_pushup = 0;                         // add some value to Z for the two legs that keep in contact with the ground to help keep the robot's
                                                      // torso at the same height. Only support from two legs will make the robot sink a little without this.
  
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
  
  if( abs(positionX) > 6 || abs(positionY) > 3 || abs(yawAngle) > 0.5 || abs(pitchAngle) > 0.5 || (prev_feet_offset_y != r_state.foot_pos_offset_y) ){
    if (stepFlag_1 == 0 && runTime - prevStepMillis_1 > trotPeriod) {
      legLength1 = positionZ - gait.step_length_z;
      fr_x = -positionX;
      bl_x = +positionX;
      fr_y = -positionY + r_state.foot_pos_offset_y;
      bl_y = +positionY + r_state.foot_pos_offset_y;
      fr_yaw = +yawAngle;
      bl_yaw = +yawAngle;
      
      stepFlag_1 = 1;              
      prevStepMillis_1 = runTime;
      
    }
    else if (stepFlag_1 == 1 && runTime - prevStepMillis_1 > trotPeriod/2) {
      legLength1 = positionZ;
      
      stepFlag_1 = 2;              
    }
    else if (stepFlag_1 == 2 && runTime - prevStepMillis_1 > trotPeriod) {
      legLength1 = positionZ + z_pushup;
      fr_x = +positionX;
      bl_x = -positionX;
      fr_y = +positionY + r_state.foot_pos_offset_y;
      bl_y = -positionY + r_state.foot_pos_offset_y;
      fr_yaw = -yawAngle;
      bl_yaw = -yawAngle;
      
      stepFlag_1 = 0;              
      prevStepMillis_1 = runTime;
    }
    else if (stepFlag_1 == 3 && runTime - prevStepMillis_1 > trotPeriod/2) {
      legLength1 = positionZ;

      stepFlag_1 = 0;              
    }

    
    unsigned long desync = UNSIG_TIME_DIF(runTime, prevStepMillis_2);
    if (stepFlag_2 == 0 && desync > trotPeriod) {
      legLength2 = positionZ - gait.step_length_z; 
      fl_x = -positionX;
      br_x = +positionX;
      fl_y = +positionY + r_state.foot_pos_offset_y;
      br_y = -positionY + r_state.foot_pos_offset_y;
      fl_yaw = +yawAngle;
      br_yaw = +yawAngle;
      
      stepFlag_2 = 1;              
      prevStepMillis_2 = runTime;
      
    }
    else if (stepFlag_2 == 1 && runTime - prevStepMillis_2 > trotPeriod/2) {
      legLength2 = positionZ; 
      
      stepFlag_2 = 2;              
    }
    else if (stepFlag_2 == 2 && runTime - prevStepMillis_2 > trotPeriod) {
      legLength2 = positionZ + z_pushup; 
      fl_x = +positionX;
      br_x = -positionX;
      fl_y = -positionY + r_state.foot_pos_offset_y;
      br_y = +positionY + r_state.foot_pos_offset_y;
      fl_yaw = -yawAngle;
      br_yaw = -yawAngle;
      
      stepFlag_2 = 0;              
      prevStepMillis_2 = runTime;
    }
    else if (stepFlag_2 == 3 && runTime - prevStepMillis_2 > trotPeriod/2) {
      legLength2 = positionZ; 
      
      stepFlag_2 = 0;              
    }
    
  }
  else{
        prevStepMillis_1 = runTime;
        prevStepMillis_2 = runTime + trotPeriod;
        stepFlag_1 = 0;  
        stepFlag_2 = 0;  
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

  prev_feet_offset_y = r_state.foot_pos_offset_y;

  gaitKinematics (0, fr_x, -fr_y, legLength1, fr_yaw, pitchAngle, 0, trotPeriod);   // front right leg
  gaitKinematics (1, fl_x, -fl_y, legLength2, fl_yaw, pitchAngle, 0, trotPeriod);   // front left leg
  gaitKinematics (2, br_x, -br_y, legLength2, br_yaw, pitchAngle, 0, trotPeriod);   // back right leg
  gaitKinematics (3, bl_x, -bl_y, legLength1, bl_yaw, pitchAngle, 0, trotPeriod);   // back left leg
  

  sendCalculatedAngles();
}
*/


void trot(int trotPeriod, float constantX, float constantY, float constantYaw, float constantPitch){
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
  static float z_pushup = 0;                         // add some value to Z for the two legs that keep in contact with the ground to help keep the robot's
                                                      // torso at the same height. Only support from two legs will make the robot sink a little without this.
  
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
    if (stoppingFlag && runTime - previousStepMillis > trotPeriod*0.8){
      stoppingFlag = false;
      if (stepFlag == 1) stepFlag = 2;
      else stepFlag = 0;
    }
    else if (stepFlag == 0 && runTime - previousStepMillis > trotPeriod) {
      legLength1 = positionZ - gait.step_length_z;
      legLength2 = positionZ + z_pushup; 
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
    else if (stepFlag == 1 && runTime - previousStepMillis > trotPeriod/2) {
      legLength1 = positionZ;
      legLength2 = positionZ; 

      stepFlag = 2;              
      //previousStepMillis = runTime;
    }
    else if (stepFlag == 2 && runTime - previousStepMillis > trotPeriod) {
      legLength1 = positionZ + z_pushup;
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
      
      stepFlag = 0;              
      previousStepMillis = runTime;
    }
    else if (stepFlag == 3 && runTime - previousStepMillis > trotPeriod/2) {
      legLength1 = positionZ;
      legLength2 = positionZ; 
      
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

  gaitKinematics (0, fr_x, -fr_y, legLength1, fr_yaw, pitchAngle, 0, trotPeriod);   // front right leg
  gaitKinematics (1, fl_x, -fl_y, legLength2, fl_yaw, pitchAngle, 0, trotPeriod);   // front left leg
  gaitKinematics (2, br_x, -br_y, legLength2, br_yaw, pitchAngle, 0, trotPeriod);   // back right leg
  gaitKinematics (3, bl_x, -bl_y, legLength1, bl_yaw, pitchAngle, 0, trotPeriod);   // back left leg
  

  sendCalculatedAngles();
}


/*
void crawl(float ratio, float stancePeriod, float swingPeriod, float constantX, float constantY, float constantYaw, float constantPitch){
  static bool stoppingFlag = false;
  static float prev_feet_offset_y = r_state.foot_pos_offset_y;
  static float positionZ = r_state.height;
  static float positionX, positionY, yawAngle, pitchAngle;
  
  static int stepFlag_fr = 0;
  static int stepFlag_fl = 0;
  static int stepFlag_br = 0;
  static int stepFlag_bl = 0;
  static unsigned long prevStepMillis_fr = runTime;
  static unsigned long prevStepMillis_fl = runTime + ratio*(swingPeriod+stancePeriod);
  static unsigned long prevStepMillis_br = runTime;
  static unsigned long prevStepMillis_bl = runTime + ratio*(swingPeriod+stancePeriod);
  static int stepPeriod_fr = stancePeriod;
  static int stepPeriod_fl = stancePeriod;
  static int stepPeriod_br = stancePeriod;
  static int stepPeriod_bl = stancePeriod;



  static float fr_z = r_state.height;
  static float fl_z = r_state.height;
  static float br_z = r_state.height;
  static float bl_z = r_state.height;
  static float fr_x = 0;
  static float fl_x = 0;
  static float br_x = 0;
  static float bl_x = 0;
  static float fr_y = 0;
  static float fl_y = 0;
  static float br_y = 0;
  static float bl_y = 0;
  static float fr_yaw = 0;
  static float fl_yaw = 0;
  static float br_yaw = 0;
  static float bl_yaw = 0;
  
  // Z input
  positionZ = holdPositionZ(1.5);
  static float z_pushup = 0;                        // add some value to Z for the two legs that keep in contact with the ground to help keep the robot's
                                                    // torso at the same height. Only support from two legs will make the robot sink a little without this.
  
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
  
  if( abs(positionX) > 6 || abs(positionY) > 3 || abs(yawAngle) > 0.5 || abs(pitchAngle) > 0.5 || (prev_feet_offset_y != r_state.foot_pos_offset_y) ){
    // Front Right ----------------------------------------------------------------
    if (stepFlag_fr == 0 && runTime - prevStepMillis_fr > stancePeriod) {
      stepPeriod_fr = swingPeriod;
      fr_z = positionZ - gait.step_length_z;
      fr_x = -positionX;
      fr_y = -positionY + r_state.foot_pos_offset_y;
      fr_yaw = +yawAngle;
      
      stepFlag_fr = 1;              
      prevStepMillis_fr = runTime;
      
    }
    else if (stepFlag_fr == 1 && runTime - prevStepMillis_fr > swingPeriod/2) {
      fr_z = positionZ;
      
      stepFlag_fr = 2;              
    }
    else if (stepFlag_fr == 2 && runTime - prevStepMillis_fr > swingPeriod) {
      stepPeriod_fr = stancePeriod;
      fr_z = positionZ + z_pushup;
      fr_x = +positionX;
      fr_y = +positionY + r_state.foot_pos_offset_y;
      fr_yaw = -yawAngle;
      
      stepFlag_fr = 0;              
      prevStepMillis_fr = runTime;
    }
    
    // Back Left ----------------------------------------------------------------
    if (stepFlag_bl == 0 && runTime - prevStepMillis_bl > stancePeriod) {
      stepPeriod_bl = swingPeriod;
      bl_z = positionZ - gait.step_length_z;
      bl_x = +positionX;
      bl_y = +positionY + r_state.foot_pos_offset_y;
      bl_yaw = +yawAngle;
      
      stepFlag_bl = 1;              
      prevStepMillis_bl = runTime;
      
    }
    else if (stepFlag_bl == 1 && runTime - prevStepMillis_bl > swingPeriod/2) {
      bl_z = positionZ;
      
      stepFlag_bl = 2;              
    }
    else if (stepFlag_bl == 2 && runTime - prevStepMillis_bl > swingPeriod) {
      stepPeriod_bl = stancePeriod;
      bl_z = positionZ + z_pushup;
      bl_x = -positionX;
      bl_y = -positionY + r_state.foot_pos_offset_y;
      bl_yaw = -yawAngle;
      
      stepFlag_bl = 0;              
      prevStepMillis_bl = runTime;
    }

    // Front Left ----------------------------------------------------------------
    unsigned long desync_fl = UNSIG_TIME_DIF(runTime, prevStepMillis_fl);
    if (stepFlag_fl == 0 && desync_fl > stancePeriod) {
      stepPeriod_fl = swingPeriod;
      fl_z = positionZ - gait.step_length_z;
      fl_x = -positionX;
      fl_y = +positionY + r_state.foot_pos_offset_y;
      fl_yaw = +yawAngle;

      stepFlag_fl = 1;              
      prevStepMillis_fl = runTime;
      
    }
    else if (stepFlag_fl == 1 && runTime - prevStepMillis_fl > swingPeriod/2) {
      fl_z = positionZ;
      
      stepFlag_fl = 2;              
    }
    else if (stepFlag_fl == 2 && runTime - prevStepMillis_fl > swingPeriod) {
      stepPeriod_fl = stancePeriod;
      fl_z = positionZ + z_pushup;
      fl_x = +positionX;
      fl_y = -positionY + r_state.foot_pos_offset_y;
      fl_yaw = -yawAngle;

      stepFlag_fl = 0;              
      prevStepMillis_fl = runTime;
    }

    // Back Right ----------------------------------------------------------------
    unsigned long desync_br = UNSIG_TIME_DIF(runTime, prevStepMillis_br);
    Serial.println(desync_br);
    if (stepFlag_br == 0 && desync_br > stancePeriod) {
      stepPeriod_br = swingPeriod;
      br_z = positionZ - gait.step_length_z;
      br_x = +positionX;
      br_y = -positionY + r_state.foot_pos_offset_y;
      br_yaw = +yawAngle;
      
      stepFlag_br = 1;              
      prevStepMillis_br = runTime;
      
    }
    else if (stepFlag_br == 1 && runTime - prevStepMillis_br > swingPeriod/2) {
      br_z = positionZ;
      
      stepFlag_br = 2;              
    }
    else if (stepFlag_br == 2 && runTime - prevStepMillis_br > swingPeriod) {
      stepPeriod_br = stancePeriod;
      br_z = positionZ + z_pushup;
      br_x = -positionX;
      br_y = +positionY + r_state.foot_pos_offset_y;
      br_yaw = -yawAngle;
      
      stepFlag_br = 0;              
      prevStepMillis_br = runTime;
    }

  }
  else{
        prevStepMillis_fr = runTime;
        prevStepMillis_fl = runTime + ratio*(swingPeriod+stancePeriod);
        prevStepMillis_br = runTime + ratio*(swingPeriod+stancePeriod);
        prevStepMillis_bl = runTime;
        stepFlag_fr = 0;
        stepFlag_fl = 0;
        stepFlag_br = 0;
        stepFlag_bl = 0; 
        fr_z = positionZ;
        fl_z = positionZ; 
        br_z = positionZ;
        bl_z = positionZ; 
        fr_x = 0;
        fl_x = 0;
        br_x = 0;
        bl_x = 0;
        fr_y = 0 + r_state.foot_pos_offset_y;
        fl_y = 0 + r_state.foot_pos_offset_y;
        br_y = 0 + r_state.foot_pos_offset_y;
        bl_y = 0 + r_state.foot_pos_offset_y;
        fr_yaw = 0;
        fl_yaw = 0;
        br_yaw = 0;
        bl_yaw = 0;
  }

  prev_feet_offset_y = r_state.foot_pos_offset_y;

  gaitKinematics (0, fr_x, -fr_y, fr_z, fr_yaw, pitchAngle, 0, stepPeriod_fr);   // front right leg
  gaitKinematics (1, fl_x, -fl_y, fl_z, fl_yaw, pitchAngle, 0, stepPeriod_fl);   // front left leg
  gaitKinematics (2, br_x, -br_y, br_z, br_yaw, pitchAngle, 0, stepPeriod_br);   // back right leg
  gaitKinematics (3, bl_x, -bl_y, bl_z, bl_yaw, pitchAngle, 0, stepPeriod_bl);   // back left leg
  

  sendCalculatedAngles();
}
*/
