#define UNSIG_TIME_DIF(a,b) (a > b) ? a - b : 0;

/*
 * Manages the crawling gait of the robot based on controller input.
 * This gait moves one leg at a time in a specific sequence for maximum stability.
 * ratio:         Sync ratio between diagonal leg pairs (not fully utilized in this specific crawl).
 * stancePeriod:  Duration in ms that a foot stays on the ground.
 * swingPeriod:   Duration in ms that a foot is in the air.
 * constantX:     Maximum step length in the X direction.
 * constantY:     Maximum step length in the Y direction.
 * constantYaw:   Maximum yaw (turning) rate.
 * constantPitch: Maximum pitch (tilting) rate.
 */
void crawl(float ratio, float stancePeriod, float swingPeriod, float constantX, float constantY, float constantYaw, float constantPitch){
  static float z_pushup = 8;                         // add some value to Z for the two legs that keep in contact with the ground to help keep the robot's
                                                     // torso at the same height. Only support from two legs will make the robot sink a little without this.
  static float anti_twist = 0;
  
  static bool motionFlag = false;
  static unsigned long stopTimer = 0;
  static float prev_feet_offset_y = r_state.foot_pos_offset_y;
  static float positionZ = r_state.height;
  static float positionX, positionY, yawAngle, pitchAngle;
  
  static int stepFlag_fr = 0;
  static int stepFlag_fl = 0;
  static int stepFlag_br = 0;
  static int stepFlag_bl = 0;
  static unsigned long prevStepMillis_fr = runTime;
  static unsigned long prevStepMillis_fl = runTime + ratio*(swingPeriod+stancePeriod);
  static unsigned long prevStepMillis_br = runTime + ratio*(swingPeriod+stancePeriod);
  static unsigned long prevStepMillis_bl = runTime;
  static int stepPeriod_fr = 0; // Start with 0 to avoid bug where z position interpolated from 0 to r_state.height making the robot jump
  static int stepPeriod_fl = 0;
  static int stepPeriod_br = 0;
  static int stepPeriod_bl = 0;

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
  
  // Y input
  float a2 = ps2x.analog(2);                           // Read left analog stick from left to right
  float newY = -((a2-128)/128) * constantY;           // Scale to a value from positive constantY to negative constantY
  positionY = 0.99*positionY + 0.01*newY;               // Apply complementary filter to smooth out the input
  
  // X input
  float a3 = ps2x.analog(3);                           // Read left analog stick from up to down
  float newX = ((a3-128)/128) * constantX;
  positionX = 0.99*positionX + 0.01*newX;
  
  // Yaw input
  float a0 = ps2x.analog(0);                           // Read right analog stick from left to right
  float newYaw = ((a0-128)/128) * constantYaw;
  yawAngle = 0.99*yawAngle + 0.01*newYaw;

  // Pitch input
  float a1 = ps2x.analog(1);                           // Read left analog stick from up to right
  float newPitch = ((a1-128)/128) * constantPitch;
  pitchAngle = 0.99*pitchAngle + 0.01*newPitch; 

  holdFeetOffsetY(1);

  if( abs(positionX) > 4 || abs(positionY) > 2 || abs(yawAngle) > 0.5 || abs(pitchAngle) > 0.5 || (prev_feet_offset_y != r_state.foot_pos_offset_y) ){
    motionFlag = true;
    stopTimer = runTime;
  }
  else{
    if(runTime - stopTimer > 1000){
      motionFlag = false;
    }
  }
  
  if( motionFlag || stepFlag_fr != 0 ){
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
      fr_y = +positionY + r_state.foot_pos_offset_y + anti_twist;
      fr_yaw = -yawAngle;
    
      stepFlag_fr = 0;              
      prevStepMillis_fr = runTime;
    }
  }
  else{
    stepPeriod_fr = 0;
    prevStepMillis_fr = runTime;
    stepFlag_fr = 0;
    fr_z = positionZ;
    fr_x = 0;
    fr_y = 0 + r_state.foot_pos_offset_y;
    fr_yaw = 0;
  }
    
  if( motionFlag || stepFlag_bl != 0 ){
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
      stepPeriod_bl = swingPeriod/3;

      bl_z = positionZ;
      
      stepFlag_bl = 3;              
    }
    else if (stepFlag_bl == 3 && runTime - prevStepMillis_bl > swingPeriod) {
      stepPeriod_bl = stancePeriod;

      bl_z = positionZ + z_pushup;
      bl_x = -positionX;
      bl_y = -positionY + r_state.foot_pos_offset_y + anti_twist;
      bl_yaw = -yawAngle;
      
      stepFlag_bl = 0;              
      prevStepMillis_bl = runTime;
    }
  }
  else{
    stepPeriod_bl = 0;
    prevStepMillis_bl = runTime;
    stepFlag_bl = 0; 
    bl_z = positionZ; 
    bl_x = 0;
    bl_y = 0 + r_state.foot_pos_offset_y;
    bl_yaw = 0;
  }

  unsigned long desync_fl = UNSIG_TIME_DIF(runTime, prevStepMillis_fl);
  if( motionFlag || stepFlag_fl != 0 ){
    // Front Left ----------------------------------------------------------------
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
      fl_y = -positionY + r_state.foot_pos_offset_y + anti_twist;
      fl_yaw = -yawAngle;

      stepFlag_fl = 0;              
      prevStepMillis_fl = runTime;
    }
  }
  else{
    stepPeriod_fl = 0;
    prevStepMillis_fl = runTime + ratio*(swingPeriod+stancePeriod);
    stepFlag_fl = 0;
    fl_z = positionZ; 
    fl_x = 0;
    fl_y = 0 + r_state.foot_pos_offset_y;
    fl_yaw = 0;
  }

  unsigned long desync_br = UNSIG_TIME_DIF(runTime, prevStepMillis_br);
  if( motionFlag || stepFlag_br != 0 ){
    // Back Right ----------------------------------------------------------------
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
      br_y = +positionY + r_state.foot_pos_offset_y + anti_twist;
      br_yaw = -yawAngle;
      
      stepFlag_br = 0;              
      prevStepMillis_br = runTime;
    }
  }
  else{
    stepPeriod_br = 0;
    prevStepMillis_br = runTime + ratio*(swingPeriod+stancePeriod);
    stepFlag_br = 0;
    br_z = positionZ;
    br_x = 0;
    br_y = 0 + r_state.foot_pos_offset_y;
    br_yaw = 0;
  }

  prev_feet_offset_y = r_state.foot_pos_offset_y;

  //gaitKinematics (FRL, fr_x, -fr_y, fr_z, fr_yaw, pitchAngle, 0, stepPeriod_fr, 1, 1);   // front right leg
  //gaitKinematics (FLL, fl_x, -fl_y, fl_z, fl_yaw, pitchAngle, 0, stepPeriod_fl, 1, 1);   // front left leg
  //gaitKinematics (BRL, br_x, -br_y, br_z, br_yaw, pitchAngle, 0, stepPeriod_br, 1, 1);   // back right leg
  //gaitKinematics (BLL, bl_x, -bl_y, bl_z, bl_yaw, pitchAngle, 0, stepPeriod_bl, 1, 1);   // back left leg

  gaitKinematics (FRL, fr_x, -fr_y, fr_z, fr_yaw, pitchAngle, 0, stepPeriod_fr, 1, (stepFlag_fr == 0) ? 1 : 0);
  gaitKinematics (FLL, fl_x, -fl_y, fl_z, fl_yaw, pitchAngle, 0, stepPeriod_fl, 1, (stepFlag_fl == 0) ? 1 : 0);
  gaitKinematics (BRL, br_x, -br_y, br_z, br_yaw, pitchAngle, 0, stepPeriod_br, 1, (stepFlag_br == 0) ? 1 : 0);
  gaitKinematics (BLL, bl_x, -bl_y, bl_z, bl_yaw, pitchAngle, 0, stepPeriod_bl, 1, (stepFlag_bl == 0) ? 1 : 0);

  sendCalculatedAngles();
}



/*
 * Manages the trotting gait of the robot based on controller input.
 * This gait moves diagonal pairs of legs simultaneously for faster movement.
 * trotPeriod:    Duration in ms for one full step cycle (swing + stance).
 * constantX:     Maximum step length in the X direction.
 * constantY:     Maximum step length in the Y direction.
 * constantYaw:   Maximum yaw (turning) rate.
 * constantPitch: Maximum pitch (tilting) rate.
 */
void trot(float trotPeriod, float constantX, float constantY, float constantYaw, float constantPitch){
  static float z_pushup = 3;                         // add some value to Z for the two legs that keep in contact with the ground to help keep the robot's
                                                     // torso at the same height. Only support from two legs will make the robot sink a little without this.
  
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
  
  // Y input
  float a2 = ps2x.analog(2);                           // Read left analog stick from left to right
  float newY = -((a2-128)/128) * constantY;           // Scale to a value from positive constantY to negative constantY
  positionY = 0.99*positionY + 0.01*newY;               // Apply complementary filter to smooth out the input
  
  // X input
  float a3 = ps2x.analog(3);                           // Read left analog stick from up to down
  float newX = ((a3-128)/128) * constantX;
  positionX = 0.99*positionX + 0.01*newX;
  
  // Yaw input
  float a0 = ps2x.analog(0);                           // Read right analog stick from left to right
  float newYaw = ((a0-128)/128) * constantYaw;
  yawAngle = 0.99*yawAngle + 0.01*newYaw;

  // Pitch input
  float a1 = ps2x.analog(1);                           // Read left analog stick from up to right
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

  // Diagonal pairs move together. Pair 1: FRL, BLL. Pair 2: FLL, BRL.
  //gaitKinematics (FRL, fr_x, -fr_y, legLength1, fr_yaw, pitchAngle, 0, trotPeriod, 0.5, 0);   // front right leg
  //gaitKinematics (FLL, fl_x, -fl_y, legLength2, fl_yaw, pitchAngle, 0, trotPeriod, 0.5, 0);   // front left leg
  //gaitKinematics (BRL, br_x, -br_y, legLength2, br_yaw, pitchAngle, 0, trotPeriod, 0.5, 0);   // back right leg
  //gaitKinematics (BLL, bl_x, -bl_y, legLength1, bl_yaw, pitchAngle, 0, trotPeriod, 0.5, 0);   // back left leg

  gaitKinematics(FRL, fr_x, -fr_y, legLength1, fr_yaw, pitchAngle, 0, trotPeriod, 0.5, (stepFlag_1 == 0) ? 1 : 0);
  gaitKinematics(FLL, fl_x, -fl_y, legLength2, fl_yaw, pitchAngle, 0, trotPeriod, 0.5, (stepFlag_2 == 0) ? 1 : 0);
  gaitKinematics(BRL, br_x, -br_y, legLength2, br_yaw, pitchAngle, 0, trotPeriod, 0.5, (stepFlag_2 == 0) ? 1 : 0);
  gaitKinematics(BLL, bl_x, -bl_y, legLength1, bl_yaw, pitchAngle, 0, trotPeriod, 0.5, (stepFlag_1 == 0) ? 1 : 0);
  

  sendCalculatedAngles();
}

/*
 * Manages the crawling gait of the robot based on controller input.
 * Diagonal pairs (FR+BL, FL+BR) step together, offset by ratio, with high duty factor.
 * ratio:         Sync ratio between diagonal leg pairs.
 * stancePeriod:  Duration in ms that a foot stays on the ground.
 * swingPeriod:   Duration in ms that a foot is in the air.
 * constantX:     Maximum step length in the X direction.
 * constantY:     Maximum step length in the Y direction.
 * constantYaw:   Maximum yaw (turning) rate.
 * constantPitch: Maximum pitch (tilting) rate.
 */
void improved_crawl(float ratio, float stancePeriod, float swingPeriod, float constantX, float constantY, float constantYaw, float constantPitch){
  const float retract = 0.85f;      // touchdown retraction: land slightly short (1.0 = off)
  const float min_step_z = 25.0f;   // minimum foot lift height [mm], floor for ground clearance at low speed
  static float z_pushup = 5;                         // extra stance extension so the torso doesn't sag.
                                                     // Smaller than the trot's value: more feet on the ground here.
  static float anti_twist = 0;

  static bool motionFlag = false;
  static unsigned long stopTimer = 0;
  static float prev_feet_offset_y = r_state.foot_pos_offset_y;
  static float positionZ = r_state.height;
  static float positionX, positionY, yawAngle, pitchAngle;

  static int stepFlag_fr = 0;
  static int stepFlag_fl = 0;
  static int stepFlag_br = 0;
  static int stepFlag_bl = 0;
  static unsigned long prevStepMillis_fr = runTime;
  static unsigned long prevStepMillis_fl = runTime + ratio*(swingPeriod+stancePeriod);
  static unsigned long prevStepMillis_br = runTime + ratio*(swingPeriod+stancePeriod);
  static unsigned long prevStepMillis_bl = runTime;
  static int stepPeriod_fr = 0; // Start with 0 to avoid bug where z position interpolated from 0 to r_state.height making the robot jump
  static int stepPeriod_fl = 0;
  static int stepPeriod_br = 0;
  static int stepPeriod_bl = 0;

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
  

  // Y input
  float a2 = ps2x.analog(2);                           // Read left analog stick from left to right
  float newY = -((a2-128)/128) * constantY;            // Scale to a value from positive constantY to negative constantY
  positionY = 0.99*positionY + 0.01*newY;              // Apply complementary filter to smooth out the input

  // X input
  float a3 = ps2x.analog(3);                           // Read left analog stick from up to down
  float newX = ((a3-128)/128) * constantX;
  positionX = 0.99*positionX + 0.01*newX;

  // Yaw input
  float a0 = ps2x.analog(0);                           // Read right analog stick from left to right
  float newYaw = ((a0-128)/128) * constantYaw;
  yawAngle = 0.99*yawAngle + 0.01*newYaw;

  // Pitch input
  float a1 = ps2x.analog(1);                           // Read left analog stick from up to right
  float newPitch = ((a1-128)/128) * constantPitch;
  pitchAngle = 0.99*pitchAngle + 0.01*newPitch;

  // Swing height scales with commanded speed
  float speed_frac = fabs(positionX) / constantX;
  if (fabs(positionY) / constantY   > speed_frac) speed_frac = fabs(positionY) / constantY;
  if (fabs(yawAngle)  / constantYaw > speed_frac) speed_frac = fabs(yawAngle)  / constantYaw;
  if (speed_frac > 1.0f) speed_frac = 1.0f;
  float step_z = min_step_z + speed_frac * (gait.step_length_z - min_step_z);

  holdFeetOffsetY(1);

  if( abs(positionX) > 4 || abs(positionY) > 2 || abs(yawAngle) > 0.5 || abs(pitchAngle) > 0.5 || (prev_feet_offset_y != r_state.foot_pos_offset_y) ){
    motionFlag = true;
    stopTimer = runTime;
  }
  else{
    if(runTime - stopTimer > 1000){
      motionFlag = false;
    }
  }

  if( motionFlag || stepFlag_fr != 0 ){
    // Front Right ----------------------------------------------------------------
    if (stepFlag_fr == 0 && runTime - prevStepMillis_fr > stancePeriod) {
      stepPeriod_fr = swingPeriod/3;

      fr_z = positionZ - step_z;

      stepFlag_fr = 1;
      prevStepMillis_fr = runTime;
    }
    else if (stepFlag_fr == 1 && runTime - prevStepMillis_fr > (swingPeriod/3)) {
      stepPeriod_fr = swingPeriod/3;

      fr_x = -retract * positionX;
      fr_y = -retract * positionY + r_state.foot_pos_offset_y;
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
      fr_y = +positionY + r_state.foot_pos_offset_y + anti_twist;
      fr_yaw = -yawAngle;

      stepFlag_fr = 0;
      prevStepMillis_fr = runTime;
    }
  }
  else{
    stepPeriod_fr = 0;
    prevStepMillis_fr = runTime;
    stepFlag_fr = 0;
    fr_z = positionZ;
    fr_x = 0;
    fr_y = 0 + r_state.foot_pos_offset_y;
    fr_yaw = 0;
  }

  if( motionFlag || stepFlag_bl != 0 ){
    // Back Left ----------------------------------------------------------------
    if (stepFlag_bl == 0 && runTime - prevStepMillis_bl > stancePeriod) {
      stepPeriod_bl = swingPeriod/3;

      bl_z = positionZ - step_z;

      stepFlag_bl = 1;
      prevStepMillis_bl = runTime;
    }
    else if (stepFlag_bl == 1 && runTime - prevStepMillis_bl > (swingPeriod/3)) {
      stepPeriod_bl = swingPeriod/3;

      bl_x = +retract * positionX;
      bl_y = +retract * positionY + r_state.foot_pos_offset_y;
      bl_yaw = +yawAngle;

      stepFlag_bl = 2;
    }
    else if (stepFlag_bl == 2 && runTime - prevStepMillis_bl > (2*swingPeriod/3)) {
      stepPeriod_bl = swingPeriod/3;

      bl_z = positionZ;

      stepFlag_bl = 3;
    }
    else if (stepFlag_bl == 3 && runTime - prevStepMillis_bl > swingPeriod) {
      stepPeriod_bl = stancePeriod;

      bl_z = positionZ + z_pushup;
      bl_x = -positionX;
      bl_y = -positionY + r_state.foot_pos_offset_y + anti_twist;
      bl_yaw = -yawAngle;

      stepFlag_bl = 0;
      prevStepMillis_bl = runTime;
    }
  }
  else{
    stepPeriod_bl = 0;
    prevStepMillis_bl = runTime;
    stepFlag_bl = 0;
    bl_z = positionZ;
    bl_x = 0;
    bl_y = 0 + r_state.foot_pos_offset_y;
    bl_yaw = 0;
  }

  unsigned long desync_fl = UNSIG_TIME_DIF(runTime, prevStepMillis_fl);
  if( motionFlag || stepFlag_fl != 0 ){
    // Front Left ----------------------------------------------------------------
    if (stepFlag_fl == 0 && desync_fl > stancePeriod) {
      stepPeriod_fl = swingPeriod/3;

      fl_z = positionZ - step_z;

      stepFlag_fl = 1;
      prevStepMillis_fl = runTime;
    }
    else if (stepFlag_fl == 1 && runTime - prevStepMillis_fl > (swingPeriod/3)) {
      stepPeriod_fl = swingPeriod/3;

      fl_x = -retract * positionX;
      fl_y = +retract * positionY + r_state.foot_pos_offset_y;
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
      fl_y = -positionY + r_state.foot_pos_offset_y + anti_twist;
      fl_yaw = -yawAngle;

      stepFlag_fl = 0;
      prevStepMillis_fl = runTime;
    }
  }
  else{
    stepPeriod_fl = 0;
    prevStepMillis_fl = runTime + ratio*(swingPeriod+stancePeriod);
    stepFlag_fl = 0;
    fl_z = positionZ;
    fl_x = 0;
    fl_y = 0 + r_state.foot_pos_offset_y;
    fl_yaw = 0;
  }

  unsigned long desync_br = UNSIG_TIME_DIF(runTime, prevStepMillis_br);
  if( motionFlag || stepFlag_br != 0 ){
    // Back Right ----------------------------------------------------------------
    if (stepFlag_br == 0 && desync_br > stancePeriod) {
      stepPeriod_br = swingPeriod/3;

      br_z = positionZ - step_z;

      stepFlag_br = 1;
      prevStepMillis_br = runTime;

    }
    else if (stepFlag_br == 1 && runTime - prevStepMillis_br > (swingPeriod/3)) {
      stepPeriod_br = swingPeriod/3;

      br_x = +retract * positionX;
      br_y = -retract * positionY + r_state.foot_pos_offset_y;
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
      br_y = +positionY + r_state.foot_pos_offset_y + anti_twist;
      br_yaw = -yawAngle;

      stepFlag_br = 0;
      prevStepMillis_br = runTime;
    }
  }
  else{
    stepPeriod_br = 0;
    prevStepMillis_br = runTime + ratio*(swingPeriod+stancePeriod);
    stepFlag_br = 0;
    br_z = positionZ;
    br_x = 0;
    br_y = 0 + r_state.foot_pos_offset_y;
    br_yaw = 0;
  }

  prev_feet_offset_y = r_state.foot_pos_offset_y;

  // Easing style matches the phase: LINEAR in stance (constant body velocity),
  // QUADRATIC in swing (soft pickup and putdown).
  gaitKinematics (FRL, fr_x, -fr_y, fr_z, fr_yaw, pitchAngle, 0, stepPeriod_fr, 1, (stepFlag_fr == 0) ? 1 : 0);   // front right leg
  gaitKinematics (FLL, fl_x, -fl_y, fl_z, fl_yaw, pitchAngle, 0, stepPeriod_fl, 1, (stepFlag_fl == 0) ? 1 : 0);   // front left leg
  gaitKinematics (BRL, br_x, -br_y, br_z, br_yaw, pitchAngle, 0, stepPeriod_br, 1, (stepFlag_br == 0) ? 1 : 0);   // back right leg
  gaitKinematics (BLL, bl_x, -bl_y, bl_z, bl_yaw, pitchAngle, 0, stepPeriod_bl, 1, (stepFlag_bl == 0) ? 1 : 0);   // back left leg

  sendCalculatedAngles();
}

/*
 * Manages the trotting gait of the robot based on controller input.
 * This gait moves diagonal pairs of legs simultaneously for faster movement.
 * trotPeriod:    Duration in ms of the stance phase. Swing is 0.8x this, so a
 *                brief 4-leg support overlap exists at each handover.
 * constantX:     Maximum step length in the X direction.
 * constantY:     Maximum step length in the Y direction.
 * constantYaw:   Maximum yaw (turning) rate.
 * constantPitch: Maximum pitch (tilting) rate.
 */
void improved_trot(float trotPeriod, float constantX, float constantY, float constantYaw, float constantPitch){
  float swingT = 0.8f * trotPeriod;       // swing shorter than stance -> 4-leg overlap window
  const float retract = 0.85f;            // touchdown retraction: land slightly short (1.0 = off)
  const float min_step_z = 25.0f;         // minimum foot lift height [mm], floor for ground clearance at low speed
  static float z_pushup = 8;              // extra stance extension so the torso doesn't sag on 2 legs.
                                          // Tune in +/-2 steps: too low = torso dips, too high = bouncing.

  static float prev_feet_offset_y = r_state.foot_pos_offset_y;
  static float positionZ = r_state.height;
  static float positionX, positionY, yawAngle, pitchAngle;

  static int stepFlag_1 = 0;
  static int stepFlag_2 = 0;
  static unsigned long prevStepMillis_1 = runTime;
  static unsigned long prevStepMillis_2 = runTime + 0.9f * trotPeriod;   // half of the new full cycle

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
  float a2 = ps2x.analog(2);                           // Read left analog stick from left to right
  float newY = -((a2-128)/128) * constantY;            // Scale to a value from positive constantY to negative constantY
  positionY = 0.99*positionY + 0.01*newY;              // Apply complementary filter to smooth out the input

  // X input
  float a3 = ps2x.analog(3);                           // Read left analog stick from up to down
  float newX = ((a3-128)/128) * constantX;
  positionX = 0.99*positionX + 0.01*newX;

  // Yaw input
  float a0 = ps2x.analog(0);                           // Read right analog stick from left to right
  float newYaw = ((a0-128)/128) * constantYaw;
  yawAngle = 0.99*yawAngle + 0.01*newYaw;

  // Pitch input
  float a1 = ps2x.analog(1);                           // Read left analog stick from up to right
  float newPitch = ((a1-128)/128) * constantPitch;
  pitchAngle = 0.99*pitchAngle + 0.01*newPitch;

  // Swing height scales with commanded speed: gentle lift when trotting slowly,
  // full gait.step_length_z lift at full stick.
  float speed_frac = fabs(positionX) / constantX;
  if (fabs(positionY) / constantY   > speed_frac) speed_frac = fabs(positionY) / constantY;
  if (fabs(yawAngle)  / constantYaw > speed_frac) speed_frac = fabs(yawAngle)  / constantYaw;
  if (speed_frac > 1.0f) speed_frac = 1.0f;
  float step_z = min_step_z + speed_frac * (gait.step_length_z - min_step_z);

  holdFeetOffsetY(1);

  if( abs(positionX) > 6 || abs(positionY) > 3 || abs(yawAngle) > 0.5 || abs(pitchAngle) > 0.5 || (prev_feet_offset_y != r_state.foot_pos_offset_y) ){

    // Pair 1: FRL + BLL ----------------------------------------------------------------
    if (stepFlag_1 == 0 && runTime - prevStepMillis_1 > trotPeriod) {
      legLength1 = positionZ - step_z;               // lift, height scaled by speed
      fr_x = -retract * positionX;                   // swing toward a slightly short touchdown
      bl_x = +retract * positionX;
      fr_y = -positionY + r_state.foot_pos_offset_y;
      bl_y = +positionY + r_state.foot_pos_offset_y;
      fr_yaw = +yawAngle;
      bl_yaw = +yawAngle;

      stepFlag_1 = 1;
      prevStepMillis_1 = runTime;
    }
    else if (stepFlag_1 == 1 && runTime - prevStepMillis_1 > swingT/2) {
      legLength1 = positionZ;                        // foot comes back down

      stepFlag_1 = 2;
    }
    else if (stepFlag_1 == 2 && runTime - prevStepMillis_1 > swingT) {
      legLength1 = positionZ + z_pushup;             // planted: push to keep torso height
      fr_x = +positionX;                             // full stride on the back-sweep
      bl_x = -positionX;
      fr_y = +positionY + r_state.foot_pos_offset_y;
      bl_y = -positionY + r_state.foot_pos_offset_y;
      fr_yaw = -yawAngle;
      bl_yaw = -yawAngle;

      stepFlag_1 = 0;
      prevStepMillis_1 = runTime;
    }

    // Pair 2: FLL + BRL ----------------------------------------------------------------
    unsigned long desync = UNSIG_TIME_DIF(runTime, prevStepMillis_2);
    if (stepFlag_2 == 0 && desync > trotPeriod) {
      legLength2 = positionZ - step_z;
      fl_x = -retract * positionX;
      br_x = +retract * positionX;
      fl_y = +positionY + r_state.foot_pos_offset_y;
      br_y = -positionY + r_state.foot_pos_offset_y;
      fl_yaw = +yawAngle;
      br_yaw = +yawAngle;

      stepFlag_2 = 1;
      prevStepMillis_2 = runTime;
    }
    else if (stepFlag_2 == 1 && runTime - prevStepMillis_2 > swingT/2) {
      legLength2 = positionZ;

      stepFlag_2 = 2;
    }
    else if (stepFlag_2 == 2 && runTime - prevStepMillis_2 > swingT) {
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

  }
  else{
        prevStepMillis_1 = runTime;
        prevStepMillis_2 = runTime + 0.9f * trotPeriod;
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

  // Interpolation gets the real time available for the current phase,
  // and the easing style matches the phase: LINEAR in stance (constant body
  // velocity), QUADRATIC in swing (soft liftoff and touchdown).
  unsigned long dur1 = (stepFlag_1 != 0) ? (unsigned long)swingT : (unsigned long)trotPeriod;
  unsigned long dur2 = (stepFlag_2 != 0) ? (unsigned long)swingT : (unsigned long)trotPeriod;

  // Diagonal pairs move together. Pair 1: FRL, BLL. Pair 2: FLL, BRL.
  gaitKinematics (FRL, fr_x, -fr_y, legLength1, fr_yaw, pitchAngle, 0, dur1, 0.5, (stepFlag_1 == 0) ? 1 : 0);   // front right leg
  gaitKinematics (FLL, fl_x, -fl_y, legLength2, fl_yaw, pitchAngle, 0, dur2, 0.5, (stepFlag_2 == 0) ? 1 : 0);   // front left leg
  gaitKinematics (BRL, br_x, -br_y, legLength2, br_yaw, pitchAngle, 0, dur2, 0.5, (stepFlag_2 == 0) ? 1 : 0);   // back right leg
  gaitKinematics (BLL, bl_x, -bl_y, legLength1, bl_yaw, pitchAngle, 0, dur1, 0.5, (stepFlag_1 == 0) ? 1 : 0);   // back left leg

  sendCalculatedAngles();

}