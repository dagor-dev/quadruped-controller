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
  static float z_pushup = 0;                         // add some value to Z for the two legs that keep in contact with the ground to help keep the robot's
                                                     // torso at the same height. Only support from two legs will make the robot sink a little without this.
  static float anti_twist = 0;
  
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
      stepPeriod_fr = swingPeriod/3;

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

  gaitKinematics (FRL, fr_x, -fr_y, fr_z, fr_yaw, pitchAngle, 0, stepPeriod_fr, 1, 1);   // front right leg
  gaitKinematics (FLL, fl_x, -fl_y, fl_z, fl_yaw, pitchAngle, 0, stepPeriod_fl, 1, 1);   // front left leg
  gaitKinematics (BRL, br_x, -br_y, br_z, br_yaw, pitchAngle, 0, stepPeriod_br, 1, 1);   // back right leg
  gaitKinematics (BLL, bl_x, -bl_y, bl_z, bl_yaw, pitchAngle, 0, stepPeriod_bl, 1, 1);   // back left leg

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
  gaitKinematics (FRL, fr_x, -fr_y, legLength1, fr_yaw, pitchAngle, 0, trotPeriod, 0.5, 0);   // front right leg
  gaitKinematics (FLL, fl_x, -fl_y, legLength2, fl_yaw, pitchAngle, 0, trotPeriod, 0.5, 0);   // front left leg
  gaitKinematics (BRL, br_x, -br_y, legLength2, br_yaw, pitchAngle, 0, trotPeriod, 0.5, 0);   // back right leg
  gaitKinematics (BLL, bl_x, -bl_y, legLength1, bl_yaw, pitchAngle, 0, trotPeriod, 0.5, 0);   // back left leg
  

  sendCalculatedAngles();
}


/*
 * Implements a stable wave gait using a centralized state machine. This approach
 * improves stability by explicitly shifting the robot's center of gravity.
 * The gait cycle is broken into 8 phases: 4 leg-swing phases and 4 body-shift phases.
 * This function reads controller input directly to determine movement direction and speed.
 */
void crawling_gemini() {
    // --- Static variables to maintain state between function calls ---
    static float gait_progress = 0; // Continuous gait cycle from 0.0 to 1.0
    static float positionX, positionY, yawAngle; // Smoothed controller inputs

    // --- Gait Configuration ---
    const int cycle_duration = 800; // Total time for one full gait cycle (ms)
    const float stance_duty_factor = 0.75; // 3 legs on the ground at all times
    const float step_height = gait.step_length_z;

    // Define the order in which legs will be lifted. Classic wave gait.
    const int leg_swing_order[4] = {FRL, BLL, FLL, BRL};
    const float leg_phase_offsets[4] = {0.0, 0.5, 0.25, 0.75};
    
    // --- Read and Smooth Controller Input ---
    float positionZ = holdPositionZ(1.5); // Allow height adjustment during crawl
    float newY = -((ps2x.analog(2) - 128) / 128.0) * gait.step_length_y;
    positionY = 0.98 * positionY + 0.02 * newY;
    float newX = ((ps2x.analog(3) - 128) / 128.0) * gait.step_length_x;
    positionX = 0.98 * positionX + 0.02 * newX;
    float newYaw = ((ps2x.analog(0) - 128) / 128.0) * gait.yaw_angle;
    yawAngle = 0.98 * yawAngle + 0.02 * newYaw;

    // --- Update Gait Progress ---
    if (abs(positionX) > 4 || abs(positionY) > 2 || abs(yawAngle) > 0.5) {
      gait_progress += (float)timeDif / cycle_duration;
      if (gait_progress > 1.0) gait_progress -= 1.0;
    }

    // --- Calculate Target Foot Positions for this frame ---
    float target_x[4], target_y[4], target_z[4];

    for (int i = 0; i < 4; i++) {
        float leg_progress = fmod(gait_progress + leg_phase_offsets[i], 1.0);
        
        if (leg_progress < stance_duty_factor) { // Leg is in STANCE phase
            float stance_progress = leg_progress / stance_duty_factor;
            target_x[i] = positionX * (0.5 - stance_progress);
            target_y[i] = positionY * (0.5 - stance_progress);
            target_z[i] = positionZ;
        } else { // Leg is in SWING phase
            float swing_progress = (leg_progress - stance_duty_factor) / (1.0 - stance_duty_factor);
            target_x[i] = positionX * (-0.5 + swing_progress);
            target_y[i] = positionY * (-0.5 + swing_progress);
            target_z[i] = positionZ + sin(swing_progress * PI) * step_height;
        }
    }

    // --- Send Commands to Legs ---
    gaitKinematics(FRL, target_x[FRL], -target_y[FRL], target_z[FRL], yawAngle, 0, 0, 50, 1, 1);
    gaitKinematics(FLL, target_x[FLL], -target_y[FLL], target_z[FLL], yawAngle, 0, 0, 50, 1, 1);
    gaitKinematics(BRL, target_x[BRL], -target_y[BRL], target_z[BRL], yawAngle, 0, 0, 50, 1, 1);
    gaitKinematics(BLL, target_x[BLL], -target_y[BLL], target_z[BLL], yawAngle, 0, 0, 50, 1, 1);

    sendCalculatedAngles();
}

/*
 * Implements a stable trotting gait using a centralized state machine. This approach
 * improves stability by explicitly shifting the robot's center of gravity before lifting each diagonal pair of legs.
 * The gait cycle is broken into 4 phases: 2 leg-swing phases and 2 body-shift phases.
 * This function reads controller input directly to determine movement direction and speed.
 */
void trot_gemini() {
    // --- Static variables to maintain state between function calls ---
    static float gait_progress = 0; // Continuous gait cycle from 0.0 to 1.0
    static float positionX, positionY, yawAngle; // Smoothed controller inputs

    // --- Gait Configuration ---
    const int cycle_duration = 400; // Total time for one full trot cycle (ms)
    const float step_height = gait.step_length_z;
    const float stance_duty_factor = 0.5; // 2 legs on the ground at all times

    // --- Read and Smooth Controller Input ---
    float positionZ = holdPositionZ(1.5); // Allow height adjustment
    float newY = -((ps2x.analog(2) - 128) / 128.0) * gait.step_length_y;
    positionY = 0.98 * positionY + 0.02 * newY;
    float newX = ((ps2x.analog(3) - 128) / 128.0) * gait.step_length_x;
    positionX = 0.98 * positionX + 0.02 * newX;
    float newYaw = ((ps2x.analog(0) - 128) / 128.0) * gait.yaw_angle;
    yawAngle = 0.98 * yawAngle + 0.02 * newYaw;

    // --- Update Gait Progress ---
    if (abs(positionX) > 4 || abs(positionY) > 2 || abs(yawAngle) > 0.5) {
      gait_progress += (float)timeDif / cycle_duration;
      if (gait_progress > 1.0) gait_progress -= 1.0;
    }

    // --- Calculate Target Foot Positions for this frame ---
    float target_x[4], target_y[4], target_z[4];
    
    // Diagonal Pair 1: FRL and BLL
    float progress1 = fmod(gait_progress, 1.0);
    // Diagonal Pair 2: FLL and BRL
    float progress2 = fmod(gait_progress + 0.5, 1.0);

    // Calculate positions for Pair 1
    if (progress1 < stance_duty_factor) { // Stance
        float stance_progress = progress1 / stance_duty_factor;
        target_x[FRL] = positionX * (0.5 - stance_progress);
        target_y[FRL] = positionY * (0.5 - stance_progress);
        target_z[FRL] = positionZ;
        target_x[BLL] = positionX * (0.5 - stance_progress);
        target_y[BLL] = positionY * (0.5 - stance_progress);
        target_z[BLL] = positionZ;
    } else { // Swing
        float swing_progress = (progress1 - stance_duty_factor) / (1.0 - stance_duty_factor);
        target_x[FRL] = positionX * (-0.5 + swing_progress);
        target_y[FRL] = positionY * (-0.5 + swing_progress);
        target_z[FRL] = positionZ + sin(swing_progress * PI) * step_height;
        target_x[BLL] = positionX * (-0.5 + swing_progress);
        target_y[BLL] = positionY * (-0.5 + swing_progress);
        target_z[BLL] = positionZ + sin(swing_progress * PI) * step_height;
    }

    // Calculate positions for Pair 2
    if (progress2 < stance_duty_factor) { // Stance
        float stance_progress = progress2 / stance_duty_factor;
        target_x[FLL] = positionX * (0.5 - stance_progress);
        target_y[FLL] = positionY * (0.5 - stance_progress);
        target_z[FLL] = positionZ;
        target_x[BRL] = positionX * (0.5 - stance_progress);
        target_y[BRL] = positionY * (0.5 - stance_progress);
        target_z[BRL] = positionZ;
    } else { // Swing
        float swing_progress = (progress2 - stance_duty_factor) / (1.0 - stance_duty_factor);
        target_x[FLL] = positionX * (-0.5 + swing_progress);
        target_y[FLL] = positionY * (-0.5 + swing_progress);
        target_z[FLL] = positionZ + sin(swing_progress * PI) * step_height;
        target_x[BRL] = positionX * (-0.5 + swing_progress);
        target_y[BRL] = positionY * (-0.5 + swing_progress);
        target_z[BRL] = positionZ + sin(swing_progress * PI) * step_height;
    }

    // --- Send Commands to Legs ---
    gaitKinematics(FRL, target_x[FRL], -target_y[FRL], target_z[FRL], yawAngle, 0, 0, 50, 1, 1);
    gaitKinematics(FLL, target_x[FLL], -target_y[FLL], target_z[FLL], yawAngle, 0, 0, 50, 1, 1);
    gaitKinematics(BRL, target_x[BRL], -target_y[BRL], target_z[BRL], yawAngle, 0, 0, 50, 1, 1);
    gaitKinematics(BLL, target_x[BLL], -target_y[BLL], target_z[BLL], yawAngle, 0, 0, 50, 1, 1);

    sendCalculatedAngles();
}