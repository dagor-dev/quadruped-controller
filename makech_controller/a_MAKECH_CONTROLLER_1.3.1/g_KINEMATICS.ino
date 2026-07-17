// Kinematicssss

float heightRestriction(float heightRes){
  if (heightRes > quad.femur_length + quad.tibia_length - 20) heightRes = quad.femur_length + quad.tibia_length - 20;
  else if (heightRes <= 20) heightRes = 20;
  return heightRes;
}

/*
 * Z Inverse Kinematics: Calculates the knee and hip angles required to achieve a specific leg extension (length).
 * leg:         A pointer to the Leg struct to modify.
 * leg_length:  The target distance from the hip joint to the foot.
 */
void z(struct Leg *leg, float leg_length){
  
  float thetaZ = ( (HALF_PI) - acos( (sq(quad.femur_length) + sq(leg_length) - sq(quad.tibia_length) ) / ( 2 * quad.femur_length * leg_length ) ) ) * quad.hipRR;
  
  float phiZ = ( acos( (sq(quad.femur_length) + sq(quad.tibia_length) - sq(leg_length)) / ( 2 * quad.femur_length * quad.tibia_length ) ) ) * quad.kneeRR;

  if ( !isnan(thetaZ) ){
    float *theta = &(*leg).theta;
    //Decide the sign of the calculated angle
    // it depends on how the actuator is physically wired and what direction it thinks is positive 
    // This logic checks the LOGICAL leg position (Front-Right and Back-Left are a diagonal pair).
    if( leg == &legs[FRL] || leg == &legs[BLL] ) *theta -= thetaZ;
    else if( leg == &legs[FLL] || leg == &legs[BRL] ) *theta += thetaZ;  
  } // Don't update calculated if it's NAN

  if ( !isnan(phiZ) ){
    float *phi = &(*leg).phi;
    if( leg == &legs[FRL] || leg == &legs[BLL] ) *phi = phiZ;
    else if( leg == &legs[FLL] || leg == &legs[BRL] ) *phi = -phiZ;
  } // Don't update calculated if it's NAN
}

/*
 * X Inverse Kinematics: Calculates the hip angle for forward/backward (X-axis) foot placement.
 * leg:    A pointer to the Leg struct to modify.
 * posZ:   The current vertical height of the foot.
 * posX:   The target forward/backward position of the foot.
 * returns: The new required leg length to achieve the X position, which is then passed to the z() function.
 */
float x(struct Leg *leg, float posZ, float posX){ 
  
  float extraTheta = ( atan( posX / posZ ) ); 
  float thetaX = extraTheta * quad.hipRR;
  float newLegLength = ( posZ / (cos(extraTheta)) );
  newLegLength = heightRestriction(abs(newLegLength));

  if ( !isnan(thetaX) ){
    float *theta = &(*leg).theta;
    // This logic checks the LOGICAL leg position (Front-Right and Back-Left are a diagonal pair).
    if( leg == &legs[FRL] || leg == &legs[BLL] ) *theta = thetaX;
    else if( leg == &legs[FLL] || leg == &legs[BRL] ) *theta = -thetaX;
  } // Don't update calculated if it's NAN

  return newLegLength;
}

/*
 * Y Inverse Kinematics: Calculates the shoulder angle for side-to-side (Y-axis) foot placement.
 * leg:    A pointer to the Leg struct to modify.
 * posZ:   The current vertical height of the foot.
 * posY:   The target side-to-side position of the foot.
 * returns: The new required leg length to achieve the Y position, which is then passed to the x() function.
 */
float y(struct Leg *leg, float posZ, float posY){
  
  float distY = quad.foot_hip_offset + posY;
  
  float gammaP = atan( distY / posZ );
  if (isnan(gammaP)) gammaP = HALF_PI;

  float hipHyp = distY / sin( gammaP );

  float lambda = asin ( quad.foot_hip_offset / hipHyp );

  float gammaY = ( ( - lambda ) + gammaP  ) * quad.shoulderRR;

  float newNewLegLength = quad.foot_hip_offset/tan(lambda);

  newNewLegLength = heightRestriction(abs(newNewLegLength));

  
  if ( !isnan(gammaY) ){
    float *gamma = &(*leg).gamma;
    *gamma = gammaY;
  } // Don't update calculated if it's NAN
  
  return newNewLegLength;
}

/*
 * Full Inverse Kinematics: Calculates all three motor angles (phi, theta, gamma) for a leg
 * to position its foot at a specific 3D coordinate.
 * leg:   A pointer to the Leg struct to modify.
 * pos_z: The target Z-coordinate (height) of the foot.
 * pos_x: The target X-coordinate (forward/backward) of the foot.
 * pos_y: The target Y-coordinate (left/right) of the foot.
 */
void inverseKinematics(struct Leg *leg, float pos_z, float pos_x, float pos_y){
  z(leg, x(leg, y(leg, pos_z, pos_y), pos_x));  
}


void moveInverseKinematics(float constantZ, float constantX, float constantY){
  static float positionZ, positionX, positionY;
  // L1 BUTTON PRESS -> Read left analog stick and command 1 actuator
  if( ps2x.button(PSB_L1) || ps2x.button(PSB_R1) ){
    float a0 = ps2x.analog(0);                  // Y input
    float adderY = -((a0-128)/128);             // Retain position
    positionY += adderY * constantY;            // Multiply change by a constant

    float a2 = ps2x.analog(2);                  // X input
    float adderX = ((a2-128)/128);
    positionX += adderX * constantX; 

    float a3 = ps2x.analog(3);                  // Z input
    float adderZ = -((a3-128)/128);
    positionZ += adderZ * constantZ;

    // Check for restrictions
    positionZ = heightRestriction(positionZ);

    //Calculate IK
    inverseKinematics(&legs[FRL], positionZ, positionX, positionY);
    inverseKinematics(&legs[FLL], positionZ, positionX, -positionY);
    inverseKinematics(&legs[BRL], positionZ, -positionX, positionY);
    inverseKinematics(&legs[BLL], positionZ, -positionX, -positionY);
    
    //sendCalculatedAngles();
    
  } 
}


/*
 * Calculates the effect of yaw (turning) rotation on a single foot's coordinates.
 * coord:     A pointer to the CartesianCoordinates struct for the foot.
 * yawAngle:  The desired yaw angle in radians.
 * positionX: The foot's current X position relative to the body center.
 * positionY: The foot's current Y position relative to the body center.
 */
void yaw(struct CartesianCoordinates *coord, float yawAngle, float positionX, float positionY){
  
  float posX, posY;
  
  // This logic depends on the LOGICAL position of the leg.
  // Using pointer comparison to identify the leg.
  if( coord == &coordinates[FRL]){
    posY = positionY + (quad.center_to_shoulder + quad.foot_hip_offset);
    posX = positionX + quad.center_to_hip;
  }
  else if( coord == &coordinates[FLL]){
    posY = positionY - (quad.center_to_shoulder + quad.foot_hip_offset);
    posX = positionX - quad.center_to_hip;
  }
  else if( coord == &coordinates[BRL]){
    posY = positionY - (quad.center_to_shoulder + quad.foot_hip_offset);
    posX = positionX - quad.center_to_hip;
  }
  else if( coord == &coordinates[BLL]){
    posY = positionY + (quad.center_to_shoulder + quad.foot_hip_offset);
    posX = positionX + quad.center_to_hip;
  }

  float currentFootAngle = atan(posY/posX);
  float radius = posY/ (sin(currentFootAngle) );

  float totalYawAngle = currentFootAngle + yawAngle;

  float newPosX = radius * cos(totalYawAngle);
  float newPosY = radius * sin(totalYawAngle);

  if( coord == &coordinates[FRL]){
    (*coord).y = newPosY - (quad.center_to_shoulder + quad.foot_hip_offset);
    (*coord).x = newPosX - quad.center_to_hip;
  }
  else if( coord == &coordinates[FLL]){
    (*coord).y = newPosY + (quad.center_to_shoulder + quad.foot_hip_offset);
    (*coord).x = newPosX + quad.center_to_hip;
  }
  else if( coord == &coordinates[BRL]){
    (*coord).y = newPosY + (quad.center_to_shoulder + quad.foot_hip_offset);
    (*coord).x = newPosX + quad.center_to_hip;
  }
  else if( coord == &coordinates[BLL]){
    (*coord).y = newPosY - (quad.center_to_shoulder + quad.foot_hip_offset);
    (*coord).x = newPosX - quad.center_to_hip;
  }

}


/*
 * Calculates the effect of pitch (tilting forward/backward) rotation on a single foot's coordinates.
 * coord:      A pointer to the CartesianCoordinates struct for the foot.
 * pitchAngle: The desired pitch angle in radians.
 * positionZ:  The foot's current Z position (height).
 * positionX:  The foot's current X position.
 */
void pitch(struct CartesianCoordinates *coord, float pitchAngle, float positionZ, float positionX){
  
  // Invert angle for back legs
  if (coord == &coordinates[BRL] || coord == &coordinates[BLL]) pitchAngle = -pitchAngle;

  float difZ = positionZ - ( quad.center_to_hip * sin(pitchAngle) );
  float difX = quad.center_to_hip - ( quad.center_to_hip * cos(pitchAngle) ) + positionX;

  float auxAngle = atan( difX / difZ );
  float virtualLeg = difZ / cos(auxAngle);

  float auxAngleTotal = auxAngle + pitchAngle;
  
  (*coord).z = cos(auxAngleTotal) * virtualLeg;

  (*coord).x = -sin(auxAngleTotal) * virtualLeg;
  
}


/*
 * Calculates the effect of roll (tilting side-to-side) rotation on a single foot's coordinates.
 * coord:     A pointer to the CartesianCoordinates struct for the foot.
 * rollAngle: The desired roll angle in radians.
 * positionZ: The foot's current Z position (height).
 * positionY: The foot's current Y position.
 */
void roll(struct CartesianCoordinates *coord, float rollAngle, float positionZ, float positionY){
  
  // Invert angle for left legs
  if (coord == &coordinates[FLL] || coord == &coordinates[BLL]) rollAngle = -rollAngle;

  float difZ = positionZ - ( quad.center_to_shoulder * sin(rollAngle) );
  float difY = quad.center_to_shoulder - ( quad.center_to_shoulder * cos(rollAngle) ) + quad.foot_hip_offset + positionY;

  float auxAngle = atan( difY / difZ );
  float virtualLeg = ( difZ / cos(auxAngle) );

  float auxAngleTotal = (auxAngle + rollAngle);
  
  (*coord).z = cos(auxAngleTotal) * virtualLeg;

  (*coord).y = (sin(auxAngleTotal) * virtualLeg) - quad.foot_hip_offset;

}


/*
 * Use arrows to change the robot's height.
 */
float holdPositionZ( float constantZ ){
  //static float positionZ = r_state.height;
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 10){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if( ps2x.button(PSB_PAD_UP) && !pressed ){
    r_state.height += constantZ;
    pressed = true;
  }
  else if( ps2x.button(PSB_PAD_DOWN) && !pressed ){
    r_state.height -= constantZ;
    pressed = true;
  }

  r_state.height = heightRestriction(r_state.height);

  return r_state.height;
}

void holdFeetOffsetY( float offsetY ){
  //static float positionZ = r_state.height;
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 10){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if( ps2x.button(PSB_PAD_LEFT) && !pressed ){
    r_state.foot_pos_offset_y -= offsetY;
    pressed = true;
  }
  else if( ps2x.button(PSB_PAD_RIGHT) && !pressed ){
    r_state.foot_pos_offset_y += offsetY;
    pressed = true;
  }
}

float holdYawAngle(float initYaw, float constantYaw){
  static float yawAngle = initYaw;
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 10){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if( ps2x.button(PSB_PAD_LEFT) && !pressed ){
    yawAngle += constantYaw;
    pressed = true;
  }
  else if( ps2x.button(PSB_PAD_RIGHT) && !pressed ){
    yawAngle -= constantYaw;
    pressed = true;
  }

  return yawAngle;
}


/*
 * Handles the "Kinematics Demo" mode, allowing direct control over the robot's body orientation and position.
 * Reads analog stick inputs to control translation (X, Y, Z) and rotation (pitch, roll, yaw) of the torso.
 * r_state: Pointer to the Robot_state struct, containing the current pose of the robot.
 * ik:      Pointer to the IK_parameters struct, containing movement limits.
 */
void holdInverseKinematics(struct Robot_state *r_state, struct IK_parameters *ik){
  static float positionZ =  (*r_state).height;
  static float positionX =  (*r_state).transl_x; 
  static float positionY =  (*r_state).transl_y;
  static float pitchAngle = (*r_state).pitch_angle;
  static float rollAngle =  (*r_state).roll_angle;
  static float yawAngle =   (*r_state).yaw_angle;
  static float z_change =   2.5;      // how many mm will the Z height of the robot change when pressing the arrows, it's basically the adjust resolution

  if( ps2x.button(PSB_L1) || ps2x.button(PSB_R1) ){
        
    // Use arrows for robot height when R1 and L1, else arrows control X/Y translation    
    if( ps2x.button(PSB_L1) && ps2x.button(PSB_R1) ){
      positionZ = holdPositionZ(z_change);     // Hold robot height
    }
    else{
      // X input
      int x_dir = 0;
      if (ps2x.button(PSB_PAD_UP)) x_dir = 1;
      else if (ps2x.button(PSB_PAD_DOWN)) x_dir = -1;
      bool down = ps2x.button(PSB_PAD_DOWN);
      float newX = x_dir * (*ik).max_x_movement;
      positionX = 0.96*positionX + 0.04*newX;
      (*r_state).transl_x = positionX;

      // Y input 
      int y_dir = 0;
      if (ps2x.button(PSB_PAD_LEFT))  y_dir = 1;
      else if (ps2x.button(PSB_PAD_RIGHT)) y_dir = -1;
      float newY = y_dir * (*ik).max_y_movement;             
      positionY = 0.96*positionY + 0.04*newY;
      (*r_state).transl_y = positionY;
    }
  
    // Z input
    float a3 = ps2x.analog(3);
    float newZ = (*r_state).height + ((a3-128)/128) * ( ((quad.femur_length+quad.tibia_length)/2)-20 );
    positionZ = 0.955*positionZ + 0.045*newZ;
    //(*r_state).height = positionZ;

    // Yaw input
    //yawAngle = holdYawAngle(0, 0.006);
    float a2 = ps2x.analog(2);
    float newYaw = -((a2-128)/128) * (*ik).max_yaw_movement;             
    yawAngle = 0.96*yawAngle + 0.04*newYaw;
    (*r_state).yaw_angle = yawAngle;
  
    // Pitch input
    float a1 = ps2x.analog(1);
    float newPitch = ((a1-128)/128) * (*ik).max_pitch_movement;
    pitchAngle = 0.96*pitchAngle + 0.04*newPitch;
    (*r_state).pitch_angle = pitchAngle;
  
    // Roll input
    float a0 = ps2x.analog(0);
    float newRoll = ((a0-128)/128) * (*ik).max_roll_movement;
    rollAngle = 0.96*rollAngle + 0.04*newRoll;
    (*r_state).roll_angle = rollAngle;
  
    yaw(&coordinates[FRL], yawAngle,  positionX,  positionY - (*r_state).foot_pos_offset_y);
    yaw(&coordinates[FLL], yawAngle,  positionX, -positionY - (*r_state).foot_pos_offset_y);
    yaw(&coordinates[BRL], yawAngle, -positionX,  positionY - (*r_state).foot_pos_offset_y);
    yaw(&coordinates[BLL], yawAngle, -positionX, -positionY - (*r_state).foot_pos_offset_y);
  
    pitch(&coordinates[FRL], pitchAngle, positionZ, coordinates[FRL].x);
    pitch(&coordinates[FLL], pitchAngle, positionZ, coordinates[FLL].x);
    pitch(&coordinates[BRL], pitchAngle, positionZ, coordinates[BRL].x);
    pitch(&coordinates[BLL], pitchAngle, positionZ, coordinates[BLL].x);
  
    roll(&coordinates[FRL], rollAngle, coordinates[FRL].z, coordinates[FRL].y);
    roll(&coordinates[FLL], rollAngle, coordinates[FLL].z, coordinates[FLL].y);
    roll(&coordinates[BRL], rollAngle, coordinates[BRL].z, coordinates[BRL].y);
    roll(&coordinates[BLL], rollAngle, coordinates[BLL].z, coordinates[BLL].y);
  
    //Calculate IK
    inverseKinematics(&legs[FRL], coordinates[FRL].z, coordinates[FRL].x, coordinates[FRL].y);
    inverseKinematics(&legs[FLL], coordinates[FLL].z, coordinates[FLL].x, coordinates[FLL].y);
    inverseKinematics(&legs[BRL], coordinates[BRL].z, coordinates[BRL].x, coordinates[BRL].y);
    inverseKinematics(&legs[BLL], coordinates[BLL].z, coordinates[BLL].x, coordinates[BLL].y);
  
    sendCalculatedAngles();  
  }
}



/*
 * Moves a single leg to a target position using interpolation and applies body rotations.
 * leg:         The logical index of the leg to move (FRL, FLL, BRL, BLL).
 * positionX:   The target X-coordinate for the foot [mm].
 * positionY:   The target Y-coordinate for the foot [mm].
 * positionZ:   The target Z-coordinate for the foot (height) [mm].
 * yawAngle:    The target yaw rotation for the foot [degrees].
 * pitchAngle:  The target pitch rotation for the body [degrees].
 * rollAngle:   The target roll rotation for the body [degrees].
 * dur:         The duration for the interpolation to take [ms].
 * mult_z:      A multiplier to adjust the duration of the Z-axis movement relative to others. Usefull when you want to make sure the foot is picked up quickly.
 * inter_style: The style of interpolation to use (e.g., 0 for QUADRATIC, 1 for LINEAR).
 */
void gaitKinematics(int leg, float positionX, float positionY, float positionZ, float yawAngle, float pitchAngle, float rollAngle, unsigned long dur, float mult_z, int inter_style){

  static float mult = 1.0;
  float multZ = mult_z; //0.5;   // all motions in Z are halfed because the foot has to go up AND down in the same time as the move moves in XY
  //Serial.println(multZ);

  if (leg == FRL) {       // front right
    positionZ = interpFRZ.go(positionZ,dur*multZ, inter_style);
    positionX = interpFRX.go(positionX,dur*mult , inter_style);
    positionY = interpFRY.go(positionY,dur*mult , inter_style);
    yawAngle  = interpFRS.go(yawAngle, dur*mult , inter_style);
  }
  
  else if (leg == FLL) {    // front left
    positionZ = interpFLZ.go(positionZ,dur*multZ, inter_style);
    positionX = interpFLX.go(positionX,dur*mult , inter_style);
    positionY = interpFLY.go(positionY,dur*mult , inter_style);
    yawAngle  = interpFLS.go(yawAngle, dur*mult , inter_style);              
  }

  else if (leg == BRL) {   // back right
    positionZ = interpBRZ.go(positionZ,dur*multZ, inter_style);
    positionX = interpBRX.go(positionX,dur*mult , inter_style);
    positionY = interpBRY.go(positionY,dur*mult , inter_style);
    yawAngle  = interpBRS.go(yawAngle, dur*mult , inter_style);
  }

  else if (leg == BLL) {    // back left
    positionZ = interpBLZ.go(positionZ,dur*multZ, inter_style);
    positionX = interpBLX.go(positionX,dur*mult , inter_style);
    positionY = interpBLY.go(positionY,dur*mult , inter_style);
    yawAngle  = interpBLS.go(yawAngle, dur*mult , inter_style);
  }

  
  yawAngle = yawAngle*PI/180;   //Passing the yawAngle to this function in Degrees because small floats don't work well with the interpolation.

  pitchAngle = pitchAngle*PI/180;

  //Calculate rotational axes
  yaw(&coordinates[leg], yawAngle,  positionX,  positionY);
  pitch(&coordinates[leg], pitchAngle, positionZ, coordinates[leg].x);
  roll(&coordinates[leg], rollAngle, coordinates[leg].z, coordinates[leg].y);

  //Calculate IK
  inverseKinematics(&legs[leg], coordinates[leg].z, coordinates[leg].x, coordinates[leg].y);
    
}