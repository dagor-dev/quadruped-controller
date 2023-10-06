void toggleStateMachine(){
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 1000){
    pressedPeriod = 0;
    pressed = false;
  }
  if(ps2x.button(PSB_R2) && !pressed){
    pressed = true;
    Serial.println("STATE MACHINE: Walking.");
    stateMachine = WALKING;
  }
  else if(ps2x.button(PSB_L2) && !pressed){
    pressed = true;
    Serial.println("STATE MACHINE: Kinematics Demo.");
    stateMachine = KINEMATICS_DEMO;
  }
}

void reset_esp_now(){
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 5000){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if(ps2x.button(PSB_SQUARE) && !pressed){
      Serial.println("[SQUARE] RESETTING ESP-NOW.");
      pressed = true;

      espNowDeinit();
      delay(2000);
      espNowInit();

      Serial.println("[SQUARE] DONE.");
  }
}

void homing(){
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 1000){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if(ps2x.button(PSB_CROSS) && !pressed){
      Serial.println("[X] SET POSITION AS HOME.");
      pressed = true;
      
      if (frl){
          sendData(0, "home", 0, "home", 0, "home", 0);
          sendData(0, "MLV5", 0, "MLV5", 0, "MLV5", 0);
      }

      if (fll){
          sendData(1, "home", 0, "home", 0, "home", 0);
          sendData(1, "MLV5", 0, "MLV5", 0, "MLV5", 0);
      }

      if (brl){
          sendData(2, "home", 0, "home", 0, "home", 0);
          sendData(2, "MLV5", 0, "MLV5", 0, "MLV5", 0);
      }

      if (bll){
          sendData(3, "home", 0, "home", 0, "home", 0);
          sendData(3, "MLV5", 0, "MLV5", 0, "MLV5", 0);
      }
  }
}

void doAFlip(){
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  static bool standDirection = false;
  static float positionX = 0;
  static float positionY = 0;

  pressedPeriod += timeDif;
  if (pressedPeriod >= 7000){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if(ps2x.button(PSB_TRIANGLE) && !pressed){
    pressed = true;

      r_state.foot_pos_offset_y = 0;
      r_state.foot_pos_offset_x = 0;

      Serial.println("[TRIANGLE] FLIPPING SIDES.");
      
      for (int positionZ = r_state.height; positionZ >= 50; positionZ--){
      //Calculate IK
        inverseKinematics(&legs[0], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[1], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[2], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[3], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(6);
      }
      
      if (ik.orientation == 1) ik.orientation = -1; 
      else if (ik.orientation == -1) ik.orientation = 1; 

      for (int positionZ = 0; positionZ <= r_state.height; positionZ++){
      //Calculate IK
        inverseKinematics(&legs[0], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[1], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[2], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[3], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(6);
      }      
    }    
}

void sideFlip(){
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  static bool standDirection = false;
  static float positionX = 0;
  static float positionY = 0;

  static int jump_starting_height = 90;
  static int long_side_leg_lenght = 290;
  static int short_side_leg_length = 140;
  static float length_increase = 24.0;
  static int land_height = 150;

  pressedPeriod += timeDif;
  if (pressedPeriod >= 4000){
    pressedPeriod = 0;
    pressed = false;
  }
  
#define JUMP_P_GAIN   "MAP1.25"
#define LAND_P_GAIN   "MAP0.3"
#define DEFAULT_P_GAIN "MAP1.0"

  if(ps2x.button(PSB_SQUARE) && !pressed){
    pressed = true;

      r_state.foot_pos_offset_y = 0;
      r_state.foot_pos_offset_x = 0;

      Serial.println("[SQUARE] Side Jump Flip.");
      
      for (int positionZ = r_state.height; positionZ >= jump_starting_height; positionZ--){
      //Calculate IK
        inverseKinematics(&legs[0], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[1], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[2], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[3], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(6);
      }
      
      for(float sp = jump_starting_height; sp <= long_side_leg_lenght; sp += length_increase){
        //sendData(legs[0].id, JUMP_P_GAIN, 0, JUMP_P_GAIN, 0, JUMP_P_GAIN, 0);
        //sendData(legs[2].id, JUMP_P_GAIN, 0, JUMP_P_GAIN, 0, JUMP_P_GAIN, 0);
        //delayMicroseconds(250);
        //sendData(legs[1].id, JUMP_P_GAIN, 0, JUMP_P_GAIN, 0, JUMP_P_GAIN, 0);
        //sendData(legs[3].id, JUMP_P_GAIN, 0, JUMP_P_GAIN, 0, JUMP_P_GAIN, 0);
        //delayMicroseconds(250);
        inverseKinematics(&legs[0], sp,  0,  0);
        inverseKinematics(&legs[2], sp,  0, 0);

        if( sp<short_side_leg_length ){
          inverseKinematics(&legs[1], sp, 0, 0);
          inverseKinematics(&legs[3], sp, 0, 0);
        }
        sendCalculatedAngles();
        delay(4);
      }

      
      //sendData(legs[0].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
      //sendData(legs[1].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
      //sendData(legs[2].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
      //sendData(legs[3].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);

      for(float sp = long_side_leg_lenght; sp >= 50; sp -= 5){
        //sendData(legs[0].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
        //sendData(legs[2].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
        //delayMicroseconds(250);
        //sendData(legs[1].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
        //sendData(legs[3].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
        //delayMicroseconds(250);
        z ( &legs[0], x(&legs[0], sp, 0) );
        z ( &legs[2], x(&legs[2], sp, 0) );

        if( sp>50 ){
          z ( &legs[1], x(&legs[1], sp, 0) );
          z ( &legs[3], x(&legs[3], sp, 0) );
        }
        sendCalculatedAngles();
        delay(4);

      }

      //sendData(legs[0].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
      //sendData(legs[1].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
      //sendData(legs[2].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);
      //sendData(legs[3].id, LAND_P_GAIN, 0, LAND_P_GAIN, 0, LAND_P_GAIN, 0);

      if (ik.orientation == 1) ik.orientation = -1; 
      else if (ik.orientation == -1) ik.orientation = 1; 

      for (int positionZ = 0; positionZ <= land_height; positionZ++){
      //Calculate IK
        inverseKinematics(&legs[0], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[1], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[2], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[3], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(4);
      }   

      for (int positionZ = land_height; positionZ <= r_state.height; positionZ++){
      //Calculate IK
        //sendData(legs[0].id, "MAP0.6", 0, "MAP0.8", 0, "MAP1.0", 0);
        //sendData(legs[2].id, "MAP0.6", 0, "MAP0.8", 0, "MAP1.0", 0);
        //delayMicroseconds(250);
        //sendData(legs[1].id, "MAP0.6", 0, "MAP0.8", 0, "MAP1.0", 0);
        //sendData(legs[3].id, "MAP0.6", 0, "MAP0.8", 0, "MAP1.0", 0);
        //delayMicroseconds(250);
        inverseKinematics(&legs[0], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[1], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[2], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[3], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(6);
      }    
      
    }    
}

