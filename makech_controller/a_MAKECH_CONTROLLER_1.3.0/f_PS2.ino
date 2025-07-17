/*
 * Toggles the robot's current behavior (state machine) based on PS2 controller input.
 * L2: Switches to KINEMATICS_DEMO mode.
 * R2: Switches to CRAWL mode.
 * R2 + R1: Switches to TROT mode.
 * L2 + R1: Switches to the new CRAWL_GEMINI mode.
 * R2 + L1: Switches to the new TROT_GEMINI mode.
 */
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
    if(ps2x.button(PSB_R1)){
      pressed = true;
      Serial.println("STATE MACHINE: Trotting.");
      stateMachine = TROT;
    } else if (ps2x.button(PSB_L1)) {
      Serial.println("STATE MACHINE: Gemini Trot.");
      stateMachine = TROT_GEMINI;
    }
    else{
      Serial.println("STATE MACHINE: Crawling.");
      stateMachine = CRAWL;
    }
  }
  else if(ps2x.button(PSB_L2) && !pressed){
    pressed = true;
    if(ps2x.button(PSB_R1)){
      Serial.println("STATE MACHINE: Gemini Crawl.");
      stateMachine = CRAWL_GEMINI;
    } else {
      Serial.println("STATE MACHINE: Kinematics Demo.");
      stateMachine = KINEMATICS_DEMO;
    }
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

/*
 * Sends a "home" command to all motor controllers when the Cross button is pressed.
 * This is used to set the zero position for all actuators.
 */
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
          sendData(legs[FRL].id, "home", 0, "home", 0, "home", 0);
          sendData(legs[FRL].id, "MLV3", 0, "MLV3", 0, "MLV3", 0);
          sendData(legs[FRL].id, "MLU10", 0, "MLU10", 0, "MLU10", 0);
          sendData(legs[FRL].id, "MQP1", 0, "MQP1", 0, "MQP1", 0);
      }

      if (fll){
          sendData(legs[FLL].id, "home", 0, "home", 0, "home", 0);
          sendData(legs[FLL].id, "MLV3", 0, "MLV3", 0, "MLV3", 0);
          sendData(legs[FLL].id, "MLU10", 0, "MLU10", 0, "MLU10", 0);
          sendData(legs[FLL].id, "MQP1", 0, "MQP1", 0, "MQP1", 0);
      }

      if (brl){
          sendData(legs[BRL].id, "home", 0, "home", 0, "home", 0);
          sendData(legs[BRL].id, "MLV3", 0, "MLV3", 0, "MLV3", 0);
          sendData(legs[BRL].id, "MLU10", 0, "MLU10", 0, "MLU10", 0);
          sendData(legs[BRL].id, "MQP1", 0, "MQP1", 0, "MQP1", 0);
      }

      if (bll){
          sendData(legs[BLL].id, "home", 0, "home", 0, "home", 0);
          sendData(legs[BLL].id, "MLV3", 0, "MLV3", 0, "MLV3", 0);
          sendData(legs[BLL].id, "MLU10", 0, "MLU10", 0, "MLU10", 0);
          sendData(legs[BLL].id, "MQP1", 0, "MQP1", 0, "MQP1", 0);
      }
  }
}

/*
 * Executes a "flip" maneuver when the Triangle button is pressed.
 * The robot lowers itself, inverts its leg orientation, and stands back up,
 * effectively flipping its orientation 180 degrees.
 */
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
        inverseKinematics(&legs[FRL], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[FLL], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BRL], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BLL], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(6);
      }
      
      if (ik.orientation == 1) ik.orientation = -1; 
      else if (ik.orientation == -1) ik.orientation = 1; 

      for (int positionZ = 0; positionZ <= r_state.height; positionZ++){
      //Calculate IK
        inverseKinematics(&legs[FRL], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[FLL], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BRL], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BLL], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(6);
      }      
    }    
}

/*
 * Executes a dynamic side flip jump maneuver when the Square button is pressed.
 * This is a complex, pre-programmed sequence of movements.
 */
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
        inverseKinematics(&legs[FRL], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[FLL], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BRL], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BLL], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(6);
      }
      
      for(float sp = jump_starting_height; sp <= long_side_leg_lenght; sp += length_increase){
        inverseKinematics(&legs[FRL], sp,  0,  0);
        inverseKinematics(&legs[BRL], sp,  0, 0);

        if( sp<short_side_leg_length ){
          inverseKinematics(&legs[FLL], sp, 0, 0);
          inverseKinematics(&legs[BLL], sp, 0, 0);
        }
        sendCalculatedAngles();
        delay(4);
      }

      
      for(float sp = long_side_leg_lenght; sp >= 50; sp -= 5){
        z ( &legs[FRL], x(&legs[FRL], sp, 0) );
        z ( &legs[BRL], x(&legs[BRL], sp, 0) );

        if( sp>50 ){
          z ( &legs[FLL], x(&legs[FLL], sp, 0) );
          z ( &legs[BLL], x(&legs[BLL], sp, 0) );
        }
        sendCalculatedAngles();
        delay(4);

      }

      if (ik.orientation == 1) ik.orientation = -1; 
      else if (ik.orientation == -1) ik.orientation = 1; 

      for (int positionZ = 0; positionZ <= land_height; positionZ++){
      //Calculate IK
        inverseKinematics(&legs[FRL], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[FLL], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BRL], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BLL], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(4);
      }  

      for (int positionZ = land_height; positionZ <= r_state.height; positionZ++){
      //Calculate IK
        inverseKinematics(&legs[FRL], positionZ,  r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[FLL], positionZ,  r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BRL], positionZ, -r_state.foot_pos_offset_x,  r_state.foot_pos_offset_y);
        inverseKinematics(&legs[BLL], positionZ, -r_state.foot_pos_offset_x, -r_state.foot_pos_offset_y);

        sendCalculatedAngles();
        delay(6);
      }      
      
    }    
}