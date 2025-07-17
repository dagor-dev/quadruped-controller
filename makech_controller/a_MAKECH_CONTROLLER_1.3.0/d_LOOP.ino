/*
 * The main control loop of the program. It runs repeatedly.
 * It reads controller input, manages timing, and executes the current state
 * (e.g., KINEMATICS_DEMO, CRAWL, TROT) at the specified update frequency.
 */
void loop() {
  unsigned long datos = ps2x.read();   // Never comment this line
  timeManagement();                    // Never comment this line either
  serialEvent();

  if(runTime > 150){
    homing();              // Cross
    doAFlip();             // Triangle
    toggleStateMachine();    // R2 and L2
    //reset_esp_now();     // Square
    //sideFlip();          // Square
  }
  
  if(stateMachine == KINEMATICS_DEMO){
    kinematicsPeriod += timeDif;
    if(kinematicsPeriod >= (1000/ik.update_freq) ){
      kinematicsPeriod = 0;
      //moveInverseKinematics(2.51, 2.25, 2.01);
      holdInverseKinematics();
    }    
  }
  else if(stateMachine == CRAWL){
    walkingPeriod += timeDif;
    if(walkingPeriod >= (1000/gait.update_freq) ){
      walkingPeriod = 0;
      crawl(); //Step section time [miliseconds], step distance in X, step distance in Y, rotation degrees in Yaw
    }
  }
  else if(stateMachine == TROT){
    walkingPeriod += timeDif;
    if(walkingPeriod >= (1000/gait.update_freq) ){
      walkingPeriod = 0;
      //crawl(0.5, 100, 100, 45, 25, 6, 10);
      trot(gait.trot_period, 55, 25, 8, 10);
    }
  }
  else if(stateMachine == CRAWL_GEMINI){
    walkingPeriod += timeDif;
    if(walkingPeriod >= (1000/gait.update_freq) ){
      walkingPeriod = 0;
      crawling_gemini();
    }
  }
  else if(stateMachine == TROT_GEMINI){
    walkingPeriod += timeDif;
    if(walkingPeriod >= (1000/gait.update_freq) ){
      walkingPeriod = 0;
      trot_gemini();
    }
  }


  /*
  jumpPeriod += timeDif;
  if(jumpPeriod >= jumpInterval){
    jumpPeriod = 0;
    jump();
  }
  */
}

/*
 * Manages the timing for the main loop to ensure consistent update rates.
 * Calculates the time difference ('timeDif') since the last loop execution.
 */
void timeManagement(){
  //Time managment for DEMOs' movements
  runTime = millis();
  timeDif = runTime - prevT;
  prevT = runTime;
}

/*
 * Handles incoming data from the serial port for debugging and manual control.
 * Allows sending direct commands to the motors.
 */
void serialEvent() {
  // a string to hold incoming data
  static String inputString;
  while(Serial.available()){
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    inputString += inChar;
    // end of user input
    if (inChar == '\n') {
      if(inputString.charAt(0) == 'T'){
       //sendData("M", inputString.substring(1).toFloat(), HT1);
       //sendData("M", inputString.substring(1).toFloat(), KT1);
       //Serial.println(inputString);
      }
      else if(inputString.charAt(0) == 'I'){
        //jumpInterval = inputString.substring(1).toFloat();
      }
      else if(inputString.charAt(0) == 'Z'){
        z( &legs[FRL], x(&legs[FRL], inputString.substring(1).toFloat(), 0) );
        z( &legs[FLL], x(&legs[FLL], inputString.substring(1).toFloat(), 0) );
        sendCalculatedAngles();
        sendCalculatedAngles();      
      }
      else{
      
        if (frl){
          sendData(legs[FRL].id, inputString, 0, inputString, 0, inputString, 0);
        }
  
        if (fll){
          sendData(legs[FLL].id, inputString, 0, inputString, 0, inputString, 0);
        }

        if (brl){
          sendData(legs[BRL].id, inputString, 0, inputString, 0, inputString, 0); 
        }

        if (bll){
          sendData(legs[BLL].id, inputString, 0, inputString, 0, inputString, 0);
        }
      
      
        Serial.println(inputString);
      } // Send any command to the actuators
      inputString = "";
    }
  }
}