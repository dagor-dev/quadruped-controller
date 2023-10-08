

void loop() {
  unsigned long datos = ps2x.read();    // Never comment this line
  timeManagement();                     // Never comment this line either
  serialEvent();

  if(runTime > 150){
    homing();                 // Cross
    doAFlip();                // Triangle
    toggleStateMachine();     // R2 and L2
    //reset_esp_now();        // Square
    //sideFlip();               // Square
  }
  
  if(stateMachine == KINEMATICS_DEMO){
    kinematicsPeriod += timeDif;
    if(kinematicsPeriod >= (1000/ik.update_freq) ){
      kinematicsPeriod = 0;
      //moveInverseKinematics(2.51, 2.25, 2.01);
      holdInverseKinematics();
    }    
  }

  else if(stateMachine == WALKING){
    walkingPeriod += timeDif;
    if(walkingPeriod >= (1000/gait.update_freq) ){
      walkingPeriod = 0;
      //trot();
      crawl(); //Step section time [miliseconds], step distance in X, step distance in Y, rotation degrees in Yaw
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

void timeManagement(){
  //Time managment for DEMOs' movements
  runTime = millis();
  timeDif = runTime - prevT;
  prevT = runTime;
}

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
        z( &legs[0], x(&legs[0], inputString.substring(1).toFloat(), 0) );
        z( &legs[1], x(&legs[1], inputString.substring(1).toFloat(), 0) );
        sendCalculatedAngles();
        sendCalculatedAngles();        
      }
      else{
      
        if (frl){
          sendData(0, inputString, 0, inputString, 0, inputString, 0);
        }
  
        if (fll){
          sendData(1, inputString, 0, inputString, 0, inputString, 0);
        }

        if (brl){
          sendData(2, inputString, 0, inputString, 0, inputString, 0); 
        }

        if (bll){
          sendData(3, inputString, 0, inputString, 0, inputString, 0);
        }
      
      
       Serial.println(inputString);
      } // Send any command to the actuators
      inputString = "";
    }
  }
}
