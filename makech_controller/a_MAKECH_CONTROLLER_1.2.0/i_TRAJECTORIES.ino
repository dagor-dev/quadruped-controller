//####_DEMO Arrays_#####


//Jumping sequence
float zJump[] = { 240, 170, 100, 100, 310, 240, 240};


void jump(){
  int sp = 0;

  if(jumping == true){

      //z ( x ( zJump[sp], 70 ) );
    
      if(sp == 0){
        //sendData("MAP",25,HT1);
        //sendData("MAP",25,KT1);
      }
      else if(sp == 6){
        //sendData("MAP",3,HT1);
        //sendData("MAP",3,KT1);
      }
      //sendData("M",-jumpA[sp],HT1);
      //sendData("M",jumpB[sp],KT1);
      
      //sendData("M", legs[0].theta, HT1);
      //sendData("M", legs[0].phi,   KT1); 
      
      sp += 1;
    }

  if(sp == 7){
    sp = 0;
    jumping = false;
  }
}
