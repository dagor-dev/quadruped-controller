
void setup(){
  
  Serial.begin(115200);
  Serial.println("[CONTROLLER] FW 1.2.0");
  Serial.println("[CONTROLLER] INIT.");
  delay(600);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  //error = ps2x.config_gamepad(15, 4, 14, 12, true, true);
  ps2x.setupPins(26, 27, 25, 32, 50);

  delay(300);

  espNowInit();

  delay(700);
  Serial.println("[CONTROLLER] READY.");
}
