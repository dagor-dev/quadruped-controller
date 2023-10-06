//MY MAC ADDRESS: 24:6F:28:51:ED:A4

#include "PsxLib.h"
#include <Ramp.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_private/wifi.h>

Psx ps2x; // create PS2 Controller instance

// Select which legs are being used.
bool frl = true;            // Front Right Leg - ID -> 0
bool fll = true;            // Front Left  Leg - ID -> 1    
bool brl = true;            // Back  Right Leg - ID -> 2
bool bll = true;            // Back  Left  Leg - ID -> 3

enum State_Machine {
  KINEMATICS_DEMO,
  WALKING,
  JUMPING
};

/*
 * Leg structure
 * 
 * Contains the angular positions of the 3 actuators of the legs, each corresponding to an ID (0-3)
 * 
 */
typedef struct Leg{
  int id;
  float theta;          // Angular position of the knee [rads]
  float phi;            // Angular position of the shoulder [rads]
  float gamma;          // Angular position of the hip [rads]
} Leg;

Leg legs[] = { {.id = 0},
               {.id = 1},
               {.id = 2},
               {.id = 3}};


/*
 * CartesianCoordinates structure
 * 
 * Contains the (x,y,z) coordinates of the feet, each corresponding to an ID (0-3)
 * 
 */
typedef struct CartesianCoordinates{
  float id;
  float z;
  float x;
  float y;
} CartesianCoordinates;

CartesianCoordinates coordinates[] = { {.id = 0},
                                       {.id = 1},
                                       {.id = 2},
                                       {.id = 3}};


// Interpolate between positions for smoother motion
class Interpolation {  
public:
    rampFloat myRamp;
    bool interpolationFlag = false;
    float savedValue;    

    float go(float input, int duration) {

      if (input != savedValue) {   // check for new data
          interpolationFlag = false;
      }
      savedValue = input;          // bookmark the old value  
    
      if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
          myRamp.go(input, duration, QUADRATIC_INOUT, ONCEFORWARD);        // start interpolation (value to go to, duration)
          interpolationFlag = true;
      }
    
    //LINEAR
    //QUADRATIC_INOUT
    //CUBIC_INOUT

      float output = myRamp.update();               
      return output;
    }
};    // end of class

Interpolation interpFRX;        // interpolation objects front right leg
Interpolation interpFRY;
Interpolation interpFRZ;
Interpolation interpFRS;

Interpolation interpFLX;        // interpolation objects front left leg
Interpolation interpFLY;
Interpolation interpFLZ;
Interpolation interpFLS;

Interpolation interpBRX;        // interpolation objects back right leg
Interpolation interpBRY;
Interpolation interpBRZ;
Interpolation interpBRS;

Interpolation interpBLX;        // interpolation objects back left leg
Interpolation interpBLY;
Interpolation interpBLZ;
Interpolation interpBLS;

//#########_STATE MACHINE_########
int stateMachine = KINEMATICS_DEMO;
bool jumping = false;

//#####_TIME MANAGEMENT_#####
unsigned long runTime, prevT = 0, timeDif; 
unsigned long kinematicsPeriod;
unsigned long walkingPeriod;

//######_FUNCTION DECLARATION_######
void holdInverseKinematics(struct Robot_state *r_state = &r_state, struct IK_parameters *ik = &ik);
//void holdInverseKinematics(float z_change, float transl_x, float transl_y, float initZ = r_state.height, float initX = r_state.foot_pos_offset_x, float initY = r_state.foot_pos_offset_y);
void moveInverseKinematics(float constantZ, float constantX, float constantY);
void walking(int stepPeriod = gait.step_period, float constantX = gait.step_length_x, float constantY = gait.step_length_y, float constantYaw = gait.yaw_angle, float constantPitch = gait.pitch_angle);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);