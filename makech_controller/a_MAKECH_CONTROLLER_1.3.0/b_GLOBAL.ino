//MY MAC ADDRESS: 24:6F:28:51:ED:A4

#include "PsxLib.h"
#include <Ramp.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_private/wifi.h>

Psx ps2x; // create PS2 Controller instance

//##########################_LEG ID CONFIGURATION_##########################
// Assign the physical hardware ID (0-3) to each logical leg position.
// For example, if the leg with physical ID 2 is your Front Right leg,
// set LEG_ID_FRONT_RIGHT = 2.
const int LEG_ID_FRONT_RIGHT = 0;
const int LEG_ID_FRONT_LEFT  = 1;
const int LEG_ID_BACK_RIGHT  = 3;
const int LEG_ID_BACK_LEFT   = 2;

//##########################_LOGICAL LEG INDICES_##########################
// These constants represent the logical position of the legs in the arrays.
// They make the code more readable. DO NOT CHANGE THESE.
const int FRL = 0; // Front Right Leg
const int FLL = 1; // Front Left Leg
const int BRL = 2; // Back Right Leg
const int BLL = 3; // Back Left Leg


// Select which legs are being used.
bool frl = true;           // Front Right Leg
bool fll = true;           // Front Left  Leg
bool brl = true;           // Back  Right Leg
bool bll = true;           // Back  Left  Leg

enum State_Machine {
  KINEMATICS_DEMO,
  CRAWL,
  JUMPING,
  TROT,
  CRAWL_GEMINI,
  TROT_GEMINI
};

/*
 * Leg structure
 * * Contains the angular positions of the 3 actuators of the legs.
 * The array index corresponds to the logical position (FRL, FLL, BRL, BLL).
 * The 'id' field holds the physical hardware ID to be sent via ESP-NOW.
 * */
typedef struct Leg{
  int id;
  float theta;           // Angular position of the knee [rads]
  float phi;             // Angular position of the shoulder [rads]
  float gamma;           // Angular position of the hip [rads]
} Leg;

// The 'legs' array is indexed by the logical position (e.g., legs[FRL] is always the front-right leg).
// The '.id' field is initialized with your configurable physical hardware ID.
Leg legs[] = { {.id = LEG_ID_FRONT_RIGHT},
               {.id = LEG_ID_FRONT_LEFT},
               {.id = LEG_ID_BACK_RIGHT},
               {.id = LEG_ID_BACK_LEFT}};


/*
 * CartesianCoordinates structure
 * * Contains the (x,y,z) coordinates of the feet.
 * The array index corresponds to the logical position (FRL, FLL, BRL, BLL).
 * */
typedef struct CartesianCoordinates{
  float id; // This now corresponds to the logical index for easier debugging.
  float z;
  float x;
  float y;
} CartesianCoordinates;

// The 'coordinates' array is indexed by logical position.
CartesianCoordinates coordinates[] = { {.id = FRL},
                                       {.id = FLL},
                                       {.id = BRL},
                                       {.id = BLL}};


// Interpolate between positions for smoother motion
class Interpolation {  
public:
    rampFloat myRamp;
    bool interpolationFlag = false;
    float savedValue;    

    /*
     * Smoothly transitions a value from its current state to a target input value over a specified duration.
     * input:       The target value to ramp towards.
     * duration:    The time in milliseconds the transition should take.
     * inter_style: The style of interpolation (0 for QUADRATIC_INOUT, 1 for LINEAR).
     * returns:     The current interpolated value for this frame.
     */
    float go(float input, int duration, int inter_style) {

      if (input != savedValue) {   // check for new data
          interpolationFlag = false;
      }
      savedValue = input;          // bookmark the old value  
    
      if (interpolationFlag == 0) {                                       // only do it once until the flag is reset
        if (inter_style){
          myRamp.go(input, duration, LINEAR, ONCEFORWARD);          // start interpolation (value to go to, duration) QUADRATIC_INOUT
        }
        else{
          myRamp.go(input, duration, QUADRATIC_INOUT, ONCEFORWARD);       // start interpolation (value to go to, duration)  
        }
          interpolationFlag = true;
      }
    
    //LINEAR
    //QUADRATIC_INOUT
    //CUBIC_INOUT

      float output = myRamp.update();              
      return output;
    }
};    // end of class

Interpolation interpFRX;         // interpolation objects front right leg
Interpolation interpFRY;
Interpolation interpFRZ;
Interpolation interpFRS;

Interpolation interpFLX;         // interpolation objects front left leg
Interpolation interpFLY;
Interpolation interpFLZ;
Interpolation interpFLS;

Interpolation interpBRX;         // interpolation objects back right leg
Interpolation interpBRY;
Interpolation interpBRZ;
Interpolation interpBRS;

Interpolation interpBLX;         // interpolation objects back left leg
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
void moveInverseKinematics(float constantZ, float constantX, float constantY);
void crawl(float ratio = gait.sync_ratio, float stancePeriod = gait.stance_period, float swingPeriod = gait.swing_period, float constantX = gait.step_length_x, float constantY = gait.step_length_y, float constantYaw = gait.yaw_angle, float constantPitch = gait.pitch_angle);
void trot(float trotPeriod = gait.trot_period, float constantX = gait.step_length_x, float constantY = gait.step_length_y, float constantYaw = gait.yaw_angle, float constantPitch = gait.pitch_angle);
void crawling_gemini();
void trot_gemini();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
