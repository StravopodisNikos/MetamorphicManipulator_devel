
/*   *********************************************************************************************
 *   Executes tests for Metamorphic Manipulator with 1 Nema34 Stepper + 3 Dynamixels
 *   using custom-made functions for Driving Steppers and Dynamixels for Ovidius Mani-
 *   pulator. Dynamixel Uno + Dynamixel Shield are used!
 *   API is provided@:https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/#library-api
 *   Custom Dxl Library is coded using this API and provided@:
 *   ********************************************************************************************* 
 */

/* 
 *  Author: Stravopodis Nikos, PhD Candidate, University of the Aegean
 *  mail: n.stravopodis@syros.aegean.gr
 * 
 *  December 2020
 */

// Include Motor Configuration files
#include <definitions.h>                            // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include <motorIDs.h>                               // Includes motor IDs as set using Dynamixel Wizard
#include <contolTableItems_LimitValues.h>           // Limit values for control table controlTableItems_LimitValues
#include <StepperMotorSettings.h>                   // Includes Stepper Motor/Driver pin StepperMotorSettings
// Libraries for Robot Motors Driving
#include "DynamixelProPlusOvidiusShield.h"
//#include "CustomStepperMetamorphicManipulator.h"
//  Used Libraries
#include "Arduino.h"                                // Main Arduino library
#include <DynamixelShield.h>                        // ROBOTIS library          
#include <Dynamixel2Arduino.h>

// Configure Serial Port Communication
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

// Namespace of used classes
using namespace std;
using namespace ControlTableItem;

/*
 * Configure IDs for Dynamixels & NEMA34 Stepper Motor -> MUST align with the IDs configured at EEPROM of each module!
 */
uint8_t dxl_id[] = {DXL1_ID, DXL2_ID, DXL3_ID};
int id_count;
int dxl_motors_used = 3;
int Nema34_id = STP1_ID;

// Declare extern variables defined in DynamixelProPlusMetamorphicManipulator
//bool dxl_addparam_result;                     // crashes serial monitor gamwthnpanagiatou
uint8_t dxl_error; 
uint32_t  dxl_present_position[sizeof(dxl_id)];
uint32_t  dxl_goal_position[sizeof(dxl_id)];
int32_t   dxl_prof_vel[sizeof(dxl_id)];
int32_t   dxl_prof_accel[sizeof(dxl_id)];
uint8_t   dxl_moving[sizeof(dxl_id)];

//int32_t   dxl_vel_limit[sizeof(dxl_id)];
//int32_t dxl_accel_limit[sizeof(dxl_id)];

int32_t DxlInitPos;                           // Variables used to initialilize DxlTrapzProfParams_forP2P[]
int32_t DxlGoalPosition;
int32_t DxlVmax;
int32_t DxlAmax;
const int TrapzProfParams_size = 4;           // Also applied to StpTrapzProfParams

/*
 * Configure Stepper/Dynamixel settings
 */
float desiredConfiguration[nDoF];
byte desiredAnatomy[nPseudoJoints];
//dxlVelLimit dxl_vel_limit = {2000, 1500, 2000};
//dxlAccelLimit dxl_accel_limit = {900, 900, 900};

// Declare extern variables defined in CustomStepperMetamorphicManipulator
bool return_function_state = false;
bool segmentExistsTrapz;
byte currentDirStatus;
double currentAbsPos_double;
unsigned long currentAbsPos;
double VelocityLimitStp;
double AccelerationLimitStp;
double MaxPosLimitStp;

double StpInitPos;                          // Variables used to initialilize StpTrapzProfParams[]
double StpGoalPosition;
double StpVmax;
double StpAmax;

const int StpTrapzProfParams_size = 4;                  // This array stores the same data as DxlTrapzProfParams_forP2P for Dxls
double StpTrapzProfParams[StpTrapzProfParams_size];

const int storage_array_for_TrajAssignedDuration_size = 5;  // This array stores properties for stepper trajectory equations for Melchiorri Theory Book
double storage_array_for_TrajAssignedDuration[storage_array_for_TrajAssignedDuration_size];

const int storage_array_for_PROFILE_STEPS_size = 4;     // This array is for Stepper Progile Trajectory Execution
unsigned long storage_array_for_PROFILE_STEPS[storage_array_for_PROFILE_STEPS_size];


/*
 * USER SERIAL INPUT VARIABLES
 */
int error_code_received;
String user_input_string;
int user_input_int;
const char * meta_exec = "MET";
const char * act_exec  = "ACT";
const char * home_exec  = "HOME";
const char * p2p_exec  = "P2P";
const char * meta_cont = "R";
const char * meta_exit = "E";
const char * YES = "Y";
const char * NO = "N";
int nl_char_shifting   = 10;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int dataNumber = 0;             // new for this version
bool MENU_EXIT;

// Create object for handling motors
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
DynamixelProPlusOvidiusShield meta_dxl(dxl_id); // Object of custom class to access custom functions for Ovidius manipulator specific 
//CustomStepperMetamorphicManipulator stp(STP1_ID, STEP_Pin, DIR_Pin, ENABLE_Pin, LED_Pin, HALL_SWITCH_PIN1, HALL_SWITCH_PIN2, HALL_SWITCH_PIN3, LOCK_Pin, SPR1, GEAR_FACTOR_PLANETARY, FT_CLOSED_LOOP);

using namespace ControlTableItem;

void setup() {
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  DEBUG_SERIAL.println("[  INFO  ] SETUP START [  SUCCESS ]");
}

void loop() {
  // PING DYNAMIXELS
  return_function_state = meta_dxl.pingDynamixels(dxl_id, sizeof(dxl_id),&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[  INFO  ] PING DYNAMIXELS [  SUCCESS ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ] PING DYNAMIXELS [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }

  
  //SET TORQUE ON
  return_function_state = meta_dxl.setDynamixelsTorqueON(dxl_id, sizeof(dxl_id), dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[  INFO  ] TORQUE ON DYNAMIXELS [  SUCCESS ]");
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ] TORQUE ON DYNAMIXELS  [  FAILED ]");
  }

  delay(5000);

  //SET TORQUE OFF
  return_function_state = meta_dxl.setDynamixelsTorqueOFF(dxl_id, sizeof(dxl_id), dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[  INFO  ] TORQUE OFF DYNAMIXELS [  SUCCESS ]");
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ] TORQUE ON DYNAMIXELS  [  FAILED ]");
  }
  
}
