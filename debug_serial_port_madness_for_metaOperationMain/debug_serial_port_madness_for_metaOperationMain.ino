
/*   *********************************************************************************************
 *   Debugs main file for Ovidius Manipulator Operation
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
#include <StepperMotorSettings.h>   

//  Used Libraries
#include "Arduino.h"                                // Main Arduino library
#include <DynamixelSDK.h>                           // Dynamixel Software Development kit/ C++ 
#include "OpenCR.h"                                 // Controller Library

// Integrate C++
#include <vector>
#include <fstream>
#include "stdlib.h"
#include "stdio.h"

using namespace std;

// Libraries for Robot Motors Driving
#include "DynamixelProPlusMetamorphicManipulator.h"
#include "CustomStepperMetamorphicManipulator.h"

/*
 * Configure IDs for Dynamixels & NEMA34 Stepper Motor -> MUST align with the IDs configured at EEPROM of each module!
 */
uint8_t dxl_id[] = {DXL1_ID, DXL2_ID, DXL3_ID};
int id_count;
int dxl_motors_used = 3;
int Nema34_id = 1;

// Declare extern variables defined in DynamixelProPlusMetamorphicManipulator
int dxl_comm_result = COMM_TX_FAIL;
//bool dxl_addparam_result;           // 
uint8_t dxl_error; 
//uint16_t dxl_model_number[3];
uint32_t  dxl_present_position[sizeof(dxl_id)];
uint32_t  dxl_goal_position[sizeof(dxl_id)];
int32_t   dxl_prof_vel[sizeof(dxl_id)];
int32_t   dxl_prof_accel[sizeof(dxl_id)];
uint8_t   dxl_moving[sizeof(dxl_id)];
//uint8_t param_torque_enable;
int32_t DxlInitPos;                           // Variables used to initialilize DxlTrapzProfParams_forP2P[]
int32_t DxlGoalPosition;
int32_t DxlVmax;
int32_t DxlAmax;
const int TrapzProfParams_size = 4;           // Also applied to StpTrapzProfParams
typeDxlTrapzProfParams_forP2P DxlTrapzProfParams_forP2P[TrapzProfParams_size];

/*
 * Configure Stepper/Dynamixel settings
 */
float desiredConfiguration[nDoF];
byte desiredAnatomy[nPseudoJoints];
dxlVelLimit dxl_vel_limit = {2000, 1500, 2000};
dxlAccelLimit dxl_accel_limit = {900, 900, 900};

// Declare extern variables defined in CustomStepperMetamorphicManipulator
bool return_function_state = false;
bool segmentExistsTrapz;
vector<double> TrajAssignedDuration;
vector<unsigned long> PROFILE_STEPS;
vector<double> vector_for_trajectoryVelocity; 
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
const int dxl_baudrate = BAUDRATE;
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
//DynamixelProPlusMetamorphicManipulator dxl(dxl_id);
CustomStepperMetamorphicManipulator stp(STP1_ID, STEP_Pin, DIR_Pin, ENABLE_Pin, LED_Pin, HALL_SWITCH_PIN1, HALL_SWITCH_PIN2, HALL_SWITCH_PIN3, LOCK_Pin, SPR1, GEAR_FACTOR_PLANETARY, FT_CLOSED_LOOP);

// Initialize PortHandler instance
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupSyncWrite instances
dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE(portHandler, packetHandler, ADDR_PRO_TORQUE_ENABLE, LEN_PRO_TORQUE_ENABLE);
dynamixel::GroupSyncWrite groupSyncWrite_RGB_LED(portHandler, packetHandler,ADDR_PRO_INDIRECTDATA_FOR_WRITE_RGB_LED, LEN_PRO_INDIRECTDATA_FOR_WRITE_RGB_LED);
dynamixel::GroupSyncWrite groupSyncWrite_BAUDRATE(portHandler, packetHandler, ADDR_PRO_BAUDRATE, LEN_PRO_BAUDRATE);

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    //while(!Serial);
    Serial.println("[   MASTER:  ]  Start...");

    /*  
     * I. Begin Communication Testing
     */

    // I.a.1. Open port
    Serial.print("[   MASTER:  ]  Opening port...");
    return_function_state = portHandler->openPort();
    if (return_function_state)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
      return;
    }
    
    delay(500);
    Serial.println("Moving to baudrate...");

    // I.a.2 Set port baudrate
    Serial.print("[   MASTER:  ]  Setting port baudrate...");
    return_function_state = portHandler->setBaudRate(dxl_baudrate);
    if (return_function_state)
    {
      int achievedBaudrate = portHandler->getBaudRate();
      Serial.print("SUCCESS -  New Baudrate is: "); Serial.println(achievedBaudrate);
    }
    else
    {
      Serial.print("FAILED");
      return;
    } 
    delay(500);
    Serial.println("Moving to syncSetBaudrate...");

     // I.a.3 Set Dynamixels Baudrate
    //  Disable Dynamixels torque to access EEPROM Are
    
    return_function_state = syncSetTorque(dxl_id, sizeof(dxl_id), 0, groupSyncWrite_TORQUE_ENABLE, packetHandler);
    if (return_function_state == true)
    {
      Serial.print("SUCCESS");
    }
    else
    {
      Serial.print("FAILED");
    }
    
    /*
    uint8_t dxl_baudrate_range = 3;                                                                                 // 3 -> 1000000
    Serial.println("[   MASTER:  ]  Setting Dynamixels BAUDRATE...");
    return_function_state = dxl.syncSetBaudrate(dxl_id, sizeof(dxl_id), dxl_baudrate_range, groupSyncWrite_BAUDRATE, packetHandler);
    if (return_function_state == true)
    {
      Serial.print("SUCCESS");
    }
    else
    {
      Serial.print("FAILED");
    }
    delay(1000);
    Serial.println("Moving to ping...");

    //  Enable Dynamixels torque to access EEPROM Area
    param_torque_enable = 1;
    Serial.println("Setting Dynamixels TORQUE to ON...");
    return_function_state = dxl.syncSetTorque(dxl_id, sizeof(dxl_id), param_torque_enable, groupSyncWrite_TORQUE_ENABLE, packetHandler);
    if (return_function_state == true)
    {
      Serial.print("SUCCESS");
    }
    else
    {
      Serial.print("FAILED");
    }
    
    // I.a.4 Ping Dynamixels
    Serial.println("[   MASTER:  ]  Pinging Dynamixels...");
    return_function_state = dxl.pingDynamixels(dxl_id, sizeof(dxl_id) , groupSyncWrite_RGB_LED,  packetHandler, portHandler);
    if(return_function_state == true)
    {
      Serial.print("SUCCESS");
    }
    else
    {
      Serial.print("FAILED");
    }
*/
} // END SETUP

void loop() {

  Serial.println( String(666) );

  
}
