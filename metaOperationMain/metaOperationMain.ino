
/*   *********************************************************************************************
 *   Executes tests for Metamorphic Manipulator with 1 Nema34 Stepper + 3 Dynamixels
 *   using custom-made functions for Driving Steppers and Dynamixels of a serial metamorphic
 *   manipulator. 
 * 
 *   Tue 19.5.20 "Fixed definitions/declarations mismatches"
 *      - This file is hard-linked locally to the folder ~/Arduino/MetaLunokhod101_Development/test_metamorphic_manipulator
 *      - Configuration file is placed locally in ~/Arduino/libraries/test_metamorphic_manipulator_configuration folder
 *      - Git repo is updated from the MetaLunokhod101_Development folder
 *   ********************************************************************************************* 
 */

/* 
 *  Author: Stravopodis Nikos, PhD Candidate, University of the Aegean
 *  mail: n.stravopodis@syros.aegean.gr
 * 
 *  February 2020
 */

// Include Motor Configuration files
#include <definitions.h>                            // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include <motorIDs.h>                               // Includes motor IDs as set using Dynamixel Wizard
#include <contolTableItems_LimitValues.h>           // Limit values for control table controlTableItems_LimitValues
#include <StepperMotorSettings.h>                   // Includes Stepper Motor/Driver pin StepperMotorSettings

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
uint8_t   dxl_ledBLUE_value[]  = {0, 255};
uint8_t   dxl_ledGREEN_value[] = {0, 255};
uint8_t   dxl_ledRED_value[]   = {0, 255};
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
String user_input_string;
int user_input_int;
const char * meta_exec = "MET";
const char * act_exec  = "ACT";
const char * act_exec  = "HOME";
const char * act_exec  = "P2P";
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
DynamixelProPlusMetamorphicManipulator dxl;
CustomStepperMetamorphicManipulator stp(STP1_ID, STEP_Pin, DIR_Pin, ENABLE_Pin, LED_Pin, HALL_SWITCH_PIN1, HALL_SWITCH_PIN2, HALL_SWITCH_PIN3, LOCK_Pin, SPR1, GEAR_FACTOR_PLANETARY, FT_CLOSED_LOOP);

// Initialize PortHandler instance
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupSyncWrite instances
dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE(portHandler, packetHandler, ADDR_PRO_TORQUE_ENABLE, LEN_PRO_TORQUE_ENABLE);
dynamixel::GroupSyncWrite groupSyncWriteGoalPos(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
dynamixel::GroupSyncWrite groupSyncWrite_GP_A_V_LED(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_WRITE_GP_A_V_LED, LEN_PRO_INDIRECTDATA_FOR_WRITE_GP_A_V_LED);

// Initialize GroupSyncRead instances
dynamixel::GroupSyncRead groupSyncRead_PP_MV(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_READ_PP_MV, LEN_PRO_INDIRECTDATA_FOR_READ_PP_MV);
dynamixel::GroupSyncRead groupSyncRead_PP_PV_PA_VL_AL(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL, LEN_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL);

void setup() {
    Serial.begin(BAUDRATE);
    while(!Serial);
    Serial.println("[   MASTER:  ]  Start...");

    /*  
     * I. Begin Communication Testing
     */

    // I.a.1. Open port
    Serial.print("[   MASTER:  ]  Opening port...");
    if (portHandler->openPort())
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
      return;
    }

    // I.a.2 Set port baudrate
    Serial.print("[   MASTER:  ]  Setting port BAUDRATE");
    if (portHandler->setBaudRate(BAUDRATE))
    {
      int achievedBaudrate = portHandler->getBaudRate();
      Serial.println("SUCCESS -  New Baudrate is: "); Serial.println(achievedBaudrate);
    }
    else
    {
      Serial.println("FAILED");
    }

    // I.a.3 Ping Dynamixels
    Serial.println("[   MASTER:  ]  Pinging Dynamixels");
    return_function_state = dxl.pingDynamixels(dxl_id, sizeof(dxl_id) , packetHandler, portHandler);
    if(return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }

} // END SETUP

void loop() {

/*
 *  HERE 2 OPERATION MODES CAN BE SELECTED
 *  1. ACTION
 *  2. METAMORPHOSIS
 * 
 *  <ACTION> MODE ENABLES:
 *  1. P2P C-SPACE, I-F POINTS ARE DEFINED IN HEADER FILE trajectories.h, USER CAN ONLY SELECT THE DESIRED SET OF POINTS
 *  2.  
 *  <METAMORPHOSIS> MODE ENABLES:
 *  1. TORQUES OFF STEPPER+DYNAMIXELS
 *  2. USER DEFINES NEW ANATOMY "ON THE FLY", USING INPUTS FROM CMD LINE, EXECUTED BASED ON PseudoSPIcommMetamorphicManipulator Library
 */


}
