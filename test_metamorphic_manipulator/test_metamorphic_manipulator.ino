
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
#include <SPI.h>

using namespace std;

// Libraries for Robot Motors Driving
#include "DynamixelProPlusMetamorphicManipulator.h"
#include "CustomStepperMetamorphicManipulator.h"
#include "PseudoSPIcommMetamorphicManipulator.h"

/*
 * Configure IDs for Dynamixels & NEMA34 Stepper Motor -> MUST align with the IDs configured at EEPROM of each module!
 */
uint8_t dxl_id[] = {DXL1_ID, DXL2_ID, DXL3_ID};
int id_count;
int dxl_motors_used = 3;
int Nema34_id = 1;
/*
 * For SPI communication Protocol
 */
int masterID          = 0;
const int TOTAL_PSEUDOS_CONNECTED = 1;
int pseudoIDs[]       = {PSEUDO1_ID};
int ssPins[]          = {SSpinPseudo1};
byte anatomy_counter  = 0;                        // just to check repeat mode
byte CURRENT_STATE[sizeof(pseudoIDs)];            // empty states initialization array
byte CURRENT_ANATOMY[sizeof(pseudoIDs)];
bool META_MODES[sizeof(pseudoIDs)];               // empty pseudo mode initialization array
bool META_EXECS[sizeof(pseudoIDs)];               // empty pseudo mode-exec initialization array

/*
 * For METAMORPHOSIS
 */
byte state_receive_from_slave;
bool metaExecution;
bool END_METAMORPHOSIS;
bool END_ACTION = false;

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
PseudoSPIcommMetamorphicManipulator MASTER_SPI(Tx, masterID, statusLED_Pin, MOSI_NANO, MISO_NANO, SCK_NANO, TXled_Pin, RXled_Pin, ssPins);
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
    Serial.println("[   MASTER:  ]  Start..");

    /*  
     * I. Begin Communication Testing
     */

    // I.a.1. Open port
    Serial.println("[   MASTER:  ]  Opening port");
    if (portHandler->openPort())
    {
      Serial.print("SUCCESS");
    }
    else
    {
      Serial.print("FAILED");
      return;
    }

    // I.a.2 Set port baudrate
    Serial.println("[   MASTER:  ]  Setting port BAUDRATE");
    if (portHandler->setBaudRate(BAUDRATE))
    {
      int achievedBaudrate = portHandler->getBaudRate();
      Serial.print("SUCCESS -  New Baudrate is: "); Serial.println(achievedBaudrate);
    }
    else
    {
      Serial.print("FAILED");
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

    // I.a.4.  Setup MASTER(OpenCR) Pinmodes for SPI Communication
    // MASTER PINMODE 
    pinMode(SCK_NANO, OUTPUT);
    pinMode(MOSI_NANO, OUTPUT);
    pinMode(MISO_NANO, INPUT);
    for (size_t i = 0; i < sizeof(ssPins); i++)
    {
      pinMode(ssPins[i], OUTPUT);
      pinMode(ssPins[i], HIGH);
    }
   
  // I.a.4.1 Start SPI Com Protocol
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV32);      // Slow down the master a bit

  // I.a.4.2 Ping Pseudos-Each pseudo pinged: Blinks(2,500) green Led(ConnectedLED)

  for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
  {
    return_function_state = MASTER_SPI.connectPseudoMaster(pseudoIDs[pseudo_cnt], ssPins);
    if (return_function_state)
    {
        //MASTER_SPI.statusLEDblink(2, 500);
        Serial.print("[   MASTER:  ]"); Serial.print(" CONNECTED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   SUCCESS");
    }
    else
    {
        //MASTER_SPI.statusLEDblink(4, 250);
        Serial.print("[   MASTER:  ]"); Serial.print(" CONNECTED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   FAILED");
    }
    delay(1);
  } // END IF PING

    /*
     *  SET/READ EEPROM SETTING FOR ACTIVE AND PASSIVE JOINT MOTORS
     */

    // II.a.1 Read current anatomy -> saved in byte array: CURRENT_ANATOMY
    Serial.println("[   MASTER:  ]  Extracting Present Anatomy");

    for (int pseudo_cnt = 0; pseudo_cnt < TOTAL_PSEUDOS_CONNECTED; pseudo_cnt++) 
    {
        return_function_state = MASTER_SPI.readCurrentAnatomyMaster(pseudoIDs[pseudo_cnt], ssPins, CURRENT_ANATOMY);
        if (return_function_state)
        {
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READ CURRENT Ci  ]  SUCCESS");
        }
        else
        {
            Serial.print("[   MASTER:  ]"); Serial.print(" TALKED TO: [   PSEUDO: "); Serial.print(pseudoIDs[pseudo_cnt]); Serial.println("  ]   STATUS:  [  READ CURRENT Ci  ]  FAILED");
        }

        // Here Anatomy MUST BE PRINTED IN SERIAL MONITOR!
    }

    // II.b.1 Setup current position/velocity/acceleration settings of stepper Joint1 
    Serial.println("[   MASTER:  ]  Setup STEPPER in OpenCR EEPROM?: [Y/N] ");

    while (Serial.available() == 0) {};
    user_input_string = Serial.readString();
      
    Serial.print("[   MASTER:  ]  [ USER INPUT ]"); Serial.print("   ->   "); Serial.println(user_input_string);
      
    if( ( strcmp(user_input_string.c_str(),YES)-nl_char_shifting == 0 ) )
    {
      // DEFAULT VALUES
      currentDirStatus = HIGH;     // [byte]
      currentAbsPos_double = 0.0;    // [rad]
      VelocityLimitStp = 5.0;        // [rad/sec]
      AccelerationLimitStp = 5.0;    // [rad/sec^2]
      MaxPosLimitStp = PI/2;         // [rad]

      stp.save_STP_EEPROM_settings(&currentDirStatus, &currentAbsPos_double, &VelocityLimitStp , &AccelerationLimitStp, &MaxPosLimitStp);
    }

    // II.b.2 Read current position/velocity/acceleration settings of stepper Joint1
    Serial.println("[   MASTER:  ]  Extracting Present Configuration and Motion Profile parameters for Stepper NEMA34");
    stp.read_STP_EEPROM_settings(&currentDirStatus, &currentAbsPos_double, &VelocityLimitStp , &AccelerationLimitStp, &MaxPosLimitStp);
    // Print the results in serial monitor
          Serial.print("[   JOINT1 STEPPER :"); Serial.print(STP1_ID); Serial.println(" ]   [READ EEPROM SETTINGS]:   SUCCESS  [UNITS]:  RAD, SEC"); 
          Serial.println("-----------------------------------------------------------------------------------------------");
          Serial.print("[  ABS_ANGLE_RAD ]    [   "); Serial.print(currentAbsPos_double);  Serial.println("     ]");
          Serial.print("[   VEL_LIMIT    ]    [   "); Serial.print(VelocityLimitStp);      Serial.println("     ]");
          Serial.print("[   ACCEL_LIMIT  ]    [   "); Serial.print(AccelerationLimitStp ); Serial.println("     ]");
          Serial.print("[   DIR_STATUS   ]    [   "); Serial.print(currentDirStatus);      Serial.println("     ]");
          Serial.println("-----------------------------------------------------------------------------------------------");

    // II.c.1  Read current position/velocity/acceleration Dynamixel settings
    Serial.println("[   MASTER:  ]  Extracting Present Configuration and Motion Profile parameters for DYNAMIXELS PRO+");
    return_function_state = dxl.syncGet_PP_PV_PA_VL_AL(dxl_id, sizeof(dxl_id), dxl_present_position, sizeof(dxl_present_position), dxl_prof_vel, sizeof(dxl_prof_vel), dxl_prof_accel, sizeof(dxl_prof_accel), dxl_vel_limit, sizeof(dxl_vel_limit), dxl_accel_limit, sizeof(dxl_accel_limit), groupSyncRead_PP_PV_PA_VL_AL, groupSyncWrite_TORQUE_ENABLE, packetHandler, portHandler);
    if(return_function_state == true)
    {
      for (size_t id_count = 0; id_count < dxl_motors_used; id_count++)
      {
        	// Print Dynamixel Settings extracted, in serial monitor
          Serial.print("[   DYNAMIXEL :"); Serial.print(dxl_id[id_count]); Serial.println(" ]   [READ EEPROM SETTINGS]:   SUCCESS  [UNITS]:  PULSES"); 
          Serial.println("-----------------------------------------------------------------------------------------------");
          Serial.print("[    ABS_ANGLE   ]    [   "); Serial.print(dxl_present_position[id_count]); Serial.println("     ]");
          Serial.print("[   VEL_LIMIT    ]    [   "); Serial.print(dxl_vel_limit[id_count]);        Serial.println("     ]");
          Serial.print("[   ACCEL_LIMIT  ]    [   "); Serial.print(dxl_accel_limit[id_count]);      Serial.println("     ]");
          Serial.print("[   PROFILE_VEL  ]    [   "); Serial.print(dxl_prof_vel[id_count]);         Serial.println("     ]");
          Serial.print("[ PROFILE_ACCEL  ]    [   "); Serial.print(dxl_prof_accel[id_count]);       Serial.println("     ]");
          Serial.println("-----------------------------------------------------------------------------------------------");
      }
    }
    else
    {
      Serial.println("FAILED");
    }

    // II.d.1 Read EEPROM setting for Pseudos


    /* 
     * III.a  USER INPUT MENU: Asks for user input to give desired commands for: 
    *  1. set home position
    *  2. set anatomy 
    *  3. set new joint actuator limits
    *  4. set configuration
    *  
    * EXECUTED IN A LOOP UNTIL USER SETS EXIT COMMAND
    */

  // START USER INPUT MENU 
  do{

    // III.a.1 HOMING
    Serial.println("[   MASTER:  ]  HOME MOTORS? [Y/N] :");

    while (Serial.available() == 0) {};

    user_input_string = Serial.readString();

    Serial.print("[   MASTER:  ]  [ USER INPUT ]"); Serial.print("   ->   "); Serial.print(user_input_string);
    if( ( strcmp(user_input_string.c_str(),YES)-nl_char_shifting == 0 ) )
    {
        Serial.println("[   MASTER:  ]  STARTED HOMING ACTIVE JOINT MOTORS");

        // HOMING STEPPER
        stp.setStepperHomePositionSlow();
        // HOMING DYNAMIXELS
        dxlGoalPos dxl_goal_pos = {0,0,0};
        return_function_state = dxl.syncSetGoalPosition(dxl_id, sizeof(dxl_id), dxl_goal_pos, sizeof(dxl_goal_pos), groupSyncWriteGoalPos,  packetHandler, portHandler);
        
        Serial.println("[   MASTER:  ]  FINISHED HOMING ACTIVE JOINT MOTORS");
    }
    else
    {
      Serial.println("[   MASTER:  ]  [WARNING] ACTIVE JOINT MOTORS NOT IN HOME POSITION");
    } 

    // III.a.2 NEW ANATOMY
    Serial.println("[   MASTER:  ]  SET NEW ANATOMY? [Y/N] :");

    while (Serial.available() == 0) {};

    user_input_string = Serial.readString();

    Serial.print("[ USER INPUT ]"); Serial.print("   ->   "); Serial.print(user_input_string);
    if( ( strcmp(user_input_string.c_str(),YES)-nl_char_shifting == 0 ) )
    {
        Serial.println("[   MASTER:  ]  SETTING NEW ANATOMY");

        // III.a.2.1 Construct desiredAnatomy
        for (size_t id_count = 0; id_count < nPseudoJoints; id_count++)
        {
          Serial.print("Give desired Ci for Pseudojoint["); Serial.print(id_count+1); Serial.println("] :");
          while (Serial.available() == 0) {};
          desiredAnatomy[id_count] = (byte) Serial.read();
        }

        // III.a.2.2 Execute routine for metamorphosis(first executed in using_ISR_simple_complete_master.ino -> now: metamorphosis_execution.ino)
        END_METAMORPHOSIS = false;
        metaExecution = true;
        metamorphosis_execution(MASTER_SPI, END_METAMORPHOSIS, metaExecution, desiredAnatomy, pseudoIDs, ssPins, CURRENT_STATE, TOTAL_PSEUDOS_CONNECTED);
    }
    else
    {
      Serial.println("[   MASTER:  ]  [INFO] ANATOMY UNCHNAGED");
    }

    // III.a.3 NEW ACTUATOR LIMITS

    // III.a.4 NEW CONFIGURATION
    Serial.println("[   MASTER:  ]  SET NEW CONFIGURATION? [Y/N] :");

    while (Serial.available() == 0) {};

    user_input_string = Serial.readString();

    Serial.print("[ USER INPUT ]"); Serial.print("   ->   "); Serial.print(user_input_string);
    if( ( strcmp(user_input_string.c_str(),YES)-nl_char_shifting == 0 ) )
    {
        Serial.println("[   MASTER:  ]  SETTING NEW CONFIGURATION ON ACTIVE JOINT MOTORS");

        for (size_t id_count = 0; id_count < nDoF; id_count++)
        {
          Serial.print("Give desired absolute angle for active joint ["); Serial.print(id_count+1); Serial.println("] :");
          while (Serial.available() == 0) {};
          desiredConfiguration[id_count] = Serial.parseFloat();
        }
        
        // HERE SYNC MUST BE EXECUTED
        // [0] -> current abs position is extracted: currentAbsPos_double is the global double variable that saves current abs angle(always initialized at setup from EEPROM)
        // [1] -> goal abs position is given by the array filled by user
        // [2] -> max vel limit is assigned
        // [3] -> max accel limit is assigned
        // Stepper Nema34
        StpInitPos          = currentAbsPos_double;                         
        StpGoalPosition     = (double) desiredConfiguration[0];
        StpVmax             = VelocityLimitStp;
        StpAmax             = AccelerationLimitStp;
        StpTrapzProfParams[0]  =  StpInitPos;
        StpTrapzProfParams[1]  =  StpGoalPosition;
        StpTrapzProfParams[2]  =  StpVmax;
        StpTrapzProfParams[3]  =  StpAmax;

        // Dynamixels
        return_function_state = dxl.syncGet_PP_MV( dxl_id, sizeof(dxl_id), dxl_moving, sizeof(dxl_moving), dxl_present_position, sizeof(dxl_present_position) , groupSyncRead_PP_MV, groupSyncWrite_TORQUE_ENABLE, packetHandler, portHandler);
        // convert double to pulses
        dxl_goal_position[0] = dxl.convertRadian2DxlPulses((double) desiredConfiguration[1]);
        dxl_goal_position[1] = dxl.convertRadian2DxlPulses((double) desiredConfiguration[2]);
        dxl_goal_position[2] = dxl.convertRadian2DxlPulses((double) desiredConfiguration[3]);

        DxlTrapzProfParams_forP2P[0][0] = dxl_present_position[0];      // Dxl1
        DxlTrapzProfParams_forP2P[0][1] = dxl_goal_position[0];
        DxlTrapzProfParams_forP2P[0][2] = dxl_vel_limit[0];
        DxlTrapzProfParams_forP2P[0][3] = dxl_accel_limit[0];
        DxlTrapzProfParams_forP2P[1][0] = dxl_present_position[1];      // Dxl2
        DxlTrapzProfParams_forP2P[1][1] = dxl_goal_position[1];
        DxlTrapzProfParams_forP2P[1][2] = dxl_vel_limit[1];
        DxlTrapzProfParams_forP2P[1][3] = dxl_accel_limit[1];
        DxlTrapzProfParams_forP2P[2][0] = dxl_present_position[2];      // Dxl3
        DxlTrapzProfParams_forP2P[2][1] = dxl_goal_position[2];
        DxlTrapzProfParams_forP2P[2][2] = dxl_vel_limit[2];
        DxlTrapzProfParams_forP2P[2][3] = dxl_accel_limit[2];

        Serial.println("[   MASTER:  ]  [INFO] SETTING NEW CONFIGURATION");
        
        syncP2Ptrapz_execution(DxlTrapzProfParams_forP2P, StpTrapzProfParams, TrapzProfParams_size, groupSyncWrite_GP_A_V_LED, groupSyncWrite_TORQUE_ENABLE, groupSyncRead_PP_MV, portHandler, packetHandler);

        Serial.println("[   MASTER:  ]  [INFO] NEW CONFIGURATION SET");
    }
    else
    {
      Serial.println("[   MASTER:  ]  [INFO] CONFIGURATION UNCHNAGED");
    } 

    // END MENU LOOP?
    Serial.println("[   MASTER:  ]  EXIT USER INPUT MENU? [Y/N] :");

    while (Serial.available() == 0) {};

    user_input_string = Serial.readString();

    Serial.print("[ USER INPUT ]"); Serial.print("   ->   "); Serial.print(user_input_string);
    if( ( strcmp(user_input_string.c_str(),YES)-nl_char_shifting == 0 ) )
    {
        MENU_EXIT = true;
    }
    else
    {
      MENU_EXIT = false;
    } 

  } while(!MENU_EXIT); // END USER INPUT MENU

  // PRINT MSG

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
