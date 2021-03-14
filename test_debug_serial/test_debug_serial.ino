/*   *********************************************************************************************
 *   Executes tests for Metamorphic Manipulator with 1 Nema34 Stepper + 3 Dynamixels
 *   using custom-made functions for Driving Steppers and Dynamixels of a serial metamorphic
 *   manipulator. 
 *
 *   ********************************************************************************************* 
 */

/* 
 *  Author: Stravopodis Nikos, PhD Candidate, University of the Aegean
 *  mail: n.stravopodis@syros.aegean.gr
 * 
 *  January 2021
 */

/*
 * INCLUDE LIBRARIES
 */
#include <definitions.h>                            // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include <motorIDs.h>                               // Includes motor IDs as set using Dynamixel Wizard
#include <contolTableItems_LimitValues.h>           // Limit values for control table controlTableItems_LimitValues
#include <StepperMotorSettings.h>                   // Includes Stepper Motor/Driver pin StepperMotorSettings
#include <task_definitions.h>                       // Includes task points for execution
#include <led_indicators.h>

// Libraries for Robot Motors Driving
#include "DynamixelProPlusOvidiusShield.h"
#include "CustomStepperOvidiusShield.h"
//  Used Libraries
#include "Arduino.h"                                // Main Arduino library
#include <DynamixelShield.h>                        // ROBOTIS library          
#include <Dynamixel2Arduino.h>

/* 
 *  CONFIGURE DEBUG_SERIAL PORT COMMUNICATION
 */
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

/* 
 *  NAMESPACE OF USED CLASSES
 */
using namespace std;
using namespace ControlTableItem;

/*
 * USER DEBUG_SERIAL INPUT VARIABLES
 */
const char * p2pcspace_exec = "p2pcsp";
const char * trajcsp_exec   = "trajcsp";
const char * home_exec   = "home";
const char * force_exec  = "force";
const char * accel_exec  = "accel";
const char * YES = "y";
const char * NO = "n";
// Auxiliary variables for DEBUG_SERIAL inputs
int error_code_received;
String user_input_string;
int user_input_int;
int nl_char_shifting   = 10;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int dataNumber = 0;             // new for this version
bool MENU_EXIT;
bool ACCESS_EEPROM;

/*
 * MAIN FLAGS USED TO CONTROL LOOPS
 */
bool END_P2PCSP;                      
bool END_TRAJCSP;
bool END_HOME;
bool END_FORCE;
bool END_ACCEL;
/*
 * SECONDARY FLAGS USED TO CONTROL LOOPS
 */
bool p2pcspExecution;
bool trajcspExecution;
bool homeExecution;
bool forceExecution;
bool accelExecution;  

/*
 * Configure IDs for Dynamixels & NEMA34 Stepper Motor -> MUST align with the IDs configured at EEPROM of each module!
 */
uint8_t dxl_id[] = {DXL1_ID, DXL2_ID, DXL3_ID};
int id_count;
const int dxl_motors_used = DXL_MOTORS;
int Nema34_id = STP1_ID;

/*
 * Configure Stepper Motor variables defined in CustomStepperOvidiusShield.h
 */
bool homingSwitchActivated = true;  // NC connection in trigger(despite signed as NO)
bool limit1SwitchActivated = true;  // Green LED is OFF at default(THIS WAS GREEN ON/true before)
bool limit2SwitchActivated = false; // Green LED is OFF at default

volatile byte currentDirStatus = CCW;     // Because SW5 is OFF=>DIR=CCW == DIR=HIGH
//uint32_t currentAbsPos;
uint32_t RELATIVE_STEPS_2_MOVE;
int stp_error;
volatile bool KILL_MOTION = false;

/*
 * Declare extern variables defined in DynamixelProPlusOvidiusShield.h
 */
uint8_t  dxl_error; 
int32_t  dxl_present_position[sizeof(dxl_id)];
int32_t  dxl_goal_position[sizeof(dxl_id)];
int32_t  dxl_prof_vel[sizeof(dxl_id)];
int32_t  dxl_prof_accel[sizeof(dxl_id)];
uint8_t  dxl_moving[sizeof(dxl_id)];

//int32_t   dxl_vel_limit[sizeof(dxl_id)];
//int32_t dxl_accel_limit[sizeof(dxl_id)];

/*
 * Timing variables 
 */
 unsigned long time_now_micros;
 unsigned long p2p_duration;
/*
 * Configure Stepper/Dynamixel settings
 */
double currentConfiguration[nDoF];
double desiredConfiguration[nDoF];
//dxlVelLimit dxl_vel_limit = {2000, 1500, 2000};
//dxlAccelLimit dxl_accel_limit = {900, 900, 900};

/* 
 * Declare extern variables defined in CustomStepperMetamorphicManipulator 
 */
bool return_function_state = false;
//bool segmentExistsTrapz;
double currentAbsPos_double;
double VelocityLimitStp;
double AccelerationLimitStp;
double MaxPosLimitStp;

double StpInitPos;                          // Variables used to initialilize StpTrapzProfParams[]
double StpGoalPosition;
double StpVmax;
double StpAmax;

double goalAbsPos_double;

const int StpTrapzProfParams_size = 4;                  // This array stores the same data as DxlTrapzProfParams_forP2P for Dxls
double StpTrapzProfParams[StpTrapzProfParams_size];

const int storage_array_for_TrajAssignedDuration_size = 5;  // This array stores properties for stepper trajectory equations for Melchiorri Theory Book
double storage_array_for_TrajAssignedDuration[storage_array_for_TrajAssignedDuration_size];

const int storage_array_for_PROFILE_STEPS_size = 4;     // This array is for Stepper Progile Trajectory Execution
unsigned long storage_array_for_PROFILE_STEPS[storage_array_for_PROFILE_STEPS_size];


/* 
 *  SYNC WRITE structure objects for sending Dynamixel data - Data type are defined in DynamixelProPlusOvidiusShield.h
 */
sw_data_t_gp sw_data_array_gp[dxl_motors_used]; // this type of array should be passed to the function
sw_data_t_pv sw_data_array_pv[dxl_motors_used]; // this type of array should be passed to the function
sw_data_t_pa sw_data_array_pa[dxl_motors_used]; // this type of array should be passed to the function

/* 
 *  SYNC READ structure objects for receiving Dynamixel data - Data type are defined in DynamixelProPlusOvidiusShield.h
 */
sr_data_t_pp sr_data_array_pp[dxl_motors_used]; // this type of array should be passed to the function

/* 
 *  Create object for handling motors
 */
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
DynamixelProPlusOvidiusShield meta_dxl(dxl_id); // Object of custom class to access custom functions for Ovidius manipulator specific 
CustomStepperOvidiusShield stp(STP1_ID, STEP_Pin, DIR_Pin, ENABLE_Pin, HOME_TRIGGER_SWITCH, HALL_SWITCH_PIN2, HALL_SWITCH_PIN3, RED_LED_PIN, GREEN_LED_PIN, BLUE_LED_PIN, SPR1, GEAR_FACTOR_PLANETARY, FT_CLOSED_LOOP);

using namespace ControlTableItem;

uint8_t incomingByte = 0;
/*
 * SETUP
 */
void setup() {
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // SET PINMODES
  pinMode(HOME_TRIGGER_SWITCH, INPUT_PULLUP);   // HOME TRIGGER SWITCH
  pinMode(HALL_SWITCH_PIN2, INPUT_PULLUP);             // LIMIT HALL SENSOR2 
  pinMode(HALL_SWITCH_PIN3, INPUT_PULLUP);             // LIMIT HALL SENSOR3

  // SET EXTERNAL INTERRUPTS
  delay(5000);
  DEBUG_SERIAL.println(" [ SETUP ] ENTERING OVIDIUS ROBOT SETUP... ");
  delay(5000);

  // SETUP DEFAULT ACTIONS
  DEBUG_SERIAL.print(" [ SETUP ] "); DEBUG_SERIAL.println("PINGING CONNECTED DYNAMIXELS PRO+");
  //ping_motors();
  
  DEBUG_SERIAL.print(" [ SETUP ] "); DEBUG_SERIAL.println("EXTRACTING GLOBAL VARIABLES FROM EEPROM ");
  //stp.read_STP_EEPROM_settings(&currentDirStatus, &currentAbsPos_double, &VelocityLimitStp, &AccelerationLimitStp, &MaxPosLimitStp); // Initialize global Stepper Variables from EEPROM Memory

  DEBUG_SERIAL.print(" [ SETUP ] "); DEBUG_SERIAL.println("EXTRACTING CURRENT CONFIGURATION");
  ACCESS_EEPROM = true;
  //read_current_configuration(ACCESS_EEPROM);
      
  // SETUP USER-SELECTED ACTIONS 
  bool stop_setup = false; // flag for setup menu robot loop

  while(!stop_setup) //wh1
  {
    /*
     * I. SLOW MOTOR HOMING
     */

    DEBUG_SERIAL.println("[ SETUP ] HOME MOTORS SLOW?");

    DEBUG_SERIAL.listen();
    DEBUG_SERIAL.read();
    while(DEBUG_SERIAL.available()==0)
    {
      DEBUG_SERIAL.print("[ INFO ]"); DEBUG_SERIAL.println(DEBUG_SERIAL.available());
      delay(500);
      incomingByte = DEBUG_SERIAL.read();
      DEBUG_SERIAL.print("[ USER INPUT ]"); DEBUG_SERIAL.print("   ->   "); DEBUG_SERIAL.println(incomingByte, DEC);
    }
    incomingByte = DEBUG_SERIAL.read();

    switch(incomingByte) {
      case '1':
        DEBUG_SERIAL.print("[ USER INPUT ]"); DEBUG_SERIAL.print("   ->   "); DEBUG_SERIAL.println(incomingByte, DEC);
        stop_setup = true;
        break;
      case '2':
        DEBUG_SERIAL.print("[ USER INPUT ]"); DEBUG_SERIAL.print("   ->   "); DEBUG_SERIAL.println(incomingByte, DEC);
        break;  
      default:
      DEBUG_SERIAL.println("fuck");
        break;
    }
        
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
