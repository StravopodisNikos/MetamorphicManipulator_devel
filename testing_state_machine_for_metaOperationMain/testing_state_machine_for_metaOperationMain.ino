/*   *********************************************************************************************
 *   Executes tests for Metamorphic Manipulator with 1 Nema34 Stepper + 3 Dynamixels
 *   using custom-made functions for Driving Steppers and Dynamixels of a serial metamorphic
 *   manipulator. 
 *
 *   In order to successfully compile UNO/MEGA BOARD + DXL SHIELD is recommended. Also custom functions 
 *   must be added inside Arduino/libraries.(TO BE DONE: CLEAN THE MESS OF LIBRARY FOLDERS)
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
#include <definitions.h>                              // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include <motorIDs.h>                                 // Includes motor IDs as set using Dynamixel Wizard
#include <contolTableItems_LimitValues.h>             // Limit values for control table controlTableItems_LimitValues
#include <StepperMotorSettings.h>                     // Includes Stepper Motor/Driver pin StepperMotorSettings
#include <task_definitions.h>                         // Includes task points for execution
#include <led_indicators.h>
//#include <ovidius_robot_controller_eeprom_addresses.h>  // Header file for all eeprom addresses used!
#include <avr/pgmspace.h>

// Libraries for Robot Motors Driving
#include "DynamixelProPlusOvidiusShield.h"
#include "CustomStepperOvidiusShield.h"
//  Used Libraries
#include "Arduino.h"                                // Main Arduino library
#include <Dynamixel2Arduino.h>

#include "OvidiusSensors.h"
#include <utility/OvidiusSensors_config.h>

/* 
 *  CONFIGURE Serial PORT COMMUNICATION

#if defined(ARDUINO_AVR_UNO) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define Serial soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(11, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define Serial soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define Serial SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define Serial SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define Serial Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define Serial Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define Serial Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 */
/* 
 *  NAMESPACE OF USED CLASSES
 */
using namespace std;
using namespace ControlTableItem;

/*
 * USER Serial INPUT VARIABLES
 */
byte YES_b     = 1;
byte NO_b      = 0;
byte p2pcsp_b  = 2;
byte trajcsp_b = 3;
byte home_b    = 4;
// Auxiliary variables for Serial inputs
int error_code_received;
byte user_input;
bool MENU_EXIT;
bool ACCESS_EEPROM;
int p2p_index;
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
double p2pcsp_joint_velocities[nDoF];
double joint_velocities_limits[] = {1, 1.57, 1.57, 1.57};     // [rad/sec] must be implemented in EEPROM Stepper+Dxl!
double p2pcsp_joint_accelerations[nDoF];
double joint_accelerations_limits[] = {20, 87, 87, 87};       // [rad/sec^2] must be implemented in EEPROM Stepper+Dxl!
double p2pcsp_Texec;
double p2pcsp_Ta;
//dxlVelLimit dxl_vel_limit = {2000, 1500, 2000};
//dxlAccelLimit dxl_accel_limit = {900, 900, 900};

/* 
 * Declare extern variables defined in CustomStepperMetamorphicManipulator 
 */
bool return_function_state = false;
//bool segmentExistsTrapz;
double currentAbsPos_double;
double VelocityLimitStp = 10;
double AccelerationLimitStp = 10;
double MaxPosLimitStp = 2.0;

double StpInitPos;                          // Variables used to initialilize StpTrapzProfParams[]
double StpGoalPosition;
double StpVmax;
double StpAmax;

double goalAbsPos_double;

CustomStepperOvidiusShield stp(STP1_ID, STEP_Pin, DIR_Pin, ENABLE_Pin, HOME_TRIGGER_SWITCH, HALL_SWITCH_PIN2, HALL_SWITCH_PIN3, RED_LED_PIN, GREEN_LED_PIN, BLUE_LED_PIN, SPR1, GEAR_FACTOR_PLANETARY, FT_CLOSED_LOOP);

void setup() {
  Serial.begin(115200);            // Serial BAUDRATE->57600
  while(!Serial);
   
  delay(1000);
  Serial.println(F(" [ SETUP ] ENTERING OVIDIUS ROBOT SETUP... "));
  delay(1000);

  // SETUP USER-SELECTED ACTIONS 
  bool stop_setup = false; // flag for setup menu robot loop

  while(!stop_setup) 
  {
    /*
     * I. TEST STEPPER
     */
    Serial.println(F("[ SETUP ] TEST THE MOTOR?"));
    Serial.parseInt();
    while (Serial.available() == 0) {};
    user_input = Serial.parseInt();
    Serial.print(F("[ USER INPUT ]")); Serial.print(F("   ->   ")); Serial.println(user_input);
  
    if( user_input == YES_b ) 
    {
      Serial.print(F(" [ INFO ] ")); Serial.println(F("TESTING STEPPER STATE MACHINE"));

      test_stepper_state_machine();
    }

            /*
     * FINISH SETUP
     */
    Serial.println(F("[ SETUP ] EXIT SETUP?"));
    Serial.parseInt();
    while (Serial.available() == 0) {};
    user_input = Serial.parseInt();
    Serial.print(F("[ USER INPUT ]")); Serial.print(F("   ->   ")); Serial.println(user_input);
  
    if( user_input == YES_b ) //start->if1
    {
      stop_setup = true;
      Serial.print(F(" [ INFO ] ")); Serial.println(F("SETUP FINISHED"));
    }
    else
    {
      Serial.print(F(" [ INFO ] ")); Serial.println(F("SETUP MENU WILL REPEAT"));
    }
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
