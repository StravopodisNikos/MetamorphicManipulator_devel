
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
#include "CustomStepperOvidiusShield.h"
//  Used Libraries
#include "Arduino.h"                                // Main Arduino library
#include <Dynamixel2Arduino.h>

// Configure Serial Port Communication
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif


// Namespace of used classes
using namespace std;
using namespace ControlTableItem;

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
bool homingSwitchActivated = true; // NC connection in trigger(despite signed as NO)
bool limit1SwitchActivated = true; // Green LED is ON at default
bool limit2SwitchActivated = false;// Green LED is OFF at default

//enum ROT_DIR{CW, CCW};
volatile byte currentDirStatus = CCW;     // Because SW5 is OFF=>DIR=CCW == DIR=HIGH
uint32_t currentAbsPos;
uint32_t RELATIVE_STEPS_2_MOVE;
int stp_error;
volatile bool KILL_MOTION = false;

/*
 * Declare extern variables defined in DynamixelProPlusOvidiusShield.h
 */
//bool dxl_addparam_result;                     // crashes serial monitor gamwthnpanagiatou
uint8_t dxl_error; 
int32_t  dxl_present_position[sizeof(dxl_id)];
int32_t  dxl_goal_position[sizeof(dxl_id)];
int32_t   dxl_prof_vel[sizeof(dxl_id)];
int32_t   dxl_prof_accel[sizeof(dxl_id)];
uint8_t   dxl_moving[sizeof(dxl_id)];

//int32_t   dxl_vel_limit[sizeof(dxl_id)];
//int32_t dxl_accel_limit[sizeof(dxl_id)];

/*
 *  LED INDICATORS
 */
unsigned char completed_move_indicator[] = {244, 164, 96};// sandybrown
unsigned char torque_off_indicator[] = {148, 0, 211};     // dark violet
unsigned char motors_homed_indicator[] = {255, 140, 0};  // orange
unsigned char motors_moving_indicator[] = {135, 206, 235};  // skyblue
unsigned char turn_off_led[] = {0, 0, 0};

/*
 * Configure Stepper/Dynamixel settings
 */
double currentConfiguration[nDoF];
double desiredConfiguration[nDoF];
//dxlVelLimit dxl_vel_limit = {2000, 1500, 2000};
//dxlAccelLimit dxl_accel_limit = {900, 900, 900};

// Declare extern variables defined in CustomStepperMetamorphicManipulator
bool return_function_state = false;
bool segmentExistsTrapz;
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
 *  USER SERIAL INPUT VARIABLES
 */
 const PROGMEM char YES[] = {"y"}; byte YES_b = 1;
const PROGMEM char NO[]  = {"no"};byte NO_b  = 0;
int error_code_received;
byte user_input;
String user_input_string;
int user_input_int;
const char * meta_exec = "MET";
const char * act_exec  = "ACT";
const char * home_exec  = "HOME";
const char * p2p_exec  = "P2P";
const char * meta_cont = "R";
const char * meta_exit = "E";
//const char * YES = "Y";
//const char * NO = "N";
int nl_char_shifting   = 10;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int dataNumber = 0;             // new for this version
bool MENU_EXIT;

bool home_switch_activated = false;
/* 
 *  Create object for handling motors
 */
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
DynamixelProPlusOvidiusShield meta_dxl(dxl_id); // Object of custom class to access custom functions for Ovidius manipulator specific 
CustomStepperOvidiusShield stp(STP1_ID, STEP_Pin, DIR_Pin, ENABLE_Pin, HOME_TRIGGER_SWITCH, HALL_SWITCH_PIN2, HALL_SWITCH_PIN3, RED_LED_PIN, GREEN_LED_PIN, BLUE_LED_PIN, SPR1, GEAR_FACTOR_PLANETARY, FT_CLOSED_LOOP);

using namespace ControlTableItem;

/*
 * SETUP
 */
void setup() {
  DEBUG_SERIAL.begin(57600);
  while(!DEBUG_SERIAL);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // SETUP STEPPER INTERRUPTS FOR HOMING-LIMIT SWITCHES
  pinMode(HOME_TRIGGER_SWITCH, INPUT_PULLUP);   // HOME TRIGGER SWITCH
  pinMode(HALL_SWITCH_PIN2, INPUT_PULLUP);             // LIMIT HALL SENSOR2 
  pinMode(HALL_SWITCH_PIN3, INPUT_PULLUP);             // LIMIT HALL SENSOR3

  //digitalWrite(HALL_SWITCH_PIN2, LOW);
  //digitalWrite(HALL_SWITCH_PIN3, LOW);
  
  attachInterrupt(digitalPinToInterrupt(HALL_SWITCH_PIN2), changeStepperDirInterrupt1, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_SWITCH_PIN3), changeStepperDirInterrupt1, RISING);

  delay(2000);
  DEBUG_SERIAL.println("[  INFO  ] SETUP START [  SUCCESS ]");

  //added on 22-1-21 just to test serial port ->
  DEBUG_SERIAL.println(F("TEST SERIAL - INSERT INT"));
  DEBUG_SERIAL.parseInt();
  while (DEBUG_SERIAL.available() == 0) {};
  user_input = DEBUG_SERIAL.parseInt();
  DEBUG_SERIAL.print("[ USER INPUT ]"); DEBUG_SERIAL.print("   ->   "); DEBUG_SERIAL.print(user_input);
  // <-
  
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
/*
  // ONLY MOVE STEPPER OUT OF HOME
  // set goal position1
  currentAbsPos_double = 0;
  goalAbsPos_double = 0.7854;
  KILL_MOTION = false;
  DEBUG_SERIAL.print("SET GOAL POSITION STARTING... -> KILL MOTION VALUE: "); DEBUG_SERIAL.println(KILL_MOTION);
  DEBUG_SERIAL.print("SET GOAL POSITION STARTING... -> CURRENT VALUE: "); DEBUG_SERIAL.println(currentAbsPos_double);
  DEBUG_SERIAL.print("SET GOAL POSITION STARTING... -> GOAL VALUE: "); DEBUG_SERIAL.println(goalAbsPos_double);
  RELATIVE_STEPS_2_MOVE = 111;  //RANDOM
  stp_error = 4; //random
  return_function_state = stp.setStepperGoalPositionFixedStep(&currentAbsPos_double, &goalAbsPos_double, &currentDirStatus, &RELATIVE_STEPS_2_MOVE, &KILL_MOTION, &stp_error  );
  if (return_function_state)
  {
    DEBUG_SERIAL.print("SET GOAL POSITION FINISHED NEW POSITION[rad]: "); DEBUG_SERIAL.println(currentAbsPos_double);
    DEBUG_SERIAL.print("SET GOAL POSITION FINISHED MOVED[pulses]: "); DEBUG_SERIAL.println(RELATIVE_STEPS_2_MOVE);
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(stp_error);
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ] SET GOAL POSITION STEPPER JOINT1 [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(stp_error);
    DEBUG_SERIAL.print("SET GOAL POSITION FINISHED -> KILL MOTION VALUE: "); DEBUG_SERIAL.println(KILL_MOTION);
  }
  delay(2000);
*/

  // HOME stepper
 
  DEBUG_SERIAL.println("[  INFO  ] HOMING STEPPER MOTOR... ");
  delay(2000);
  currentAbsPos = 55000; //random
  DEBUG_SERIAL.print("NOT HOMED CURRENT POSITION[STEP]: "); DEBUG_SERIAL.println(currentAbsPos);
  DEBUG_SERIAL.print("NOT HOMED CURRENT DIRECTION: "); DEBUG_SERIAL.println(currentDirStatus);
  stp_error = 4; //random
  stp.setStepperHomePositionSlow( &currentAbsPos_double, &currentDirStatus, &KILL_MOTION, &stp_error);
  //stp.setStepperHomePositionFast( &currentAbsPos_double, &currentDirStatus, &KILL_MOTION, &stp_error);
  DEBUG_SERIAL.print("HOMING FINISHED NEW POSITION[STEP]: "); DEBUG_SERIAL.println(currentAbsPos);
  DEBUG_SERIAL.print("HOMING FINISHED NEW DIRECTION: "); DEBUG_SERIAL.println(currentDirStatus);
  DEBUG_SERIAL.print("HOMING FINISHED STP ERROR CODE: "); DEBUG_SERIAL.println(stp_error);
  DEBUG_SERIAL.print("HOMING FINISHED KILL MOTION VALUE: "); DEBUG_SERIAL.println(KILL_MOTION);
  delay(2000);

  
  // SAVE EEPROM
  VelocityLimitStp = 10.0;     // [rad/sec]
  AccelerationLimitStp = 100.0; // [rad/sec^2]
  MaxPosLimitStp = 2.618;      // [rad] =~150 deg
  stp.save_STP_EEPROM_settings(&currentDirStatus, &currentAbsPos_double, &VelocityLimitStp, &AccelerationLimitStp, &MaxPosLimitStp);
  DEBUG_SERIAL.print("[  INFO  ] STEPPER SAVED EEPROM  [  SUCCESS ]");
  delay(2000);

  
  // SYNC WRITE DXL+STP
    //SET TORQUE ON
  return_function_state = meta_dxl.setDynamixelsTorqueON(dxl_id, sizeof(dxl_id), dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[  INFO  ] TORQUE ON DYNAMIXELS [  SUCCESS ]");
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ] TORQUE ON DYNAMIXELS  [  FAILED ]");
  }

  // SYNC MOVE MOTORS 1
  // GIVE P2P PROPERTIES
  // 1.STEPPER PRE SETTING TO COMPUTE Texec,Ta
  //currentAbsPos_double = 0;
  goalAbsPos_double = 0.4;  // just change sign for each pair of points defined in l.260-264
  double Vexec;
  double Aexec;
  double Texec = 1.5;
  double Taccel;
  RELATIVE_STEPS_2_MOVE = 111;
  stp_error = 4;
  bool LIN_SEG_EXISTS;
  uint32_t P2P_PROF_STEPS[4];
  return_function_state =  stp.syncPreSetStepperGoalPositionVarStep(&currentAbsPos_double, &goalAbsPos_double, &Vexec, &Aexec, &Texec, &Taccel, &currentDirStatus, &LIN_SEG_EXISTS, P2P_PROF_STEPS,  &stp_error);
  if (return_function_state)
  {
    DEBUG_SERIAL.println("[  INFO  ] PRE SET STEPPER JOINT1 [  SUCCESS ]");
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ] PRE SET STEPPER JOINT1 [  FAILED ]");
  }
  // 2.DYNAMIXELS CALCULATE VEL/ACCEL PROFILE FOR SYNCED MOTION
  dxl_prof_vel[0] = 100; 
  dxl_prof_vel[1] = 500;
  dxl_prof_vel[2] = 200;
  dxl_prof_accel[0] = 500; 
  dxl_prof_accel[1] = 500;
  dxl_prof_accel[2] = 500;
  // TEST C-SPACE POINTS
  double Qdxl_i[] = {0,0,0};
  double Qdxl_f[] = {0.3,-0.7,-0.5};
  
  //double Qdxl_i[] = {0.3,-0.7,-0.5};
  //double Qdxl_f[] = {0,0,0};
  
  // convert Delta_Pos matrices to pulses
  double Qdxl_rel_dpos[3];
  Qdxl_rel_dpos[0] = Qdxl_f[0] - Qdxl_i[0];
  Qdxl_rel_dpos[1] = Qdxl_f[1] - Qdxl_i[1];
  Qdxl_rel_dpos[2] = Qdxl_f[2] - Qdxl_i[2];
  return_function_state = meta_dxl.calculateProfVelAccel_preassignedVelTexec2(dxl_prof_vel, dxl_prof_accel, Qdxl_rel_dpos, Taccel,Texec,&error_code_received);
  if (return_function_state)
  {
    DEBUG_SERIAL.println("[  INFO  ] PRE SET DYNAMIXELS [  SUCCESS ]");
    DEBUG_SERIAL.print("[  INFO  ] RECALCULATED PV[0] : "); DEBUG_SERIAL.println(dxl_prof_vel[0]);
    DEBUG_SERIAL.print("[  INFO  ] RECALCULATED PV[1] : "); DEBUG_SERIAL.println(dxl_prof_vel[1]);
    DEBUG_SERIAL.print("[  INFO  ] RECALCULATED PV[2] : "); DEBUG_SERIAL.println(dxl_prof_vel[2]);
    
    DEBUG_SERIAL.print("[  INFO  ] RECALCULATED PA[0] : "); DEBUG_SERIAL.println(dxl_prof_accel[0]);
    DEBUG_SERIAL.print("[  INFO  ] RECALCULATED PA[1] : "); DEBUG_SERIAL.println(dxl_prof_accel[1]);
    DEBUG_SERIAL.print("[  INFO  ] RECALCULATED PA[2] : "); DEBUG_SERIAL.println(dxl_prof_accel[2]);
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ] PRE SET DYNAMIXELS [  FAILED ]");
  }

  // 3.DYNAMIXELS SET VEL/ACCEL PROFILE
  return_function_state = meta_dxl.syncSetDynamixelsProfVel(dxl_id, sizeof(dxl_id), dxl_prof_vel, sw_data_array_pv,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[    INFO    ] SYNC WRITE PROFILE VELOCITY DYNAMIXELS  [  SUCCESS ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println("[    ERROR   ] SYNC WRITE PROFILE VELOCITY DYNAMIXELS [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  return_function_state = meta_dxl.syncSetDynamixelsProfAccel(dxl_id, sizeof(dxl_id), dxl_prof_accel, sw_data_array_pa,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[    INFO    ] SYNC WRITE PROFILE ACCELERATION DYNAMIXELS  [  SUCCESS ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println("[    ERROR   ] SYNC WRITE PROFILE ACCELERATION DYNAMIXELS [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }

  // 4.DYNAMIXELS SET GOAL POSITION
  unsigned long motor_movement_start = millis();
  
  dxl_goal_position[0] = meta_dxl.convertRadian2DxlPulses(Qdxl_f[0]);
  dxl_goal_position[1] = meta_dxl.convertRadian2DxlPulses(Qdxl_f[1]);
  dxl_goal_position[2] = meta_dxl.convertRadian2DxlPulses(Qdxl_f[2]);
  return_function_state = meta_dxl.syncSetDynamixelsGoalPosition(dxl_id, sizeof(dxl_id), dxl_goal_position, sw_data_array_gp,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[    INFO    ] SYNC WRITE GOAL POSITION DYNAMIXELS [  SUCCESS ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println("[    ERROR   ] SYNC WRITE GOAL POSITION DYNAMIXELS [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }

  return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), turn_off_led, dxl);
  return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), completed_move_indicator, dxl);

  // 5.STEPPER SET GOAL POSITION
  return_function_state =  stp.syncSetStepperGoalPositionVarStep(&currentAbsPos_double, &goalAbsPos_double, &Aexec, &Texec,  &currentDirStatus, &KILL_MOTION, &LIN_SEG_EXISTS, P2P_PROF_STEPS,   &error_code_received);
  if (return_function_state){
    DEBUG_SERIAL.println("[    INFO    ] SYNC WRITE GOAL POSITION STEPPER [  SUCCESS ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println("[    ERROR   ] SYNC WRITE GOAL POSITION STEPPER [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  unsigned long time_duration = millis() - motor_movement_start;                                              // Calculates Stepper Motion Execution Time 
  double time_duration_double = time_duration / 1000.0;
  DEBUG_SERIAL.print("[    INFO    ] TOTAL P2P MOTION DURATION  [sec]:"); DEBUG_SERIAL.println(time_duration_double);
  delay(1000);

  //*/
 // read current position
   stp.read_STP_EEPROM_settings(&currentDirStatus, &currentAbsPos_double, &VelocityLimitStp, &AccelerationLimitStp, &MaxPosLimitStp);

  currentConfiguration[0] = currentAbsPos_double;

  // Sync Read Dynamixels Present Position 
  return_function_state = meta_dxl.syncGetDynamixelsPresentPosition(dxl_id, sizeof(dxl_id), dxl_present_position, sr_data_array_pp,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[    INFO    ] SYNC READ PRESENT POSITION DYNAMIXELS [  SUCCESS ]");
  }
  else
  {
    DEBUG_SERIAL.println("[    ERROR   ] SYNC READ PRESENT POSITION DYNAMIXELS [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]"); DEBUG_SERIAL.println(error_code_received);
  }
  currentConfiguration[1] = dxl_present_position[0];
  currentConfiguration[2] = dxl_present_position[1];
  currentConfiguration[3] = dxl_present_position[2];

  DEBUG_SERIAL.print(" [ INFO ] "); DEBUG_SERIAL.println("CURRENT ROBOT CONFIGURATION:"); 
  for (size_t i = 0; i < nDoF; i++)
  {
    DEBUG_SERIAL.print(" [ JOINT "); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(" ] "); DEBUG_SERIAL.println(currentConfiguration[i],4);
  }
  
  
}

void loop() {

  /*
  delay(2000);
  
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

  // MOVE MOTORS 1
  dxl_goal_position[0] = 200000;
  dxl_goal_position[1] = 50000;
  dxl_goal_position[2] = 150000;
  return_function_state = meta_dxl.syncSetDynamixelsGoalPosition(dxl_id, sizeof(dxl_id), dxl_goal_position, sw_data_array,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[  INFO  ] SYNC WRITE GOAL POSITION DYNAMIXELS [  SUCCESS ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ]SYNC WRITE GOAL POSITION DYNAMIXELS [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }

  return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), turn_off_led, dxl);
  return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), completed_move_indicator, dxl);
  
  delay(1000);

  // MOVE MOTORS 2
  dxl_goal_position[0] = 0;
  dxl_goal_position[1] = 0;
  dxl_goal_position[2] = 0;
  return_function_state = meta_dxl.syncSetDynamixelsGoalPosition(dxl_id, sizeof(dxl_id), dxl_goal_position, sw_data_array,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[  INFO  ] SYNC WRITE GOAL POSITION DYNAMIXELS [  SUCCESS ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ]SYNC WRITE GOAL POSITION DYNAMIXELS [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
  }
  return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), turn_off_led, dxl);
  return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), completed_move_indicator, dxl);
  
  delay(1000);
  
  //SET TORQUE OFF
  return_function_state = meta_dxl.setDynamixelsTorqueOFF(dxl_id, sizeof(dxl_id), dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[  INFO  ] TORQUE OFF DYNAMIXELS [  SUCCESS ]");
  }
  else
  {
    DEBUG_SERIAL.println("[  ERROR  ] TORQUE ON DYNAMIXELS  [  FAILED ]");
  }
  return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), turn_off_led, dxl);
  return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), torque_off_indicator, dxl);
  delay(1000);

  */
}

/*
 *  INTERRUPT FUNCTIONS USED
 */
void changeStepperDirInterrupt1()
{
  currentDirStatus = !currentDirStatus;
  KILL_MOTION = true;
}

void changeStepperDirInterrupt2()
{
  currentDirStatus = !currentDirStatus;
  KILL_MOTION = false;
}
