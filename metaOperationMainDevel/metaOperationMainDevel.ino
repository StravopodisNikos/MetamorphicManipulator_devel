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
 // Core Libraries for Robot Motors Driving
#include "DynamixelProPlusOvidiusShield.h"
#include "CustomStepperOvidiusShield.h"
//  Used Libraries
#include "Arduino.h"                                // Main Arduino library
#include <Dynamixel2Arduino.h>
// Auxiliary Libraries
#include <definitions.h>                              // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include <motorIDs.h>                                 // Includes motor IDs as set using Dynamixel Wizard
#include <contolTableItems_LimitValues.h>             // Limit values for control table controlTableItems_LimitValues
#include <utility/StepperMotorSettings.h>             // Includes Stepper Motor/Driver pin StepperMotorSettings
#include <task_definitions.h>                         // Includes task points for execution
#include <led_indicators.h>
//#include <ovidius_robot_controller_eeprom_addresses.h>  // Header file for all eeprom addresses used!
#include <avr/pgmspace.h>
#include "OvidiusSensors.h"
#include <utility/OvidiusSensors_config.h>

/* 
 *  CONFIGURE DEBUG_SERIAL PORT COMMUNICATION
 */
#if defined(ARDUINO_AVR_UNO) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(11, 8); // DYNAMIXELShield UART RX/TX
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

/* 
 *  NAMESPACE OF USED CLASSES
 */
using namespace std;
using namespace ControlTableItem;

/*
 * USER DEBUG_SERIAL INPUT VARIABLES
 */
byte YES_b     = 1;
byte NO_b      = 0;
byte p2pcsp_b  = 2;
byte trajcsp_b = 3;
byte home_b    = 4;
// Auxiliary variables for DEBUG_SERIAL inputs
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
int32_t  dxl_present_velocity[sizeof(dxl_id)];
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
double VelocityLimitStp;
double AccelerationLimitStp;
double MaxPosLimitStp;

double StpInitPos;                          // Variables used to initialilize StpTrapzProfParams[]
double StpGoalPosition;
double StpVmax;
double StpAmax;

double goalAbsPos_double;

/* 
 *  SYNC WRITE structure objects for sending Dynamixel data - Data type are defined in DynamixelProPlusOvidiusShield.h
 */
sw_data_t_gp sw_data_array_gp[DXL_MOTORS]; // this type of array should be passed to the function
sw_data_t_pv sw_data_array_pv[DXL_MOTORS]; // this type of array should be passed to the function
sw_data_t_pa sw_data_array_pa[DXL_MOTORS]; // this type of array should be passed to the function

/* 
 *  SYNC READ structure objects for receiving Dynamixel data - Data type are defined in DynamixelProPlusOvidiusShield.h
 */
sr_data_t_pp sr_data_array_pp[DXL_MOTORS]; // this type of array should be passed to the function
sr_data_t_pv sr_data_array_pv[DXL_MOTORS]; // this type of array should be passed to the function

/*
 *  Create struct objects for communicating between CustomStepper+DynamixelOvidius libraries
 */
DXL_PP_PACKET dxl_pp_packet, *PTR_2_dxl_pp_packet;
DXL_PV_PACKET dxl_pv_packet, *PTR_2_dxl_pv_packet; 

/* 
 *  Create object for handling motors
 */
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
DynamixelProPlusOvidiusShield meta_dxl(dxl_id), *PTR_2_meta_dxl; // Object of custom class to access custom functions for Ovidius manipulator specific 
CustomStepperOvidiusShield stp(STP1_ID, STEP_Pin, DIR_Pin, ENABLE_Pin, HOME_TRIGGER_SWITCH, HALL_SWITCH_PIN2, HALL_SWITCH_PIN3, RED_LED_PIN, GREEN_LED_PIN, BLUE_LED_PIN, SPR1, GEAR_FACTOR_PLANETARY, FT_CLOSED_LOOP);

/* 
 *  Create object for handling sensors+tools
 */
Servo OvidiusGripperServo, *ptr2OvidiusGripperServo;
//HX711 ForceSensorX, *ptr2ForceSensorX;
HX711 ForceSensorHX711[3];

// Force Sensor Globals
debug_error_type sensor_error;
// Gripper Tool Globals
int offset_analog_reading;                          // Returned from setupGripper(), can be executed at setup
unsigned long grasp_force_limit_newton = MIN_GRASP_FORCE; // Max value is 10[N], must be saved at EEPROM

tools::gripper_states GripperCurrentState;
sensors::force_sensor_states ForceCurrentState;

tools::gripper OvidiusGripper(GRIPPER_SERVO_PIN,FSR_ANAL_PIN1);
sensors::force3axis ForceSensor[num_FORCE_SENSORS] = {
    sensors::force3axis(DOUT_PIN_X, SCK_PIN_X),   //ForceSensor[0] -> ForceSensorX
    sensors::force3axis(DOUT_PIN_Y, SCK_PIN_Y),   //ForceSensor[1] -> ForceSensorY
    sensors::force3axis(DOUT_PIN_Z, SCK_PIN_Z),   //ForceSensor[2] -> ForceSensorZ
};

/*
 * ADAFRUIT IMU sensor 
 */
sensors::imu9dof SingleIMUSensor(GYRO_RANGE_250DPS, ACCEL_RANGE_2G, FILTER_UPDATE_RATE_HZ, 0x0021002C, 0x8700A, 0x8700B), *IMUSensor;
Adafruit_FXAS21002C gyro;
Adafruit_FXOS8700 accelmag;
Adafruit_Madgwick filter;  // added 2/3/21 - original adafruit
//SF fusion;           // removed 2/3/21 - will implement original adafruit class
Adafruit_Sensor_Calibration_EEPROM adafr_calibr;  
sensors_event_t gyro_event, accel_event, magnet_event;
float pitch, roll, yaw;
sensors::imu_packet IMU_DATA_PKG, *PTR_2_imu_packet;
sensors::imu_sensor_states ImuCurrentState;

/*
 * SETUP
 */
void setup() {
  // User can select which functions to be executed and finally accept "finish seup"(enter "Y") and proceed to main loop() execution
  
  DEBUG_SERIAL.begin(SERIAL_BAUDRATE);            // Serial BAUDRATE->57600
  while(!DEBUG_SERIAL);
  dxl.begin(DXL_BAUDRATE2);                       // UART BAUDRATE->1000000
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // IF OPENCR USED ->
  /*
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN,HIGH);
  delay(1000);
  DEBUG_SERIAL.println(" [ SETUP ] ENABLED DXL POWER PIN IN OPENCR... ");
  */
  // IF OPENCR USED <-
  
  // SET PINMODES
  pinMode(HOME_TRIGGER_SWITCH, INPUT_PULLUP);   // HOME TRIGGER SWITCH
  pinMode(HALL_SWITCH_PIN2, INPUT_PULLUP);             // LIMIT HALL SENSOR2 
  pinMode(HALL_SWITCH_PIN3, INPUT_PULLUP);             // LIMIT HALL SENSOR3
  
  // SET EXTERNAL INTERRUPTS
  // IF OPENCR USED ->
  /*
  attachInterrupt(digitalPinToInterrupt(EXTI_Pin2), changeStepperDirInterrupt1, RISING);
  attachInterrupt(digitalPinToInterrupt(EXTI_Pin3), changeStepperDirInterrupt1, RISING);
  */  
  // IF OPENCR USED <-  
  // IF DXL SHIELD USED ->
  attachInterrupt(digitalPinToInterrupt(HALL_SWITCH_PIN2), changeStepperDirInterrupt1, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_SWITCH_PIN3), changeStepperDirInterrupt1, RISING);
  // IF DXL SHIELD USED <-
  
  delay(1000);
  DEBUG_SERIAL.println(F(" [ SETUP ] ENTERING OVIDIUS ROBOT SETUP... "));
  delay(1000);

  // SETUP DEFAULT ACTIONS
  DEBUG_SERIAL.print(F(" [ SETUP ] ")); DEBUG_SERIAL.println(F("PINGING CONNECTED DYNAMIXELS PRO+"));
  ping_motors();
  
  DEBUG_SERIAL.print(F(" [ SETUP ] ")); DEBUG_SERIAL.println(F("EXTRACTING GLOBAL VARIABLES FROM EEPROM "));
  stp.read_STP_EEPROM_settings(&currentDirStatus, &currentAbsPos_double, &VelocityLimitStp, &AccelerationLimitStp, &MaxPosLimitStp); // Initialize global Stepper Variables from EEPROM Memory
  print_stp_eeprom();

  // initialize data packets for libraries communication
  DEBUG_SERIAL.print(F(" [ SETUP ] ")); DEBUG_SERIAL.println(F("BUILDING DATA PACKETS FOR LIBRARIES DATA EXCHANGE"));
  setup_libraries_packets();
  
  DEBUG_SERIAL.print(F(" [ SETUP ] ")); DEBUG_SERIAL.println(F("EXTRACTING CURRENT CONFIGURATION"));
  ACCESS_EEPROM = true;
  read_current_configuration(ACCESS_EEPROM);
      
  // SETUP USER-SELECTED ACTIONS 
  bool stop_setup = false; // flag for setup menu robot loop

  while(!stop_setup) //wh1
  {
    /*
     * I. SLOW MOTOR HOMING
     */
    DEBUG_SERIAL.println(F("[ SETUP ] HOME MOTORS SLOW?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) 
    {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SLOW HOMING MOTORS"));

      slow_home_motors();
    }
  
    /*
     * II. FAST MOTOR HOMING
     */
    DEBUG_SERIAL.println(F("[ SETUP ] HOME MOTORS FAST?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.print(user_input);
  
    if( user_input == YES_b )  
    {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("FAST HOMING MOTORS"));
      fast_home_motors();
    }
 
    /*
     * III. DISPLAY EEPROM
     */
    DEBUG_SERIAL.println("[ SETUP ] DISPLAY EEPROM?");
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print("[ USER INPUT ]"); DEBUG_SERIAL.print("   ->   "); DEBUG_SERIAL.print(user_input);
  
    if( user_input == YES_b ) 
    {
      print_stp_eeprom();
    }

    /*
     * IV. SETUP EEPROM
     */
     // HERE I WRITE DEFAULT VALUES FOR MOTORS/SENSORS/TOOLS
    DEBUG_SERIAL.println(F("[ SETUP ] SETUP EEPROM?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b  ) //start->if1
    {
      setup_ovidius_eeprom();      
    }
    
    /*
     * V. TEST GRIPPER
     */
    DEBUG_SERIAL.println(F("[ SETUP ] SETUP GRIPPER?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) //start->if1
    {
      setup_gripper();
    }

    /*
     * VI. TEST FORCE SENSOR
     */
    DEBUG_SERIAL.println(F("[ SETUP ] SETUP FORCE SENSOR?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) //start->if1
    {
      setup_force_sensor();
    }
        
    /*
     * FINISH SETUP
     */
    DEBUG_SERIAL.println(F("[ SETUP ] EXIT SETUP?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) //start->if1
    {
      stop_setup = true;
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SETUP FINISHED"));
    }
    else
    {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SETUP MENU WILL REPEAT"));
    }
    
  } // wh1

} // END SETUP

/*
 * MAIN LOOP
 */
void loop() {
  /*
   * I. User must specify operating
   */
  DEBUG_SERIAL.println(F("Set ROBOT OPERATION MODE: FORMAT 'mode':"));
  DEBUG_SERIAL.parseInt();
  while (DEBUG_SERIAL.available() == 0) {};
  user_input = DEBUG_SERIAL.parseInt();
  DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.print(user_input);

  /*
   * II. <p2pcsp>
   */
   if( user_input == p2pcsp_b  ) //start->if1
   {
   
   while( (!END_P2PCSP) ) //start->wh1
   {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("BEGIN CONFIGURATION SPACE P2P EXECUTION..."));

      // II.1 Display to user the current configuration
      ACCESS_EEPROM = false;
      read_current_configuration(ACCESS_EEPROM); 
      
      // II.2 Specify desired configuration
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SPECIFY TASK POINT INDEX"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      p2p_index = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(p2p_index);
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SELECTED CONFIGURATION SPACE POINT:")); 
      for (size_t i = 0; i < nDoF; i++)
      {
        DEBUG_SERIAL.print(F(" [ JOINT ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(p2p_list[p2p_index][i],4);

        desiredConfiguration[i] = p2p_list[p2p_index][i];
      }
      
      
      // II.3 SIMPE P2P EXECUTION FUNCTION TO BE PLACED HERE!
      p2pcsp_Texec = p2p_dur[p2p_index];
      p2pcsp2(currentConfiguration, desiredConfiguration, p2pcsp_Texec);
      
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("P2P EXECUTED"));
      
      /*
       * FINISH p2pcsp
       */
      DEBUG_SERIAL.println(F("[ INFO ] EXIT <p2pcsp>?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        END_P2PCSP    = true;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<p2pcsp> FINISHED"));
      }
      else
      {
        END_P2PCSP    = false;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<p2pcsp> WILL REPEAT"));
      } 

   }//end->wh1
   
   }//end->if1

  /*
   * III. <trajcsp>
   */
   if( user_input == trajcsp_b ) //start->if2
   {
   
   while( (!END_TRAJCSP) ) //start->wh2
   {
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("BEGIN CONFIGURATION SPACE TRAJECTORY EXECUTION..."));

      // SIMPE TRAJ EXECUTION FUNCTION TO BE PLACED HERE!
      DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("TRAJECTORY EXECUTED"));
      
      /*
       * FINISH trajcsp
       */
      DEBUG_SERIAL.println(F("[ INFO ] EXIT <trajcsp>?"));
      DEBUG_SERIAL.parseInt();
      while (DEBUG_SERIAL.available() == 0) {};
      user_input = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
      
      if( user_input == YES_b ) //start->if1
      {
        END_TRAJCSP    = true;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<trajcsp> FINISHED"));
      }
      else
      {
        END_TRAJCSP    = false;
        DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("<trajcsp> WILL REPEAT"));
      } 

   }//end->wh2
   
   }//end->if2  

  /*
   *  RESET FLAGS
   */
    END_P2PCSP    = false;
    END_TRAJCSP   = false;
    END_HOME      = false;
    END_FORCE     = false;
    END_ACCEL     = false;
      
  /*
   *  SAVE EEPROM AFTER MAIN OPERATION
   */
   /* UNDER DEVEL - MUST DRAW DRAFT SKETCH FOR CURRENT ABS POS!
    DEBUG_SERIAL.println(F("[ LOOP ] SAVE CHANGES TO EEPROM?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();
    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b  ) //start->if1
    {
      loop_ovidius_eeprom();      
    }
   */
}// END LOOP 

/*
 *  INTERRUPT FUNCTIONS USED
 */
void changeStepperDirInterrupt1()
{
  currentDirStatus = !currentDirStatus;
  KILL_MOTION = true;
}
