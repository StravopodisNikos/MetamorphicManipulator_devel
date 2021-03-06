

/*   *********************************************************************************************
 *   Executes tests for Metamorphic Manipulator with 1 Nema34 Stepper + 3 Dynamixels
 *   using Trapezoidal Velocity profile
 *   Parent File: testing_AccelStepper/test2_StepperWithDynamixel/sync2DxlStp_TrapzVelProf_SDK.ino
 *   ********************************************************************************************* 
 */

/* 
 *  Author: Stravopodis Nikos
 */

// Used Libraries
#include "Arduino.h"
#include <DynamixelSDK.h>
#include "CustomStepperMetamorphicManipulator.h"                    // Uses Library for Stepper Motor Driving
#include "OpenCR.h"
#include "stdlib.h"
#include "stdio.h"
#include <Vector.h>
#include <Streaming.h>

// Control Table Used Items Address
#define ADDR_PRO_TORQUE_ENABLE                  512                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION                  564
#define ADDR_PRO_GOAL_VELOCITY                  552
#define ADDR_PRO_PROF_ACCEL                     556
#define ADDR_PRO_PROF_VEL                       560
#define ADDR_PRO_PRESENT_VELOCITY               576
#define ADDR_PRO_MOVING                         570
#define ADDR_PRO_MOVING_STATUS                  571
#define ADDR_PRO_PRESENT_POSITION               580
#define ADDR_PRO_ACCEL_LIMIT                    40
#define ADDR_PRO_VEL_LIMIT                      44
#define ADDR_PRO_MAX_POS_LIMIT                  48
#define ADDR_PRO_MIN_POS_LIMIT                  52
#define ADDR_PRO_INDIRECTADDRESS_FOR_WRITE      168                  // EEPROM region for free Indirect Address for PH54!!!
#define ADDR_PRO_INDIRECTADDRESS_FOR_READ       196                  // ...
#define ADDR_PRO_INDIRECTDATA_FOR_WRITE         634                  // RAM region for the corresponding free Indirect Data for PH54!!!
#define ADDR_PRO_INDIRECTDATA_FOR_READ          648                  // ...
#define ADDR_PRO_LED_BLUE                       515
#define ADDR_PRO_LED_GREEN                      514
#define ADDR_PRO_RETURN_DELAY_TIME              9
#define ADDR_PRO_BAUDRATE                       8
#define ADDR_PRO_ID                             7
#define ADDR_PRO_SECONDARY_ID                   12
#define ADDR_PRO_FIRMWARE_VER                   6
#define ADDR_PRO_MODEL_INFO                     2

// Data Byte Length
#define LEN_PRO_TORQUE_ENABLE                   1
#define LEN_PRO_GOAL_POSITION                   4
#define LEN_PRO_GOAL_VELOCITY                   4
#define LEN_PRO_PROF_ACCEL                      4
#define LEN_PRO_PROF_VEL                        4
#define LEN_PRO_PRESENT_VELOCITY                4
#define LEN_PRO_PRESENT_POSITION                4
#define LEN_PRO_ACCEL_LIMIT                     4
#define LEN_PRO_VEL_LIMIT                       4
#define LEN_PRO_MAX_POS_LIMIT                   4
#define LEN_PRO_MAX_POS_LIMIT                   4
#define LEN_PRO_MOVING                          1
#define LEN_PRO_MOVING_STATUS                   1 
#define LEN_PRO_LED                             1
#define LEN_PRO_RETURN_DELAY_TIME               1
#define LEN_PRO_BAUDRATE                        1
#define LEN_PRO_ID                              1
#define LEN_PRO_SECONDARY_ID                    1
#define LEN_PRO_FIRMWARE_VER                    1
#define LEN_PRO_MODEL_INFO                      4
#define LEN_PRO_INDIRECTDATA_FOR_WRITE          14          // 4*3 for pos/vel/accel and 1+1 for leds
#define LEN_PRO_INDIRECTDATA_FOR_READ           6           // 4 for present position and 1+1 for dxl_moving

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL3_ID                         3                   // Dynamixel#3 ID: 3
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyACM0"      // This definition only has a symbolic meaning and does not affect to any functionality
#define CMD_SERIAL                      Serial
#define ESC_ASCII_VALUE                 0x1b

// Control Command Values
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE     -150000              // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

// Stepper Motor and Driver Settings
// For Nema34 motor used for Active Joint1
#define stepPin                         10
#define dirPin                          11
#define enblPin                         12
#define ledPin                          13                  // the number of the LED pin
#define hallSwitchPin                   9                   // the number of the pushbutton pin - This is the Hall effect-homing switch
#define STP_ID                          1
#define STP_MOVING_STATUS_THRESHOLD     1
int spr               = 3200;                               // Speps per Revolution from Dip Switch Driver Configuration [steps/rev]
int GEAR_FACTOR       = 40;                                 // Gear Reduction used for Joint 1
int ft                = 200000;                             // Stepper Driver Frequency [1/micros]->[*10^-6 * 1/sec]

// Timing variables
unsigned long time_now_micros = 0;                           // Set timer counter
unsigned long time_now_millis = 0;                           // ...
const uint8_t handler_index   = 0;
unsigned long curMillis;
unsigned long prevStepMillis  = 0;

// PreAllocate Memory for Parameters/Data TxRx
uint8_t dxl_id[] = {DXL1_ID, DXL2_ID, DXL3_ID};
int dxl_motors_used = 3;
int id_count;
uint8_t stp1_id = STP_ID;
uint8_t dxl_moving[2];
uint8_t dxl_moving_status[2];                                                             // Dynamixel moving status
// Dynamixel moving status
uint8_t param_goal_position[4];
uint8_t param_accel_limit[4];
uint8_t param_vel_limit[4];
uint8_t param_indirect_data_for_write[LEN_PRO_INDIRECTDATA_FOR_WRITE];
uint8_t param_led_for_write;
uint8_t param_torque_enable;
uint8_t dxl_ledBLUE_value[2] = {0, 255};                                                  // Dynamixel LED value
uint8_t dxl_ledGREEN_value[2] = {0, 255};                                                 // Dynamixel LED value
uint32_t dxl_present_position[2];                                                         // Present position
uint32_t dxl_present_position1;                                                           // Present position
  
// Set Desired Limit Values for Control Table Items
int dxl_accel_limit = 10000;
int dxl_vel_limit   = 2000;

// Debugging
// int index = 0;
int dxl_comm_result = COMM_TX_FAIL;                                                        // Communication result
bool dxl_addparam_result = false;                                                          // addParam result
bool dxl_getdata_result = false;                                                           // GetParam result
//bool return_function_state = false;
uint8_t dxl_error = 0;                                                                     // Dynamixel error
uint16_t dxl_model_number[2];                                                              // Dynamixel model number
int32_t dxl1_present_position = 0, dxl2_present_position = 0;                              // Present position
//double hRel;

void setup() {
    Serial.begin(BAUDRATE);
    while(!Serial);
    Serial.println("Start..");

    /*  
     * I. Initialize DYNAMIXEL SDK
     */

    // Initialize PortHandler instance
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Initialize GroupSyncWrite instances

    // Initialize Groupsyncread instances

    // Initialize GroupBulkRead instances

    /*  
     * II. Initialize Stepper Motor
     */
    CustomStepperMetamorphicManipulator Joint1Stepper(STP_ID, stepPin, dirPin, enblPin, ledPin, hallSwitchPin, spr, GEAR_FACTOR, ft);
    
    /*  
     * III. Begin Communication Testing
     */
    // III.a.1. Open port
    Serial.println("Opening port");
    if (portHandler->openPort())
    {
      Serial.print("Succeeded to open the port!\n");
    }
    else
    {
      Serial.print("Failed to open the port!\n");
      return;
    }

    // III.a.2 Set port baudrate
    Serial.println("Setting port BAUDRATE");
    if (portHandler->setBaudRate(BAUDRATE))
    {
      int achievedBaudrate = portHandler->getBaudRate();
      Serial.print("Succeeded to change the baudrate! New Baudrate is: "); Serial.println(achievedBaudrate);
    }
    else
    {
      Serial.print("Failed to change the baudrate!\n");
    }
    
    // III.a.3 Ping Dynamixels
    Serial.println("Pinging Dynamixels");
    return_function_state = pingDynamixels(dxl_id, dxl_motors_used, packetHandler, portHandler);
    if(return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }

    /*  
     * IV. Begin Motion Testing
     */
    // IV.a.1 Stepper Execute Trapezoidal Velocity profile => TESTED OK
    /*
    double setStepperTestTime  = 2.0;
    double setStepperGoalAngle = 1.5708;
    double *StepperTestAssignedProperties;
    
    StepperTestAssignedProperties = Joint1Stepper.returnTrajAssignedDurationProperties(setStepperTestTime, setStepperGoalAngle);
    
    return_function_state = Joint1Stepper.syncTrajPreassignedAccelVel(StepperTestAssignedProperties);
    if(return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }
*/
    // IV.a.2 Stepper Homing and Goal Position with Assigned Duration

    // Homing
/*
    return_function_state = Joint1Stepper.setStepperHomePosition();
    if(return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }

    Serial.print("HOME = "); Serial.println(Joint1Stepper.currentAbsPos);
*/    
    // Goal Position1 with Assigned Duration

    Joint1Stepper.currentAbsPos = 0;
    Joint1Stepper.currentMoveRel = 0;
    Joint1Stepper.currentDirStatus = LOW;
    
    const int ELEMENT_COUNT_MAX1 = 5;
    const int ELEMENT_COUNT_MAX2 = 4;
    
    double setStepperTestTime  = 2.0;
    double setStepperGoalAngle = 1.5708;

    double hRel_Stepper = Joint1Stepper.setStepperGoalPositionAssignedDuration(setStepperTestTime, setStepperGoalAngle);
    Serial.print("hRel_Stepper= "); Serial.println(hRel_Stepper,6);
    
    //double storage_array_for_returnTraj[ELEMENT_COUNT_MAX1];
    //Vector<double> vector_for_returnTraj;
    //vector_for_returnTraj.setStorage(storage_array_for_returnTraj);

    Vector<double> TrajAssignedDuration1 = Joint1Stepper.returnTrajAssignedDurationProperties(setStepperTestTime, hRel_Stepper);
    //TrajAssignedDuration = Joint1Stepper.returnTrajAssignedDurationProperties(setStepperTestTime, hRel_Stepper);
    
    Serial << "vector_for_returnTraj.max_size(): " << TrajAssignedDuration1.max_size() << endl;
    Serial << "vector_for_returnTraj:" << TrajAssignedDuration1 << endl;

    Serial.print("h2="); Serial.println(TrajAssignedDuration1[0],6);
    
    bool returned_segment_exists = Joint1Stepper.segmentExists_TrapzVelProfile(TrajAssignedDuration1);

    unsigned long storage_array_for_returnTrapz[ELEMENT_COUNT_MAX2];
    Vector<unsigned long> vector_for_returnTrapz(storage_array_for_returnTrapz);
    
    vector_for_returnTrapz = Joint1Stepper.returnTrapzVelProfileSteps(TrajAssignedDuration1, returned_segment_exists);

    double initial_step_delay = Joint1Stepper.calculateInitialStepDelay(TrajAssignedDuration1);
         
    return_function_state = Joint1Stepper.executeStepperTrapzProfile(returned_segment_exists, vector_for_returnTrapz, setStepperTestTime, initial_step_delay);
    if(return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }

    Serial.print("GOAL1 = "); Serial.println(Joint1Stepper.currentAbsPos);
    Serial.print("MOVED FOR GOAL1 = "); Serial.println(Joint1Stepper.currentMoveRel);
    
    delay(1000);

    /*
    // Goal Position2 with Assigned Duration
    setStepperTestTime  = 5.0;
    setStepperGoalAngle = 0.7854;
    
    Joint1Stepper.hRel = Joint1Stepper.setStepperGoalPositionAssignedDuration(setStepperTestTime, setStepperGoalAngle);
    Serial.print("hRel2="); Serial.println(Joint1Stepper.hRel,6);
    
    StepperTestAssignedProperties2 = Joint1Stepper.returnTrajAssignedDurationProperties(setStepperTestTime, Joint1Stepper.hRel);
    Serial.print("h2="); Serial.println(StepperTestAssignedProperties2[0],6);
    Serial.print("Texec2="); Serial.println(StepperTestAssignedProperties2[1],6);
    Serial.print("Ta2="); Serial.println(StepperTestAssignedProperties2[2],6);
    Serial.print("Vmax2="); Serial.println(StepperTestAssignedProperties2[3],6);
    Serial.print("Amax2="); Serial.println(StepperTestAssignedProperties2[4],6);
    return_function_state = Joint1Stepper.syncTrajPreassignedAccelVel(StepperTestAssignedProperties2);
    
    if(return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }

    Serial.print("GOAL2 = "); Serial.println(Joint1Stepper.currentAbsPos);
    Serial.print("MOVED FOR GOAL2 = "); Serial.println(Joint1Stepper.currentMoveRel);
    */
} // END SETUP

void loop() {

} // END LOOP
