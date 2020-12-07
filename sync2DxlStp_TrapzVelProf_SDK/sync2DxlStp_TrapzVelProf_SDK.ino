/*******************************************************************************
* Executes simple Sync of 2 Dynamixels and Nema34 Stepper motor using Velocity 
* and Acceleration profiles
* Based on sync2DxlStp_TrapzVelProf.ino which magically doesn't work for Dxl Wb
*******************************************************************************/

/* Author: Stravopodis Nikos */

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

// Used Libraries
#include <DynamixelSDK.h>
//#include <AccelStepper.h>
#include <DynamixelWorkbench.h>
#include <stdlib.h>
#include <stdio.h>

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
#define ADDR_PRO_INDIRECTADDRESS_FOR_READ       196                  // ...previous: 198
#define ADDR_PRO_INDIRECTDATA_FOR_WRITE         634                  // RAM region for the corresponding free Indirect Data for PH54!!!
#define ADDR_PRO_INDIRECTDATA_FOR_READ          648                  // ...previous: 649
#define ADDR_PRO_LED_BLUE                       515
#define ADDR_PRO_LED_GREEN                      514
#define ADDR_PRO_RETURN_DELAY_TIME              9
#define ADDR_PRO_BAUDRATE                       8
#define ADDR_PRO_ID                             7
#define ADDR_PRO_SECONDARY_ID                   12

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
#define LEN_PRO_INDIRECTDATA_FOR_WRITE          14          // 4*3 for pos/vel/accel and 1+1 for leds
#define LEN_PRO_INDIRECTDATA_FOR_READ           6           // 4 for present position and 1+1 for dxl_moving

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyACM0"   // This definition only has a symbolic meaning and does not affect to any functionality
#define CMD_SERIAL                      Serial
#define ESC_ASCII_VALUE                 0x1b

// Control Command Values
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -100000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      100000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

// Stepper Definition
#define stepPin 10
#define dirPin 11
#define enblPin 12
#define ledPin 13                                                                       // the number of the LED pin
#define hallSwitchPin 9                                                                 // the number of the pushbutton pin - This is the manual-homing switch - Here Hall effect sensor!!!
#define motorInterfaceType 8
#define STP_ID    1
#define STP_MOVING_STATUS_THRESHOLD     1

// Numeric Type Definitions
const float pi          = 3.14159265359;
const int spr           = 3200;             // [steps/rev]
const int GEAR_FACTOR   = 40;

// Timing variables
const int ft       = 10;                                                                 // [5~10 @ 200KHz and @ 0.1~0.5 RAMP_FACTOR]
int stpWritePeriod = 50;                                                                 // millis
int dxlWritePeriod = 50;                                                                 // millis
unsigned long time_now_micros = 0;                                                       // Set timer counter
unsigned long time_now_millis = 0;                                                       // Set timer counter
const uint8_t handler_index = 0;
unsigned long curMillis;
unsigned long prevStepMillis = 0;
unsigned long millisBetweenSteps = 1; // milliseconds

// Classes Objects definition
//AccelStepper stepper1(motorInterfaceType, stepPin, dirPin);                               // Stepper Motor, it has STP_ID
//DynamixelWorkbench dxl_wb;                                                              // Dynamixel H54

// PreAllocate Memory for Parameters/Data TxRx
uint8_t dxl_id[] = {DXL1_ID, DXL2_ID};
uint8_t stp1_id = STP_ID;
uint8_t dxl_moving[2];
uint8_t dxl_moving_status[2];                                                             // Dynamixel moving status
// Dynamixel moving status
uint8_t param_goal_position[4];
uint8_t param_accel_limit[4];
uint8_t param_vel_limit[4];
uint8_t param_indirect_data_for_write[LEN_PRO_INDIRECTDATA_FOR_WRITE];
uint8_t param_led_for_write;
uint8_t dxl_ledBLUE_value[2] = {0, 255};                                                  // Dynamixel LED value
uint8_t dxl_ledGREEN_value[2] = {0, 255};                                                 // Dynamixel LED value
uint32_t dxl_present_position[2];                                                         // Present position
uint32_t dxl_present_position1;                                                           // Present position
  
// Set Desired Limit Values for Control Table Items
int dxl_accel_limit = 1000;
int dxl_vel_limit   = 1000;

// Debugging
// int index = 0;
int dxl_comm_result = COMM_TX_FAIL;                                                        // Communication result
bool dxl_addparam_result = false;                                                          // addParam result
bool dxl_getdata_result = false;                                                           // GetParam result
bool return_function_state = false;
uint8_t dxl_error = 0;                                                                     // Dynamixel error 
int32_t dxl1_present_position = 0, dxl2_present_position = 0;                              // Present position


DynamixelWorkbench dxl_wb;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


void setup()
{
  Serial.begin(BAUDRATE);
  while(!Serial);

  Serial.println("Start..");

  /*  
   * I. Initialize Dynamixel SDK 
   */
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWriteGoalPos(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
  dynamixel::GroupSyncWrite groupSyncWriteAccelLim(portHandler, packetHandler, ADDR_PRO_ACCEL_LIMIT, LEN_PRO_ACCEL_LIMIT);
  dynamixel::GroupSyncWrite groupSyncWriteVelLim(portHandler, packetHandler, ADDR_PRO_VEL_LIMIT, LEN_PRO_VEL_LIMIT);
  dynamixel::GroupSyncWrite groupSyncWrite_GP_A_V_LED(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_WRITE, LEN_PRO_INDIRECTDATA_FOR_WRITE);
  dynamixel::GroupSyncWrite groupSyncWrite_GREEN_LED(portHandler, packetHandler, ADDR_PRO_LED_GREEN, LEN_PRO_LED);
  dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE(portHandler, packetHandler, ADDR_PRO_TORQUE_ENABLE, LEN_PRO_TORQUE_ENABLE);
  dynamixel::GroupSyncWrite groupSyncWrite_RETURN_DELAY_TIME(portHandler, packetHandler, ADDR_PRO_RETURN_DELAY_TIME, LEN_PRO_RETURN_DELAY_TIME);
  dynamixel::GroupSyncWrite groupSyncWrite_BAUDRATE(portHandler, packetHandler, ADDR_PRO_BAUDRATE, LEN_PRO_BAUDRATE);
  dynamixel::GroupSyncWrite groupSyncWrite_ID(portHandler, packetHandler, ADDR_PRO_ID, LEN_PRO_ID);
  dynamixel::GroupSyncWrite groupSyncWrite_SECONDARY_ID(portHandler, packetHandler, ADDR_PRO_SECONDARY_ID, LEN_PRO_SECONDARY_ID);

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncReadPresentPos(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  dynamixel::GroupSyncRead groupSyncRead_PP_M_MS(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_INDIRECTDATA_FOR_READ);

  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);
    
  // I. Variables intialization
  const char *log;
  bool result = false;
  uint16_t model_number   =   0;
  long current_stepper_position;
  int buttonState         =   0;   // variable for reading the pushbutton status
  int move_finished       =   1;   // Used to check if move is completed

  /*  
   * II. Stepper Motor is tested
   */
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);
  pinMode(ledPin, OUTPUT);          // initialize the LED pin as an output:
  pinMode(hallSwitchPin, INPUT); // initialize the pushbutton pin as an input: Button open(unpressed) => we read high(1)
  digitalWrite(stepPin, LOW);
  //digitalWrite(ledPin, LOW);
  digitalWrite(enblPin, LOW);
  digitalWrite(dirPin, LOW);

  // Stepper Check // MUST be put inside while loop!
  //InitialStepperCheck(stepper1, stp1_id);
  
  /*  
   * III. Dynamixel Motors are tested
   */

    // III.a. Open port
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
  
    // III.b.1 Set port baudrate
    Serial.println("Setting port BAUDRATE");
    if (portHandler->setBaudRate(BAUDRATE))
    {
      int achievedBaudrate = portHandler->getBaudRate();
      Serial.print("Succeeded to change the baudrate! New Baudrate is: "); Serial.println(achievedBaudrate);
    }
    else
    {
      Serial.print("Failed to change the baudrate!\n");
      return;
    }
/*
    // III.b.2 Disable Dynamixels torque to access EEPROM Area
    uint8_t param_torque_enable = 0;
    Serial.println("Setting Dynamixels TORQUE");
    return_function_state = syncSetTorque(dxl_id, 2, param_torque_enable, groupSyncWrite_TORQUE_ENABLE, packetHandler);
    if (return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }
*/    
    // III.b.3 Set Dynamixels Baudrate
    uint8_t dxl_baudrate_range = 3;                                                                                 // 3 -> 1000000
    Serial.println("Setting Dynamixels BAUDRATE");
    return_function_state = syncSetBaudrate(dxl_id, 2, dxl_baudrate_range, groupSyncWrite_BAUDRATE, packetHandler);
    if (return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }

    // III.b.4 Set Return_Delay_Time for Dynamixels to minimum
    uint8_t param_return_delay_time = 10;
    Serial.println("Setting Dynamixels Return_Delay_Time");
    return_function_state = syncSetReturnDelayTime(dxl_id, 2, param_return_delay_time, groupSyncWrite_RETURN_DELAY_TIME, packetHandler);
    if (return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }
/*
    // III.b.5 Set ID's for Dynamixels
    uint8_t param_id_range = 1;
    Serial.println("Setting Dynamixels ID's");
    return_function_state = syncSetID(dxl_id, 2, param_id_range, groupSyncWrite_ID, packetHandler);
    if (return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }
*/    
    // III.c. Set Velocity and Acceleration Limits in Dynamixels
    Serial.println("Setting Dynamixels Velocity & Acceleration Limit values");
    return_function_state = syncSetVelAccelLimit(dxl_id, 2, dxl_vel_limit,dxl_accel_limit,groupSyncWriteVelLim,groupSyncWriteAccelLim, packetHandler, portHandler);
    if (return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }
  
    // IV. Homing Stepper Motor[Joint1] using AccelStepper 
    // To make new...
    delay(1000);
  
    // V.  Homing Dynamixel Motors[Joint2,3] using Dynamixel SDK
    Serial.println("Started Homing Dynamixels");
    return_function_state == false;
    return_function_state = HomingDynamixelProfilesSDK(dxl_id, 2, groupSyncWrite_GP_A_V_LED, groupSyncWrite_GREEN_LED, groupSyncRead_PP_M_MS, packetHandler, portHandler);
    if (return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }
  
    // VI. Simple P2P Motion StepperNema34 + 2 Dynamixels
    int32_t DxlInitPos = 0;
    int32_t DxlGoalPosition = 100000;
    int32_t DxlVmax = dxl_vel_limit;
    int32_t DxlAmax = dxl_accel_limit/10;
    float StpInitPos = 0;
    //double StpGoalPosition = 0.25*6.28318531;
    float StpGoalPosition = 0.15*6.28312;           // 0.15 for 100000
    float StpVmax = 5.0000;
    float StpAmax = 5.0000;
    uint8_t MotorsIDs[] = {DXL1_ID, DXL2_ID, STP_ID};
    int32_t DxlTrapzProfParams[] = {DxlInitPos, DxlGoalPosition, DxlVmax, DxlAmax};
    float   StpTrapzProfParams[] = {StpInitPos, StpGoalPosition, StpVmax, StpAmax};
    int MotorsIds_size = 3;
    int TrapzProfParams_size =4;
  
    // Simple Sync Stepper-Dynamixel motors for P2P motion:
    Serial.println("Started Sync P2P Dynamixels-Stepper");
    return_function_state = SimpleSyncP2P_TrapzVelProf_SDK(MotorsIDs, MotorsIds_size, DxlTrapzProfParams, StpTrapzProfParams, TrapzProfParams_size, groupSyncWrite_GP_A_V_LED, groupSyncWrite_TORQUE_ENABLE, groupSyncRead_PP_M_MS, packetHandler, portHandler);
    //return_function_state = SimpleSyncP2Pbulk_TrapzVelProf_SDK(MotorsIDs, MotorsIds_size, DxlTrapzProfParams, StpTrapzProfParams, TrapzProfParams_size, groupSyncWrite_GP_A_V_LED, groupSyncWrite_TORQUE_ENABLE, groupBulkRead, packetHandler, portHandler);
    if (return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }

    // Read present position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result); Serial.println("dxl_present_position1 FAILED");
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error); Serial.println("dxl_present_position2 FAILED");
    }
    Serial.print("[ID: "); Serial.print(DXL1_ID); Serial.print("] Present Position: "); Serial.println(dxl_present_position1);
         
    // Disable Torque
    uint8_t param_torque_enable = 0;
    Serial.println("Setting Dynamixels TORQUE");
    return_function_state = syncSetTorque(dxl_id, 2, param_torque_enable, groupSyncWrite_TORQUE_ENABLE, packetHandler);
    if (return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }
    
    // Close port
    portHandler->closePort();
}

void loop()
{
}


// ====================================================== >>>>>>>   Custom Functions Definitions    <<<<<<< ===================================================== //

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


// InitialStepperCheck
/*bool InitialStepperCheck(AccelStepper AccelStepperMotor, uint8_t stp_ID){
  time_now_micros = micros();
  bool  check_stepper_running = AccelStepperMotor.isRunning();
  if (check_stepper_running == true)
  {
    Serial.printf("Stepper Motor: %d is running! \n",stp_ID);
    Serial.printf("Waiting for Stepper Motor: %d to stop to comlete check! \n",stp_ID);
    AccelStepperMotor.stop();
    //delay(1000);
    while(micros() < time_now_micros + stpWritePeriod){
        //wait approx. [stpWritePeriod] μs
    }
  }
  else
  {
    long current_stepper_position = AccelStepperMotor.currentPosition();
    Serial.printf("Stepper Motor: %d is stall at Position: %ld! \n",stp_ID,current_stepper_position);
    Serial.printf("Stepper Motor: %d Check OK. \n",stp_ID);
  }

}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


int getch()
{
  while(1)
  {
    if( CMD_SERIAL.available() > 0 )
    {
      break;
    }
  }

  return CMD_SERIAL.read();
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


int kbhit(void)
{
  return CMD_SERIAL.available();
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


void singleStepVarDelay(unsigned long delayTime) {
  // Custom Stepping Function for Velocity-Acceleration Profiles 
    unsigned long time_now_micros = micros();
    digitalWrite(stepPin, HIGH);
    while(micros() < time_now_micros + delayTime){}                   //wait approx. [μs]
    digitalWrite(stepPin, LOW);
} // END function singleStepVarDelay
    

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
