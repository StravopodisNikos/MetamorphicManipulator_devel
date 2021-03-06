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
#include <AccelStepper.h>
//#include <DynamixelWorkbench.h>
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
#define ADDR_PRO_INDIRECTADDRESS_FOR_READ       198                  // ...
#define ADDR_PRO_INDIRECTDATA_FOR_WRITE         634                  // RAM region for the corresponding free Indirect Data for PH54!!!
#define ADDR_PRO_INDIRECTDATA_FOR_READ           649                 // ...
#define ADDR_PRO_LED_BLUE                       515
#define ADDR_PRO_LED_GREEN                      514
#define LEN_PRO_INDIRECTDATA_FOR_WRITE          14                   // 4*3 for pos/vel/accel and 1+1 for leds
#define LEN_PRO_INDIRECTDATA_FOR_READ           6                    // 4 for present position and 1+1 for dxl_moving

// Data Byte Length
#define LEN_PRO_TORQUE_ENABLE           4
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_GOAL_VELOCITY           4
#define LEN_PRO_PROF_ACCEL              4
#define LEN_PRO_PROF_VEL                4
#define LEN_PRO_PRESENT_VELOCITY        4
#define LEN_PRO_PRESENT_POSITION        4
#define LEN_PRO_ACCEL_LIMIT             4
#define LEN_PRO_VEL_LIMIT               4
#define LEN_PRO_MAX_POS_LIMIT           4
#define LEN_PRO_MAX_POS_LIMIT           4
#define LEN_PRO_MOVING                  1
#define LEN_PRO_MOVING_STATUS           1 
#define LEN_PRO_LED                     1
  
// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        57600
#define DEVICENAME                      "OpenCR_DXL_Port"   // This definition only has a symbolic meaning and does not affect to any functionality
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
const int ft       = 5;                // [1 MHz]
int stpWritePeriod = 50;                                                                 // millis
int dxlWritePeriod = 50;                                                                 // millis
unsigned long time_now_micros = 0;                                                       // Set timer counter
unsigned long time_now_millis = 0;                                                       // Set timer counter
const uint8_t handler_index = 0;
unsigned long curMillis;
unsigned long prevStepMillis = 0;
unsigned long millisBetweenSteps = 1; // milliseconds

// Classes Objects definition
AccelStepper stepper1(motorInterfaceType, stepPin, dirPin);                               // Stepper Motor, it has STP_ID
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
int32_t dxl_present_position[2];                                                          // Present position
  
// Set Desired Limit Values for Control Table Items
int dxl_accel_limit = 1000;
int dxl_vel_limit   = 1000;

// Debugging
// int index = 0;
int dxl_comm_result = COMM_TX_FAIL;                                                        // Communication result
bool dxl_addparam_result = false;                                                          // addParam result
bool dxl_getdata_result = false;                                                           // GetParam result
uint8_t dxl_error = 0;                                                                     // Dynamixel error 
int32_t dxl1_present_position = 0, dxl2_present_position = 0;                              // Present position



// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


void setup()
{
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Start..");

  /*  
   * I. Initialize Dynamixel SDK 
   */
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
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
  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncReadPresentPos(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  dynamixel::GroupSyncRead groupSyncRead_PP_M_MS(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_INDIRECTDATA_FOR_READ);
  
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
  InitialStepperCheck(stepper1, stp1_id);
  
  /*  
   * III. Dynamixel Motors are tested
   */

  // III.a. Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    return;
  }

  // III.b. Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    return;
  }

  // III.c. Set Limit values for Velocity/Acceleration to Dynamixels
    // III.c.1. Allocate Acceleration Limit value into byte array
    param_accel_limit[0] = DXL_LOBYTE(DXL_LOWORD(dxl_accel_limit));
    param_accel_limit[1] = DXL_HIBYTE(DXL_LOWORD(dxl_accel_limit));
    param_accel_limit[2] = DXL_LOBYTE(DXL_HIWORD(dxl_accel_limit));
    param_accel_limit[3] = DXL_HIBYTE(DXL_HIWORD(dxl_accel_limit));
    // III.c.2.Add Dynamixel#1 goal acceleration value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAccelLim.addParam(DXL1_ID, param_accel_limit);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] groupSyncWriteAccelLim addparam failed");
      return;
    }
    else{Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] groupSyncWriteAccelLim1 addparam successful"); }


    // III.c.3.Add Dynamixel#2 goal acceleration value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAccelLim.addParam(DXL2_ID, param_accel_limit);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("] groupSyncWrite addparam failed");
      return;
    }
    else{Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("] groupSyncWriteAccelLim2 addparam successful"); }

    // III.c.4.Syncwrite goal acceleration
    dxl_comm_result = groupSyncWriteAccelLim.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) Serial.print(packetHandler->getTxRxResult(dxl_comm_result));

    // III.c.5.Clear syncwrite parameter storage
    groupSyncWriteAccelLim.clearParam();
    
    // III.c.6. Allocate Velocity Limit value into byte array
    param_vel_limit[0] = DXL_LOBYTE(DXL_LOWORD(dxl_vel_limit));
    param_vel_limit[1] = DXL_HIBYTE(DXL_LOWORD(dxl_vel_limit));
    param_vel_limit[2] = DXL_LOBYTE(DXL_HIWORD(dxl_vel_limit));
    param_vel_limit[3] = DXL_HIBYTE(DXL_HIWORD(dxl_vel_limit));
    // III.c.7.Add Dynamixel#1 goal velocity value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteVelLim.addParam(DXL1_ID, param_vel_limit);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] groupSyncWriteVelLim addparam failed");
      return;
    }
    else{Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] groupSyncWriteVelLim1 addparam successful"); }

    // III.c.8.Add Dynamixel#2 goal velocity value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteVelLim.addParam(DXL2_ID, param_vel_limit);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("] groupSyncWrite addparam failed");
      return;
    }
    else{Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("] groupSyncWriteVelLim2 addparam successful"); }

    // III.c.9.Syncwrite goal velocity
    dxl_comm_result = groupSyncWriteVelLim.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) Serial.print(packetHandler->getTxRxResult(dxl_comm_result));

    // III.c.10.Clear syncwrite parameter storage
    groupSyncWriteVelLim.clearParam();

  // III.d. Enable Dynamixel#1 Torque -> Gets Error!!!
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] TorqueOn - Error in getTxRxResult1 ");
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
    Serial.println("Error in getRxPacketError");
  }
  else
  {
    Serial.print("Dynamixel#1 has been successfully connected \n");
  }

  // III.d. Enable Dynamixel#2 Torque -> Gets Error!!!
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:"); Serial.print(DXL2_ID);  Serial.println("] TorqueOn - Error in getTxRxResult");
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
    Serial.println("Error in getRxPacketError");
  }
  else
  {
    Serial.print("Dynamixel#2 has been successfully connected \n");
  }

  // IV. Homing Stepper Motor[Joint1] using AccelStepper 
  //HomingStepper(stepper1, stp1_id);
  //delay(1000);

  // V.  Homing Dynamixel Motors[Joint2,3] using Dynamixel SDK
  //HomingDynamixelSDK(groupSyncWriteGoalPos,packetHandler,portHandler); 

  // VI.  Homing Dynamixel Motors[Joint2,3] using Dynamixel SDK
  Serial.println("Started Homing Dynamixels");
  HomingDynamixelProfilesSDK(groupSyncWrite_GP_A_V_LED, groupSyncWrite_GREEN_LED, groupSyncRead_PP_M_MS, packetHandler, portHandler);
  Serial.println("Finished Homing Dynamixels");
  delay(1000);

  // VII. SImple P2P Motion StepperNema34 + 2 Dynamixels
  int32_t DxlInitPos = 0;
  int32_t DxlGoalPosition = 100000;
  int32_t DxlVmax = dxl_vel_limit;
  int32_t DxlAmax = dxl_accel_limit/10;
  float StpInitPos = 0;
  //double StpGoalPosition = 0.25*6.28318531;
  float StpGoalPosition = 0.15*6.28312;           // 0.15 for 151000
  float StpVmax = 5.0000;
  float StpAmax = 5.0000;
  uint8_t MotorsIDs[] = {DXL1_ID, DXL2_ID, STP_ID};
  int32_t DxlTrapzProfParams[] = {DxlInitPos, DxlGoalPosition, DxlVmax, DxlAmax};
  float   StpTrapzProfParams[] = {StpInitPos, StpGoalPosition, StpVmax, StpAmax};
  int MotorsIds_size = 3;
  int TrapzProfParams_size =4;

  // Simple Sync Stepper-Dynamixel motors for P2P motion:
  Serial.println("Started Sync P2P Dynamixels-Stepper");
  SimpleSyncP2P_TrapzVelProf_SDK(MotorsIDs, MotorsIds_size, DxlTrapzProfParams, StpTrapzProfParams, TrapzProfParams_size, groupSyncWrite_GP_A_V_LED, groupSyncRead_PP_M_MS, packetHandler, portHandler);
  Serial.println("Finished Sync P2P Dynamixels-Stepper");
  delay(1000);
  
  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  // Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
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
bool InitialStepperCheck(AccelStepper AccelStepperMotor, uint8_t stp_ID){
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


// HomingStepper
  bool HomingStepper(AccelStepper AccelStepperMotor, uint8_t stp_ID){
  time_now_micros = micros();
  long init_homing_step =  -1;
  
  Serial.print("Stepper is Homing . . . . . . . . . . .\n ");
  
  //digitalWrite(ledPin, HIGH);                                                                           // Orange LED is ONN while HOMING
  
  while (digitalRead(hallSwitchPin)) {                                                                      // Make the Stepper move CCW until the switch is activated   
    AccelStepperMotor.moveTo(init_homing_step);                                                         // Set the position to move to
    init_homing_step--;                                                                                 // Decrease by 1 for next move if needed
    AccelStepperMotor.run();                                                                            // Start moving the stepper
    } // while
    
  // since while breaks, button is pressed => HOME is reached!
  AccelStepperMotor.setCurrentPosition(0);
  //AccelStepperMotor.setMaxSpeed(500);                                                                 // Caution: the maximum speed achievable depends on your processor and clock speed. The default maxSpeed is 1.0 steps per second.
  //AccelStepperMotor.setAcceleration(300);
  init_homing_step=1;
  
  while (!digitalRead(hallSwitchPin)) {                 // Make the Stepper move CCW until the switch is activated   
    AccelStepperMotor.moveTo(init_homing_step);     // Set the position to move to
    init_homing_step++;                             // Decrease by 1 for next move if needed
    AccelStepperMotor.run();                        // Start moving the stepper
    } // while

  Serial.print("Homing finished. \n");
  delay(1000);                                      // Pause for 1 sec
  
  Serial.print("Setup finished. Ready for action.  \n");
  //AccelStepperMotor.setMaxSpeed(1000.0);            // Set Max Speed of Stepper (Faster for regular movements)
  //AccelStepperMotor.setAcceleration(1000.0);        // Set Acceleration of Stepper

  digitalWrite(ledPin, LOW);                        // Orange LED is OFF
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

/*
bool HomingDynamixelSDK(dynamixel::GroupSyncWrite groupSyncWriteGoalPos, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){
  time_now_millis = millis();

  // I. Here Set Acceleration and Profile Velocity on Dynamixels

  // II. Here write Home Position Value and Homing Velocity/Acceleration
  int32_t dxl_home_position = 0;

    // II.a. Allocate goal position value into byte array
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_home_position));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_home_position));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_home_position));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_home_position));

      // II.b. Add Dynamixel#1 goal position value to the Syncwrite storage
      dxl_addparam_result = groupSyncWriteGoalPos.addParam(DXL1_ID, param_goal_position);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] groupSyncWriteHomePos1 addparam failed");
        return false;
      }

      // II.c. Add Dynamixel#2 goal position value to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWriteGoalPos.addParam(DXL2_ID, param_goal_position);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("] groupSyncWriteHomePos2 addparam failed");
        return false;
      }

      // II.d. Syncwrite goal position
      dxl_comm_result = groupSyncWriteGoalPos.txPacket();
      if (dxl_comm_result != COMM_SUCCESS){
       Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      }else{
        Serial.println("Dynamixels  Homing...");
      }

      // II.e. Clear syncwrite parameter storage
      groupSyncWriteGoalPos.clearParam();

  // III. LED status while homing
  int k=0;
  do{
    k++;
    //blink led;
    Serial.println("BLUE LED blinks");
    // MUST undestand read1ByteTxRx vs read1ByteRx
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler,DXL1_ID,ADDR_PRO_MOVING_STATUS,&dxl_moving,&dxl_error);        // if 0 => success of TxRx
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,DXL1_ID,ADDR_PRO_MOVING, dxl_moving ,&dxl_error);              // if 0 => success of TxRx
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
  
    Serial.printf("dxl_moving= %d \n",dxl_moving);
    Serial.printf("readStateMovingStatus= %d \n",readStateMovingStatus);
  }while(k<100);    
  //}while(!dxl_moving);
}
*/

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

// Function
bool HomingDynamixelProfilesSDK(dynamixel::GroupSyncWrite groupSyncWrite_GP_A_V_LED, dynamixel::GroupSyncWrite groupSyncWrite_GREEN_LED, dynamixel::GroupSyncRead groupSyncRead_PP_M_MS, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){
  time_now_millis = millis();
    

    
// FOR loop for DXL_ID starts here...
for(int id_count = 0; id_count <2; id_count++){
  // I. Disable Dynamixel Torque because Indirect address would not be accessible when the torque is already enabled!
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  else
  {
    printf("DXL has been successfully connected \n");
  }


  // II. Indirect Parameter Storage for WRITE

    // II.a.1. Allocate goal position value into byte array
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_GOAL_POSITION + 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_GOAL_POSITION + 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 4, ADDR_PRO_GOAL_POSITION + 2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 6, ADDR_PRO_GOAL_POSITION + 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    // II.a.2. Allocate Profile Acceleration value into byte array
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_PROF_ACCEL + 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 10, ADDR_PRO_PROF_ACCEL + 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 12, ADDR_PRO_PROF_ACCEL + 2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 14, ADDR_PRO_PROF_ACCEL + 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    // II.a.3. Allocate Profile Velocity value into byte array
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 16, ADDR_PRO_PROF_VEL + 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 18, ADDR_PRO_PROF_VEL + 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 20, ADDR_PRO_PROF_VEL + 2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 22, ADDR_PRO_PROF_VEL + 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    // II.a.3. Allocate Profile Velocity value into byte array
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 24, ADDR_PRO_LED_BLUE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 26, ADDR_PRO_LED_GREEN, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    // III. Indirect Parameter Storage for READ
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 0, ADDR_PRO_PRESENT_POSITION + 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 2, ADDR_PRO_PRESENT_POSITION + 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 4, ADDR_PRO_PRESENT_POSITION + 2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 6, ADDR_PRO_PRESENT_POSITION + 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 8, ADDR_PRO_MOVING, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 10, ADDR_PRO_MOVING_STATUS, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

// FINISHED WRITING IN EEPROM MEMORY => TORQUE CAN BE ENABLED

  // IV. Enable DXL Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  // Add parameter storage for the present position value
  dxl_addparam_result = groupSyncRead_PP_M_MS.addParam(dxl_id[id_count]);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed\n", dxl_id[id_count]);
    return 0;
  }

}
// FOR loop for DXL_ID ends here...

      // V. Allocate params for Write from Byte Array
      int32_t dxl_home_position = 0;
      int32_t dxl_home_velocity = dxl_vel_limit;                                            // Homes with HALF of MAX Velocity
      int32_t dxl_home_acceleration = dxl_accel_limit/10;                                        // Homes with FULL Acceleration

      param_indirect_data_for_write[0] = DXL_LOBYTE(DXL_LOWORD(dxl_home_position));
      param_indirect_data_for_write[1] = DXL_HIBYTE(DXL_LOWORD(dxl_home_position));
      param_indirect_data_for_write[2] = DXL_LOBYTE(DXL_HIWORD(dxl_home_position));
      param_indirect_data_for_write[3] = DXL_HIBYTE(DXL_HIWORD(dxl_home_position));
      
      param_indirect_data_for_write[4] = DXL_LOBYTE(DXL_LOWORD(dxl_home_velocity));
      param_indirect_data_for_write[5] = DXL_HIBYTE(DXL_LOWORD(dxl_home_velocity));
      param_indirect_data_for_write[6] = DXL_LOBYTE(DXL_HIWORD(dxl_home_velocity));
      param_indirect_data_for_write[7] = DXL_HIBYTE(DXL_HIWORD(dxl_home_velocity));
      
      param_indirect_data_for_write[8] = DXL_LOBYTE(DXL_LOWORD(dxl_home_acceleration));
      param_indirect_data_for_write[9] = DXL_HIBYTE(DXL_LOWORD(dxl_home_acceleration));
      param_indirect_data_for_write[10] = DXL_LOBYTE(DXL_HIWORD(dxl_home_acceleration));
      param_indirect_data_for_write[11] = DXL_HIBYTE(DXL_HIWORD(dxl_home_acceleration));
      
      param_indirect_data_for_write[12] = dxl_ledBLUE_value[0];
      param_indirect_data_for_write[13] = dxl_ledGREEN_value[1];

      // VI.a Add Dynamixel#1 WRITE values to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWrite_GP_A_V_LED.addParam(DXL1_ID, param_indirect_data_for_write);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] groupSyncWriteHomePos1 addparam failed");
        return false;
      }

      // VI.b. Add Dynamixel#2 WRITE values to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWrite_GP_A_V_LED.addParam(DXL2_ID, param_indirect_data_for_write);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("] groupSyncWriteHomePos2 addparam failed");
        return false;
      }

      // VII. Syncwrite Packet is sent to Dynamixels
      dxl_comm_result = groupSyncWrite_GP_A_V_LED.txPacket();
      /*if (dxl_comm_result != COMM_SUCCESS){
       Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      }else{
       Serial.println("Dynamixels  Homing...");
      }*/

      // VIII. Clear syncwrite parameter storage
      groupSyncWrite_GP_A_V_LED.clearParam();

      // Motor Movement Timing
      unsigned long t1 = (600*dxl_home_velocity)/dxl_home_acceleration;
      int32_t Dpos = abs(dxl_home_position-150000);
      unsigned long t2 = (5.97701241*Dpos)/dxl_home_velocity;
      unsigned long DxlTimeExec = t1+t2; // Time Units [millisecs]
      unsigned long time_duration;                                  
      Serial.print("DxlTimeExec[millisecs]="); Serial.println(DxlTimeExec);
      
      unsigned long motor_movement_start = millis();
      // IX. EXECUTING AND READING
      do
      {
      /*
       * // IX.a. Syncread present position from indirectdata2
      dxl_comm_result = groupSyncRead_PP_M_MS.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

      // IX.b.1. Check if groupsyncread data of Dyanamixels is available
      for(int id_count = 0; id_count <2; id_count++){
            dxl_getdata_result = groupSyncRead_PP_M_MS.isAvailable(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);
            if (dxl_getdata_result != true)
            {
              fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id[id_count]);
              return 0;
            }

            dxl_getdata_result = groupSyncRead_PP_M_MS.isAvailable(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);
            if (dxl_getdata_result != true)
            {
              fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id[id_count]);
              return 0;
            }

            dxl_getdata_result = groupSyncRead_PP_M_MS.isAvailable(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION + LEN_PRO_MOVING, LEN_PRO_MOVING_STATUS);
            if (dxl_getdata_result != true)
            {
              fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id[id_count]);
              return 0;
            }
      }

      // IX.b.2.Get Dynamixel values
      for(int id_count = 0; id_count <2; id_count++){
            dxl_present_position[id_count] = groupSyncRead_PP_M_MS.getData(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);

            dxl_moving[id_count] = groupSyncRead_PP_M_MS.getData(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);

            dxl_moving_status[id_count] = groupSyncRead_PP_M_MS.getData(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION + LEN_PRO_MOVING, LEN_PRO_MOVING_STATUS);
      }

      Serial.printf("[Dynamixel Motor ID:%03d] GoalPos:%d  PresPos:%d  IsMoving:%d \n", dxl_id[0], dxl_home_position, dxl_present_position[0], dxl_moving[0]);
      Serial.printf("[Dynamixel Motor ID:%03d] GoalPos:%d  PresPos:%d  IsMoving:%d \n", dxl_id[1], dxl_home_position, dxl_present_position[1], dxl_moving[1]);
      Serial.println("Dynamixels  Homing... Please Wait... ");
      //}while(abs(dxl_home_position - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD);*/

      time_duration = millis() - motor_movement_start;
      //Serial.println(time_duration);
      }while( time_duration <  DxlTimeExec);

  // X.1.1 Turn on Green LED
  /*param_led_for_write = dxl_ledGREEN_value[1];
  // Add Dynamixels GREEN LED value to the Syncwrite storage
  for(int id_count = 0; id_count <2; id_count++){
      dxl_addparam_result = groupSyncWrite_GREEN_LED.addParam(dxl_id[id_count], &param_led_for_write);
      if (dxl_addparam_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dxl_id[id_count]);
        return 0;
      }
  }
  // Syncwrite Dynamixels GREEN LED
  dxl_comm_result = groupSyncWrite_GREEN_LED.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  // Clear syncwrite parameter storage
  groupSyncWrite_GREEN_LED.clearParam();
  */
  //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_LED_BLUE, dxl_ledBLUE_value[0], &dxl_error);
  //dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_LED_GREEN, dxl_ledGREEN_value[1], &dxl_error);  
    
  // X.1.2 Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // X.1.3. Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // X.1.4 Close port
  portHandler->closePort();

  return 0;
} // END OF FUNCTION

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


// Function
bool SimpleSyncP2P_TrapzVelProf_SDK( uint8_t *MotorsIDs, int MotorsIds_size, int32_t *DxlTrapzProfParams, float *StpTrapzProfParams, int TrapzProfParams_size, dynamixel::GroupSyncWrite groupSyncWrite_GP_A_V_LED, dynamixel::GroupSyncRead groupSyncRead_PP_M_MS, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){
    time_now_millis = millis();

    /* 
     * I.
     * Dynamixel Initialization 
     * Here only Dxl Position change is considered for position status monitoring
     * DxlTimeExec is calculated only for 1 motor since Sunc is used!
     */
    unsigned long t1 = (600*DxlTrapzProfParams[2])/DxlTrapzProfParams[3];
    int32_t Dpos = abs(DxlTrapzProfParams[1]-DxlTrapzProfParams[0]);
    unsigned long t2 = (5.97701241*Dpos)/DxlTrapzProfParams[2];
    unsigned long DxlTimeExec = t1+t2;                                              // Time Units [millisecs]
    Serial.print("DxlTimeExec[millisecs]="); Serial.println(DxlTimeExec);
    int32_t DxlPresentPosition = 0;                                                 // Dynamixel is previously Homed!
    bool result = false;
    const char *log;
    //int32_t get_data = 0;

    /* 
     * II.
     * Stepper Initialization
     * Checks for Stepper Trapezoidal profile
     * Based on trapzPreAssignedVelAcceleP2P.cpp
     */
    float a  = ( 2 * pi ) / (  spr );                                                   // Stepper Motor Step Angle(Angular Position of Motor shaft)[rad]
    float ag = ( 2 * pi ) / ( GEAR_FACTOR * spr );                                      // Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]
    Serial.print("Angle per step(a) = "); Serial.print(ag, 6); Serial.println(" [rad] "); 
    float h  = StpTrapzProfParams[1]-StpTrapzProfParams[0];                             // Calculate displacement in [rad]
    //h = abs(h);                                                                       // ...
    float h_step = round( h / ag );                                                     // Calculate displacement in [steps]
    Serial.print("Total number of Stepper Motor steps(h_step)  = "); Serial.print(h_step); Serial.println(" [steps] "); 

    long max_s_lim = pow(StpTrapzProfParams[2],2) / (2 * ag * StpTrapzProfParams[3] * 100 );  // Number of needed steps to accelerate to desired SPEED
    long accel_lim = h_step * StpTrapzProfParams[3] / ( 2 *StpTrapzProfParams[3]  );          // Number of needed steps to accelerate to desired ACCELERATION
     
    float Ta; float T;
    long  nmov_Ta;
    long  nmov_Td;
    bool segmentExists;
    long nmov_linseg;
    if(max_s_lim <= accel_lim)                                                           // Trapezoidal Profile => Reaches max Speed with MAX Acceleration
    {
      Serial.println("Trapezoidal Profile!"); segmentExists = true;
      
      // Calculate Theoretical Time Execution Values!
      Ta = StpTrapzProfParams[2] / StpTrapzProfParams[3];
      Serial.print("Acceleration Phase Time(Ta) = "); Serial.print(Ta, 6); Serial.println(" [sec] ");                         // Display accelration time
      T  = ( h*StpTrapzProfParams[3]+pow(StpTrapzProfParams[2],2) ) / ( StpTrapzProfParams[3]*StpTrapzProfParams[2] );        // Calculate total execution time
      Serial.print("Total Execution Time(T) = "); Serial.print(T, 6); Serial.println(" [sec] ");                              // Display accelration time

      nmov_Ta =  max_s_lim;                                                                                                   // Steps of Acceleration Phase;
      nmov_linseg = h_step - 2 * nmov_Ta;                                                                                     // Steps of liner segment if Accel=Deccel
      nmov_Td = max_s_lim ;
      Serial.print("Steps of Accel/Deccel  Phase(nmov_Ta/nmov_Td) = "); Serial.print(nmov_Ta); Serial.println(" [steps] ");
      Serial.print("Steps of Constant Velocity Phase(nmov_linseg) = "); Serial.print(nmov_linseg); Serial.println(" [steps] ");
    } 
    else
    {
      Serial.println("Triangular Profile! Vmax is recalculated!"); segmentExists = false;

      // Calculate Theoretical Time Execution Values!
      Ta = sqrt(h/StpTrapzProfParams[3]);
      Serial.print("Acceleration Phase Time(Ta) = "); Serial.print(Ta, 6); Serial.println(" [sec] ");                           // Display accelration time
      T  = 2*Ta;
      Serial.print("Total Execution Time(T) = "); Serial.print(T, 6); Serial.println(" [sec] ");                                // Display accelration time
      float nVmax = StpTrapzProfParams[3]*Ta;
      Serial.printf("New maximum Velocity: %f \n",nVmax);
      
      nmov_Ta = max_s_lim;
      nmov_linseg = 0;
      nmov_Td = h_step-nmov_Ta;
      Serial.print("Steps of Accel  Phase(nmov_Ta) = "); Serial.print(nmov_Ta); Serial.println(" [steps] ");
      Serial.print("Steps of Ct Velocity Phase(nmov_linseg) = "); Serial.print(nmov_linseg); Serial.println(" [steps] ");
      Serial.print("Steps of Deccel  Phase(nmov_Td) = "); Serial.print(nmov_Ta); Serial.println(" [steps] ");
    }

    long StpPresentPosition = 0;                                                                    // Moves motor until specified number of steps is reached
    //float delta_t = 0.0001;
    float delta_t =  0.676 * ft *(2*ag/StpTrapzProfParams[3]);                                     // c0 with ignored inaccuracy factor [sec]
    Serial.print("Initial Step Delay Time(c0) = "); Serial.print(delta_t,6); Serial.println(" [sec] ");
    delay(1000);
    
    float new_delta_t;
    unsigned long rest  = 0;
    long accel_count = 0; 
    long ctVel_count = 0;
    long decel_count = -nmov_Td;                                                    // counter of steps executed for Acceleration Phase

/*
 * III. Initiate Sync write for Dynamixels using Indirect Addressing 
 */
// FOR loop for DXL_ID starts here...
        for(int id_count = 0; id_count <2; id_count++){
          // III.1. Disable Dynamixel Torque because Indirect address would not be accessible when the torque is already enabled!
          dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          else
          {
            printf("DXL has been successfully connected \n");
          }


        // III.2 Indirect Parameter Storage for WRITE

          // III.2.1. Allocate goal position value into byte array
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_GOAL_POSITION + 0, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_GOAL_POSITION + 1, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 4, ADDR_PRO_GOAL_POSITION + 2, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 6, ADDR_PRO_GOAL_POSITION + 3, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          // III.2.2. Allocate Profile Acceleration value into byte array
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_PROF_ACCEL + 0, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 10, ADDR_PRO_PROF_ACCEL + 1, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 12, ADDR_PRO_PROF_ACCEL + 2, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 14, ADDR_PRO_PROF_ACCEL + 3, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          // III.2.3. Allocate Profile Velocity value into byte array
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 16, ADDR_PRO_PROF_VEL + 0, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 18, ADDR_PRO_PROF_VEL + 1, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 20, ADDR_PRO_PROF_VEL + 2, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 22, ADDR_PRO_PROF_VEL + 3, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          // III.2.3. Allocate LEDs value into byte array
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 24, ADDR_PRO_LED_BLUE, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 26, ADDR_PRO_LED_GREEN, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          // III.3. Indirect Parameter Storage for READ
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 0, ADDR_PRO_PRESENT_POSITION + 0, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 2, ADDR_PRO_PRESENT_POSITION + 1, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 4, ADDR_PRO_PRESENT_POSITION + 2, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 6, ADDR_PRO_PRESENT_POSITION + 3, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 8, ADDR_PRO_MOVING, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 10, ADDR_PRO_MOVING_STATUS, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

      // FINISHED WRITING IN EEPROM MEMORY => TORQUE CAN BE ENABLED

        // IV. Enable DXL Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        // Add parameter storage for the present position value
        dxl_addparam_result = groupSyncRead_PP_M_MS.addParam(dxl_id[id_count]);
        if (dxl_addparam_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed\n", dxl_id[id_count]);
          return 0;
        }

      }
      // FOR loop for DXL_ID ends here...

      // V. Allocate params for Write from Byte Array, arguments are given from user in function call
      param_indirect_data_for_write[0] = DXL_LOBYTE(DXL_LOWORD(DxlTrapzProfParams[1]));
      param_indirect_data_for_write[1] = DXL_HIBYTE(DXL_LOWORD(DxlTrapzProfParams[1]));
      param_indirect_data_for_write[2] = DXL_LOBYTE(DXL_HIWORD(DxlTrapzProfParams[1]));
      param_indirect_data_for_write[3] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[1]));
      
      param_indirect_data_for_write[4] = DXL_LOBYTE(DXL_LOWORD(DxlTrapzProfParams[2]));
      param_indirect_data_for_write[5] = DXL_HIBYTE(DXL_LOWORD(DxlTrapzProfParams[2]));
      param_indirect_data_for_write[6] = DXL_LOBYTE(DXL_HIWORD(DxlTrapzProfParams[2]));
      param_indirect_data_for_write[7] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[2]));
      
      param_indirect_data_for_write[8] = DXL_LOBYTE(DXL_LOWORD(DxlTrapzProfParams[3]));
      param_indirect_data_for_write[9] = DXL_HIBYTE(DXL_LOWORD(DxlTrapzProfParams[3]));
      param_indirect_data_for_write[10] = DXL_LOBYTE(DXL_HIWORD(DxlTrapzProfParams[3]));
      param_indirect_data_for_write[11] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[3]));
      
      param_indirect_data_for_write[12] = dxl_ledBLUE_value[1];
      param_indirect_data_for_write[13] = dxl_ledGREEN_value[0];

      // VI.a Add Dynamixel#1 WRITE values to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWrite_GP_A_V_LED.addParam(DXL1_ID, param_indirect_data_for_write);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] groupSyncWriteHomePos1 addparam failed");
        return false;
      }

      // VI.b. Add Dynamixel#2 WRITE values to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWrite_GP_A_V_LED.addParam(DXL2_ID, param_indirect_data_for_write);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("] groupSyncWriteHomePos2 addparam failed");
        return false;
      }

      // VII. Syncwrite Packet is sent to Dynamixels
      dxl_comm_result = groupSyncWrite_GP_A_V_LED.txPacket();
      if (dxl_comm_result != COMM_SUCCESS){
       Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      }else{
        Serial.println("Dynamixels  Moving...");
      }

      // VIII. Clear syncwrite parameter storage
      groupSyncWrite_GP_A_V_LED.clearParam();


      // IX. EXECUTING AND READING
      do
      {
      // IX.aa. Move stepper using Trapz Vel Profile
// =============================================================================================================================================
          StpPresentPosition++;
          // IV.b.1.I. Locate position in ramp
          const float RAMP_FACTOR = 0.1;
          if(segmentExists)                                                                             // Linear Segment exists
          {
                Serial.println("Segment exists");                               
                if(StpPresentPosition<nmov_Ta){
                  Serial.println("Acceleration Phase");
                  accel_count++;                                                                        // Acceleration Phase: delta_t -> minimizes
                  new_delta_t =  delta_t - RAMP_FACTOR * ( (2*delta_t+rest)/(4*accel_count+1) );                // c_n [sec]
                  //new_rest = (2*delta_t + rest)*mod(4*accel_count+1);                                 // r_n
                }else if( StpPresentPosition>nmov_Ta && StpPresentPosition<(nmov_Ta+nmov_linseg) ){     // Linear Segment: delta_t -> constant
                  ctVel_count++;
                  Serial.printf("Accel Phase: %ld \n",accel_count);
                  Serial.println("Constant Velocity Phase");
                  new_delta_t = delta_t;  
                }
                else{
                  Serial.printf("CtVel Phase steps: %ld \n",ctVel_count);
                  Serial.println("Decelleration Phase");
                  decel_count++;                                                                        // Negative Value!
                  Serial.println(decel_count);
                  new_delta_t =  delta_t - RAMP_FACTOR * ( (2*delta_t+rest)/(4*decel_count+1) );                // Deceleration Phase: delta_t -> maximizes [sec] 
                }                                                                         
          }
          else
          {                                                                                             // Linear Segment doesn't exist
                Serial.println("Segment doesn't exist");
                if(StpPresentPosition<nmov_Ta)                                                          // Acceleration Phase: delta_t -> minimizes
                {
                  Serial.println("Acceleration Phase");
                  accel_count++;
                  new_delta_t = delta_t - RAMP_FACTOR * ((2*delta_t+rest)/(4*accel_count+1) );                           // c_n [sec]

                }                                   
                else{                                                                                   // Deceleration Phase: delta_t -> maximizes
                  Serial.println("Decelleration Phase");
                  decel_count++;                                                                        // Negative Value!
                  new_delta_t = delta_t - RAMP_FACTOR * ((2*delta_t+rest)/(4*decel_count+1) );                           // Deceleration Phase: delta_t -> maximizes [sec] 
                }                                                                       
          }
          
          Serial.printf("New step delay time[s]: %f \n",new_delta_t);
          unsigned long new_delta_t_micros = (new_delta_t*1000000);
          //unsigned long new_delta_t_micros = new_delta_t;
          Serial.printf("New step delay time[micros]: %lu \n",new_delta_t_micros);
          
          // IV.b.1.II. Steps Motor with variable time delay step
          singleStepVarDelay(new_delta_t_micros);       

          // Updates time step delay for next step
          delta_t = new_delta_t;
            
          Serial.print("StpPresentPosition="); Serial.println(StpPresentPosition);

          // =============================================================================================================================================      
/*
      // IX.a. Syncread present position from indirectdata2
          dxl_comm_result = groupSyncRead_PP_M_MS.txRxPacket();
          if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

          // IX.b.1. Check if groupsyncread data of Dyanamixels is available
          for(int id_count = 0; id_count <2; id_count++){
                dxl_getdata_result = groupSyncRead_PP_M_MS.isAvailable(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);
                if (dxl_getdata_result != true)
                {
                  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id[id_count]);
                  return 0;
                }

                dxl_getdata_result = groupSyncRead_PP_M_MS.isAvailable(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);
                if (dxl_getdata_result != true)
                {
                  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id[id_count]);
                  return 0;
                }

                dxl_getdata_result = groupSyncRead_PP_M_MS.isAvailable(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION + LEN_PRO_MOVING, LEN_PRO_MOVING_STATUS);
                if (dxl_getdata_result != true)
                {
                  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id[id_count]);
                  return 0;
                }
          }

          // IX.b.2.Get Dynamixel values
          for(int id_count = 0; id_count <2; id_count++){
                dxl_present_position[id_count] = groupSyncRead_PP_M_MS.getData(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);

                dxl_moving[id_count] = groupSyncRead_PP_M_MS.getData(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);

                dxl_moving_status[id_count] = groupSyncRead_PP_M_MS.getData(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION + LEN_PRO_MOVING, LEN_PRO_MOVING_STATUS);
          }

          Serial.printf("[Dynamixel Motor ID:%03d] GoalPos:%d  PresPos:%d  IsMoving:%d \n", dxl_id[0], DxlTrapzProfParams[1], dxl_present_position[0], dxl_moving[0]);
          Serial.printf("[Dynamixel Motor ID:%03d] GoalPos:%d  PresPos:%d  IsMoving:%d \n", dxl_id[1], DxlTrapzProfParams[1], dxl_present_position[1], dxl_moving[1]);
*/
      //}while(  ( abs(DxlTrapzProfParams[1] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD ) && ( (abs(h_step - StpPresentPosition) != 0) ) );
      //}while(   (millis() < time_now_millis + DxlTimeExec)  );
      }while( ( (abs(h_step - StpPresentPosition) != 0) ));

  // X.1.1 Turn on Green LED
    
  // X.1.2 Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // X.1.3. Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // X.1.4 Close port
  portHandler->closePort();

  return 0;
} // END OF FUNCTION


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
