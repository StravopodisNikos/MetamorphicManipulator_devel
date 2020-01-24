/*******************************************************************************
* Executes simple Sync of 2 Dynamixels and Nema23 Stepper motor using Velocity 
* and Acceleration profiles
* Based on sync2DxlStp_TrapzVelProf.ino which magically doesn't work for Dxl Wb
*******************************************************************************/

/* Author: Stravopodis Nikos */

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

// Used Libraries
#include <DynamixelSDK.h>
#include <AccelStepper.h>
#include <DynamixelWorkbench.h>


// Control Table Used Items Address
#define ADDR_PRO_TORQUE_ENABLE          512                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          564
#define ADDR_PRO_GOAL_VELOCITY          552
#define ADDR_PRO_PROF_ACCEL             556
#define ADDR_PRO_PROF_VEL               560
#define ADDR_PRO_PRESENT_VELOCITY       576
#define ADDR_PRO_MOVING_STATUS          571
#define ADDR_PRO_PRESENT_POSITION       580
#define ADDR_PRO_ACCEL_LIMIT            40
#define ADDR_PRO_VEL_LIMIT              44
#define ADDR_PRO_MAX_POS_LIMIT          48
#define ADDR_PRO_MIN_POS_LIMIT          52

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
#define dirPin 11
#define stepPin 10
#define enblPin 12
#define ledPin 13                                                                                                       // the number of the LED pin
#define hallSwitchPin 3                                                                                                     // the number of the pushbutton pin - This is the manual-homing switch - Here Hall effect sensor!!!
#define motorInterfaceType 8
#define STP_ID    1
#define STP_MOVING_STATUS_THRESHOLD     1

// Numeric Type Definitions
const float pi = 3.14159265359;

// Timing variables
int stpWritePeriod = 50;                                                                                                // millis
int dxlWritePeriod = 50;                                                                                                // millis
unsigned long time_now_micros = 0;                                                                                      // Set timer counter
unsigned long time_now_millis = 0;                                                                                      // Set timer counter
const uint8_t handler_index = 0;
unsigned long curMillis;
unsigned long prevStepMillis = 0;
unsigned long millisBetweenSteps = 1; // milliseconds

// Classes Objects definition
AccelStepper stepper1(motorInterfaceType, stepPin, dirPin);                                                             // Stepper Motor, it has STP_ID
DynamixelWorkbench dxl_wb;                                                                                              // Dynamixel H54

// PreAllocate Memory for Parameters/Data TxRx
uint8_t dxl_id[] = {DXL1_ID, DXL2_ID};
uint8_t stp1_id = STP_ID;
uint8_t param_goal_position[4];
uint8_t param_accel_limit[4];
uint8_t param_vel_limit[4];

// Set Desired Limit Values for Control Table Items
int dxl_accel_limit = 20000;
int dxl_vel_limit   = 1000;

// Debugging
// int index = 0;
int dxl_comm_result = COMM_TX_FAIL;                                                           // Communication result
bool dxl_addparam_result = false;                                                             // addParam result
//bool dxl_getdata_result = false;                                                            // GetParam result
uint8_t dxl_error = 0;                                                                        // Dynamixel error 
int32_t dxl1_present_position = 0, dxl2_present_position = 0;                                 // Present position



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

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncReadPresentPos(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

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
 HomingStepper(stepper1, stp1_id);
 delay(1000);

 // V.  Homing Dynamixel Motors[Joint2,3] using Dynamixel SDK
 HomingDynamixelSDK(groupSyncWriteGoalPos,packetHandler,portHandler); 
 
/*
  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncReadPresentPos.addParam(DXL1_ID);
  if (dxl_addparam_result != true)
  {
    Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("groupSyncRead addparam failed");
    return;
  }

  // Add parameter storage for Dynamixel#2 present position value
  dxl_addparam_result = groupSyncReadPresentPos.addParam(DXL2_ID);
  if (dxl_addparam_result != true)
  {
    Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("groupSyncRead addparam failed");
    return;
  }
*/

/*
  while(1)
  {
    Serial.print("Press any key to continue! (or press q to quit!)\n");
    if (getch() == 'q')
      break;

    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteGoalPos.addParam(DXL1_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("groupSyncWrite addparam failed");
      return;
    }

    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteGoalPos.addParam(DXL2_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("groupSyncWrite addparam failed");
      return;
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWriteGoalPos.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) Serial.print(packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWriteGoalPos.clearParam();

    do
    {
      // Syncread present position
      dxl_comm_result = groupSyncReadPresentPos.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS) Serial.print(packetHandler->getTxRxResult(dxl_comm_result));

      // Check if groupsyncread data of Dynamixel#1 is available
      groupSyncReadPresentPos.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
      if (dxl_addparam_result != true)
      {dodo
        Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("groupSyncRead getdata failed");
        return;
      }

      // Check if groupsyncread data of Dynamixel#2 is available
      groupSyncReadPresentPos.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("groupSyncRead getdata failed");
        return;
      }

      // Get Dynamixel#1 present position value
      dxl1_present_position = groupSyncReadPresentPos.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      // Get Dynamixel#2 present position value
      dxl2_present_position = groupSyncReadPresentPos.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      Serial.print("[ID:"); Serial.print(DXL1_ID);
      Serial.print("] GoalPos:"); Serial.print(dxl_goal_position[index]);
      Serial.print("  PresPos:"); Serial.print(dxl1_present_position);
      Serial.print(" [ID:"); Serial.print(DXL2_ID);
      Serial.print("] GoalPos:"); Serial.print(dxl_goal_position[index]);
      Serial.print("  PresPos:"); Serial.print(dxl2_present_position);
      Serial.println(" ");

    }while((abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  } */ //here comment for while(1) 

  
/*
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
*/

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
  
  digitalWrite(ledPin, HIGH);                                                                           // Orange LED is ONN while HOMING
  
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


bool HomingDynamixelSDK(dynamixel::GroupSyncWrite groupSyncWriteGoalPos, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){
  time_now_millis = millis();

  // I. Here Set Acceleration and Profile Velocity on Dynamixels

  // II. Here write Home Position Value
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
  uint8_t dxl_moving;
  // III. LED status while homing
  int k=0;
  do{
    k++;
    //blink led;
    Serial.println("BLUE LED blinks");
    // MUST undestand read1ByteTxRx vs read1ByteRx
    int readStateMovingStatus = packetHandler->read1ByteTxRx(portHandler,DXL1_ID,ADDR_PRO_MOVING_STATUS,&dxl_moving,&dxl_error);       // if 0 => success of TxRx
    //int readStateMovingStatus = packetHandler->read1ByteRx(portHandler,dxl_moving,&dxl_error);
    Serial.printf("dxl_moving= %d \n",dxl_moving);
    Serial.printf("readStateMovingStatus= %d \n",readStateMovingStatus);
  }while(k<100);    
  //}while(!dxl_moving);
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
