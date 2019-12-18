/* Synchronize Stepper Nema23 and Dynamixel Motors */
/* Reference for Stepper:   test1_SimpleStepperControl.ino
                 Dynamixel: k_Read_Write from OpenCR Examples */
#include <AccelStepper.h> 
#include <MultiStepper.h>
#include <DynamixelWorkbench.h>

// Stepper Definition
#define dirPin 11
#define stepPin 10
#define enblPin 12
#define ledPin 13             // the number of the LED pin
#define buttonPin 2           // the number of the pushbutton pin - This is the manual-homing switch
#define motorInterfaceType 8
#define STP_ID    1

// Dynamixel Definition
#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   
#define BAUDRATE  57600
#define DXL_ID    1


// Classes Objects definition
AccelStepper stepper1(motorInterfaceType, stepPin, dirPin); // Stepper Motor, it has STP_ID
DynamixelWorkbench dxl_wb;                                  // Dynamixel H54


void setup() {
  // Serial Communication Baudrate:
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor

  // Variables intialization
  const char *log;
  bool result = false;
  uint8_t dxl_id = DXL_ID;
  uint8_t stp1_id = STP_ID;
  uint16_t model_number = 0;
  long current_stepper_position;
  int buttonState         =   0;   // variable for reading the pushbutton status
  int move_finished       =   1;   // Used to check if move is completed
  long initial_homing     =  -1;   // Used to Home Stepper at startup

  // stepper
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);
  pinMode(ledPin, OUTPUT);          // initialize the LED pin as an output:
  pinMode(buttonPin, INPUT_PULLUP); // initialize the pushbutton pin as an input: Button open(unpressed) => we read high(1)

  digitalWrite(stepPin, LOW);
  //digitalWrite(ledPin, LOW);
  digitalWrite(enblPin, LOW);
  digitalWrite(dirPin, LOW);
  
  // Dynamixel Check
    result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);  
  }

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to ping");
  }
  else
  {
    Serial.println("Succeeded to ping");
    Serial.print("id : ");
    Serial.print(dxl_id);
    Serial.print(" model_number : ");
    Serial.println(model_number);
  }

  result = dxl_wb.itemWrite(dxl_id, "LED", 1, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to LED On");
  }
  else
  {
    Serial.println("Succeed to LED On");
  }

  int32_t get_data = 0;
  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_data, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to get present position");
  }
  else
  {
    Serial.print("Succeed to get present position(value : ");
    Serial.print(get_data);
    Serial.println(")");
  }

  result = dxl_wb.torqueOn(dxl_id, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to enable torque on Dynamixel:");
    Serial.print(dxl_id);
    Serial.println(")");
  }
  else
  {
    Serial.println(log);
    Serial.printf("Torque Enabled on Dynamixel: %d \n",dxl_id);
  }
  
  // Stepper Check // MUST be put inside while loop!
  InitialStepperCheck(stepper1, stp1_id);


  // Homing Dynamixel
  HomingDynamixel(dxl_wb, dxl_id);

  
  // Homing Stepper using AccelStepper - Blocks Code until Stepper Homing Finishes
  HomingStepper(stepper1, stp1_id, initial_homing);



  result = dxl_wb.torqueOff(dxl_id, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to disable torque on Dynamixel:");
    Serial.print(dxl_id);
    Serial.println(")");
  }
  else
  {
    Serial.println(log);
    Serial.printf("Torque Disabled on Dynamixel: %d \n",dxl_id);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

// Custom Function Definitions

// InitialStepperCheck
bool InitialStepperCheck(AccelStepper AccelStepperMotor, uint8_t stp_ID){
  bool  check_stepper_running = AccelStepperMotor.isRunning();
  if (check_stepper_running == true)
  {
    Serial.printf("Stepper Motor: %d is running! \n",stp_ID);
    Serial.printf("Waiting for Stepper Motor: %d to stop to comlete check! \n",stp_ID);
    AccelStepperMotor.stop();
    delay(1000);
  }
  else
  {
    long current_stepper_position = AccelStepperMotor.currentPosition();
    Serial.printf("Stepper Motor: %d is stall at Position: %ld! \n",stp_ID,current_stepper_position);
    Serial.printf("Stepper Motor: %d Check OK. \n",stp_ID);
  }

}

// HomingStepper
bool HomingStepper(AccelStepper AccelStepperMotor, uint8_t stp_ID, long init_homing_step){
  Serial.print("Stepper is Homing . . . . . . . . . . .\n ");
  
  digitalWrite(ledPin, HIGH);           // Orange LED is ONN while HOMING
  
  while (digitalRead(buttonPin)) {      // Make the Stepper move CCW until the switch is activated   
    AccelStepperMotor.moveTo(init_homing_step);    // Set the position to move to
    init_homing_step--;                   // Decrease by 1 for next move if needed
    AccelStepperMotor.run();                     // Start moving the stepper
    delayMicroseconds(500);
    } // while
    
  // since while breaks, button is pressed => HOME is reached!
  AccelStepperMotor.setCurrentPosition(0);
  AccelStepperMotor.setMaxSpeed(500);            // Caution: the maximum speed achievable depends on your processor and clock speed. The default maxSpeed is 1.0 steps per second.
  AccelStepperMotor.setAcceleration(300);
  init_homing_step=1;
  
  while (!digitalRead(buttonPin)) {      // Make the Stepper move CCW until the switch is activated   
    AccelStepperMotor.moveTo(init_homing_step);    // Set the position to move to
    init_homing_step++;                   // Decrease by 1 for next move if needed
    AccelStepperMotor.run();                     // Start moving the stepper
    delayMicroseconds(500);
    } // while

  Serial.print("Homing finished. \n");
  delay(1000);                          // Pause for 1 sec
  
  Serial.print("Setup finished. Ready for action.  \n");
  AccelStepperMotor.setMaxSpeed(1000.0);         // Set Max Speed of Stepper (Faster for regular movements)
  AccelStepperMotor.setAcceleration(1000.0);     // Set Acceleration of Stepper

  digitalWrite(ledPin, LOW);           // Orange LED is OFF
}

// HomingDynamixel
  bool HomingDynamixel(DynamixelWorkbench DxlMotor, uint8_t MOTOR_DXL_ID){
  bool result = false;
  int32_t dxl_homing_position = 251000;
  result = DxlMotor.itemWrite(MOTOR_DXL_ID, "Goal_Position", dxl_homing_position);
  delay(200);
  if (result == false)
  {
  //Serial.println(log);
  Serial.println("Failed to write");
  }
  else
  {
  Serial.printf("Dynamixel Motor moves to position: %d Check OK. \n",dxl_homing_position);
  }

}