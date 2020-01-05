#include <AccelStepper.h> 
#include <MultiStepper.h>
#include <DynamixelWorkbench.h>                                 

#define DXL_MOVING_STATUS_THRESHOLD     20

struct PseudoJointStruct{
  String name;
  uint8_t id;
  uint8_t homingLedPin_ID;
  uint8_t metLedPin_ID;
  uint8_t metErrPin_ID;
  uint8_t dirPin;
  uint8_t stepPin;
  uint8_t hallsens0Pin;
  int motorInterfaceType;
  long rad2stpFactor;                                                    
} pseudo1;


void setup() {
  // Declaration of Pseudojoint1
  uint8_t MOTOR_DXL_ID=1;
  DynamixelWorkbench DxlMotor;
  // Declaration of Pseudojoint1
  pseudo1.id = 1;
  pseudo1.homingLedPin_ID=14;
  pseudo1.metLedPin_ID = 13;
  pseudo1.metErrPin_ID = 12;
  pseudo1.dirPin = 11;
  pseudo1.stepPin = 10;
  pseudo1.hallsens0Pin = 9;
  pseudo1.motorInterfaceType = 8;
  pseudo1.rad2stpFactor = 1273.24;                                                      // 1 rad = 1273.24 steps when motorInterfaceType = 8 for DM542 Driver
  AccelStepper stepper1(pseudo1.motorInterfaceType, pseudo1.stepPin, pseudo1.dirPin);   // Stepper Motor1

  //bool result = Homing(stepper1,pseudo1);
  float thpGoalPosition = 1.5708;
  bool result = Metamorphosis(DxlMotor,MOTOR_DXL_ID,stepper1,pseudo1,thpGoalPosition);
}

void loop() {
}

bool Homing(AccelStepper AccelStepperMotor, PseudoJointStruct pseudo){

// Moves motor until Homing Hall Sensor(hsens0) is ENABLED
  bool status;
  int stpWritePeriod = 500;    // [micros]
  long init_homing_step =  -1;
  float homingSpeed = 400.0*0.000277778;                // Set Homing Speed @ 100 steps/second
  AccelStepperMotor.setSpeed(homingSpeed);
  digitalWrite(pseudo.homingLedPin_ID, HIGH);                      // Orange LED is ONN while HOMING, ledPin is passed as array element
  
  unsigned long time_now_micros = micros();
  while (digitalRead(pseudo.hallsens0Pin)== 0) {                 // Make the Stepper move until hallsens0 is ENABLED  
      AccelStepperMotor.moveTo(init_homing_step);
      AccelStepperMotor.setSpeed(homingSpeed);
      init_homing_step--;                  
      AccelStepperMotor.runSpeed();                             // Start moving the stepper - Implementing a constant speed
      while(micros() < time_now_micros + stpWritePeriod){}      // Wait approx. stpWritePeriod==minimimum step execution time
    } // while
  
  AccelStepperMotor.setCurrentPosition(0);
  init_homing_step=1;

  time_now_micros = micros();
  while (digitalRead(pseudo.hallsens0Pin)!=0) {                        // Make the Stepper move reverse until the switch is de-activated   
      AccelStepperMotor.moveTo(init_homing_step);             // Set the position to move to
      AccelStepperMotor.setSpeed(homingSpeed);
      init_homing_step++;                                     // Increase by 1 for next move if needed
      AccelStepperMotor.runSpeed();                             // Start moving the stepper - Implementing a constant speed
      while(micros() < time_now_micros + stpWritePeriod){}      // Wait approx. stpWritePeriod==minimimum step execution time
    } // while

// fival check motor status and return state
  if(AccelStepperMotor.isRunning()){
    Serial.printf("[Stepper Motor %d   ] Homing ERROR.    \n",pseudo.id);
    digitalWrite(pseudo.metErrPin_ID, HIGH);                   // Red LED is ON
    bool status = false;
  }
  else{
    Serial.printf("[Stepper Motor %d   ] Homing FINISHED. \n",pseudo.id);
    digitalWrite(pseudo.homingLedPin_ID, LOW);               // Orange LED is OFF
    bool status = true;
  }
  return status;
} // END of function

// ===========================================================================================================================================

bool Metamorphosis(DynamixelWorkbench DxlMotor, uint8_t MOTOR_DXL_ID,AccelStepper AccelStepperMotor, PseudoJointStruct pseudo, float thpGoalPosition){
// Variables/Pin to declare/set rad2stpFactor, metErrPin_ID, metLedPin_ID

// gets Stepper Objects, ID's and pseudojoints angles
// Changes pseudojoint's configuration
  //String thpStrGoalPosition = String(thpGoalPosition,4);              // Convert float to string array for printf use
  char outputString[100];
  const char *log;
  bool status;
  int stpWritePeriod = 500;    // [micros]
// Sets Torque off for connected Dynamixel Motors
  bool result = DxlMotor.torqueOff(MOTOR_DXL_ID, &log);
  if (result == false)
  {
      Serial.printf("Failed to disable torque on Dynamixel Motor: %d.    ABORTING Metamorphosis!\n",MOTOR_DXL_ID);
  }
  else
  {
      Serial.printf("Succeeded to disable torque on Dynamixel Motor: %d. PROCEEDING to Homing steppers!\n",MOTOR_DXL_ID);
  }

  delay(1000);


//  Always Homing Steppers before Metamorphosis
  result = Homing(AccelStepperMotor,pseudo);
  if (result == false)
  {
      Serial.printf("[Stepper Motor %d   ] Homing ERROR.\n",pseudo.id);
      Serial.println("Cannot continue! ABORTING!");
  }
  else
  {
    Serial.printf("[Stepper Motor %d   ] Homing SUCCESSFUL. PROCEEDING to Metamorphosis!\n",pseudo.id);
    AccelStepperMotor.setCurrentPosition(0);                            // This is zero position
    sprintf(outputString,"[Stepper Motor %d   ] Goal Position %f rad. \n",pseudo.id,thpGoalPosition);
    Serial.println(outputString);
  }

  delay(1000);
  digitalWrite(pseudo.metLedPin_ID, HIGH);                                       // Blue LED is ONN while METAMORPHOSIS

// Set Goal Anatomy -> Converts thp to steps
    float metSpeed = 600.0*0.000277778;
  long stpGoalPosition = thpGoalPosition*pseudo.rad2stpFactor;                       // Convert rad to steps
    AccelStepperMotor.moveTo(stpGoalPosition);                                // Set the absolute position to move to
    AccelStepperMotor.setSpeed(metSpeed);                               // Always set speed after moveTo if ct speed is desired

// Move Steppers to Goal Position, moves until distance2Go->0
    unsigned long time_now_micros =  micros();
    while(AccelStepperMotor.distanceToGo() > 5)                             // Stepping Threshold=5
    {                           
      // Run the motor
      AccelStepperMotor.runSpeed();
    while(micros() < time_now_micros + stpWritePeriod){}
    // Display current stepper angular posiion
    long stpPresentPosition = AccelStepperMotor.currentPosition();
    float thpPresentPosition = (1/pseudo.rad2stpFactor)*stpPresentPosition;
    sprintf(outputString,"[Stepper  Motor %d  ] Present Position : %f rad Goal Position : %f rad. \n",pseudo.id,thpPresentPosition,thpGoalPosition);
    Serial.println(outputString);
    }

// when steppers reach Goal Position => Log to Screen:Anatomy Reached 
  sprintf(outputString,"Pseudojoint: %d reached: %f rad. \n",pseudo.id,thpGoalPosition);
  Serial.println(outputString);

// Locking
  /*result = Locking()
  if (result == false)
  {
      Serial.printf("[Stepper Motor %d   ] Locking ERROR.\n",pseudo.id);
      Serial.println("Cannot continue! ABORTING!");
  }
  else
  {
    Serial.printf("[Stepper Motor %d   ] Locking SUCCESSFUL. PROCEEDING to Turning off Motor!\n",pseudo.id);
    // Turn off stepper

  }*/

// fival check motor status and return state
  result = AccelStepperMotor.isRunning();
  if(result == true){
    Serial.printf("[Stepper Motor %d   ] Metamorphosis ERROR.    \n",pseudo.id);
    digitalWrite(pseudo.metErrPin_ID, HIGH);                               // Red LED is ON
    status = false;
  }
  else{
    Serial.printf("[Stepper Motor %d   ] Metamorphosis FINISHED. \n",pseudo.id);
    digitalWrite(pseudo.metLedPin_ID, LOW);                                // Blue LED is OFF
    status = true;
  }

return status;
} // END OF FUNCTION

// ====================================================================================================================

bool SimpleSyncStepperDynamixel(DynamixelWorkbench DxlMotor, uint8_t MOTOR_DXL_ID, AccelStepper AccelStepperMotor, PseudoJointStruct pseudo, long StpPresentPosition){
// Since stepper doesn't return current position(but only relative to home) the current position is passed from calling function
// #define DXL_MOVING_STATUS_THRESHOLD     20  

  int stpWritePeriod = 500;                                                                          // [micros]
  int32_t DxlInitPos = 0;
  int32_t DxlCurrentPos = 0;
  int32_t DxlCurrentVel = 0;
  
  // initialize logging variables
  bool status1 = false;
  bool status2 = false;
  bool status = false;
  bool result = false;
  const char *log;
  char outputString[100];

  // Reads Present Position
    // Dynamixel
    result = DxlMotor.readRegister(MOTOR_DXL_ID, "Present_Position", &DxlInitPos , &log);
    if (result == false)
    {
      Serial.println(log);
    }
    else
    {
      Serial.printf("[Dynamixel Motor %d ] Initial Position : %d \n",MOTOR_DXL_ID,DxlInitPos);
    }

    // Stepper
    long StpInitPos = StpPresentPosition;
    AccelStepperMotor.setCurrentPosition(0);
    float thpInitPos =  (1/pseudo.rad2stpFactor)*StpInitPos;
    sprintf(outputString,"[Stepper Motor %d   ] Goal Position %f rad. \n",pseudo.id,thpInitPos);
    Serial.println(outputString);

  // Set Dynamixel & Stepper movement step
  int32_t DxlMotorMovement = 100000;                                                                        // Dynamixel motor moves for 100.000 steps
  int32_t StpMotorMovement = 1000;                                                                          // Stepper motor moves for 1000 steps

  // Reads user input and breaks/executes
  while(1){
    // Reads current Dynamixel Position(for 1st time it will repeat the first values)
    result = DxlMotor.readRegister(MOTOR_DXL_ID, "Present_Position", &DxlCurrentPos , &log);

    // Reads current Stepper Position(for 1st time it will repeat the first values)
    long stpPresentPosition = AccelStepperMotor.currentPosition();

    // Writes Goal Stepper Position
    float movSpeed = 1000.0*0.000277778;                                                            // stepper miving speed 1000 steps/sec
    long StpGoalPosition = stpPresentPosition + StpMotorMovement;                                   // Convert rad to steps
    AccelStepperMotor.moveTo(StpGoalPosition);                                                      // Set the absolute position to move to
    AccelStepperMotor.setSpeed(movSpeed);                                                           // Always set speed after moveTo if ct speed is desired

    // Writes Goal Dynamixel Position & and moves Dynamixel motor
    int32_t DxlGoalPosition = DxlCurrentPos + DxlMotorMovement;
    result = DxlMotor.itemWrite(MOTOR_DXL_ID, "Goal_Position",DxlGoalPosition, &log);
    // Executes(moves stepper) and reads movement data
    unsigned long time_now_micros =  micros();
    do{
        // Run stepper
        AccelStepperMotor.runSpeed();
        //while(micros() < time_now_micros + stpWritePeriod){}
        // Show movement data
          // Dynamixel 
          result = DxlMotor.readRegister(MOTOR_DXL_ID, "Present_Position", &DxlCurrentPos , &log);
          Serial.printf("[Dynamixel Motor %d ] Present Position : %ld Goal Position : %d \n",MOTOR_DXL_ID,DxlCurrentPos, DxlGoalPosition);
          // Stepper
          StpPresentPosition = AccelStepperMotor.currentPosition();
        Serial.printf("[Stepper  Motor %d  ] Present Position : %ld Goal Position : %ld \n",pseudo.id,StpPresentPosition, StpGoalPosition);
    }while( (abs(DxlGoalPosition - DxlCurrentPos) > DXL_MOVING_STATUS_THRESHOLD) && (AccelStepperMotor.distanceToGo() == 0) );
    
    // After do...while BOTH motors should have stopped(only for debugging)
    // Checks if Stepper stopped
    result = AccelStepperMotor.isRunning();
    if(result == true){
      Serial.printf("[Stepper Motor %d   ] Stepper still RUNNING. OUT OF SYNC  \n",pseudo.id);
      status1 = false;
    }
    else{
      Serial.printf("[Stepper Motor %d   ] Stepper is idle. SYNCED \n",pseudo.id);
      status1 = true;
    }
    // Checks if Dynamixel stopped
    result = DxlMotor.getPresentVelocityData(MOTOR_DXL_ID, &DxlCurrentVel , &log);
    if(DxlCurrentVel != 0){
      Serial.printf("[Dynamixel Motor %d   ] Dynamixel still RUNNING. OUT OF SYNC  \n",MOTOR_DXL_ID);
      status2 = false;
    }
    else{
      Serial.printf("[Dynamixel Motor %d   ] Dynamixel is idle. SYNCED \n",MOTOR_DXL_ID);
      status2 = true;
    }

    // waits for user to decide if repeat/brake
    Serial.print("Press any key to continue! (or press q to quit!)\n");
    // If q is pressed: Turning Off torque in Dynamixel
    if (getch() == 'q')
    {
        result = DxlMotor.torqueOff(MOTOR_DXL_ID, &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to disable torque on Dynamixel:");
          Serial.print(MOTOR_DXL_ID);
          Serial.println(")");
        }
        else
        {
          Serial.println(log);
          Serial.printf("Torque Disabled on Dynamixel: %d \n",MOTOR_DXL_ID);
        }

        break;
    }
  } // END while(1)

  // Returns true if completed
  if(status1 == false || status2 == false){
    Serial.println("MOTORS OUT OF SYNC");
    status = false;
  }
  else{
    Serial.println("MOTORS SYNCED");
    status = true;
  }

return status;
} // END FUNCTION

// ====================================================================================================================
// get serial monitor cmd line value
int getch()
{
  while(1)
  {
    if( Serial.available() > 0 )
    {
      break;
    }
  }

  return Serial.read();
}
