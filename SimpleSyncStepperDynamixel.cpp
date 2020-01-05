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