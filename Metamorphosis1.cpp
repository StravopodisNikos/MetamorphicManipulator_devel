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