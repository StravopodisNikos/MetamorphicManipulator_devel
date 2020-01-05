bool Metamorphosis(DynamixelWorkbench DxlMotor, uint8_t MOTOR_DXL_ID,AccelStepper AccelStepperMotor, PseudoJointStruct pseudo, long thpGoalPosition){

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
      Serial.printf("[Stepper Motor %d   ] Goal Position %ld rad. \n",pseudo.id,thpGoalPosition);
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
    long thpPresentPosition = (1/pseudo.rad2stpFactor)*stpPresentPosition;
        Serial.printf("[Stepper  Motor %d  ] Present Position : %ld Goal Position : %ld \n",pseudo.id,thpPresentPosition,thpGoalPosition);
    }

// when steppers reach Goal Position => Log to Screen:Anatomy Reached 
  Serial.printf("Pseudojoint: %d reached: %ld rad.",pseudo.id,thpGoalPosition);

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