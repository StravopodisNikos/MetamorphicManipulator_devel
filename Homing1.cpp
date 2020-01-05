bool Homing(AccelStepper AccelStepperMotor, PseudoJointStruct pseudo){
// Compiles successfully Sun. 5/1/2020
// Only for 1 motor

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