bool Metamorphosis(AccelStepper AccelStepperMotor, uint8_t stp_ID, float thp){
// gets Stepper Objects, ID's and pseudojoints angles
// Changes pseudojoint's configuration

// if Halls Sensors in each pseudo angle
// 		Reads current anatomy=>Reads ENABLED Hall sensor
// else
//		always Homing before Metamorphosis
	result = Homing1();
// end

// Logs out to screen Current->Goal
  if (result == false)
  {
  	erial.printf("[Stepper Motor %d   ] Homing ERROR.    \n",stp_ID);
    Serial.println("Cannot continue! ABORTING!");
  }
  else
  {
	Serial.printf("[Stepper Motor %d   ] Homing SUCCESSFUL. \n",stp_ID);
    Serial.println("Continues to METAMORPHOSIS!");
    AccelStepperMotor.setCurrentPosition(0);							// This is zero position
    Serial.printf("[Stepper Motor %d   ] Goal Position %ld rad. \n",stp_ID,thp);
  }

delay(100);
	digitalWrite(metLedPin_ID, HIGH);           						// Blue LED is ONN while METAMORPHOSIS

// Set Goal Anatomy -> Converts thp to steps
  	float metSpeed = 600.0*0.000277778;
	long thp_stp = thp*rad2stpFactor;
   	AccelStepperMotor.moveTo(thp_stp);     								// Set the absolute position to move to
   	AccelStepperMotor.setSpeed(metSpeed);

// Move Steppers to Goal Position, moves until distance2Go->0
   	time_now_micros =  micros();
   	while(AccelStepperMotor.distanceToGo() != 0){
   		AccelStepperMotor.runSpeed();
		while(micros() < time_now_micros + stpWritePeriod){}	
   	}

// when steppers reach Goal Position => Log to Screen:Anatomy Reached 
Serial.println("Pseudojoint: %d reached: %ld rad.",stp_ID,thp);

// when locks ok => Log to Screen:Anatomy Locked
result = locking1()
//Serial.println("Pseudojoint: %d LOCKED",stp_ID);
// when locks ok => Turn off motor

// fival check motor status and return state
	if(AccelStepperMotor.isRunning()){
		Serial.printf("[Stepper Motor %d   ] Metamorphosis ERROR.    \n",stp_ID);
		digitalWrite(errPin, HIGH);										// Red LED is ON
		status = false;
	}
	else{
		Serial.printf("[Stepper Motor %d   ] Metamorphosis FINISHED. \n",stp_ID);
		digitalWrite(metLedPin_ID, LOW); 								// Blue LED is OFF
		status = true;
	}

return status;
} // END OF FUNCTION