bool Homing(AccelStepper AccelStepperMotor, uint8_t stp_ID, uint8_t homingLedPin_ID){
// GLOBAL VARIABLES USED: 	stpWritePeriod:  int stpWritePeriod = 500;    // [micros] 
// 							hallsens0Pin:    pinMode(hallsens0Pin, INPUT);
// Only for 1 motor

// Moves motor until Homing Hall Sensor(hsens0) is ENABLED
	long init_homing_step =  -1;
	float homingSpeed = 400.0*0.000277778;								// Set Homing Speed @ 100 steps/second
	AccelStepperMotor.setSpeed(homingSpeed);
	digitalWrite(homingLedPin_ID, HIGH);           						// Orange LED is ONN while HOMING, ledPin is passed as array element
	
	time_now_micros = micros();
	while (digitalRead(hallsens0Pin)== 0) {      						// Make the Stepper move until hallsens0 is ENABLED  
    	AccelStepperMotor.moveTo(init_homing_step);
    	AccelStepperMotor.setSpeed(homingSpeed);
    	init_homing_step--;                  
    	AccelStepperMotor.runSpeed();                     				// Start moving the stepper - Implementing a constant speed
    	while(micros() < time_now_micros + stpWritePeriod){}			// Wait approx. stpWritePeriod==minimimum step execution time
    } // while
	
	AccelStepperMotor.setCurrentPosition(0);
	init_homing_step=1;

	time_now_micros = micros();
	while (digitalRead(hallsens0Pin)!=0) {                 				// Make the Stepper move reverse until the switch is de-activated   
    	AccelStepperMotor.moveTo(init_homing_step);     				// Set the position to move to
    	AccelStepperMotor.setSpeed(homingSpeed);
    	init_homing_step++;                             				// Increase by 1 for next move if needed
    	AccelStepperMotor.runSpeed();                        			// Start moving the stepper - Implementing a constant speed
    	while(micros() < time_now_micros + stpWritePeriod){}			// Wait approx. stpWritePeriod==minimimum step execution time
    } // while

// fival check motor status and return state
	if(AccelStepperMotor.isRunning()){
		Serial.printf("[Stepper Motor %d   ] Homing ERROR.    \n",stp_ID);
		digitalWrite(errPin, HIGH);										// Red LED is ON
		status = false;
	}
	else{
		Serial.printf("[Stepper Motor %d   ] Homing FINISHED. \n",stp_ID);
		digitalWrite(homingLedPin_ID, LOW); 							// Orange LED is OFF
		status = true;
	}

return status;
} // END of function