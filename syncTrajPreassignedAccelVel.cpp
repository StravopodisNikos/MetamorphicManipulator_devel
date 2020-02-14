bool syncTrajPreassignedAccelVel(float *StpTrapzProfParams) {
// Based on Par. 3.2.2. Trajectory Planning for Machines and Robots

	/* 
	 *	StpTrapzProfParams_CT = {h, Texec, Ta, StpVmax, StpAmax }: All given in SI units
	 *  StpInitPos,StpGoalPosition -> [rads]
	 *  StpVmax -> [rad/sec]
	 *  StpAmax -> [rad/sec^2]
	 */

	float h_cond = pow(StpTrapzProfParams[3],2.0) / StpTrapzProfParams[4];				// Condition factor for linear segment existence in
																						// Trapezoidal Velocity Profile
	unsigned long h_step = round( StpTrapzProfParams[0] / ag );                                             // Calculate displacement in [steps]
	Serial.print("Total number of Stepper Motor steps(h_step)  = "); Serial.print(h_step); Serial.println(" [steps] "); 

	// Check condition for determination of Trapezoidal/Triangular Profile:
	if(StpTrapzProfParams[0] >= h_cond)
	{
		// In this case: p2p is executed for Texec with the preassigned Vmax,Amax!
		Serial.println("Trapezoidal Profile!"); segmentExists = true;

        // Determine Profile Step Variables based on Real Time Profiles vs Melchiorri
		float h_accel   = 0.5 * StpTrapzProfParams[4] * pow(StpTrapzProfParams[2],2.0);
		float h_lin_seg = StpTrapzProfParams[4] * accel_width * pow(StpTrapzProfParams[1],2.0) * ( 1 - 2 * accel_width);
		
		unsigned long h_accel_step = round( h_accel / ag );
		unsigned long h_lin_seg_step = round( h_lin_seg / ag );

		nmov_Ta      = h_accel_step;                                                                                           // Steps of Acceleration Phase;
	    nmov_linseg  = h_lin_seg_step                                                                                          // Steps of liner segment 
	    nmov_Td      = h_step - ( h_accel_step + h_lin_seg_step ) ;															   // Steps of Decceleration Phase; 			

	}
	else
	{
		// In this case: p2p is executed for Texec with the recalculated Vmax!
		Serial.println("Triangular Profile! Vmax is recalculated!"); segmentExists = false;
		// Calculate Theoretical Time Execution Values!
		float nTa = sqrt(StpTrapzProfParams[0]/StpTrapzProfParams[4]);

		float nVmax = StpTrapzProfParams[4] * nTa;

		Serial.print("New maximum Velocity:"); Serial.print(nVmax,6); Serial.println("[rad/sec]");

		float h_accel  = 0.5 * StpTrapzProfParams[4] * pow(nTa,2.0);
		unsigned long h_accel_step = round( h_accel / ag );

		nmov_Ta      = h_accel_step;
		nmov_linseg  = 0;
		nmov_Td      = h_step - h_accel_step ;

	}

	// Print info
	Serial.print("Steps of Accel Phase(nmov_Ta)                 = "); Serial.print(nmov_Ta);     Serial.println(" [steps] ");
    Serial.print("Steps of Constant Velocity Phase(nmov_linseg) = "); Serial.print(nmov_linseg); Serial.println(" [steps] ");
    Serial.print("Steps of Deccel Phase(nmov_Td)                = "); Serial.print(nmov_Td);     Serial.println(" [steps] ");   

// Execute Real time profile - > Calls function to execute Stepper motion

	return_function_state = executeStepperTrapzProfile_rad();
    if (return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }


} // END OF FUNCTION