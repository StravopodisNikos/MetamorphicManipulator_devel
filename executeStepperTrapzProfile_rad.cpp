bool executeStepperTrapzProfile_rad(bool segmentExists, long *PROFILE_STEPS){

	/*
	 *  Runs Stepper for predefined angle with Trapezoidal Velocity Profile
	 *  INPUT: segmentExists, PROFILE_STEPS = {nmov_Ta, nmov_linseg, nmov_Td}, 
	 */

    // Initialize counters for Profile Execution
	long StpPresentPosition = 0;													
    long accel_count = 0; 
    long ctVel_count = 0;
    long decel_count = -nmov_Td; 

    // Determine initial step delay time for real-time profile generation
    float sqrt_for_c0_calc =  (2 * ag) / StpTrapzProfParams[4] ;
    double delta_t         =  0.676 * ft * sqrt(sqrt_for_c0_calc);							// c0 with ignored inaccuracy factor [sec]
    Serial.print("Initial Step Delay Time(c0)                   = "); Serial.print(delta_t,6);   Serial.println(" [secs] ");

    // Initialize variable for timing/derivative calculations
    unsigned long time_duration;
    unsigned long motor_movement_start = millis();

    // Stepping loop
    do
    {
    		// IX.aa. Move stepper using Trapz Vel Profile
          StpPresentPosition++;
          // IV.b.1.I. Locate position in ramp
          if(segmentExists)                                                                             // Linear Segment exists
          {
                //Serial.println("Segment exists");                               
                if(StpPresentPosition<nmov_Ta){
                  //Serial.println("Acceleration Phase");
                  accel_count++;                                                                        // Acceleration Phase: delta_t -> minimizes
                  new_delta_t =  delta_t - RAMP_FACTOR * ( (2*delta_t+rest)/(4*accel_count+1) );                // c_n [sec]
                  //new_rest = (2*delta_t + rest)*mod(4*accel_count+1);                                 // r_n
                }else if( StpPresentPosition>nmov_Ta && StpPresentPosition<(nmov_Ta+nmov_linseg) ){     // Linear Segment: delta_t -> constant
                  ctVel_count++;
                  //Serial.printf("Accel Phase: %ld \n",accel_count);
                  //Serial.println("Constant Velocity Phase");
                  new_delta_t = delta_t;  
                }
                else{
                  //Serial.printf("CtVel Phase steps: %ld \n",ctVel_count);
                  //Serial.println("Decelleration Phase");
                  decel_count++;                                                                        
                  //Serial.println(decel_count);
                  new_delta_t =  delta_t - RAMP_FACTOR * ( (2*delta_t+rest)/(4*decel_count+1) );        // Deceleration Phase: delta_t -> maximizes [sec] 
                }                                                                         
          }
          else
          {                                                                                             // Linear Segment doesn't exist
                //Serial.println("Segment doesn't exist");
                if(StpPresentPosition<nmov_Ta)                                                          // Acceleration Phase: delta_t -> minimizes
                {
                  //Serial.println("Acceleration Phase");
                  accel_count++;
                  new_delta_t = delta_t - RAMP_FACTOR * ((2*delta_t+rest)/(4*accel_count+1) );          // c_n [sec]

                }                                   
                else{                                                                                   // Deceleration Phase: delta_t -> maximizes
                  //Serial.println("Decelleration Phase");
                  decel_count++;                                                                        // Negative Value!
                  new_delta_t = delta_t - RAMP_FACTOR * ((2*delta_t+rest)/(4*decel_count+1) );          // Deceleration Phase: delta_t -> maximizes [sec] 
                }                                                                       
          }

          unsigned long new_delta_t_micros = (new_delta_t*1000000);

          singleStepVarDelay(new_delta_t_micros);

          delta_t = new_delta_t;

          time_duration = millis() - motor_movement_start;
          float_time_duration = time_duration / 1000.0;

    // loop RUNS for: running period shorter than dynamixel running AND number of stepps less than target      
    }while( ( float_time_duration < ( ETDF * Texec) ) && (abs(h_step - StpPresentPosition) != 0) );

return true;

} // END OF FUNCTION