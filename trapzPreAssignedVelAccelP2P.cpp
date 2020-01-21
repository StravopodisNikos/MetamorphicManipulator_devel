/*
// Stepper Definition
#define dirPin 11
#define stepPin 10
#define enblPin 12

const float pi = 3.14159265359;

void setup() {
  // Serial Communication Baudrate:
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);

  digitalWrite(enblPin, LOW);
  
  const uint8_t id_steppper1 =1;
  float vmax = 10;
  float amax = 0.00005;
  float step0 = 0;
  float step1 = 6.28;
  Serial.println("Axecuting P2P motion with acceleration!");
  trapzPreAssignedVelAccelP2P(id_steppper1,vmax,amax,step0,step1);
}


void loop() {
}
*/


/*
 *	Linear trajectory with Parabolic Blends(Trapezoidal Trajectory)
 *  Trajectory with preassigned acceleration and velocity
 *  I give a_max, v_max and execute simple P2P motion
 *  We don't know if trapezoidal or triangular! => Condition must be formed!
 */

bool trapzPreAssignedVelAccelP2P(uint8_t stp_ID, float Vmax, float Amax, float q0, float q1)	// q:[rad]
{
	unsigned long fn_duration_counter = millis();
	
	float a = 2*pi/1000;																		// [rad], q angle @ spr=1000[steps/rev] 
	float h  = q1 - q0;
	//h = abs(h);																				// Calculate displacement in [rad]
	float h_step1 = h/a;																			// Calculate displacement in [steps]
	long h_step = round(h_step1);
  Serial.println(h_step);
	float hcond = pow(Vmax,2)/Amax;
	float Ta;
  long  nmov_Ta;
  long  nmov_Td;
  bool segmentExists;
  long nmov_linseg;
  
	if(h>=hcond)																			// Checks if linear segment exists Melchiorri
	{
		// equations 3.9 will be executed
		Serial.println("Trapezoidal Profile!");
		Ta = (1.5*Vmax)/Amax;																// Calculate accelration time
		float T  = (h*Amax+pow(Vmax,2))/(Amax*Vmax);										// Calculate total execution time
		segmentExists = true;
		// Calculate Intermediate phase steps
		float qmov_Ta = 0.5*Amax*pow(Ta,2);
		nmov_Ta = qmov_Ta/a;                                        						// Steps of Acceleration Phase;
		nmov_linseg = h_step-2*nmov_Ta;														// Steps of liner segment if Accel=Deccel
	  nmov_Td = nmov_Ta;
	}	
	else
	{
		// equations 3.10 will be executed
		Serial.println("Triangular Profile! Vmax is recalculated!");
		Ta = sqrt(h/Amax);
		float T  = 2*Ta;
		float nVmax = Amax*Ta;
		Serial.printf("New maximum Velocity: %f \n",nVmax);
		segmentExists = false;
		// Calculate Intermediate phase steps
		float qmov_Ta = 0.5*Amax*pow(Ta,2);
		nmov_Ta = qmov_Ta/a;
		nmov_linseg = 0;
		nmov_Td = h_step-nmov_Ta;
	}


	///*
	long step_count = 0;																// Moves motor until specified number of steps is reached
	float delta_t = 0.676*(1/Ta)*(2*a/Amax);											// c0 with ignored inaccuracy factor
	float new_delta_t;
	unsigned long	rest  = 0;
	long accel_count = 0;	
  long ctVel_count = 0;
	long decel_count = -nmov_Td;														// counter of steps executed for Acceleration Phase
	do
	{
		step_count++;
		
		// I. Locate position in ramp

		if(segmentExists)		// Linear Segment exists
		{
      Serial.println("Segment exists");																
			if(step_count<nmov_Ta){
      	Serial.println("Acceleration Phase");
				accel_count++;															// Acceleration Phase: delta_t -> minimizes
				new_delta_t =  delta_t - (2*delta_t+rest)/(4*accel_count+1);			// c_n
				//new_rest = (2*delta_t + rest)*mod(4*accel_count+1);					// r_n
			}else if( step_count>nmov_Ta && step_count<(nmov_Ta+nmov_linseg) ){			// Linear Segment: delta_t -> constant
				ctVel_count++;
				Serial.printf("Accel Phase: %ld \n",accel_count);
				Serial.println("Constant Velocity Phase");
				new_delta_t = delta_t;	
			}
			else{
        Serial.printf("CtVel Phase steps: %ld \n",ctVel_count);
        Serial.println("Decelleration Phase");
				decel_count++;															// Negative Value!
        Serial.println(decel_count);
				new_delta_t =  delta_t - (2*delta_t+rest)/(4*decel_count+1) ;			// Deceleration Phase: delta_t -> maximizes	
			}																																					
		}
		else
		{																				// Linear Segment doesn't exist
      Serial.println("Segment doesn't exist");
			if(step_count<nmov_Ta)														// Acceleration Phase: delta_t -> minimizes
			{
        Serial.println("Acceleration Phase");
				accel_count++;
				new_delta_t = delta_t - (2*delta_t+rest)/(4*accel_count+1);				// c_n

			}																		
			else{   																	// Deceleration Phase: delta_t -> maximizes
        Serial.println("Decelleration Phase");
				decel_count++;															// Negative Value!
				new_delta_t = delta_t - (2*delta_t+rest)/(4*decel_count+1);				// Deceleration Phase: delta_t -> maximizes	
			}																																				
		}
    Serial.printf("New step delay time[s]: %f \n",new_delta_t);

    
		// II. Step Motor
    unsigned long new_delta_t_micros = new_delta_t*1000000;
    Serial.printf("New step delay time[micros]: %lu \n",new_delta_t_micros);
	singleStep(new_delta_t_micros);				

	delta_t = new_delta_t;
    
  	Serial.println(step_count);
	} while (step_count <= h_step);
	//*/
  	Serial.printf("Step count: %ld Target_step: %ld",step_count,h_step);          		// Check if move is finished
	
	unsigned long fn_duration = millis() - fn_duration_counter;
  	//Serial.printf("Real execution time: %lu[millis] Expected execution time: %f[seconds] \n", fn_duration, T);
  
} // END Function

void singleStep(unsigned long delayTime) {
    unsigned long time_now_micros = micros();

    digitalWrite(stepPin, HIGH);
    while(micros() < time_now_micros + delayTime){}										//wait approx. [μs]
    digitalWrite(stepPin, LOW);
}