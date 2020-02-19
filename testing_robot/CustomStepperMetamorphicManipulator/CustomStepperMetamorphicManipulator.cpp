 /*
  * CustomStepperMetamorphicManipulator.cpp - Library for controlling Steppers of a Metamorphic Manipulator
  * Created by N.A. Stravopodis, February, 2020.
  */

#include "Arduino.h"
//#include <vector>
#include <Vector.h>
#include <Streaming.h>
#include "CustomStepperMetamorphicManipulator.h"

//using namespace std;

// Constructor
CustomStepperMetamorphicManipulator::CustomStepperMetamorphicManipulator(int stepID, int stepPin, int dirPin, int enblPin, int ledPin, int hallSwitchPin, int spr, int GEAR_FACTOR, int ft )
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enblPin, OUTPUT);
    pinMode(ledPin, OUTPUT);          
    pinMode(hallSwitchPin, INPUT);

    digitalWrite(stepPin, LOW);
    digitalWrite(ledPin, LOW);
    digitalWrite(enblPin, LOW);
    digitalWrite(dirPin, LOW);

    _stepID         = stepID;
    _stepPin        = stepPin;
    _dirPin         = dirPin;
    _enblPin        = enblPin;
    _ledPin         = ledPin;
    _hallSwitchPin  = hallSwitchPin;

    _spr            = spr;                                  // Steps per revolution[found from driver dip switch configuration]
    _GEAR_FACTOR    = GEAR_FACTOR;                          // Gearbox Reduction of stepper motor
    _ft             = ft;                                   // Frequency of stepper driver

    _a              = ( 2 * pi ) / (  _spr );               // Stepper Motor Step Angle(Angular Position of Motor shaft)[rad]
    _ag             = ( 2 * pi ) / ( _GEAR_FACTOR * _spr ); // Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]
    _accel_width    = 0.10;                                 // Acceleration Phase width

}

// =========================================================================================================== //

// singleStepVarDelay
void CustomStepperMetamorphicManipulator::singleStepVarDelay(unsigned long delayTime) {
  // Custom Stepping Function for Velocity-Acceleration Profiles

    unsigned long time_now_micros = micros();
    digitalWrite(_stepPin, HIGH);
    while(micros() < time_now_micros + delayTime){}                   //wait approx. [μs]
    digitalWrite(_stepPin, LOW);
} // END function singleStepVarDelay

// =========================================================================================================== //

// returnTrajAssignedDurationProperties
Vector<double> CustomStepperMetamorphicManipulator::returnTrajAssignedDurationProperties(double Texec, double hRel) {

  // Define return vector
  double storage_array_for_TrajAssignedDuration[5];

	// Pre - determine the width of acceleration phase
	// Ta = accel_width * Texec
	double Ta = _accel_width * Texec;

	// Compute Vmax, Amax

	double Vmax =   abs( hRel / ( (1 - _accel_width) * Texec ) ) ;
  Serial.print("Vmax = "); Serial.println(Vmax,6);
	double Amax = abs( hRel / ( _accel_width * ( 1 - _accel_width) * pow(Texec,2.0) ) );
 	Serial.print("Amax = "); Serial.println(Amax,6);
  
  //double array_to_fill_vector[] = {hRel, Texec, Ta, Vmax, Amax};
  storage_array_for_TrajAssignedDuration[0] = hRel;
  storage_array_for_TrajAssignedDuration[1] = Texec;
  storage_array_for_TrajAssignedDuration[2] = Ta;
  storage_array_for_TrajAssignedDuration[3] = Vmax;
  storage_array_for_TrajAssignedDuration[4] = Amax;

  Vector<double> TrajAssignedDuration(storage_array_for_TrajAssignedDuration, 5);

  Serial << "TrajAssignedDuration.max_size(): " << TrajAssignedDuration.max_size() << endl;
  Serial << "TrajAssignedDuration:" << TrajAssignedDuration << endl;

	return TrajAssignedDuration;
} // END function returnTrajAssignedDurationProperties

// =========================================================================================================== //

// executeStepperTrapzProfile
bool CustomStepperMetamorphicManipulator::executeStepperTrapzProfile(bool segmentExists, Vector<unsigned long> PROFILE_STEPS, double Texec, double delta_t){

	/*
	 *  Runs Stepper for predefined angle with Trapezoidal Velocity Profile
	 *  INPUT: segmentExists, PROFILE_STEPS = {h_step, nmov_Ta, nmov_linseg, nmov_Td}, delta_t = c0(initial step delay), Texec=Motion Execution Time, delta_t = Initian step delay
	 */

    const float RAMP_FACTOR       = 0.90;                        // Velocity Profile ramp slope factor
    const float ETDF              = 1.40;                        // Execution Time Deviation Factor (experimental calculation)

    // Initialize counters for Profile Execution
    long StpPresentPosition = 0;													
    long accel_count = 0; 
    long ctVel_count = 0;
    long decel_count = -PROFILE_STEPS[3]; 

    // Initialize variable for timing/derivative calculations
    unsigned long time_duration;
    float float_time_duration;
    double new_delta_t;
    double rest = 0;
    unsigned long motor_movement_start = millis();

    // Stepping loop
    do
    {
    		// IX.aa. Move stepper using Trapz Vel Profile
          StpPresentPosition++;
          // IV.b.1.I. Locate position in ramp
          if(segmentExists)                                                                                                     // Linear Segment exists
          {
                if(StpPresentPosition < PROFILE_STEPS[1]){
                  accel_count++;                                                                                                // Acceleration Phase: delta_t -> minimizes
                  new_delta_t =  delta_t - RAMP_FACTOR * ( (2*delta_t+rest)/(4*accel_count+1) );                                // c_n [sec]
                }else if( (StpPresentPosition > PROFILE_STEPS[1]) && (StpPresentPosition < PROFILE_STEPS[1]+PROFILE_STEPS[2]) ){// Linear Segment: delta_t -> constant
                  ctVel_count++;
                  new_delta_t = delta_t;  
                }
                else{
                  decel_count++;                                                                        
                  new_delta_t =  delta_t - RAMP_FACTOR * ( (2*delta_t+rest)/(4*decel_count+1) );                                // Deceleration Phase: delta_t -> maximizes [sec] 
                }                                                                         
          }
          else
          {                                                                                             // Linear Segment doesn't exist
                if(StpPresentPosition < PROFILE_STEPS[1])                                               // Acceleration Phase: delta_t -> minimizes
                {
                  accel_count++;
                  new_delta_t = delta_t - RAMP_FACTOR * ((2*delta_t+rest)/(4*accel_count+1) );          // c_n [sec]
                }                                   
                else{                                                                                   // Deceleration Phase: delta_t -> maximizes
                  decel_count++;                                                                        // Negative Value!
                  new_delta_t = delta_t - RAMP_FACTOR * ((2*delta_t+rest)/(4*decel_count+1) );          // Deceleration Phase: delta_t -> maximizes [sec] 
                }                                                                       
          }

          unsigned long new_delta_t_micros = (new_delta_t*1000000);

          CustomStepperMetamorphicManipulator::singleStepVarDelay(new_delta_t_micros);                  // Executes Motor Step 

          delta_t = new_delta_t;                                                                        // Changes Motor Step Delay

          time_duration = millis() - motor_movement_start;                                              // Calculates Stepper Motion Execution Time 
          float_time_duration = time_duration / 1000.0;
        
        //Serial.print("new_delta_t_micros = "); Serial.println(new_delta_t_micros);
        //Serial.print("StpPresentPosition = "); Serial.println(StpPresentPosition);

    // loop RUNS for: running period shorter than dynamixel running AND number of stepps less than target      
    }while( ( float_time_duration < ( ETDF * Texec) ) && (abs(PROFILE_STEPS[0] - StpPresentPosition) != 0) );


if (abs(PROFILE_STEPS[0] - StpPresentPosition) == 0)
{
  Serial.println("Stepper Trapezoidal Profile IN Goal Position");
  return true;
}
else
{
  Serial.println("Stepper Trapezoidal Profile NOT IN Goal Position");
  return false;
}

} // END OF FUNCTION

// =========================================================================================================== //

// segmentExists_TrapzVelProfile
bool CustomStepperMetamorphicManipulator::segmentExists_TrapzVelProfile(Vector<double> StpTrapzProfParams){

  bool segmentExists;
  float h_cond = pow(StpTrapzProfParams[3],2.0) / StpTrapzProfParams[4];				        // Condition factor for linear segment existence in Trapezoidal Velocity Profile

  if(StpTrapzProfParams[0] >= h_cond)
  {
      Serial.println("Trapezoidal Profile!"); 
      segmentExists = true;
  }
  else
  {
      Serial.println("Triangular Profile! Vmax is recalculated!");
      segmentExists = false;
  }
  
  return segmentExists;
}

// =========================================================================================================== //

// returnTrapzVelProfileSteps
Vector<unsigned long> CustomStepperMetamorphicManipulator::returnTrapzVelProfileSteps(Vector<double> StpTrapzProfParams, bool segmentExists){


  unsigned long storage_array_for_PROFILE_STEPS[4];
  Vector<unsigned long> PROFILE_STEPS; 
  PROFILE_STEPS.setStorage(storage_array_for_PROFILE_STEPS);

  float h_accel;
  unsigned long nmov_Ta;
  unsigned long nmov_Td; 
  unsigned long nmov_linseg;
  unsigned long h_accel_step;
  
  unsigned long h_step = round( StpTrapzProfParams[0] / _ag );                          // Calculate displacement in [steps]

  if(segmentExists)
  {
      // Determine Profile Step Variables based on Real Time Profiles vs Melchiorri
      h_accel   = 0.5 * StpTrapzProfParams[4] * pow(StpTrapzProfParams[2],2.0);
      
      float h_lin_seg = StpTrapzProfParams[4] * _accel_width * pow(StpTrapzProfParams[1],2.0) * ( 1 - 2 * _accel_width);
      
      h_accel_step = round( h_accel / _ag );
      unsigned long h_lin_seg_step = round( h_lin_seg / _ag );

      nmov_Ta      = h_accel_step;                                                         // Steps of Acceleration Phase;
      nmov_linseg  = h_lin_seg_step;                                                       // Steps of liner segment 
      nmov_Td      = h_step - ( h_accel_step + h_lin_seg_step ) ;													 // Steps of Decceleration Phase; 
  }
  else
  {
      // In this case: p2p is executed for Texec with the recalculated Vmax!
      float nTa = sqrt(StpTrapzProfParams[0]/StpTrapzProfParams[4]);

      float nVmax = StpTrapzProfParams[4] * nTa;

      Serial.print("New maximum Velocity:"); Serial.print(nVmax,6); Serial.println("[rad/sec]");

      h_accel  = 0.5 * StpTrapzProfParams[4] * pow(nTa,2.0);
      h_accel_step = round( h_accel / _ag );

      nmov_Ta      = h_accel_step;
      nmov_linseg  = 0;
      nmov_Td      = h_step - h_accel_step ;
  }

  // Print info
	Serial.print("Steps of Accel Phase(nmov_Ta)                 = "); Serial.print(nmov_Ta);     Serial.println(" [steps] ");
  Serial.print("Steps of Constant Velocity Phase(nmov_linseg) = "); Serial.print(nmov_linseg); Serial.println(" [steps] ");
  Serial.print("Steps of Deccel Phase(nmov_Td)                = "); Serial.print(nmov_Td);     Serial.println(" [steps] "); 

  const unsigned long array_to_fill_PROFILE_STEPS[] = {h_step, nmov_Ta, nmov_linseg, nmov_Td};
  PROFILE_STEPS.assign(4, array_to_fill_PROFILE_STEPS);

  //unsigned long array_to_fill_PROFILE_STEPS[] = {h_step, nmov_Ta, nmov_linseg, nmov_Td};
  //Vector<unsigned long> PROFILE_STEPS(array_to_fill_PROFILE_STEPS, 4);

  Serial << "PROFILE_STEPS:" << endl;
  Serial << PROFILE_STEPS << endl;

  return PROFILE_STEPS;
}

// =========================================================================================================== //

// calculateInitialStepDelay
double CustomStepperMetamorphicManipulator::calculateInitialStepDelay(Vector<double> StpTrapzProfParams) {
// Based on Par. 3.2.2. Trajectory Planning for Machines and Robots

	/* 
	 *	StpTrapzProfParams = {h, Texec, Ta, StpVmax, StpAmax }: All given in SI units
	 *  StpInitPos,StpGoalPosition -> [rads]
	 *  StpVmax -> [rad/sec]
	 *  StpAmax -> [rad/sec^2]
	 */

  // Determine initial step delay time for real-time profile generation: c0 with ignored inaccuracy factor [sec]
  float sqrt_for_c0_calc =  (2 * _ag) / StpTrapzProfParams[4] ;
  double c0              =  0.676 * _ft * pow(10.0,-6.0) * sqrt(sqrt_for_c0_calc);							                  
  Serial.print("Initial Step Delay Time(c0)                   = "); Serial.print(c0,6);        Serial.println("  [secs] ");

return c0;

} // END OF FUNCTION

// =========================================================================================================== //

// setStepperHomePosition
bool CustomStepperMetamorphicManipulator::setStepperHomePosition(){

  unsigned long homing_stepping_delay = 250;

  Serial.print("[ Stepper ID: "); Serial.print(_stepID); Serial.println("] HOMING ... ");
  
  digitalWrite(_ledPin, HIGH);                                                                          
  
  while( (digitalRead(_hallSwitchPin) == 0 ) )
  {                                                                        
      CustomStepperMetamorphicManipulator::singleStepVarDelay(homing_stepping_delay);                  
  }

  long currentAbsPos = 10;

  Serial.print("[ Stepper ID: "); Serial.print(_stepID); Serial.println("] HOMING FINISHED ");
  
  digitalWrite(_ledPin, LOW);                        

return true;
} // END OF FUNCTION

// =========================================================================================================== //

// setStepperGoalPosition
double CustomStepperMetamorphicManipulator::setStepperGoalPositionAssignedDuration(double Texec, double hAbs){

 	/*
	 *  Moves Stepper to goal position with Trapezoidal Velocity Profile
	 *  INPUT: Texec = Motion Execution Time [secs] , h = Absolute Angular Position [rad]
   *  OUTPUT: hRelative = Relative Angular Movement [rad]
	 */ 
  double hRelative;

  Serial.print("Texec="); Serial.println(Texec,6);
  Serial.print("hAbs="); Serial.println(hAbs,6);

  int newDirStatus;

  long inputAbsPos = round( hAbs / _ag);

  int previousDirStatus = currentDirStatus;
  long previousMoveRel  = currentMoveRel; 
  long previousAbsPos   = currentAbsPos;
  
  if (inputAbsPos == previousAbsPos){
      return false;                                 // Already there
  }

  long moveRel = inputAbsPos - previousAbsPos;
  hRelative = abs( moveRel * _ag);
  Serial.print("hRelative = "); Serial.println(hRelative,6);

  if( moveRel*previousMoveRel >=0 ){
      newDirStatus = previousDirStatus;
      digitalWrite(_dirPin, newDirStatus);     	  // Direction doesn't change
  }else{
      //Serial.println("ok1");
      newDirStatus = !previousDirStatus;
      digitalWrite(_dirPin, newDirStatus);       	// Direction changes
  }
    Serial.print("newDirStatus = "); Serial.println(newDirStatus);
    //Serial.println(moveRel);

    // Saves for next call
    currentDirStatus = newDirStatus;
    currentMoveRel   = moveRel;
    currentAbsPos    = inputAbsPos;

    // Calculate Velocity - Acceleration
    return hRelative;
}

// =========================================================================================================== //
