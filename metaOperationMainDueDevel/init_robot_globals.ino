void init_robot_globals()
{
  // assign values to STEPPER globals
  currentAbsPos_double  = 0;     // must print message that if stepper is not homed user MUST execute homing!
  currentDirStatus      = CCW;   // Because SW5 is OFF=>DIR=CCW == DIR=HIGH 
  VelocityLimitStp      = joint_velocities_limits[0];
  AccelerationLimitStp  = joint_accelerations_limits[0];
  MaxPosLimitStp        = 2.0944; // [rad] ~ 120 [deg]
  // assign values to GRIPPER globals
  GripperCurrentState   = tools::GRIPPER_OPENED;

  //assign value to JOINT1 current sensor
  joint1_current_measurement = 380.0; // [mA] default voltage when stepper doesn't move
  
  // serial monitor print the data assigned
  DEBUG_SERIAL.println(F(" [ INFO ]  STP GLOBALS INITIALIZATION  "));
  DEBUG_SERIAL.println(F("[========================================]"));
  DEBUG_SERIAL.print(F("| CURRENT STEPPER POSITION   [rad]       | ")); DEBUG_SERIAL.println(currentAbsPos_double);
  DEBUG_SERIAL.print(F("| CURRENT STEPPER DIRECTION  [CW=0/CCW=1]| ")); DEBUG_SERIAL.println(currentDirStatus);
  DEBUG_SERIAL.print(F("| STEPPER VELOCITY LIMIT     [rad/sec]   | ")); DEBUG_SERIAL.println(VelocityLimitStp);
  DEBUG_SERIAL.print(F("| STEPPER ACCELERATION LIMIT [rad/sec2]  | ")); DEBUG_SERIAL.println(AccelerationLimitStp);
  DEBUG_SERIAL.print(F("| STEPPER POSITION LIMIT     [rad]       | ")); DEBUG_SERIAL.println(MaxPosLimitStp);
  DEBUG_SERIAL.println(F("[========================================]"));

  return;
}
