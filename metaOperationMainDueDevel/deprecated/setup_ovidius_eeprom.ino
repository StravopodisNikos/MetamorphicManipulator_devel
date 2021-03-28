void setup_ovidius_eeprom()
{
  // STEPPER
  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SETTING STEPPER EEPROM MEMORY"));
  
  VelocityLimitStp = joint_velocities_limits[0];     // [rad/sec]
  AccelerationLimitStp = joint_accelerations_limits[0]; // [rad/sec^2]
  MaxPosLimitStp = 2.618;      // [rad] =~150 deg
  
  //stp.save_STP_EEPROM_settings(&currentDirStatus, &currentAbsPos_double, &VelocityLimitStp, &AccelerationLimitStp, &MaxPosLimitStp);
  DEBUG_SERIAL.println(F("[  INFO  ] STEPPER SAVED EEPROM  [  SUCCESS ]"));

  // GRIPPER
  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("SETTING GRIPPER EEPROM MEMORY"));
  
  GripperCurrentState = tools::GRIPPER_OPENED;
  
  //OvidiusGripper.writeGripperStateEEPROM(&GripperCurrentState);

  DEBUG_SERIAL.println(F("[  INFO  ] GRIPPER SAVED EEPROM  [  SUCCESS ]"));
      
  return;
}
