void print_stp_eeprom()
{
  DEBUG_SERIAL.println(F("[  STP EEPROM  ]"));
  DEBUG_SERIAL.println(F("[==============]"));
  DEBUG_SERIAL.print(F("CURRENT STEPPER POSITION   [rad]       ")); DEBUG_SERIAL.println(currentAbsPos_double);
  DEBUG_SERIAL.print(F("CURRENT STEPPER DIRECTION  [CW=0/CCW=1]")); DEBUG_SERIAL.println(currentDirStatus);
  DEBUG_SERIAL.print(F("STEPPER VELOCITY LIMIT     [rad/sec]   ")); DEBUG_SERIAL.println(VelocityLimitStp);
  DEBUG_SERIAL.print(F("STEPPER ACCELERATION LIMIT [rad/sec2]  ")); DEBUG_SERIAL.println(AccelerationLimitStp);
  DEBUG_SERIAL.print(F("STEPPER POSITION LIMIT     [rad]       ")); DEBUG_SERIAL.println(MaxPosLimitStp);
  DEBUG_SERIAL.println(F("[==============]"));

  return;
}
