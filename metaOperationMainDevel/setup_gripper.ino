void setup_gripper()
{
  /*
   * Reads gripper state from eeprom
   */
   DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("READING CURRENT GRIPPER STATE FROM EEPROM"));
   
   OvidiusGripper.readGripperStateEEPROM(&GripperCurrentState);
    
  /*
   * Opens gripper if closed in order to continue
   */
  if ( GripperCurrentState == tools::GRIPPER_CLOSED)
  {
    DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("GRIPPER IS CLOSED: OPENING..."));
    OvidiusGripper.openGripper(ptr2OvidiusGripperServo,&GripperCurrentState);
  }
  else
  {
    DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("GRIPPER IS OPENED"));
  }

  // just for show
  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("GRIPPER CLOSED"));
  OvidiusGripper.closeGripper(ptr2OvidiusGripperServo,&GripperCurrentState);  
  
  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("GRIPPER OPENED"));
  OvidiusGripper.openGripper(ptr2OvidiusGripperServo,&GripperCurrentState); 
  
  /*
   * Calibrates FSR sensor-returns the default voltage measured and stores it as offset value
   */
  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("CALIBRATING DEFAULT VOLTAGE(ANALOG)"));
  offset_analog_reading = OvidiusGripper.setupGripper();
  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.print(F("DEFAULT VOLTAGE[mV]: ")); DEBUG_SERIAL.println(offset_analog_reading);

  // just for show
  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("GRIPPER CLOSED WITH FORCE"));
  OvidiusGripper.closeGripperForce(ptr2OvidiusGripperServo,grasp_force_limit_newton, &GripperCurrentState);

  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("GRIPPER OPENED"));
  OvidiusGripper.openGripper(ptr2OvidiusGripperServo,&GripperCurrentState); 
    
  return;
}
