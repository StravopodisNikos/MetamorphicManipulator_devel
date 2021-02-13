void read_current_configuration(bool access_eeprom)
{

  // Read Joint1(Stepper Motor)
  if(access_eeprom)
  {
    stp.read_STP_EEPROM_settings(&currentDirStatus, &currentAbsPos_double, &VelocityLimitStp, &AccelerationLimitStp, &MaxPosLimitStp);
    currentConfiguration[0] = currentAbsPos_double;
  }
  else
  {
    currentConfiguration[0] = currentAbsPos_double;
  }

  // Sync Read Dynamixels Present Position 
  return_function_state = meta_dxl.syncGetDynamixelsPresentPosition(dxl_id, sizeof(dxl_id), dxl_present_position, sr_data_array_pp,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println("[    INFO    ] SYNC READ PRESENT POSITION DYNAMIXELS [  SUCCESS ]");
  }
  else
  {
    DEBUG_SERIAL.println("[    ERROR   ] SYNC READ PRESENT POSITION DYNAMIXELS [  FAILED ]");
    DEBUG_SERIAL.print("[  ERROR CODE  ]"); DEBUG_SERIAL.println(error_code_received);
  }

  // convert DxlPulses to rad and save to global array
  currentConfiguration[1] = meta_dxl.convertDxlPulses2Radian(dxl_present_position[0]);
  currentConfiguration[2] = meta_dxl.convertDxlPulses2Radian(dxl_present_position[1]);
  currentConfiguration[3] = meta_dxl.convertDxlPulses2Radian(dxl_present_position[2]);

  DEBUG_SERIAL.print(" [ INFO ] "); DEBUG_SERIAL.println("CURRENT ROBOT CONFIGURATION:"); 
  for (size_t i = 0; i < nDoF; i++)
  {
    DEBUG_SERIAL.print(" [ JOINT "); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(" ] "); DEBUG_SERIAL.println(currentConfiguration[i],4);
  }
      
  return;
}
