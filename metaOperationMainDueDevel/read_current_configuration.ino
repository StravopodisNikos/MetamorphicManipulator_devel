void read_current_configuration()
{
  // [21-3-21] Modified in order no EEPROM usage! 
  
  // Read Joint1(Stepper Motor)
  currentConfiguration[0] = currentAbsPos_double;

  // Sync Read Dynamixels Present Position 
  return_function_state = meta_dxl.syncGetDynamixelsPresentPosition(dxl_id, sizeof(dxl_id), dxl_present_position, sr_data_array_pp,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC READ PRESENT POSITION DYNAMIXELS [  SUCCESS ]"));
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC READ PRESENT POSITION DYNAMIXELS [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(error_code_received);
  }

  // convert DxlPulses to rad and save to global array
  currentConfiguration[1] = meta_dxl.convertDxlPulses2Radian(dxl_present_position[0]);
  currentConfiguration[2] = meta_dxl.convertDxlPulses2Radian(dxl_present_position[1]);
  currentConfiguration[3] = meta_dxl.convertDxlPulses2Radian(dxl_present_position[2]);

  DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("CURRENT ROBOT CONFIGURATION:")); 
  for (size_t i = 0; i < nDoF; i++)
  {
    DEBUG_SERIAL.print(F(" [ JOINT ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(currentConfiguration[i],4);
  }

  // [26-3-21] Check if getJointsAbsPosition_rad update function works
  // This must return the same values with above!!! - TESTED-OK
  // [27-3-21] Added timing counter to help me understand real time data logging limits -> removed: Found that need 5.5 msec to respond!
  /*
  bool check_robot_configuration_at_setup = true;
  uint32_t first_step_value = 0; // this makes it to always return zero pos for stepper
  time_now_micros = micros();
  stp.getJointsAbsPosition_rad(currentConfiguration, PTR_2_meta_dxl, PTR_2_dxl_pp_packet, first_step_value, check_robot_configuration_at_setup, &error_code_received);
  total_time_trying = micros() - time_now_micros;
  DEBUG_SERIAL.print(F("[    INFO    ] SYNC READ CONFIGURATION TOTAL TIME: ")); DEBUG_SERIAL.print(total_time_trying); DEBUG_SERIAL.println(F("[us]"));
  if (error_code_received == NO_ERROR){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC READ POSITION [  SUCCESS ]"));
    DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("CURRENT ROBOT JOINTS POSITION [rad]:")); 
    for (size_t i = 0; i < nDoF; i++)
    {
      DEBUG_SERIAL.print(F(" [ JOINT ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(currentConfiguration[i],4);
    }
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC READ POSITION [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(error_code_received);
  }
  */
      
  return;
}
