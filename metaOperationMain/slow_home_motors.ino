  void slow_home_motors()
  {
    // 1.HOME stepper 
    DEBUG_SERIAL.println(F("[  INFO  ] HOMING STEPPER MOTOR... "));

    return_function_state = stp.setStepperHomePositionSlow( &currentAbsPos_double, &currentDirStatus, &KILL_MOTION, &stp_error);
    if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SLOW HOMING STEPPER MOTOR [  SUCCESS ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
    }
    else
    {
      DEBUG_SERIAL.println(F("[    ERROR   ] SLOW HOMING STEPPER MOTOR [  FAILED ]"));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
    }

    // 2.HOME DYNAMIXELS
    DEBUG_SERIAL.println(F("[  INFO  ] HOMING DYNAMIXEL MOTORS... "));

    // 2.1 Torque ON Dynamixels
    return_function_state = meta_dxl.setDynamixelsTorqueON(dxl_id, sizeof(dxl_id), dxl);
    if (return_function_state){
      DEBUG_SERIAL.println(F("[  INFO  ] TORQUE ON DYNAMIXELS [  SUCCESS ]"));
    }
    else
    {
      DEBUG_SERIAL.println(F("[  ERROR  ] TORQUE ON DYNAMIXELS  [  FAILED ]"));
    }
  
    for(size_t id_count = 0; id_count < sizeof(dxl_id); id_count++){
      dxl_goal_position[id_count] = 0;
    }

    // 2.2 Move Dynamixels with homing velocity
    dxl_prof_vel[0] = 500;
    dxl_prof_vel[1] = 500;
    dxl_prof_vel[2] = 500;
    return_function_state = meta_dxl.syncSetDynamixelsProfVel(dxl_id, sizeof(dxl_id), dxl_prof_vel, sw_data_array_pv, &error_code_received, dxl);
    
    return_function_state = meta_dxl.syncSetDynamixelsGoalPosition(dxl_id, sizeof(dxl_id), dxl_goal_position, sw_data_array_gp,&error_code_received, dxl);
    if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC WRITE HOME POSITION DYNAMIXELS [  SUCCESS ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
    }
    else
    {
      DEBUG_SERIAL.println(F("[    ERROR   ] SYNC WRITE HOME POSITION DYNAMIXELS [  FAILED ]"));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
    }

    delay(3000); // wait for Dynamixels to HOME
    
    // 2.3 LED INDICATOR
    return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), turn_off_led, dxl);
    return_function_state = meta_dxl.setDynamixelLeds(dxl_id, sizeof(dxl_id), motors_homed_indicator, dxl);

    // 2.4 Torque Off Dynamixels
    return_function_state = meta_dxl.setDynamixelsTorqueOFF(dxl_id, sizeof(dxl_id), dxl);
    if (return_function_state){
      DEBUG_SERIAL.println(F("[  INFO  ] TORQUE OFF DYNAMIXELS [  SUCCESS ]"));
    }
    else
    {
      DEBUG_SERIAL.println(F("[  ERROR  ] TORQUE OFF DYNAMIXELS  [  FAILED ]"));
    }
    
    return;
  } //end function
