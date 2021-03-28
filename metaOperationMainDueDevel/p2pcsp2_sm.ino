  void p2pcsp2_sm(float * q_i, float * q_f, float Texec)
  {
  /*
   * This function is based on p2pcsp BUT the corresponding functions
   * for state-machine implementation are executed. Enables real-time
   * monitoring/logging of the manipulator! [7-3-21]
   * 
   * The main devel file is: testing_state_machine_for_metaOperationMain/
   * test_stepper_state_machine.ino 
   * 
   * Var matching:
   * q_f -> [ goalAbsPos_double,Qdxl_f[3] ] is a 4x1 array
   * q_i -> [ currentAbsPos_double,Qdxl_i[3] ] is a 4x1 array
   * Author N. Stravopodis
   */
  DEBUG_SERIAL.println(F("[  INFO  ] EXECUTING P2PCSP... "));

  // pointer to the dataLogger class object
  PTR2RobotDataLog = &RobotDataLog;

  // Create the files for data logging 
  // [15-3-21] Yet only 3 data logs will be used(pos/vel/force)
  // open/close and write in log will be called inside:
  // syncSetStepperGoalPositionVarStep2/execute_StpTrapzProfile2
  // [22-3-21] Added current sensor log file
  // [25-3-21] Crashes when successfully executed at setup and then called here. Also crashes when it is called for the second time!
  // [26-3-21] In order to create files for each p2p a dynamic memory reallocation must be implemented(will be developed in another file.)
  //create_logfiles(); // here FINAL_ACCESSED_FILES array is created (not yet tested) -> [27-3-21] DYN MEMORY FOR CHAR ARRAYS -> TO DO!
  // [27-3-21] Just Serial print the logfiles names-paths -> CHECKED OK [27-3-21]
  //DEBUG_SERIAL.print(F("[ INFO ] WILL WRITE TO LOG FILE:")); DEBUG_SERIAL.print(FINAL_ACCESSED_FILES[0]); 
  //DEBUG_SERIAL.print(F("[ INFO ] WILL WRITE TO LOG FILE:")); DEBUG_SERIAL.print(FINAL_ACCESSED_FILES[1]); 
  //DEBUG_SERIAL.print(F("[ INFO ] WILL WRITE TO LOG FILE:")); DEBUG_SERIAL.print(FINAL_ACCESSED_FILES[2]); 
  //DEBUG_SERIAL.print(F("[ INFO ] WILL WRITE TO LOG FILE:")); DEBUG_SERIAL.print(FINAL_ACCESSED_FILES[3]);
  
  // give value to global variables for current/goal joint 1 position
  currentAbsPos_double = q_i[0];
  goalAbsPos_double = q_f[0];
  float Vexec;
  float Aexec;
  
  // build the Delta_Pos matrix
  float Qrel_dpos[nDoF];
  for (size_t i = 0; i < nDoF; i++)
  {
    Qrel_dpos[i] = q_f[i] - q_i[i];
  }

  p2pcsp_Ta = (float) (1.0 / ACCEL_WIDTH_DENOM) * Texec;
  /*
   * First sync motion V,A and new Texec are calculated
   */
  return_function_state = meta_dxl.calculateProfVelAccel_preassignedVelTexec3(p2pcsp_joint_velocities, p2pcsp_joint_accelerations, Qrel_dpos, &p2pcsp_Ta, &Texec, joint_velocities_limits, joint_accelerations_limits, &error_code_received);
  if (return_function_state)
  {
    DEBUG_SERIAL.println(F("[============================================]"));
    DEBUG_SERIAL.println(F("[  INFO  ] PRE SET STEPPER JOINT1 [  SUCCESS ]"));
    DEBUG_SERIAL.println(F("[  INFO  ] FINAL P2P MOTION PARAMETERS"));
    DEBUG_SERIAL.print(F("Texec -> ")); DEBUG_SERIAL.print(Texec); DEBUG_SERIAL.println(F("[sec]"));
    for (size_t i = 0; i < nDoF; i++)
    {
      DEBUG_SERIAL.print(F(" [ VELOCITY JOINT ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(p2pcsp_joint_velocities[i],4);
    }
    for (size_t i = 0; i < nDoF; i++)
    {
      DEBUG_SERIAL.print(F(" [ ACCELERATION JOINT ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(p2pcsp_joint_accelerations[i],4);
    }    
    DEBUG_SERIAL.println(F("[============================================]"));       
  }
  else
  {
    DEBUG_SERIAL.println(F("[  ERROR  ] PRE SET STEPPER JOINT1 [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }
  /*
   * Now Stepper Parameters must be transformed to steps/var delay
   */
  Vexec = p2pcsp_joint_velocities[0];
  Aexec = p2pcsp_joint_accelerations[0];
  bool LIN_SEG_EXISTS;
  uint32_t P2P_PROF_STEPS[4];
  return_function_state =  stp.syncPreSetStepperGoalPositionVarStep2(&currentAbsPos_double, &goalAbsPos_double, &Vexec, &Aexec, &Texec, &p2pcsp_Ta, &currentDirStatus, &LIN_SEG_EXISTS, P2P_PROF_STEPS,  &error_code_received);
  if (return_function_state)
  {
    DEBUG_SERIAL.println(F("[  INFO  ] STEPPPR VELOCITY PROFILE STEPS [  SUCCESS ]"));
    DEBUG_SERIAL.println(F("[====================================================]"));
    for (size_t i = 0; i < 4; i++)
    {
      DEBUG_SERIAL.print(F(" [ P2P_PROF_STEPS ")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(P2P_PROF_STEPS[i]);
    }
    DEBUG_SERIAL.println(F("[====================================================]"));
  }
  else
  {
    DEBUG_SERIAL.println(F("[  ERROR  ] PRE SET STEPPER JOINT1 [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }

  /*
   * Dxl pulses must be converted
   */
   for(size_t id_count = 0; id_count < sizeof(dxl_id); id_count++)
   {
      dxl_prof_vel[id_count]   = meta_dxl.convertRadPsec2DxlVelUnits(p2pcsp_joint_velocities[id_count+1]);
      dxl_prof_accel[id_count] = meta_dxl.convertRadPsec2_2_DxlAccelUnits(p2pcsp_joint_accelerations[id_count+1]);

      // [27-3-21] Modify acceleration for (yet) undefined bug
      if(dxl_prof_accel[id_count] < MIN_DXL_PA )
      {
        dxl_prof_accel[id_count] = MIN_DXL_PA;
      }
       
      DEBUG_SERIAL.print(F("[  INFO  ] SETTING DXL PV[")); DEBUG_SERIAL.print(id_count); DEBUG_SERIAL.print(F("] : ")); DEBUG_SERIAL.println(dxl_prof_vel[id_count]);
      DEBUG_SERIAL.print(F("[  INFO  ] SETTING DXL PA[")); DEBUG_SERIAL.print(id_count); DEBUG_SERIAL.print(F("] : ")); DEBUG_SERIAL.println(dxl_prof_accel[id_count]);
   }
   
  /*
   * Torque On Dynamixels.
   */
  return_function_state = meta_dxl.setDynamixelsTorqueON(dxl_id, sizeof(dxl_id), dxl);
  if (return_function_state){
    DEBUG_SERIAL.println(F("[  INFO  ] TORQUE ON DYNAMIXELS [  SUCCESS ]"));
  }
  else
  {
    DEBUG_SERIAL.println(F("[  ERROR  ] TORQUE ON DYNAMIXELS  [  FAILED ]"));
  }
  
  // 3.DYNAMIXELS SET VEL/ACCEL PROFILE
  return_function_state = meta_dxl.syncSetDynamixelsProfVel(dxl_id, sizeof(dxl_id), dxl_prof_vel, sw_data_array_pv, &error_code_received, dxl);
  if (return_function_state)
  {
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC WRITE PROFILE VELOCITY DYNAMIXELS  [  SUCCESS ]"));
    //DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC WRITE PROFILE VELOCITY DYNAMIXELS [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }
   
  return_function_state = meta_dxl.syncSetDynamixelsProfAccel(dxl_id, sizeof(dxl_id), dxl_prof_accel, sw_data_array_pa,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC WRITE PROFILE ACCELERATION DYNAMIXELS  [  SUCCESS ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC WRITE PROFILE ACCELERATION DYNAMIXELS [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }

  // [26-3-21] HERE ASKS USER IF @5 LOG DATA MUST BE IMPLEMENTED - MUST BE DONE BEFORE SYNC SET GOAL POS TO DYNAMIXELS!
  set_sensor_updates();
  
  // 4.DYNAMIXELS SET GOAL POSITION
  for(size_t id_count = 0; id_count < sizeof(dxl_id); id_count++)
  {
    dxl_goal_position[id_count] = meta_dxl.convertRadian2DxlPulses(q_f[id_count+1]);
    DEBUG_SERIAL.print(F("[  INFO  ] SETTING DXL GP[")); DEBUG_SERIAL.print(id_count); DEBUG_SERIAL.print(F("] : ")); DEBUG_SERIAL.println(dxl_goal_position[id_count]);
  }
  
  unsigned long motor_movement_start = millis();
  return_function_state = meta_dxl.syncSetDynamixelsGoalPosition(dxl_id, sizeof(dxl_id), dxl_goal_position, sw_data_array_gp,&error_code_received, dxl);
  if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC WRITE GOAL POSITION DYNAMIXELS [  SUCCESS ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC WRITE GOAL POSITION DYNAMIXELS [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }

  // 5.STEPPER SET GOAL POSITION  
  // [7-3-21]  - STATE MACHINE FUNCTION IS IMPLEMENTED
  // [21-3-21] - ADDED STEPPER CHECK IF DXL HAVE FINISHED - MASSIVE CHANGES IN syncSetStepperGoalPositionVarStep3
  // [25-3-21] - asks user which sensor to update/log during p2p execution
  
  PTR_2_meta_dxl = &meta_dxl; PTR_2_dxl_pp_packet = &dxl_pp_packet; PTR_2_dxl_pv_packet = &dxl_pv_packet; PTR_2_dxl_pc_packet = &dxl_pc_packet;
  return_function_state =  stp.syncSetStepperGoalPositionVarStep3(PTR2RobotDataLog, LOGFILES, FINAL_ACCESSED_FILES, ForceSensor, ForceSensorHX711, ptr2joint1_cur_sensor, &sensor_error, &currentAbsPos_double, &goalAbsPos_double, &Aexec, &LIN_SEG_EXISTS, update_sensor_measurements , P2P_PROF_STEPS,  PTR_2_meta_dxl, PTR_2_dxl_pp_packet, PTR_2_dxl_pv_packet, PTR_2_dxl_pc_packet, PTR_2_dxl_mov_packet,  &error_code_received);  
  if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC WRITE GOAL POSITION STEPPER [  SUCCESS ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE - SENSOR]"));DEBUG_SERIAL.println(sensor_error);
    DEBUG_SERIAL.print(F("[  ERROR CODE - STP   ]"));DEBUG_SERIAL.println(error_code_received);
    stp.setStepperLed(stepper_synced_motion_success);
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC WRITE GOAL POSITION STEPPER [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE - SENSOR]"));DEBUG_SERIAL.println(sensor_error);
    DEBUG_SERIAL.print(F("[  ERROR CODE - STP   ]"));DEBUG_SERIAL.println(error_code_received);
    stp.setStepperLed(stepper_critical_error_led);
  }

  // [26-3-21] SYNC MOTION WILL BE EVALUATED HERE!
  // 6. EVALUATE SYNC MOTION
  // Stepper will have terminated since (5) finished, so only dxl must be checked.
  // Waits until dxl respond that finished OR timeouts.
  time_now_millis = millis();
  STOP_WAITING = false;
  while (!stp.getDynamixelsMotionState(&meta_dxl, PTR_2_dxl_mov_packet, &stp_error) && (!STOP_WAITING) )
  {
    DEBUG_SERIAL.println(F("[  INFO  ] DYNAMIXELS STILL MOVING !!!"));

    delay(100);

    total_time_trying = millis() - time_now_millis;
    if (total_time_trying > MOTION_TIMEOUT_MILLIS)
    {
      STOP_WAITING = true;
    }
  }
  
  if( STOP_WAITING )
  {
      DEBUG_SERIAL.println(F("[  INFO  ] P2P CSP  [  FAILED ]"));
  }
  else
  {
      DEBUG_SERIAL.println(F("[  INFO  ] P2P CSP  [  NEEDS ADJUSTMENT ]"));
  }

  p2p_duration = millis() - motor_movement_start;                                              // Calculates Stepper Motion Execution Time 
  double p2p_duration_double = p2p_duration / 1000.0;
  
  DEBUG_SERIAL.print(F("[    INFO    ] TOTAL P2P MOTION DURATION  [sec]:")); DEBUG_SERIAL.println(p2p_duration_double);

  return_function_state = meta_dxl.setDynamixelsTorqueOFF(dxl_id, sizeof(dxl_id), dxl);
  if (return_function_state)
  {
    DEBUG_SERIAL.println(F("[  INFO  ] TORQUE OFF DYNAMIXELS [  SUCCESS ]"));
  }
  else
  {
    DEBUG_SERIAL.println(F("[  ERROR  ] TORQUE OFF DYNAMIXELS  [  FAILED ]"));
  }
  
  return;
} // END FUNCTION
