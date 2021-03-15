  void p2pcsp2_sm(double * q_i, double * q_f, double Texec)
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

  // Open files for data logging 
  // until 15-3-21 only 3 data logs will be used(pos/vel/force)
  // open the log files for writing. Writing in log will be called inside:
  // syncSetStepperGoalPositionVarStep2/execute_StpTrapzProfile2 
  //open_logfiles();
  
  // give value to global variables for current/goal joint 1 position
  currentAbsPos_double = q_i[0];
  goalAbsPos_double = q_f[0];
  double Vexec;
  double Aexec;
  
  // build the Delta_Pos matrix
  double Qrel_dpos[4];
  for (size_t i = 0; i < nDoF; i++)
  {
    Qrel_dpos[i] = q_f[i] - q_i[i];
  }

  p2pcsp_Ta = (double) (1.0 / ACCEL_WIDTH_DENOM) * Texec;
  /*
   * First sync motion V,A and new Texec are calculated
   */
  return_function_state = meta_dxl.calculateProfVelAccel_preassignedVelTexec3(p2pcsp_joint_velocities, p2pcsp_joint_accelerations, Qrel_dpos, &p2pcsp_Ta, &Texec, joint_velocities_limits, joint_accelerations_limits, &error_code_received);
  if (return_function_state)
  {
    DEBUG_SERIAL.println(F("[  INFO  ] PRE SET STEPPER JOINT1 [  SUCCESS ]"));
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
    DEBUG_SERIAL.println(F("[  INFO  ] PRE SET STEPPER JOINT1 [  SUCCESS ]"));
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
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
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

  // 4.DYNAMIXELS SET GOAL POSITION
  //dxl_goal_position[0] = meta_dxl.convertRadian2DxlPulses(q_f[1]);
  //dxl_goal_position[1] = meta_dxl.convertRadian2DxlPulses(q_f[2]);
  //dxl_goal_position[2] = meta_dxl.convertRadian2DxlPulses(q_f[3]);
  //DEBUG_SERIAL.print(F("[  INFO  ] SETTING DXL GP[0] : ")); DEBUG_SERIAL.println(dxl_goal_position[0]);
  //DEBUG_SERIAL.print(F("[  INFO  ] SETTING DXL GP[1] : ")); DEBUG_SERIAL.println(dxl_goal_position[1]);
  //DEBUG_SERIAL.print(F("[  INFO  ] SETTING DXL GP[2] : ")); DEBUG_SERIAL.println(dxl_goal_position[2]);
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

  // 5.STEPPER SET GOAL POSITION - HERE STATE MACHINE FUNCTION IS IMPLEMENTED!!! [7-3-21]
  bool update_force_during_exec = true;
  bool update_imu_during_exec = false;
  float FORCE_MEASUREMENTS[num_FORCE_SENSORS];
  sensors::imu_filter selected_filter = sensors::imu_filter::MAHONY_F;
  PTR_2_meta_dxl = &meta_dxl; PTR_2_dxl_pp_packet = &dxl_pp_packet; PTR_2_dxl_pv_packet = &dxl_pv_packet;
  return_function_state =  stp.syncSetStepperGoalPositionVarStep2(PTR2RobotDataLog, LOGFILES, IMUSensor, PTR_2_imu_packet, selected_filter, ForceSensor, ForceSensorHX711, FORCE_MEASUREMENTS, &sensor_error, &currentAbsPos_double, &goalAbsPos_double, &Aexec, &p2pcsp_Texec,  &currentDirStatus, &KILL_MOTION, &LIN_SEG_EXISTS, update_force_during_exec,update_imu_during_exec,  P2P_PROF_STEPS,  PTR_2_meta_dxl, PTR_2_dxl_pp_packet, PTR_2_dxl_pv_packet,  &error_code_received);
  if (return_function_state){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC WRITE GOAL POSITION STEPPER [  SUCCESS ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC WRITE GOAL POSITION STEPPER [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
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
