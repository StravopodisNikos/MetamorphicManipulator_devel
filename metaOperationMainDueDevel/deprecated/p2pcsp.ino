  void p2pcsp(double * q_i, double * q_f, double Texec)
  {
  /*
   * This function is based on code executed @ metaOperationMainDxlShield for sync move Stp+Dxls.
   * Var matching:
   * q_f -> [ goalAbsPos_double,Qdxl_f[3] ] is a 4x1 array
   * q_i -> [ currentAbsPos_double,Qdxl_i[3] ] is a 4x1 array
   * Author N. Stravopodis
   */
  //int ERROR_RETURNED;
  DEBUG_SERIAL.println(F("[  INFO  ] EXECUTING P2PCSP... "));
  
  // give value to global variables for current/goal joint 1 position
  currentAbsPos_double = q_i[0];
  goalAbsPos_double = q_f[0];

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
  
  /*
   * Declare local variables for p2p motion.
   */
  double Vexec;
  double Aexec;
  double Taccel; 
  uint32_t RELATIVE_STEPS_2_MOVE;
  bool LIN_SEG_EXISTS;

  // 1.STEPPER PRE SETTING TO COMPUTE Texec,Ta
  uint32_t P2P_PROF_STEPS[4];
  return_function_state =  stp.syncPreSetStepperGoalPositionVarStep(&currentAbsPos_double, &goalAbsPos_double, &Vexec, &Aexec, &Texec, &Taccel, &currentDirStatus, &LIN_SEG_EXISTS, P2P_PROF_STEPS,  &error_code_received);
  if (return_function_state)
  {
    DEBUG_SERIAL.println(F("[  INFO  ] PRE SET STEPPER JOINT1 [  SUCCESS ]"));
  }
  else
  {
    DEBUG_SERIAL.println(F("[  ERROR  ] PRE SET STEPPER JOINT1 [  FAILED ]"));
  }
  
  // 2.DYNAMIXELS CALCULATE VEL/ACCEL PROFILE FOR SYNCED MOTION
  dxl_prof_vel[0] = 1000; 
  dxl_prof_vel[1] = 1000;
  dxl_prof_vel[2] = 1000;
  dxl_prof_accel[0] = 1000; 
  dxl_prof_accel[1] = 1000;
  dxl_prof_accel[2] = 1000;

  // convert Delta_Pos matrices to pulses
  double Qdxl_rel_dpos[sizeof(dxl_id)];
  double Qdxl_i[sizeof(dxl_id)];
  double Qdxl_f[sizeof(dxl_id)];
  Qdxl_f[0] = q_f[1];
  Qdxl_f[1] = q_f[2];
  Qdxl_f[2] = q_f[3];
  Qdxl_i[0] = q_i[1];
  Qdxl_i[1] = q_i[2];
  Qdxl_i[2] = q_i[3];
  Qdxl_rel_dpos[0] = Qdxl_f[0] - Qdxl_i[0];
  Qdxl_rel_dpos[1] = Qdxl_f[1] - Qdxl_i[1];
  Qdxl_rel_dpos[2] = Qdxl_f[2] - Qdxl_i[2];
  return_function_state = meta_dxl.calculateProfVelAccel_preassignedVelTexec(dxl_prof_vel, dxl_prof_accel, Qdxl_rel_dpos, Taccel, Texec, &error_code_received);
  if (return_function_state)
  {
    DEBUG_SERIAL.println(F("[  INFO  ] PRE SET DYNAMIXELS [  SUCCESS ]"));
    DEBUG_SERIAL.print(F("[  INFO  ] RECALCULATED PV[0] : ")); DEBUG_SERIAL.println(dxl_prof_vel[0]);
    DEBUG_SERIAL.print(F("[  INFO  ] RECALCULATED PV[1] : ")); DEBUG_SERIAL.println(dxl_prof_vel[1]);
    DEBUG_SERIAL.print(F("[  INFO  ] RECALCULATED PV[2] : ")); DEBUG_SERIAL.println(dxl_prof_vel[2]);
    
    DEBUG_SERIAL.print(F("[  INFO  ] RECALCULATED PA[0] : ")); DEBUG_SERIAL.println(dxl_prof_accel[0]);
    DEBUG_SERIAL.print(F("[  INFO  ] RECALCULATED PA[1] : ")); DEBUG_SERIAL.println(dxl_prof_accel[1]);
    DEBUG_SERIAL.print(F("[  INFO  ] RECALCULATED PA[2] : ")); DEBUG_SERIAL.println(dxl_prof_accel[2]);
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }
  else
  {
    DEBUG_SERIAL.println(F("[  ERROR  ] PRE SET DYNAMIXELS [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(error_code_received);
  }

  //dxl_prof_vel[0] = 500;
  //dxl_prof_vel[1] = 500;
  //dxl_prof_vel[2] = 500;
  
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
  
  //dxl_prof_accel[0] = 1000;
  //dxl_prof_accel[1] = 1000;
  //dxl_prof_accel[2] = 1000;
   
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
  unsigned long motor_movement_start = millis();
  
  dxl_goal_position[0] = meta_dxl.convertRadian2DxlPulses(Qdxl_f[0]);
  dxl_goal_position[1] = meta_dxl.convertRadian2DxlPulses(Qdxl_f[1]);
  dxl_goal_position[2] = meta_dxl.convertRadian2DxlPulses(Qdxl_f[2]);
  DEBUG_SERIAL.print(F("[  INFO  ] SETTING DXL GP[0] : ")); DEBUG_SERIAL.println(dxl_goal_position[0]);
  DEBUG_SERIAL.print(F("[  INFO  ] SETTING DXL GP[1] : ")); DEBUG_SERIAL.println(dxl_goal_position[1]);
  DEBUG_SERIAL.print(F("[  INFO  ] SETTING DXL GP[2] : ")); DEBUG_SERIAL.println(dxl_goal_position[2]);
  
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
  return_function_state =  stp.syncSetStepperGoalPositionVarStep(&currentAbsPos_double, &goalAbsPos_double, &Aexec, &Texec,  &currentDirStatus, &KILL_MOTION, &LIN_SEG_EXISTS, P2P_PROF_STEPS,   &error_code_received);
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
