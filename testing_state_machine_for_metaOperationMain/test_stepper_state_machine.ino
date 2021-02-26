void test_stepper_state_machine()
{
  // =========================================================
  currentAbsPos_double = 0;                               
  p2pcsp_Texec = 1.0;
  double Vexec = 5.0;
  double Aexec = 50.0;
  goalAbsPos_double = 1.5708;
  p2pcsp_Ta = (1/ACCEL_WIDTH_DENOM) * p2pcsp_Texec;
  // =========================================================

  bool LIN_SEG_EXISTS;
  uint32_t P2P_PROF_STEPS[4];
  return_function_state =  stp.syncPreSetStepperGoalPositionVarStep2(&currentAbsPos_double, &goalAbsPos_double, &Vexec, &Aexec, &p2pcsp_Texec, &p2pcsp_Ta, &currentDirStatus, &LIN_SEG_EXISTS, P2P_PROF_STEPS,  &error_code_received);
  if (return_function_state)
  {
    Serial.println(F("[  INFO  ] PRE SET STEPPER JOINT1 [  SUCCESS ]"));
  }
  else
  {
    Serial.println(F("[  ERROR  ] PRE SET STEPPER JOINT1 [  FAILED ]"));
    Serial.print(F("[  ERROR CODE  ]"));Serial.println(error_code_received);
  }

  return_function_state =  stp.syncSetStepperGoalPositionVarStep(&currentAbsPos_double, &goalAbsPos_double, &Aexec, &p2pcsp_Texec,  &currentDirStatus, &KILL_MOTION, &LIN_SEG_EXISTS, P2P_PROF_STEPS,   &error_code_received);
  if (return_function_state){
    Serial.println(F("[    INFO    ] SYNC WRITE GOAL POSITION STEPPER [  SUCCESS ]"));
    Serial.print(F("[  ERROR CODE  ]"));Serial.println(error_code_received);
  }
  else
  {
    Serial.println(F("[    ERROR   ] SYNC WRITE GOAL POSITION STEPPER [  FAILED ]"));
    Serial.print(F("[  ERROR CODE  ]"));Serial.println(error_code_received);
  }
  
  
  return;
}
