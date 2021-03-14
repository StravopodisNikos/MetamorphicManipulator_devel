void test_stepper_state_machine()
{
  // =========================================================
  currentAbsPos_double = 0;                               
  p2pcsp_Texec = 1.5;
  double Vexec = 0.5;
  double Aexec = 1.0;
  goalAbsPos_double = 1.5708;
  p2pcsp_Ta = 0.2 * p2pcsp_Texec;
  // =========================================================
  ForceSensor = & SingleForceSensor;
  ForceSensorHX711 = & SingleForceSensorHX711;

  bool LIN_SEG_EXISTS;
  uint32_t P2P_PROF_STEPS[PROF_STEPS_SIZE];
  return_function_state =  stp.syncPreSetStepperGoalPositionVarStep2(&currentAbsPos_double, &goalAbsPos_double, &Vexec, &Aexec, &p2pcsp_Texec, &p2pcsp_Ta, &currentDirStatus, &LIN_SEG_EXISTS, P2P_PROF_STEPS,  &error_code_received);
  if (return_function_state)
  {
    Serial.println(F("[  INFO  ] PRE SET STEPPER JOINT1 [  SUCCESS ]"));
    //Serial.print("STEPS 0 = "); Serial.println(P2P_PROF_STEPS[0]);
    //Serial.print("STEPS 1 = "); Serial.println(P2P_PROF_STEPS[1]);
    //Serial.print("STEPS 2 = "); Serial.println(P2P_PROF_STEPS[2]);
    //Serial.print("STEPS 3 = "); Serial.println(P2P_PROF_STEPS[3]);
  }
  else
  {
    Serial.println(F("[  ERROR  ] PRE SET STEPPER JOINT1 [  FAILED ]"));
    Serial.print(F("[  ERROR CODE  ]"));Serial.println(error_code_received);
  }
  
  bool update_force_during_exec = true;
  bool update_imu_during_exec = true;
  float FORCE_MEASUREMENTS[num_FORCE_SENSORS];
  sensors::imu_filter selected_filter = sensors::imu_filter::MAHONY_F;
  //return_function_state =  stp.syncSetStepperGoalPositionVarStep2(IMUSensor, PTR_2_imu_packet, selected_filter, ForceSensor, ForceSensorHX711, FORCE_MEASUREMENTS, &sensor_error, &currentAbsPos_double, &goalAbsPos_double, &Aexec, &p2pcsp_Texec,  &currentDirStatus, &KILL_MOTION, &LIN_SEG_EXISTS, update_force_during_exec,update_imu_during_exec,  P2P_PROF_STEPS,    &error_code_received);
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
