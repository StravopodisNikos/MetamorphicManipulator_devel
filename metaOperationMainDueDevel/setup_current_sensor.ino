void setup_current_sensor()
{
  // Adafruit 219b functions commented out
  /*
  // build the current struct packet
  CUR_DATA_PKG.ptr2ina219     = & ina219;
  CUR_DATA_PKG.current_measurement_mA = joint1_current_measurement;
  
  PTR_2_CUR_DATA_PKG = & CUR_DATA_PKG;          // pointer to data pkg
  ptr2joint1_cur_sensor = & joint1_cur_sensor;  // pointer to sensor class object

  ptr2joint1_cur_sensor->setupCurrentSensor(PTR_2_CUR_DATA_PKG,  &sensor_error);
  if ( sensor_error == NO_ERROR)
  {
    DEBUG_SERIAL.println(F("[  INFO  ] INIT STEPPER CURRENT SENSOR [  SUCCESS ]"));
  }
  else
  {
    DEBUG_SERIAL.println(F("[  INFO  ] INIT STEPPER CURRENT SENSOR [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(sensor_error);
  }

  ptr2joint1_cur_sensor->measureCurrent_mA(PTR_2_CUR_DATA_PKG, &sensor_error);
  if ( sensor_error == NO_ERROR)
  {
    DEBUG_SERIAL.println(F("[  INFO  ] JOINT 1 CURRENT MEASURED [mA]  [  SUCCESS ]"));
    DEBUG_SERIAL.print(F("[  INFO  ] STEPPER CURRENT[mA] = ")); DEBUG_SERIAL.println(PTR_2_CUR_DATA_PKG->current_measurement_mA,4);
  }
  else
  {
    DEBUG_SERIAL.println(F("[  INFO  ] JOINT 1 CURRENT MEASURED [mA] [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(sensor_error);
  }
  */

  // ACS712 Module function
  ptr2joint1_cur_sensor = & joint1_cur_sensor;  // pointer to sensor class object;
  
  ptr2joint1_cur_sensor->measureCurrentACS712_A(joint1_current_measurement, &sensor_error);
  if ( sensor_error == NO_ERROR)
  {
    DEBUG_SERIAL.println(F("[  INFO  ] SETUP ACS712 MODULE [  SUCCESS ]"));
    DEBUG_SERIAL.print(F("[  INFO  ] STEPPER CURRENT[A] = ")); DEBUG_SERIAL.println(joint1_current_measurement,4);
  }
  else
  {
    DEBUG_SERIAL.println(F("[  INFO  ] SETUP ACS712 MODULE [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  INFO  ] STEPPER CURRENT[A] = ")); DEBUG_SERIAL.println(joint1_current_measurement,4);
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(sensor_error);
  }

  // Measure current from all joints
  bool check_robot_current_at_setup = true;
  stp.getJointsCurrent_A(p2pcsp_joint_currents, ptr2joint1_cur_sensor, PTR_2_meta_dxl, PTR_2_dxl_pc_packet, check_robot_current_at_setup, &error_code_received);
  if (error_code_received == NO_ERROR){
    DEBUG_SERIAL.println(F("[    INFO    ] SYNC READ CURRENT [  SUCCESS ]"));
    DEBUG_SERIAL.print(F(" [ INFO ] ")); DEBUG_SERIAL.println(F("CURRENT ROBOT JOINTS CURRENT [A]:")); 
    for (size_t i = 0; i < nDoF; i++)
    {
      DEBUG_SERIAL.print(F(" [ JOINT ")); DEBUG_SERIAL.print(i+1); DEBUG_SERIAL.print(F(" ] ")); DEBUG_SERIAL.println(p2pcsp_joint_currents[i],4);
    }
  }
  else
  {
    DEBUG_SERIAL.println(F("[    ERROR   ] SYNC READ CURRENT [  FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(error_code_received);
  }
  
  return;
}
