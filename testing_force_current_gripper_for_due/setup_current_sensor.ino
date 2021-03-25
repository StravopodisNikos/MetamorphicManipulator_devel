void setup_current_sensor()
{
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
  
  return;
}
