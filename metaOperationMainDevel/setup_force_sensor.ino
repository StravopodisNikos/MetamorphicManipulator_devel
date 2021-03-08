void setup_force_sensor()
{
  // =======================================
  // CALIBRATION MUST BE EXECUTED FROM SKETCH
  // calibrate_3axis_force_sensor.ino
  // in order to determine the array:
  // float manual_calibration_scale_factors[3]
  float manual_calibration_scale_factors[num_FORCE_SENSORS];
  manual_calibration_scale_factors[0] = 195000;
  manual_calibration_scale_factors[1] = 1;
  manual_calibration_scale_factors[2] = 500000;
  // =======================================

/*
 * Initializes pins for each sensor and timeout pings, sets state to FORCE_READY 
 */
 bool pinged_all_sensors = true;
 for (size_t i = 0; i < num_FORCE_SENSORS; i++)
 {
  return_function_state = ForceSensor[i].setupForceSensor((ForceSensorHX711+i), manual_calibration_scale_factors[i] , &ForceCurrentState, &sensor_error);
  
  if (!return_function_state)
  {
    pinged_all_sensors =  false; break;
  }
  else
  {
      DEBUG_SERIAL.print(F("[    INFO    ] SENSOR AXIS ")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println(F(" PINGED  [ SUCCESS ]"));
  }
  delay(5);
 }

 if(pinged_all_sensors)
 {
    DEBUG_SERIAL.println(F("[    INFO    ] 3 AXIS FORCE SENSOR PINGED [ SUCCESS ]")); // ALL SENSOR STATES->FORCE_IDLE
 }
 else
 {
    DEBUG_SERIAL.println(F("[    INFO    ] 3 AXIS FORCE SENSOR PINGED [ FAILED ]"));
    DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(sensor_error);
 }

/*
 * Serial prints the z-axis offset Z-AXIS->2
 */
 long Z_FORCE_OFFSET;
 return_function_state = ForceSensor[2].getPermanentZeroOffset((ForceSensorHX711+2), &Z_FORCE_OFFSET);
 if (return_function_state)
 {
    DEBUG_SERIAL.print(F("[    INFO    ] Z-AXIS OFFSET: "));  DEBUG_SERIAL.println(Z_FORCE_OFFSET); 
 }
 else
 {
    DEBUG_SERIAL.println(F("[    INFO    ] Z-AXIS OFFSET [ FAILED ]"));
 }

/*
 * Get single measurement
 */
 float force_measurements_nwts[num_FORCE_SENSORS];
 float force_measurements_kgs[num_FORCE_SENSORS];
 for (size_t i = 0; i < num_FORCE_SENSORS; i++)
 {
    return_function_state = ForceSensor[i].measureForceKilos((ForceSensorHX711+i), (force_measurements_kgs+i), &sensor_error);
    if (return_function_state)
    {
      DEBUG_SERIAL.print(F("[    INFO    ] FORCE SENSOR")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println(F(" MEASURED  [ SUCCESS ]"));
      DEBUG_SERIAL.print(F("[MEASUREMENT: "));  DEBUG_SERIAL.print(force_measurements_kgs[i]); DEBUG_SERIAL.println(F(" [N]"));
    }
    else
    {
      DEBUG_SERIAL.print(F("[    INFO    ] FORCE SENSOR")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println(F(" MEASURED  [ FAILED ]"));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]")); DEBUG_SERIAL.println(sensor_error);
    }
}

return;
    
}
