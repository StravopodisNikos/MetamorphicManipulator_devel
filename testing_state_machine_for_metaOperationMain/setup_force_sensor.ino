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
ForceSensor = & SingleForceSensor;
ForceSensorHX711 = & SingleForceSensorHX711;
/*
 * Initializes pins for each sensor and timeout pings, sets state to FORCE_READY 
 */
 bool pinged_all_sensors = true;
 for (size_t i = 0; i < 1; i++)
 {
  return_function_state = ForceSensor->setupForceSensor(ForceSensorHX711, manual_calibration_scale_factors[i] , &ForceCurrentState, &sensor_error);
  
  if (!return_function_state)
  {
    pinged_all_sensors =  false; break;
  }
  else
  {
      Serial.print(F("[    INFO    ] SENSOR AXIS ")); Serial.print(i); Serial.println(F(" PINGED  [ SUCCESS ]"));
  }
  delay(500);
 }

 if(pinged_all_sensors)
 {
    Serial.println(F("[    INFO    ] 1 AXIS FORCE SENSOR PINGED [ SUCCESS ]")); // ALL SENSOR STATES->FORCE_IDLE
 }
 else
 {
    Serial.println(F("[    INFO    ] 1 AXIS FORCE SENSOR PINGED [ FAILED ]"));
    Serial.print(F("[  ERROR CODE  ]")); Serial.println(sensor_error);
 }

/*
 * Get single measurement
 */
 float *force_measurements_nwts;
 float *force_measurements_kgs;
 for (size_t i = 0; i < 1; i++)
 {
    return_function_state = ForceSensor->measureForceKilos(ForceSensorHX711, force_measurements_kgs, &sensor_error);
    Serial.print(F("[    INFO    ] FORCE SENSOR")); Serial.print(i); Serial.println(F(" MEASURED  [ SUCCESS ]"));
    Serial.print(F("[MEASUREMENT: "));  Serial.print(force_measurements_kgs[i]); Serial.println(F(" [N]"));

}

return;
    
}
