void setup_imu_sensor()
{

/*  
 *   First build the package and set the pointer to each first element
 */
  IMU_DATA_PKG.ptr2_fxas    = &gyro;
  IMU_DATA_PKG.ptr2_fxos    = &accelmag;
  //IMU_DATA_PKG.ptr2_filter  = &fusion;
  IMU_DATA_PKG.ptr2_filter  = &filter;
  IMU_DATA_PKG.cal          = &adafr_calibr;
  IMU_DATA_PKG.gyr_event    = &gyro_event;
  IMU_DATA_PKG.acc_event    = &accel_event;
  IMU_DATA_PKG.mag_event    = &magnet_event;
  IMU_DATA_PKG.roll_c       = roll;
  IMU_DATA_PKG.pitch_c      = pitch;
  IMU_DATA_PKG.yaw_c        = yaw;
  
  PTR_2_imu_packet = &IMU_DATA_PKG;
  
  IMUSensor = & SingleIMUSensor;
/*
 * Connects to sensors via I2C and sets state to IMU_READY
 */

  return_function_state = IMUSensor->setupIMU2(PTR_2_imu_packet, &ImuCurrentState, &sensor_error);
  
  if (!return_function_state)
  {
       Serial.print(F("[     INFO     ] IMU PINGED [ FAILED ]"));
       Serial.print(F("[  ERROR CODE  ]")); Serial.println(sensor_error);
  }
  else
  {
      Serial.println(F("[    INFO    ] IMU PINGED [ SUCCESS ]"));
  }

  return;
    
}
