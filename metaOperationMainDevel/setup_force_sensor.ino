void setup_force_sensor()
{
/*
 * Pings HX711 chips: Changes state OFF->IDLE
 */
 bool pinged_all_sensors = true;
 for (size_t i = 0; i < nDoF; i++)
 {
  return_function_state = ForceSensor[i].setupForceSensor((ForceSensorHX711+i), &ForceCurrentState, &force_error);

  if (!return_function_state)
  {
    pinged_all_sensors =  false; break;
  }
 }

 if(pinged_all_sensors)
 {
  DEBUG_SERIAL.println(F("[    INFO    ] FORCE SENSOR PINGED [  SUCCESS ]")); // ALL SENSOR STATES->FORCE_IDLE
 }
 else
 {
  DEBUG_SERIAL.println(F("[    INFO    ] FORCE SENSOR PINGED [  FAILED ]"));
 }

/*
 * Takes single measurement in each axis
 */

/*
 * Asks for calibration
 */

/*
 * Exits setup: Changes state IDLE->READY
 */


  return;
}
