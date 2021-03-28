void set_sensor_updates()
{
  DEBUG_SERIAL.println(F("[ AVAILABLE SENSORS ]"));
  DEBUG_SERIAL.println(F("SENSOR [0]: POSITION"));
  DEBUG_SERIAL.println(F("SENSOR [1]: VELOCITY"));
  DEBUG_SERIAL.println(F("SENSOR [2]: FORCE-Z"));
  DEBUG_SERIAL.println(F("SENSOR [3]: CURRENT"));

  for (size_t i = 0; i < TOTAL_SENSORS_USED; i++)
  {
    DEBUG_SERIAL.print(F("[ MENU ] UPDATE SENSOR [")); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println(F("] ?"));
    DEBUG_SERIAL.parseInt();
    while (DEBUG_SERIAL.available() == 0) {};
    user_input = DEBUG_SERIAL.parseInt();

    DEBUG_SERIAL.print(F("[ USER INPUT ]")); DEBUG_SERIAL.print(F("   ->   ")); DEBUG_SERIAL.println(user_input);
  
    if( user_input == YES_b ) 
    {
      update_sensor_measurements[i] = true;
    }
    else
    {
      update_sensor_measurements[i] = false;
    }
  }
  
  return;
}
