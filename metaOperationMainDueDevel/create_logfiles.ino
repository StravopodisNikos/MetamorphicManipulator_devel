void create_logfiles()
{
  unsigned long data_cnt=0;
  unsigned long TIMESTAMP;

  // Create an array of pointer, that each points to the log file path created for each sensor
  char * LOG_FILE_PATH[3];
  LOG_FILE_PATH[POS_LOG_ID] = LOG_POS;
  LOG_FILE_PATH[VEL_LOG_ID] = LOG_VEL;
  LOG_FILE_PATH[FOR_LOG_ID] = LOG_FORCE;
  
  // Create a array of pointers, that each point to a sensor dir path created in session_mkdir
  const char * PTRS2SENSOR_DIRS[3]; // must be expanded to 5!
  PTRS2SENSOR_DIRS[POS_LOG_ID] = SESSION_MAIN_DIR_POS;
  PTRS2SENSOR_DIRS[VEL_LOG_ID] = SESSION_MAIN_DIR_VEL;
  PTRS2SENSOR_DIRS[FOR_LOG_ID] = SESSION_MAIN_DIR_FORCE;
  
  for (size_t i = 0; i < sizeof(LOG_FILES); i++) // sizeof(LOG_FILES)
  {    
    RobotDataLog.createFile(LOGFILES+i, PTRS2SENSOR_DIRS[i], LOG_FILE_PATH[i], &data_error);
    if (data_error == NO_ERROR)
    {
      DEBUG_SERIAL.print(F("[ SETUP ] CREATE LOG FILE:")); DEBUG_SERIAL.print(LOG_FILE_PATH[i]); DEBUG_SERIAL.println(F(" SUCCESS "));
    }
    else
    {
      DEBUG_SERIAL.print(F("[ SETUP ] CREATE LOG FILE:")); DEBUG_SERIAL.print(LOG_FILE_PATH[i]); DEBUG_SERIAL.println(F(" FAILED "));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(data_error);
    }
    delay(500);
  }

  double first_data = 0;
  
  // OPEN - WRITE - CLOSE SEQUENCE
  for (size_t i = 0; i < sizeof(LOG_FILES); i++) // sizeof(LOG_FILES)
  {
    // OPEN LOG FILE
    RobotDataLog.openFile(LOGFILES+i, LOG_FILE_PATH[i] , FILE_WRITE,  &data_error);
    if (data_error == NO_ERROR)
    {
      DEBUG_SERIAL.print(F("[ SETUP ] OPEN LOG FILE:")); DEBUG_SERIAL.print(LOG_FILE_PATH[i]); DEBUG_SERIAL.println(F(" SUCCESS "));
    }
    else
    {
      DEBUG_SERIAL.print(F("[ SETUP ] OPEN LOG FILE:")); DEBUG_SERIAL.print(LOG_FILE_PATH[i]); DEBUG_SERIAL.println(F(" FAILED "));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(data_error);
    }

    // WRITE SINGLE LINE
    data_cnt++;
    TIMESTAMP = millis();
    RobotDataLog.writeData(first_data, TIMESTAMP ,data_cnt, LOGFILES+i, &data_error);
    if (data_error == NO_ERROR)
    {
      DEBUG_SERIAL.print(F("[ SETUP ] FIRST WRITE LOG FILE:")); DEBUG_SERIAL.print(LOG_FILES[i]); DEBUG_SERIAL.println(F(" SUCCESS "));
    }
    else
    {
      DEBUG_SERIAL.print(F("[ SETUP ] FIRST WRITE LOG FILE:")); DEBUG_SERIAL.print(LOG_FILES[i]); DEBUG_SERIAL.println(F(" FAILED "));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(data_error);
    }    

    // CLOSE LOG FILE
    RobotDataLog.closeFile(LOGFILES+i,&data_error); 
  }
  
  return;
}
