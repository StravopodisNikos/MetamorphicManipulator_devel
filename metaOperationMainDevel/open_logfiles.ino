void open_logfiles()
{
 // creates the logfile objects based on testing_dataLogger_for_metaOperationMain  l.116
  LOG_FILES_LOC[POS_LOG_ID] = "pos.log";
  LOG_FILES_LOC[VEL_LOG_ID] = "vel.log";
  LOG_FILES_LOC[FOR_LOG_ID] = "force.log";

  LOG_FILES_GL[POS_LOG_ID]  = SESSION_MAIN_DIR_POS   + "/" + LOG_FILES_LOC[POS_LOG_ID];
  LOG_FILES_GL[VEL_LOG_ID]  = SESSION_MAIN_DIR_VEL   + "/" + LOG_FILES_LOC[VEL_LOG_ID];
  LOG_FILES_GL[FOR_LOG_ID]  = SESSION_MAIN_DIR_FORCE + "/" + LOG_FILES_LOC[FOR_LOG_ID];

  double first_data = 0;
  
  // OPEN LOG FILES
  for (size_t i = 0; i < sizeof(LOG_FILES_LOC); i++)
  {
    LOGFILES[i] = SD.open(LOG_FILES_GL[i], FILE_WRITE);
    if (LOGFILES[i])
    {
        TIMESTAMP = millis();
        RobotDataLog.writeData(first_data, TIMESTAMP ,data_cnt, LOGFILES+i, &data_error);
        if (data_error == NO_ERROR)
        {
          DEBUG_SERIAL.print(F("[ SETUP ] WRITE TO FILE:")); DEBUG_SERIAL.print(i);   DEBUG_SERIAL.println(F("SUCCESS"));
        }
        else
        {
          DEBUG_SERIAL.print(F("[ SETUP ] WRITE TO FILE:")); DEBUG_SERIAL.print(i);   DEBUG_SERIAL.println(F("FAILED"));
        }
     }
     else
     {
        DEBUG_SERIAL.print(F("[ SETUP ] OPEN LOG FILE:")); DEBUG_SERIAL.print(i);   DEBUG_SERIAL.println(F("FAILED"));
     }
  }
  
  return;
}
