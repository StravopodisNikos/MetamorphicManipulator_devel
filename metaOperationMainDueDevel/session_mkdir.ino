void session_mkdir()
{
   // Here all session's sensor folders are created
   // based on testing_dataLogger_for_metaOperationMain
   // the log files will be created in start of the
   // .ino file that calls robot execution(i.e. p2pcsp2_sm)

   // SESSION FOLDER
   if (RobotDataLog.createSessionDir(SESSION_MAIN_DIR))
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION DIR SUCCESS"));
      DEBUG_SERIAL.print(F("[ INFO  ] SESSION DIR:")); DEBUG_SERIAL.println(SESSION_MAIN_DIR); 
   }
   else
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION DIR FAILED"));
      DEBUG_SERIAL.print(F("[ INFO  ] SESSION DIR:")); DEBUG_SERIAL.println(SESSION_MAIN_DIR);
   }
   
   // JOINT POSITION SUBFOLDER
   if(RobotDataLog.createSensorDir(POS_FOLDER, SESSION_MAIN_DIR, SESSION_MAIN_DIR_POS,&data_error))
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION POSITION DIR SUCCESS"));
      DEBUG_SERIAL.print(F("[ SETUP ] POSITION DIR:")); DEBUG_SERIAL.println(SESSION_MAIN_DIR_POS);
   }
   else
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION POSITION DIR FAILED"));
      DEBUG_SERIAL.print(F("[ SETUP ] POSITION DIR:")); DEBUG_SERIAL.println(SESSION_MAIN_DIR_POS);
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(data_error);
   }   
   
   // JOINT VELOCITY SUBFOLDER
   if(RobotDataLog.createSensorDir(VEL_FOLDER, SESSION_MAIN_DIR, SESSION_MAIN_DIR_VEL,&data_error))
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION VELOCITY DIR SUCCESS"));
      DEBUG_SERIAL.print(F("[ SETUP ] VELOCITY DIR:")); DEBUG_SERIAL.println(SESSION_MAIN_DIR_VEL);
   }
   else
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION VELOCITY DIR FAILED"));
      DEBUG_SERIAL.print(F("[ SETUP ] VELOCITY DIR:")); DEBUG_SERIAL.println(SESSION_MAIN_DIR_VEL);
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(data_error);
   }   
   
   // FORCE SUBFOLDER
   if(RobotDataLog.createSensorDir(FORCE_FOLDER, SESSION_MAIN_DIR, SESSION_MAIN_DIR_FORCE,&data_error))
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION FORCE DIR SUCCESS"));
      DEBUG_SERIAL.print(F("[ SETUP ] FORCE DIR:")); DEBUG_SERIAL.println(SESSION_MAIN_DIR_FORCE);
   }
   else
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION FORCE DIR FAILED"));
      DEBUG_SERIAL.print(F("[ SETUP ] FORCE DIR:")); DEBUG_SERIAL.println(SESSION_MAIN_DIR_FORCE);
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(data_error);
   }
   
   return;
}
