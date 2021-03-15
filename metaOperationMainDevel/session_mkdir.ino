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
   }
   
   // FORCE SUBFOLDER
   if(RobotDataLog.createSensorDir(FORCE_FOLDER, SESSION_MAIN_DIR, SESSION_MAIN_DIR_FORCE))
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION FORCE DIR SUCCESS"));
   }
   else
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION FORCE DIR FAILED"));
   }

   // JOINT POSITION SUBFOLDER
   if(RobotDataLog.createSensorDir(POS_FOLDER, SESSION_MAIN_DIR, SESSION_MAIN_DIR_POS))
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION POSITION DIR SUCCESS"));
   }
   else
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION POSITION DIR FAILED"));
   }   
   
   // JOINT VELOCITY SUBFOLDER
   if(RobotDataLog.createSensorDir(VEL_FOLDER, SESSION_MAIN_DIR, SESSION_MAIN_DIR_VEL))
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION VELOCITY DIR SUCCESS"));
   }
   else
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION VELOCITY DIR FAILED"));
   }   
      
   return;
}
