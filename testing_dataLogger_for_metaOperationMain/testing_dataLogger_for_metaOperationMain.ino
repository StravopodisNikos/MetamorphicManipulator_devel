/*   *********************************************************************************************
 *   Used to check data Logger in OvidiusSensors.h
 *   
 *   ********************************************************************************************* 
 */

/* 
 *  Author: Stravopodis Nikos, PhD Candidate, University of the Aegean
 *  mail: n.stravopodis@syros.aegean.gr
 * 
 *  March 2021
 */

/*
 * INCLUDE LIBRARIES
 */
 // Core Libraries for Robot Motors Driving
//#include "DynamixelProPlusOvidiusShield.h"
//#include "CustomStepperOvidiusShield.h"
//  Used Libraries
#include "Arduino.h"                                // Main Arduino library
#include <Dynamixel2Arduino.h>
// Auxiliary Libraries
#include <definitions.h>                              // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include <motorIDs.h>                                 // Includes motor IDs as set using Dynamixel Wizard
#include <contolTableItems_LimitValues.h>             // Limit values for control table controlTableItems_LimitValues
//#include <utility/StepperMotorSettings.h>             // Includes Stepper Motor/Driver pin StepperMotorSettings
//#include <task_definitions.h>                         // Includes task points for execution
#include <led_indicators.h>
//#include <ovidius_robot_controller_eeprom_addresses.h>  // Header file for all eeprom addresses used!
#include <avr/pgmspace.h>
#include "OvidiusSensors.h"
#include <utility/OvidiusSensors_config.h>
#include <utility/OvidiusSensors_debug.h>
#include <TimeLib.h>

tools::dataLogger RobotDataLog;
debug_error_type data_error;
File root,dir, *PTR2ROOT, *ptr2dir;
File FORCE_LOG;
sensors::sensors_list FORCE_FOLDER = sensors::FORCE_3AXIS;
String SESSION_MAIN_DIR,SESSION_MAIN_DIR_FORCE;
unsigned long data_cnt=0;
unsigned long TIMESTAMP;
double data = 1.5708;

void setup() {
  Serial.begin(SERIAL_BAUDRATE);            // Serial BAUDRATE->57600
  while(!Serial);

  delay(1000);
  Serial.println(F(" [ SETUP ] ENTERING OVIDIUS ROBOT SETUP... "));
  delay(1000);

  /*
   *  Set time to enter main loop()
   */

   setSyncProvider(requestSync);
   Serial.println(F("[ SETUP ] SET ROBOT TIME STARTED - INSERT TIME:"));
   Serial.parseInt();
   bool TIME_NOT_SET = true;
   while (TIME_NOT_SET)
   {
      if (Serial.available())
      {
        processSyncMessage();
        TIME_NOT_SET = false;
        Serial.println(F("[ SETUP ] SET ROBOT TIME FINISHED"));
      }      
   };

  /*
   * Create Session dir
   */
    
   RobotDataLog.setupDataLogger(PTR2ROOT, &data_error);
   if (data_error == SD_INIT_FAILED)
   {
      Serial.println(F("[ SETUP ] SETUP SD CARD FAILED"));
   }
   else
   {
     if (RobotDataLog.createSessionDir(SESSION_MAIN_DIR))
     {
        Serial.println(F("[ SETUP ] CREATED SESSION DIR SUCCESS"));
        Serial.print(F("[ INFO  ] SESSION DIR:")); Serial.println(SESSION_MAIN_DIR); 
     }
     else
     {
        Serial.println(F("[ SETUP ] CREATED SESSION DIR FAILED"));
     }
   }
   
   if(RobotDataLog.createSensorDir(FORCE_FOLDER, SESSION_MAIN_DIR, SESSION_MAIN_DIR_FORCE))
   {
      Serial.println(F("[ SETUP ] CREATED SESSION FORCE DIR SUCCESS"));
   }
   else
   {
      Serial.println(F("[ SETUP ] CREATED SESSION FORCE DIR FAILED"));
   }

   String forceZ_log_loc = "forceZ.log";
   String forceZ_log_gl  = SESSION_MAIN_DIR_FORCE+"/"+forceZ_log_loc;
   Serial.println(forceZ_log_gl);

   // WORKS UNTIL THIS POINT!
   /*
   //RobotDataLog.openFile(&FORCE_LOG, forceZ_log_gl, FILE_WRITE,  &data_error);
   if (SD.exists("gamw.txt"))
   {  
      Serial.println("1");
      SD.remove("gamw.txt");
   }
   else
   {
      Serial.println("2");
   }
   
   FORCE_LOG = SD.open("gamw.txt", FILE_WRITE);
   FORCE_LOG.close();
   //if (data_error == NO_ERROR)
   if (SD.exists("gamw.txt"))
   {
      Serial.println("3");
      data_cnt++;
      TIMESTAMP = millis();
      RobotDataLog.writeData(data, TIMESTAMP ,data_cnt, &FORCE_LOG, &data_error);
      Serial.println(F("[ SETUP ] WRITE TO FORCE LOG FILE SUCCESS"));
   }
   else
   {
      Serial.println("4");
      Serial.println(F("[ SETUP ] CREATED FORCE LOG FILE FAILED"));
   }
*/
}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
 * TIMING ARDUINO THROUGH SERIAL PORT
 */
 time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}
