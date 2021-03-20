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

#if defined(ARDUINO_AVR_UNO) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial       
  #define DEBUG_SERIAL soft_serial  
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(11, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#else defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 
tools::dataLogger RobotDataLog, *PTR2RobotDataLog;
debug_error_type data_error;
File root,dir, *PTR2ROOT, *ptr2dir;
File FORCE_LOG;
sensors::sensors_list FORCE_FOLDER = sensors::FORCE_3AXIS;
char SESSION_MAIN_DIR[10];
char SESSION_MAIN_DIR_FORCE[20];
unsigned long data_cnt=0;
unsigned long TIMESTAMP;
double data = 1.5708;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() 
{
  //DEBUG_SERIAL.print(F("STARTING DEBUG PC SERIAL BUS..."));
  DEBUG_SERIAL.begin(SERIAL_BAUDRATE);            // Serial BAUDRATE->115200
  while(!DEBUG_SERIAL);
  DEBUG_SERIAL.print(F("STARTING DEBUG PC SERIAL BUS..."));
  DEBUG_SERIAL.println(F("SUCCESS"));
  
  DEBUG_SERIAL.print(F("STARTING DYNAMIXEL SERIAL BUS..."));
  dxl.begin(DXL_BAUDRATE2);                       // UART BAUDRATE->1000000
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  DEBUG_SERIAL.println(F("SUCCESS"));
  
  /*
  Serial.begin(SERIAL_BAUDRATE);            // Serial BAUDRATE->57600
  while(!Serial);
  */
  
  delay(1000);
  DEBUG_SERIAL.println(F(" [ SETUP ] ENTERING OVIDIUS ROBOT SETUP... "));
  delay(1000);

  /*
   *  Set time to enter main loop()
   */
   setSyncProvider(requestSync);
   DEBUG_SERIAL.println(F("[ SETUP ] SET ROBOT TIME STARTED - INSERT TIME:"));
   DEBUG_SERIAL.parseInt();
   bool TIME_NOT_SET = true;
   while (TIME_NOT_SET)
   {
      if (DEBUG_SERIAL.available())
      {
        processSyncMessage();
        TIME_NOT_SET = false;
        DEBUG_SERIAL.println(F("[ SETUP ] SET ROBOT TIME FINISHED"));
      }      
   };
  /*
   * Create Session dir
   */

   if (!SD.begin(SD_CARD_CS_PIN)) {
      DEBUG_SERIAL.println(F("[ SETUP ] SD INITIALIZATION FAILED!"));
      while (1);
   }
   DEBUG_SERIAL.println(F("[ SETUP ] SD INITIALIZATION SUCCESS!"));
/*
   RobotDataLog.setupDataLogger(PTR2ROOT, &data_error);
   if (data_error == SD_INIT_FAILED)
   {
      Serial.println(F("[ SETUP ] SETUP SD CARD FAILED"));
   }
   else
   {
      Serial.println(F("[ SETUP ] SETUP SD CARD SUCCESS"));
   }
*/

   if (RobotDataLog.createSessionDir(SESSION_MAIN_DIR))
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION DIR SUCCESS"));
      DEBUG_SERIAL.print(F("[ INFO  ] SESSION DIR:")); DEBUG_SERIAL.println(SESSION_MAIN_DIR); 
   }
   else
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION DIR FAILED"));
   }
   
   
   if(RobotDataLog.createSensorDir(FORCE_FOLDER, SESSION_MAIN_DIR, SESSION_MAIN_DIR_FORCE,&data_error))
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION FORCE DIR SUCCESS"));
      DEBUG_SERIAL.println(SESSION_MAIN_DIR_FORCE);
   }
   else
   {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATED SESSION FORCE DIR FAILED"));
      DEBUG_SERIAL.println(SESSION_MAIN_DIR_FORCE);
   }

   
   
/* // this works but file name must alse have minute digits/added a create-open-write-close sequence [17-3-21]
   String forceZ_log_loc = "force.log";
   String forceZ_log_gl  = SESSION_MAIN_DIR_FORCE+"/"+forceZ_log_loc;

   // TRYING TO OPEN FILE HERE
   FORCE_LOG = SD.open(forceZ_log_gl, FILE_WRITE);
   if (FORCE_LOG)
   {
      data_cnt++;
      TIMESTAMP = millis();
      RobotDataLog.writeData(data, TIMESTAMP ,data_cnt, &FORCE_LOG, &data_error);
      if (data_error == NO_ERROR)
      {
        Serial.println(F("[ SETUP ] WRITE TO FORCE LOG FILE SUCCESS"));
      }
      else
      {
        Serial.println(F("[ SETUP ] WRITE TO FORCE LOG FILE FAILED"));
      }
   }
   else
   {
      Serial.println(F("[ SETUP ] OPEN FORCE LOG FILE FAILED"));
   }
  */
  
    // create file
    char LOG_FILE_TEMP[33];
    RobotDataLog.createFile(&FORCE_LOG, SESSION_MAIN_DIR_FORCE, LOG_FILE_TEMP, &data_error);
    if (data_error == NO_ERROR)
    {
      DEBUG_SERIAL.println(LOG_FILE_TEMP);
      DEBUG_SERIAL.println(F("[ SETUP ] CREATE FORCE LOG FILE SUCCESS"));
    }
    else
    {
      DEBUG_SERIAL.println(F("[ SETUP ] CREATE FORCE LOG FILE FAILED"));
      DEBUG_SERIAL.println(LOG_FILE_TEMP);
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(data_error);
    }

    // open file
    RobotDataLog.openFile(&FORCE_LOG, LOG_FILE_TEMP , FILE_WRITE,  &data_error);
    if (data_error == NO_ERROR)
    {
      DEBUG_SERIAL.println(F("[ SETUP ] OPEN FORCE LOG FILE SUCCESS"));
    }
    else
    {
      DEBUG_SERIAL.println(F("[ SETUP ] OPEN FORCE LOG FILE FAILED"));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(data_error);
    }
    
    // write data 2 file
    data_cnt++;
    TIMESTAMP = millis();
    RobotDataLog.writeData(data, TIMESTAMP ,data_cnt, &FORCE_LOG, &data_error);
    if (data_error == NO_ERROR)
    {
      DEBUG_SERIAL.println(F("[ SETUP ] WRITE TO FORCE LOG FILE SUCCESS"));
    }
    else
    {
      DEBUG_SERIAL.println(F("[ SETUP ] WRITE TO FORCE LOG FILE FAILED"));
      DEBUG_SERIAL.print(F("[  ERROR CODE  ]"));DEBUG_SERIAL.println(data_error);
    }
    
    // close file
    RobotDataLog.closeFile(&FORCE_LOG, &data_error);
    
}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
 * TIMING ARDUINO THROUGH SERIAL PORT
 */
 time_t requestSync()
{
  DEBUG_SERIAL.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(DEBUG_SERIAL.find(TIME_HEADER)) {
     pctime = DEBUG_SERIAL.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}
