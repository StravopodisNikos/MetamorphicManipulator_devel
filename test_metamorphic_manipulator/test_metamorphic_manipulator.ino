
/*   *********************************************************************************************
 *   Executes tests for Metamorphic Manipulator with 1 Nema34 Stepper + 3 Dynamixels
 *   using custom-made functions for Driving Steppers and Dynamixels of a serial metamorphic
 *   manipulator. 
 *   ********************************************************************************************* 
 */

/* 
 *  Author: Stravopodis Nikos, PhD Candidate, University of the Aegean
 *  mail: n.stravopodis@syros.aegean.gr
 * 
 *  February 2020
 */

// Include definitions
#include "definitions.h"                            // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include "motorIDs.h"                               // Includes motor IDs as set using Dynamixel Wizard
#include "contolTableItems_LimitValues.h"           // Limit values for control table controlTableItems_LimitValues
#include "StepperMotorSettings.h"                   // Includes Stepper Motor/Driver pin StepperMotorSettings

//  Used Libraries
#include "Arduino.h"                                // Main Arduino library
#include <DynamixelSDK.h>                           // Dynamixel Software Development kit/ C++ 
#include "OpenCR.h"                                 // Controller Library

// Integrate C++
#include <vector>
#include <fstream>
#include "stdlib.h"
#include "stdio.h"

using namespace std;

// Libraries for Robot Motors Driving
#include "DynamixelProPlusMetamorphicManipulator.h"
#include "CustomStepperMetamorphicManipulator.h"

// Declare extern variables defined in DynamixelProPlusMetamorphicManipulator

// Declare extern variables defined in CustomStepperMetamorphicManipulator
bool return_function_state = false;
vector<double> TrajAssignedDuration;
vector<double> StpTrapzProfParams;
vector<unsigned long> PROFILE_STEPS;
vector<double> vector_for_trajectoryVelocity; 


uint8_t dxl_id[] = {DXL1_ID, DXL2_ID, DXL3_ID};


void setup() {
    Serial.begin(BAUDRATE);
    while(!Serial);
    Serial.println("Start..");

    // Initialize PortHandler instance
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Create object for handling dynamixels
    DynamixelProPlusMetamorphicManipulator dxl;

        /*  
     * I. Begin Communication Testing
     */
    // I.a.1. Open port
    Serial.println("Opening port");
    if (portHandler->openPort())
    {
      Serial.print("Succeeded to open the port!\n");
    }
    else
    {
      Serial.print("Failed to open the port!\n");
      return;
    }

    // I.a.2 Set port baudrate
    Serial.println("Setting port BAUDRATE");
    if (portHandler->setBaudRate(BAUDRATE))
    {
      int achievedBaudrate = portHandler->getBaudRate();
      Serial.print("Succeeded to change the baudrate! New Baudrate is: "); Serial.println(achievedBaudrate);
    }
    else
    {
      Serial.print("Failed to change the baudrate!\n");
    }

    // I.a.3 Ping Dynamixels
    Serial.println("Pinging Dynamixels");
    return_function_state = dxl.pingDynamixels(dxl_id, sizeof(dxl_id) , packetHandler, portHandler);
    if(return_function_state == true)
    {
      Serial.println("SUCCESS");
    }
    else
    {
      Serial.println("FAILED");
    }



}

void loop() {
  // put your main code here, to run repeatedly:

}
