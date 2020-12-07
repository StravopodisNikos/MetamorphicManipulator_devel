// Function is called in sync2DxlStp_TrapzVelProfSDK.ino
// Based on SimpleSyncP2P_TrapzVelProf() called in syncDxlStp_TrapzVelProf.ino

// Function
bool SimpleSyncP2P_TrapzVelProf_SDK( uint8_t *MotorsIDs, int MotorsIds_size, int32_t *DxlTrapzProfParams, float *StpTrapzProfParams, int TrapzProfParams_size, dynamixel::GroupSyncWrite groupSyncWrite_GP_A_V_LED, dynamixel::GroupSyncRead groupSyncRead_PP_M_MS, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){
    time_now_millis = millis();

    /* 
     * I.
     * Dynamixel Initialization 
     * Here only Dxl Position change is considered for position status monitoring
     * DxlTimeExec is calculated only for 1 motor since Sunc is used!
     */
    unsigned long t1 = (600*DxlTrapzProfParams[2])/DxlTrapzProfParams[3];
    int32_t Dpos = abs(DxlTrapzProfParams[1]-DxlTrapzProfParams[0]);
    unsigned long t2 = (5.97701241*Dpos)/DxlTrapzProfParams[2];
    unsigned long DxlTimeExec = t1+t2;                            // Time Units [millisecs]
    Serial.println(DxlTimeExec);
    int32_t DxlPresentPosition = 0;                               // Dynamixel is previously Homed!
    bool result = false;
    const char *log;
    //int32_t get_data = 0;

    /* 
     * II.
     * Stepper Initialization
     * Checks for Stepper Trapezoidal profile
     * Based on trapzPreAssignedVelAcceleP2P.cpp
     */
    float a = 2*pi/(40*3200);                                     // [rad], q angle @ spr=3200[steps/rev] , GEAR_RATIO = 1:40
    float h  = StpTrapzProfParams[1]-StpTrapzProfParams[0];
    //h = abs(h);                                                 // Calculate displacement in [rad]
    float h_step1 = h/a;                                          // Calculate displacement in [steps]
    long h_step = round(h_step1);
    //Serial.println(h_step);    
    float hcond = pow(StpTrapzProfParams[2],2)/StpTrapzProfParams[3];
    float Ta;
    long  nmov_Ta;
    long  nmov_Td;
    bool segmentExists;
    long nmov_linseg;
    if(h>=hcond)                                            // Checks if linear segment exists Melchiorri
    {
      // equations 3.9 will be executed
      Serial.println("Trapezoidal Profile!");
      Ta = (1.5*StpTrapzProfParams[2])/StpTrapzProfParams[3];                                 // Calculate accelration time
      float T  = (h*StpTrapzProfParams[3]+pow(StpTrapzProfParams[2],2))/(StpTrapzProfParams[3]*StpTrapzProfParams[2]);          // Calculate total execution time
      segmentExists = true;
      // Calculate Intermediate phase steps
      float qmov_Ta = 0.5*StpTrapzProfParams[3]*pow(Ta,2);
      nmov_Ta = qmov_Ta/a;                                  // Steps of Acceleration Phase;
      nmov_linseg = h_step-2*nmov_Ta;                       // Steps of liner segment if Accel=Deccel
      nmov_Td = nmov_Ta;
    } 
    else
    {
      // equations 3.10 will be executed
      Serial.println("Triangular Profile! Vmax is recalculated!");
      Ta = sqrt(h/StpTrapzProfParams[3]);
      float T  = 2*Ta;
      Serial.println(T);
      float nVmax = StpTrapzProfParams[3]*Ta;
      Serial.printf("New maximum Velocity: %f \n",nVmax);
      segmentExists = false;
      // Calculate Intermediate phase steps
      float qmov_Ta = 0.5*StpTrapzProfParams[3]*pow(Ta,2);
      nmov_Ta = qmov_Ta/a;
      nmov_linseg = 0;
      nmov_Td = h_step-nmov_Ta;
    }

    long StpPresentPosition = 0;                                                // Moves motor until specified number of steps is reached
    float delta_t = 0.676*(1/Ta)*(2*a/StpTrapzProfParams[3]);                   // c0 with ignored inaccuracy factor
    float new_delta_t;
    unsigned long rest  = 0;
    long accel_count = 0; 
    long ctVel_count = 0;
    long decel_count = -nmov_Td;                                                // counter of steps executed for Acceleration Phase

/*
 * III. Initiate Sync write for Dynamixels using Indirect Addressing 
 */
// FOR loop for DXL_ID starts here...
        for(int id_count = 0; id_count <2; id_count++){
          // III.1. Disable Dynamixel Torque because Indirect address would not be accessible when the torque is already enabled!
          dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          else
          {
            printf("DXL has been successfully connected \n");
          }


        // III.2 Indirect Parameter Storage for WRITE

          // III.2.1. Allocate goal position value into byte array
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_GOAL_POSITION + 0, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_GOAL_POSITION + 1, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 4, ADDR_PRO_GOAL_POSITION + 2, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 6, ADDR_PRO_GOAL_POSITION + 3, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          // III.2.2. Allocate Profile Acceleration value into byte array
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_PROF_ACCEL + 0, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 10, ADDR_PRO_PROF_ACCEL + 1, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 12, ADDR_PRO_PROF_ACCEL + 2, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 14, ADDR_PRO_PROF_ACCEL + 3, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          // III.2.3. Allocate Profile Velocity value into byte array
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 16, ADDR_PRO_PROF_VEL + 0, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 18, ADDR_PRO_PROF_VEL + 1, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 20, ADDR_PRO_PROF_VEL + 2, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 22, ADDR_PRO_PROF_VEL + 3, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          // III.2.3. Allocate LEDs value into byte array
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 24, ADDR_PRO_LED_BLUE, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 26, ADDR_PRO_LED_GREEN, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          // III.3. Indirect Parameter Storage for READ
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 0, ADDR_PRO_PRESENT_POSITION + 0, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 2, ADDR_PRO_PRESENT_POSITION + 1, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 4, ADDR_PRO_PRESENT_POSITION + 2, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 6, ADDR_PRO_PRESENT_POSITION + 3, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 8, ADDR_PRO_MOVING, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ + 10, ADDR_PRO_MOVING_STATUS, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }

      // FINISHED WRITING IN EEPROM MEMORY => TORQUE CAN BE ENABLED

        // IV. Enable DXL Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[id_count], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        // Add parameter storage for the present position value
        dxl_addparam_result = groupSyncRead_PP_M_MS.addParam(dxl_id[id_count]);
        if (dxl_addparam_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed\n", dxl_id[id_count]);
          return 0;
        }

      }
      // FOR loop for DXL_ID ends here...

      // V. Allocate params for Write from Byte Array, arguments are given from user in function call
      param_indirect_data_for_write[0] = DXL_LOBYTE(DXL_LOWORD(DxlTrapzProfParams[1]));
      param_indirect_data_for_write[1] = DXL_HIBYTE(DXL_LOWORD(DxlTrapzProfParams[1]));
      param_indirect_data_for_write[2] = DXL_LOBYTE(DXL_HIWORD(DxlTrapzProfParams[1]));
      param_indirect_data_for_write[3] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[1]));
      param_indirect_data_for_write[4] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[2]));
      param_indirect_data_for_write[5] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[2]));
      param_indirect_data_for_write[6] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[2]));
      param_indirect_data_for_write[7] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[2]));
      param_indirect_data_for_write[8] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[3]));
      param_indirect_data_for_write[9] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[3]));
      param_indirect_data_for_write[10] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[3]));
      param_indirect_data_for_write[11] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[3]));
      param_indirect_data_for_write[12] = dxl_ledBLUE_value[1];
      param_indirect_data_for_write[13] = dxl_ledGREEN_value[0];

      // VI.a Add Dynamixel#1 WRITE values to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWrite_GP_A_V_LED.addParam(DXL1_ID, param_indirect_data_for_write);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] groupSyncWriteHomePos1 addparam failed");
        return false;
      }

      // VI.b. Add Dynamixel#2 WRITE values to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWrite_GP_A_V_LED.addParam(DXL2_ID, param_indirect_data_for_write);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("] groupSyncWriteHomePos2 addparam failed");
        return false;
      }

      // VII. Syncwrite Packet is sent to Dynamixels
      dxl_comm_result = groupSyncWrite_GP_A_V_LED.txPacket();
      if (dxl_comm_result != COMM_SUCCESS){
       Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      }else{
        Serial.println("Dynamixels  Moving...");
      }

      // VIII. Clear syncwrite parameter storage
      groupSyncWrite_GP_A_V_LED.clearParam();


      // IX. EXECUTING AND READING
      do
      {
      // IX.aa. Move stepper using Trapz Vel Profile
// =============================================================================================================================================
          StpPresentPosition++;
          // IV.b.1.I. Locate position in ramp

          if(segmentExists)                                                                             // Linear Segment exists
          {
                Serial.println("Segment exists");                               
                if(StpPresentPosition<nmov_Ta){
                  Serial.println("Acceleration Phase");
                  accel_count++;                                                                        // Acceleration Phase: delta_t -> minimizes
                  new_delta_t =  delta_t - (2*delta_t+rest)/(4*accel_count+1);                          // c_n
                  //new_rest = (2*delta_t + rest)*mod(4*accel_count+1);                                 // r_n
                }else if( StpPresentPosition>nmov_Ta && StpPresentPosition<(nmov_Ta+nmov_linseg) ){     // Linear Segment: delta_t -> constant
                  ctVel_count++;
                  Serial.printf("Accel Phase: %ld \n",accel_count);
                  Serial.println("Constant Velocity Phase");
                  new_delta_t = delta_t;  
                }
                else{
                  Serial.printf("CtVel Phase steps: %ld \n",ctVel_count);
                  Serial.println("Decelleration Phase");
                  decel_count++;                                                                        // Negative Value!
                  Serial.println(decel_count);
                  new_delta_t =  delta_t - (2*delta_t+rest)/(4*decel_count+1) ;                         // Deceleration Phase: delta_t -> maximizes 
                }                                                                         
          }
          else
          {                                                                                             // Linear Segment doesn't exist
                Serial.println("Segment doesn't exist");
                if(StpPresentPosition<nmov_Ta)                                                          // Acceleration Phase: delta_t -> minimizes
                {
                  Serial.println("Acceleration Phase");
                  accel_count++;
                  new_delta_t = delta_t - (2*delta_t+rest)/(4*accel_count+1);                           // c_n

                }                                   
                else{                                                                                   // Deceleration Phase: delta_t -> maximizes
                  Serial.println("Decelleration Phase");
                  decel_count++;                                                                        // Negative Value!
                  new_delta_t = delta_t - (2*delta_t+rest)/(4*decel_count+1);                           // Deceleration Phase: delta_t -> maximizes 
                }                                                                       
          }
          Serial.printf("New step delay time[s]: %f \n",new_delta_t);

          
          // IV.b.1.II. Steps Motor with variable time delay step
          unsigned long new_delta_t_micros = (new_delta_t*1000000)/40;
          Serial.printf("New step delay time[micros]: %lu \n",new_delta_t_micros);
          singleStepVarDelay(new_delta_t_micros);       

          delta_t = new_delta_t;
            
          Serial.println(StpPresentPosition);

          // =============================================================================================================================================      

      // IX.a. Syncread present position from indirectdata2
          dxl_comm_result = groupSyncRead_PP_M_MS.txRxPacket();
          if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

          // IX.b.1. Check if groupsyncread data of Dyanamixels is available
          for(int id_count = 0; id_count <2; id_count++){
                dxl_getdata_result = groupSyncRead_PP_M_MS.isAvailable(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);
                if (dxl_getdata_result != true)
                {
                  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id[id_count]);
                  return 0;
                }

                dxl_getdata_result = groupSyncRead_PP_M_MS.isAvailable(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);
                if (dxl_getdata_result != true)
                {
                  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id[id_count]);
                  return 0;
                }

                dxl_getdata_result = groupSyncRead_PP_M_MS.isAvailable(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION + LEN_PRO_MOVING, LEN_PRO_MOVING_STATUS);
                if (dxl_getdata_result != true)
                {
                  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id[id_count]);
                  return 0;
                }
          }

          // IX.b.2.Get Dynamixel values
          for(int id_count = 0; id_count <2; id_count++){
                dxl_present_position[id_count] = groupSyncRead_PP_M_MS.getData(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);

                dxl_moving[id_count] = groupSyncRead_PP_M_MS.getData(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);

                dxl_moving_status[id_count] = groupSyncRead_PP_M_MS.getData(dxl_id[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION + LEN_PRO_MOVING, LEN_PRO_MOVING_STATUS);
          }

          Serial.printf("[Dynamixel Motor ID:%03d] GoalPos:%d  PresPos:%d  IsMoving:%d \n", dxl_id[0], dxl_home_position, dxl_present_position[0], dxl_moving[0]);
          Serial.printf("[Dynamixel Motor ID:%03d] GoalPos:%d  PresPos:%d  IsMoving:%d \n", dxl_id[1], dxl_home_position, dxl_present_position[1], dxl_moving[1]);

      //}while(abs(dxl_home_position - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD);
      //}while(   (millis() < time_now_millis + DxlTimeExec)  );
      }while( ( (abs(h_step - StpPresentPosition) != 0) ));

  // X.1.1 Turn on Green LED
    
  // X.1.2 Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // X.1.3. Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // X.1.4 Close port
  portHandler->closePort();

  return 0;
} // END OF FUNCTION