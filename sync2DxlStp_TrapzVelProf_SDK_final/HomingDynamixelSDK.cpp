bool HomingDynamixelSDK(dynamixel::GroupSyncWrite groupSyncWriteGoalPos, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){
  time_now_millis = millis();

  // I. Here Set Acceleration and Profile Velocity on Dynamixels

  // II. Here write Home Position Value
  int32_t dxl_home_position = 0;

    // II.a. Allocate goal position value into byte array
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_home_position));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_home_position));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_home_position));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_home_position));

      // II.b. Add Dynamixel#1 goal position value to the Syncwrite storage
      dxl_addparam_result = groupSyncWriteGoalPos.addParam(DXL1_ID, param_goal_position);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL1_ID); Serial.println("] groupSyncWriteHomePos1 addparam failed");
        return false;
      }

      // II.c. Add Dynamixel#2 goal position value to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWriteGoalPos.addParam(DXL2_ID, param_goal_position);
      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL2_ID); Serial.println("] groupSyncWriteHomePos2 addparam failed");
        return false;
      }

      // II.d. Syncwrite goal position
      dxl_comm_result = groupSyncWriteGoalPos.txPacket();
      if (dxl_comm_result != COMM_SUCCESS){
       Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      }else{
        Serial.println("Dynamixels  Homing...");
      }

      // II.e. Clear syncwrite parameter storage
      groupSyncWriteGoalPos.clearParam();
  uint8_t dxl_moving;
  // III. LED status while homing
  int k=0;
  do{
    k++;
    //blink led;
    Serial.println("BLUE LED blinks");
    // MUST undestand read1ByteTxRx vs read1ByteRx
    int readStateMovingStatus = packetHandler->read1ByteTxRx(portHandler,DXL1_ID,ADDR_PRO_MOVING_STATUS,&dxl_moving,&dxl_error);       // if 0 => success of TxRx
    //int readStateMovingStatus = packetHandler->read1ByteRx(portHandler,dxl_moving,&dxl_error);
    Serial.printf("dxl_moving= %d \n",dxl_moving);
    Serial.printf("readStateMovingStatus= %d \n",readStateMovingStatus);
  }while(k<100);    
  //}while(!dxl_moving);
}