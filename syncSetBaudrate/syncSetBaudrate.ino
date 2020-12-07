bool syncSetBaudrate(uint8_t *DxlIDs, int DxlIds_size, uint8_t data, dynamixel::GroupSyncWrite groupSyncWrite_BAUDRATE, dynamixel::PacketHandler *packetHandler)
{     
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        dxl_addparam_result = groupSyncWrite_BAUDRATE.addParam(DxlIDs[id_count], &data);
        if (dxl_addparam_result != true)
        {
          Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("groupSyncWrite_BAUDRATE.addParam FAILED");
          return false;
        }
    }

    dxl_comm_result = groupSyncWrite_BAUDRATE.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result)); Serial.println("groupSyncWrite_BAUDRATE.txPacket() FAILED");
      return false;
    }
    else
    {
      Serial.print("BAUDRATE CHANGED"); Serial.print(" New status:"); Serial.println(data);
    }
    
    groupSyncWrite_BAUDRATE.clearParam();
    return true;
} // END FUNCTION 
