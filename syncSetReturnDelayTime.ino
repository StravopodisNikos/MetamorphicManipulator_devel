bool syncSetReturnDelayTime(uint8_t *DxlIDs, int DxlIds_size, uint8_t data, dynamixel::GroupSyncWrite groupSyncWrite_RETURN_DELAY_TIME, dynamixel::PacketHandler *packetHandler)
{     
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        dxl_addparam_result = groupSyncWrite_RETURN_DELAY_TIME.addParam(DxlIDs[id_count], &data);
        if (dxl_addparam_result != true)
        {
          Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("groupSyncWrite_RETURN_DELAY_TIME.addParam FAILED");
          return false;
        }
    }

    dxl_comm_result = groupSyncWrite_RETURN_DELAY_TIME.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result)); Serial.println("groupSyncWrite_RETURN_DELAY_TIME.txPacket() FAILED");
      return false;
    }
    else
    {
      Serial.print("RETURN_DELAY_TIME CHANGED"); Serial.print(" New status:"); Serial.println(data);
    }
    
    groupSyncWrite_RETURN_DELAY_TIME.clearParam();
    return true;
} // END FUNCTION 
