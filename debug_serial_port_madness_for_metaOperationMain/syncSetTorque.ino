bool syncSetTorque(uint8_t *DxlIDs, int DxlIds_size, uint8_t data, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE, dynamixel::PacketHandler *packetHandler)
{     
/*
 *  Sets the same toque status to all connected Dynamixels
 * torque status value is given through data variable and range value is: 0~1 
 */
Serial.println("Mphka syncSetTorque");

    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        bool dxl_addparam_result = groupSyncWrite_TORQUE_ENABLE.addParam(DxlIDs[id_count], &data);
        if (dxl_addparam_result != true)
        {
          Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("groupSyncWrite_TORQUE_ENABLE.addParam FAILED");
          //return false;
        }
        else
        {
          Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("groupSyncWrite_TORQUE_ENABLE.addParam SUCCESS");
        }
    }

    dxl_comm_result = groupSyncWrite_TORQUE_ENABLE.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result)); Serial.println("groupSyncWrite_TORQUE_ENABLE.txPacket() FAILED");
      //return false;
    }
    else
    {
      Serial.print("Torque STATUS CHANGED"); Serial.print(" New status:"); Serial.println(data);
    }
    
    groupSyncWrite_TORQUE_ENABLE.clearParam();
    return true;
} // END FUNCTION 
