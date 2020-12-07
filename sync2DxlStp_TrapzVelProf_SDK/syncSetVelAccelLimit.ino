bool syncSetVelAccelLimit(uint8_t *DxlIDs, int DxlIds_size, int32_t dxl_vel_limit, int32_t dxl_accel_limit, dynamixel::GroupSyncWrite groupSyncWriteVelLim, dynamixel::GroupSyncWrite groupSyncWriteAccelLim, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){
// Torque MUST be DISABLED!

// I. Set Limit values for Velocity/Acceleration to Dynamixels

    // I.1. Allocate Acceleration Limit value into byte array
    param_accel_limit[0] = DXL_LOBYTE(DXL_LOWORD(dxl_accel_limit));
    param_accel_limit[1] = DXL_HIBYTE(DXL_LOWORD(dxl_accel_limit));
    param_accel_limit[2] = DXL_LOBYTE(DXL_HIWORD(dxl_accel_limit));
    param_accel_limit[3] = DXL_HIBYTE(DXL_HIWORD(dxl_accel_limit));

for(int id_count = 0; id_count < DxlIds_size; id_count++){
    // I.2.Add Dynamixel#1 goal acceleration value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAccelLim.addParam(DxlIDs[id_count], param_accel_limit);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteAccelLim addparam failed");
      return false;
    }
    else
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteAccelLim addparam successful");
    }
}

    // I.4.Syncwrite goal acceleration
    dxl_comm_result = groupSyncWriteAccelLim.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) Serial.print(packetHandler->getTxRxResult(dxl_comm_result));

    // I.5.Clear syncwrite parameter storage
    groupSyncWriteAccelLim.clearParam();
    
    // I.6. Allocate Velocity Limit value into byte array
    param_vel_limit[0] = DXL_LOBYTE(DXL_LOWORD(dxl_vel_limit));
    param_vel_limit[1] = DXL_HIBYTE(DXL_LOWORD(dxl_vel_limit));
    param_vel_limit[2] = DXL_LOBYTE(DXL_HIWORD(dxl_vel_limit));
    param_vel_limit[3] = DXL_HIBYTE(DXL_HIWORD(dxl_vel_limit));
    
    // I.7.Add Dynamixel#1 goal velocity value to the Syncwrite storage
for(int id_count = 0; id_count < DxlIds_size; id_count++){
    dxl_addparam_result = groupSyncWriteVelLim.addParam(DxlIDs[id_count], param_vel_limit);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteVelLim addparam failed");
      return false;
    }
    else
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteVelLim addparam successful");
      }
}

    // III.c.9.Syncwrite goal velocity
    dxl_comm_result = groupSyncWriteVelLim.txPacket();
    if (dxl_comm_result != COMM_SUCCESS){
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      return false;
    }
    else
    {
      Serial.print("Velocity Limit CHANGED"); Serial.print(" New Velocity Limit:"); Serial.println(dxl_vel_limit);
      Serial.print("Acceleration Limit CHANGED"); Serial.print(" New Acceleration Limit:"); Serial.println(dxl_accel_limit);
    }

    // III.c.10.Clear syncwrite parameter storage
    groupSyncWriteVelLim.clearParam();

return true;
} // END FUNCTION
