  void ping_motors()
  {
    // PING STEPPER
    // NO NEED YET(VISUAL EVALUATION IS ASSUMED)
    
    // PING DYNAMIXELS
    return_function_state = meta_dxl.pingDynamixels(dxl_id, sizeof(dxl_id),&error_code_received, dxl);
    if (return_function_state){
      DEBUG_SERIAL.println("[  INFO  ] PING DYNAMIXELS [  SUCCESS ]");
      DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
    }
    else
    {
      DEBUG_SERIAL.println("[  ERROR  ] PING DYNAMIXELS [  FAILED ]");
      DEBUG_SERIAL.print("[  ERROR CODE  ]");DEBUG_SERIAL.println(error_code_received);
    }

  return;
  } // end function
