void syncP2Ptrapz_execution(typeDxlTrapzProfParams_forP2P DxlTrapzProfParams_forP2P[] , float * StpTrapzProfParams, int TrapzProfParams_size, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)
{
    /*
     *  Emulates SimpleSyncP2P_TrapzVelProf_SDK
     */

    /* 
     * I.
     * Dynamixel Initialization 
     * Here only Dxl Position change is considered for trajectory velocity profile calculation
     */

    /* 
     * II.
     * Stepper synchronization
     */

    /*
     * III. Execute Sync write for Dynamixels using Indirect Addressing 
     */

    /*
     * IV. Execute Stepper Motor Motion
     */

    /* 
     * V. After movement finishes, reads current position (for Dynamixels) and save to global variable (for stepper only)
     */

}