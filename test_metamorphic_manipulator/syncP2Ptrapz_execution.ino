void syncP2Ptrapz_execution(typeDxlTrapzProfParams_forP2P DxlTrapzProfParams_forP2P[] , double * StpTrapzProfParams, int TrapzProfParams_size)
{
    /*
     *  Emulates SimpleSyncP2P_TrapzVelProf_SDK
     *  Accepts 2 arrays 1: 3x4 for 3 DynamixelsProPlus and 1x4 for Nema34 -> Here for Nema34 the following arrays are extracted:
     *  1. storage_array_for_TrajAssignedDuration[5] = {h, Texec, Ta, StpVmax, StpAmax }
     *  2. storage_array_for_PROFILE_STEPS[4] = {h_step, nmov_Ta, nmov_linseg, nmov_Td}
     */

    /* 
     * I.
     * Dynamixel Initialization 
     * Here only Dxl Position change is considered for trajectory velocity profile calculation
     * Time is calculated using only One Dxl
     */
    unsigned long DxlTimeExec = dxl.calculateDxlExecTime(DxlTrapzProfParams_forP2P[0][3], DxlTrapzProfParams_forP2P[0][4], DxlTrapzProfParams_forP2P[0][1], DxlTrapzProfParams_forP2P[0][2]);
    double DxlTimeExec_sec = DxlTimeExec / 1000.0; 
    /* 
     * II.
     * Stepper Motor Properties for task execution
     */

    double hRelStp = abs( StpTrapzProfParams[1] - StpTrapzProfParams[0]);

    TrajAssignedDuration = stp.returnTrajAssignedDurationProperties(DxlTimeExec_sec, hRelStp, storage_array_for_TrajAssignedDuration, storage_array_for_TrajAssignedDuration_size);

    segmentExistsTrapz = stp.segmentExists_TrapzVelProfile(storage_array_for_TrajAssignedDuration, storage_array_for_TrajAssignedDuration_size);

    PROFILE_STEPS = stp.returnTrapzVelProfileSteps(storage_array_for_TrajAssignedDuration, storage_array_for_TrajAssignedDuration_size, storage_array_for_PROFILE_STEPS, storage_array_for_PROFILE_STEPS_size, segmentExistsTrapz);
    
    /*
     * III. Execute Sync write for Dynamixels using Indirect Addressing 
     */


    /*
     * IV. Execute Stepper Motor Motion
     */

    double initial_step_delay_time = stp.calculateInitialStepDelay(storage_array_for_TrajAssignedDuration, storage_array_for_TrajAssignedDuration_size);

    return_function_state = stp.executeStepperTrapzProfile(storage_array_for_PROFILE_STEPS, storage_array_for_PROFILE_STEPS_size, segmentExistsTrapz, DxlTimeExec_sec, initial_step_delay_time);

    /* 
     * V. After movement finishes, reads current position (for Dynamixels) and save to global variable (for stepper only)
     */

}