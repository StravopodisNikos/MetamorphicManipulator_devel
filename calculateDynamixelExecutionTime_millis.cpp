unsigned long Texec_dxl = calculateDynamixelExecutionTime_millis(int32_t *DxlTrapzProfParams_CT) {
	// Based on CT(Control Table) of the e-Manual of H54P PRO+ Series

	/* 
	 *	DxlTrapzProfParams_CT = {DxlInitPos, DxlGoalPosition, DxlVmax, }: All given in the units of Control Table
	 *  DxlInitPos,DxlGoalPosition -> [steps]
	 *  DxlVmax -> [0.01 rev/min]
	 *  DxlAmax -> [1    rev/min^2]
	 */

	unsigned long t1 = round( (600*DxlTrapzProfParams_CT[2])/DxlTrapzProfParams_CT[3] );
	
	int32_t Dpos = abs(DxlTrapzProfParams_CT[1]-DxlTrapzProfParams_CT[0]);

	unsigned long t2 = round( (5.977*Dpos)/DxlTrapzProfParams_CT[2] );

	unsigned long Texec_dxl = t1 + t2;                                              				
    Serial.print("Dynamixels Execution Time[millisecs] = "); Serial.println(Texec_dxl);

	return Texec_dxl;
}