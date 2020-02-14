float Texec_dxl = calculateDynamixelExecutionTime_secs(float *DxlTrapzProfParams_SI) {
	// Based on eManual of H54P PRO+ Series but conversion to SI Units

	/* 
	 *	DxlTrapzProfParams_SI = {DxlInitPos, DxlGoalPosition, DxlVmax, }: All given in SI units
	 *  DxlInitPos,DxlGoalPosition -> [rad]
	 *  DxlVmax -> [rad/sec]
	 *  DxlAmax -> [rad/sec^2]
	 */

	float t1 = 0.360065 * ( DxlTrapzProfParams_SI[2]/DxlTrapzProfParams_SI[3] ) ;
	
	float Dpos = abs(DxlTrapzProfParams_SI[1]-DxlTrapzProfParams_SI[0]);

	float t2 =  (911897.345737*Dpos)/DxlTrapzProfParams_SI[2] ;

	float Texec_dxl = t1 + t2;                                              				
    Serial.print("Dynamixels Execution Time[seconds] = "); Serial.println(Texec_dxl,6);

	return Texec_dxl;
}