float * returnTrajAssignedDurationProperties(float Texec, float h) {

	// Pre - determine the width of acceleration phase
	// Ta = accel_width * Texec
	const float accel_width = 0.5;
	
	float Ta = accel_width * Texec;

	// Compute Vmax, Amax

	float Vmax = h / ( (1 - accel_width) * Texec );

	float Amax = h / ( accel_width * ( 1 - accel_width) * pow(Texec,2.0) );

 	float TrajAssignedDuration = {h, Texec, Ta, Vmax, Amax};
	return TrajAssignedDuration;
}