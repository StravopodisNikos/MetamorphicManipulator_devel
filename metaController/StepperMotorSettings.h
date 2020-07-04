// Stepper Motor and Driver Settings

// MOTOR ID: STP1_ID Joint1 axis Stepper Nema34

#define HALL_SWITCH_PIN1        		10                                // This is the Hall effect-homing switch
#define HALL_SWITCH_PIN2        		9						   // This is the Hall effect-MIN limit switch			
#define HALL_SWITCH_PIN3        		8						   // This is the Hall effect-MAX limit switch

#define STEP_Pin               		5
#define DIR_Pin                		6
#define ENABLE_Pin               		7

#define LED_Pin                		15                               // the number of the LED pin
#define LOCK_Pin               		14                               // pin number to control relay for lock

#define SPR1                   		3200                             // Speps per Revolution from Dip Switch Driver Configuration [steps/rev]
#define GEAR_FACTOR_PLANETARY  	     40                               // Gear Reducer used
#define FT_CLOSED_LOOP                  200000                           // Stepper Driver Frequency [1/micros]->[*10^-6 * 1/sec]

#define STP_MOVING_STATUS_THRESHOLD     1
