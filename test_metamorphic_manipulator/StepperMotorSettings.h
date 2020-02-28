// Stepper Motor and Driver Settings

// MOTOR ID: STP1_ID Joint1 axis Stepper Nema34
int stepPin           =     10;
int dirPin            =     11;
int enblPin           =     12;
int ledPin            =     13;                                 // the number of the LED pin
int hallSwitchPin     =     9;                                  // the number of the pushbutton pin - This is the Hall effect-homing switch
int lockPin           =     14;                                 // pin number to control relay for lock
int spr               =     3200;                               // Speps per Revolution from Dip Switch Driver Configuration [steps/rev]
int GEAR_FACTOR       =     40;                                 // Gear Reducer used
int ft                =     200000;                             // Stepper Driver Frequency [1/micros]->[*10^-6 * 1/sec]

#define STP_MOVING_STATUS_THRESHOLD     1
