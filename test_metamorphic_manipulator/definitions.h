// Control Table Used Items Address
#define ADDR_PRO_TORQUE_ENABLE                  512                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION                  564
#define ADDR_PRO_GOAL_VELOCITY                  552
#define ADDR_PRO_PROF_ACCEL                     556
#define ADDR_PRO_PROF_VEL                       560
#define ADDR_PRO_PRESENT_VELOCITY               576
#define ADDR_PRO_MOVING                         570
#define ADDR_PRO_MOVING_STATUS                  571
#define ADDR_PRO_PRESENT_POSITION               580
#define ADDR_PRO_ACCEL_LIMIT                    40
#define ADDR_PRO_VEL_LIMIT                      44
#define ADDR_PRO_MAX_POS_LIMIT                  48
#define ADDR_PRO_MIN_POS_LIMIT                  52
#define ADDR_PRO_LED_BLUE                       515
#define ADDR_PRO_LED_GREEN                      514
#define ADDR_PRO_RETURN_DELAY_TIME              9
#define ADDR_PRO_BAUDRATE                       8
#define ADDR_PRO_ID                             7
#define ADDR_PRO_SECONDARY_ID                   12
#define ADDR_PRO_FIRMWARE_VER                   6
#define ADDR_PRO_MODEL_INFO                     2


// Data Byte Length of default control table items
#define LEN_PRO_TORQUE_ENABLE                   1
#define LEN_PRO_GOAL_POSITION                   4
#define LEN_PRO_GOAL_VELOCITY                   4
#define LEN_PRO_PROF_ACCEL                      4
#define LEN_PRO_PROF_VEL                        4
#define LEN_PRO_PRESENT_VELOCITY                4
#define LEN_PRO_PRESENT_POSITION                4
#define LEN_PRO_ACCEL_LIMIT                     4
#define LEN_PRO_VEL_LIMIT                       4
#define LEN_PRO_MAX_POS_LIMIT                   4
#define LEN_PRO_MAX_POS_LIMIT                   4
#define LEN_PRO_MOVING                          1
#define LEN_PRO_MOVING_STATUS                   1 
#define LEN_PRO_LED                             1
#define LEN_PRO_RETURN_DELAY_TIME               1
#define LEN_PRO_BAUDRATE                        1
#define LEN_PRO_ID                              1
#define LEN_PRO_SECONDARY_ID                    1
#define LEN_PRO_FIRMWARE_VER                    1
#define LEN_PRO_MODEL_INFO                      4

// Data Byte Length of custom command items for indirect addressing
#define LEN_PRO_INDIRECTDATA_FOR_WRITE          14                   // 4*3 for pos/vel/accel and 1+1 for leds
#define LEN_PRO_INDIRECTDATA_FOR_READ           6                    // 4 for present position and 1+1 for dxl_moving

#define ADDR_PRO_INDIRECTADDRESS_FOR_WRITE      168                  // EEPROM region for free Indirect Address for PH54!!!
#define ADDR_PRO_INDIRECTADDRESS_FOR_READ       196                  // ...
#define ADDR_PRO_INDIRECTDATA_FOR_WRITE         634                  // RAM region for the corresponding free Indirect Data for PH54!!!
#define ADDR_PRO_INDIRECTDATA_FOR_READ          648                  // ...

// Protocol version
#define PROTOCOL_VERSION                        2.0                 // Protocol Version of Dunamixels used

// Serial Communication Properties
#define BAUDRATE                                2000000
#define DEVICENAME                              "/dev/ttyACM0"
#define CMD_SERIAL                              Serial
#define ESC_ASCII_VALUE                         0x1b