/*  Synchronize Stepper Nema23 and 2 Dynamixel Motors 
 *  Trapezoidal Velocity Profiles Used for Position Control
 *  
 *  Devel Update: Wed 22/1/20 Simple function for Synchronize stp-dxl for P2P task
 *              
 */ 
#include <AccelStepper.h>
#include <DynamixelWorkbench.h>

// Stepper Definition
#define dirPin 11
#define stepPin 10
#define enblPin 12
#define ledPin 13                                                                                                       // the number of the LED pin
#define buttonPin 2                                                                                                     // the number of the pushbutton pin - This is the manual-homing switch - Here Hall effect sensor!!!
#define motorInterfaceType 8
#define STP_ID    1
#define STP_MOVING_STATUS_THRESHOLD     1
// Dynamixel Definition
#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   
#define BAUDRATE  57600
#define DXL_ID    1
#define DXL_MOVING_STATUS_THRESHOLD     10
const float pi = 3.14159265359;

// Timing variables
int stpWritePeriod = 50;                                                                                                // millis
int dxlWritePeriod = 50;                                                                                                // millis
unsigned long time_now_micros = 0;                                                                                      // Set timer counter
unsigned long time_now_millis = 0;                                                                                      // Set timer counter
const uint8_t handler_index = 0;
unsigned long curMillis;
unsigned long prevStepMillis = 0;
unsigned long millisBetweenSteps = 1; // milliseconds

// Classes Objects definition
AccelStepper stepper1(motorInterfaceType, stepPin, dirPin);                                                             // Stepper Motor, it has STP_ID
DynamixelWorkbench dxl_wb;                                                                                              // Dynamixel H54
uint8_t dxl_id = DXL_ID;
uint8_t stp1_id = STP_ID;
  
// P2P point
int pos_index = 0;
//int32_t DxlGoalPosition[2] = {501000, -501000};                                                                           // +-90 deg // with no sync
//long StpGoalPosition[2] = {500, -500};
//int32_t DxlGoalPosition[2] = {-251000, 251000};                                                                         // with sync
//long StpGoalPosition[2] = {250, -250};                                                                                


void setup() {
  // Serial Communication Baudrate:
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor

  // Variables intialization
  const char *log;
  bool result = false;
  uint16_t model_number = 0;
  long current_stepper_position;
  int buttonState         =   0;   // variable for reading the pushbutton status
  int move_finished       =   1;   // Used to check if move is completed

  // 1 Stepper Motor is tested
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);
  pinMode(ledPin, OUTPUT);          // initialize the LED pin as an output:
  pinMode(buttonPin, INPUT_PULLUP); // initialize the pushbutton pin as an input: Button open(unpressed) => we read high(1)

  digitalWrite(stepPin, LOW);
  //digitalWrite(ledPin, LOW);
  digitalWrite(enblPin, LOW);
  digitalWrite(dirPin, LOW);
  
  // 2 Dynamixel Motors Tested
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);  
  }

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to ping");
  }
  else
  {
    Serial.println("Succeeded to ping");
    Serial.print("id : ");
    Serial.print(dxl_id);
    Serial.print(" model_number : ");
    Serial.println(model_number);
  }


  int32_t get_data = 0;
  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_data, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to get present position");
  }
  else
  {
    Serial.print("Succeed to get present position(value : ");
    Serial.print(get_data);
    Serial.println(")");
  }

  // Set Limit values for Velocity/Acceleration to Dynamixel
  result = dxl_wb.itemWrite(dxl_id, "Acceleration_Limit", 20000, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to set Acceleration Limit");
  }
  else
  {
    Serial.println("Succeed to set Acceleration Limit");
  }
  result = dxl_wb.itemWrite(dxl_id, "Velocity_Limit", 2500, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to set Velocity Limit");
  }
  else
  {
    Serial.println("Succeed to set Velocity Limit");
  }

  // Torque on Dynamixel
  result = dxl_wb.torqueOn(dxl_id, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to enable torque on Dynamixel:");
    Serial.print(dxl_id);
    Serial.println(")");
  }
  else
  {
    Serial.println(log);
    Serial.printf("Torque Enabled on Dynamixel: %d \n",dxl_id);
  }

  // Set Profiles Velocity/Acceleration to Dynamixel
  /*
   * Profiles are given based on Limits at each call of jointMode function!
   */

  // Stepper Check // MUST be put inside while loop!
  InitialStepperCheck(stepper1, stp1_id);

  delay(1000);

  // Homing Stepper using AccelStepper 
  HomingStepper(stepper1, stp1_id);

  // Homing Dynamixel using Dynamixel Workbench
  HomingDynamixel(dxl_wb, dxl_id);
  
}

void loop() {

  int32_t DxlInitPos = 0;
  int32_t DxlGoalPosition = 501000;
  int32_t DxlVmax = 1200;
  int32_t DxlAmax = 10000;
  float StpInitPos = 0;
  float StpGoalPosition = 0.5*6.28318531;
  float StpVmax = 10;
  float StpAmax = 10;
  uint8_t MotorsIDs[] = {dxl_id, stp1_id};
  int32_t DxlTrapzProfParams[] = {DxlInitPos, DxlGoalPosition, DxlVmax, DxlAmax};
  float   StpTrapzProfParams[] = {StpInitPos, StpGoalPosition, StpVmax, StpAmax};
  int MotorsIds_size = 2;
  int TrapzProfParams_size =4;

  // Simple Sync Stepper-Dynamixel motor for P2P motion:
  SimpleSyncP2P_TrapzVelProf(dxl_wb,MotorsIDs, MotorsIds_size,DxlTrapzProfParams,StpTrapzProfParams, TrapzProfParams_size);
}

// ====================================================== >>>>>>>   Custom Functions Definitions    <<<<<<< ===================================================== //

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


// get serial monitor cmd line value
int getch()
{
  while(1)
  {
    if( Serial.available() > 0 )
    {
      break;
    }
  }

  return Serial.read();
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


// InitialStepperCheck
bool InitialStepperCheck(AccelStepper AccelStepperMotor, uint8_t stp_ID){
  time_now_micros = micros();
  bool  check_stepper_running = AccelStepperMotor.isRunning();
  if (check_stepper_running == true)
  {
    Serial.printf("Stepper Motor: %d is running! \n",stp_ID);
    Serial.printf("Waiting for Stepper Motor: %d to stop to comlete check! \n",stp_ID);
    AccelStepperMotor.stop();
    //delay(1000);
    while(micros() < time_now_micros + stpWritePeriod){
        //wait approx. [stpWritePeriod] μs
    }
  }
  else
  {
    long current_stepper_position = AccelStepperMotor.currentPosition();
    Serial.printf("Stepper Motor: %d is stall at Position: %ld! \n",stp_ID,current_stepper_position);
    Serial.printf("Stepper Motor: %d Check OK. \n",stp_ID);
  }

}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


// HomingStepper
bool HomingStepper(AccelStepper AccelStepperMotor, uint8_t stp_ID){
  time_now_micros = micros();
  long init_homing_step =  -1;
  
  Serial.print("Stepper is Homing . . . . . . . . . . .\n ");
  
  digitalWrite(ledPin, HIGH);                                                                           // Orange LED is ONN while HOMING
  
  while (digitalRead(buttonPin)) {                                                                      // Make the Stepper move CCW until the switch is activated   
    AccelStepperMotor.moveTo(init_homing_step);                                                         // Set the position to move to
    init_homing_step--;                                                                                 // Decrease by 1 for next move if needed
    AccelStepperMotor.run();                                                                            // Start moving the stepper
    /*while(micros() < time_now_micros + stpWritePeriod){
        //wait approx. [stpWritePeriod] μs
    }*/
    } // while
    
  // since while breaks, button is pressed => HOME is reached!
  AccelStepperMotor.setCurrentPosition(0);
  //AccelStepperMotor.setMaxSpeed(500);                                                                 // Caution: the maximum speed achievable depends on your processor and clock speed. The default maxSpeed is 1.0 steps per second.
  //AccelStepperMotor.setAcceleration(300);
  init_homing_step=1;
  
  while (!digitalRead(buttonPin)) {                 // Make the Stepper move CCW until the switch is activated   
    AccelStepperMotor.moveTo(init_homing_step);     // Set the position to move to
    init_homing_step++;                             // Decrease by 1 for next move if needed
    AccelStepperMotor.run();                        // Start moving the stepper
    //delayMicroseconds(500);
    /*while(micros() < time_now_micros + stpWritePeriod){
    //wait approx. [stpWritePeriod] μs
    }*/
    } // while

  Serial.print("Homing finished. \n");
  delay(1000);                                      // Pause for 1 sec
  
  Serial.print("Setup finished. Ready for action.  \n");
  //AccelStepperMotor.setMaxSpeed(1000.0);            // Set Max Speed of Stepper (Faster for regular movements)
  //AccelStepperMotor.setAcceleration(1000.0);        // Set Acceleration of Stepper

  digitalWrite(ledPin, LOW);                        // Orange LED is OFF
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


// HomingDynamixel
  bool HomingDynamixel(DynamixelWorkbench DxlMotor, uint8_t MOTOR_DXL_ID){
        time_now_millis = millis();
        const char *log;  
        bool result = false;
        int32_t dxl_homing_position = 0;

        // Starts homing Dynamixel
        result = dxl_wb.jointMode(MOTOR_DXL_ID, 2400, 10000, &log);

        if (result == false)
        {
              Serial.println(log);
              Serial.println("Failed to change joint mode");
        }
        else
        {
              result = DxlMotor.goalPosition(MOTOR_DXL_ID, dxl_homing_position);
              Serial.println("dynamixel homing...");
              while(millis() < time_now_millis + 5000){
              //wait approx. [dxlWritePeriod] ms
              }
              if (result == false)
              {
              //Serial.println(log);
              Serial.println("Failed to write");
              }
              else
              {
              Serial.printf("Dynamixel Motor moves to position: %d Check OK. \n",dxl_homing_position);
              }
        }

}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


// CustomStepping Function
void singleStep() {
    if (curMillis - prevStepMillis >= millisBetweenSteps) {
        prevStepMillis = curMillis;
        digitalWrite(stepPin,HIGH);
        time_now_micros = micros();
        while(micros() < time_now_micros + 100){} //wait approx. [stpWritePeriod] μs
        digitalWrite(stepPin,LOW);
        //time_now_micros = micros();
        //while(micros() < time_now_micros + 100){} //wait approx. [stpWritePeriod] μs
    }
} // END function singleStep


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


void singleStepVarDelay(unsigned long delayTime) {
  // Custom Stepping Function for Velocity-Acceleration Profiles 
    unsigned long time_now_micros = micros();
    digitalWrite(stepPin, HIGH);
    while(micros() < time_now_micros + delayTime){}                   //wait approx. [μs]
    digitalWrite(stepPin, LOW);
} // END function singleStepVarDelay
    

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

// Function Parameters
/*
 * int32_t DxlTrapzProfParams[] = {DxlInitPos, DxlGoalPosition, DxlStpTrapzProfParams[2]=1150, DxlStpTrapzProfParams[3]=10000};
 * float   StpTrapzProfParams[] = {StpInitPos, StpGoalPosition, StpStpTrapzProfParams[2], StpStpTrapzProfParams[3]};
 * int MotorsIds_size = 2;
 * int TrapzProfParams_size =4;
 */
// SimpleSyncP2P_TrapzVelProf
bool SimpleSyncP2P_TrapzVelProf(DynamixelWorkbench DxlMotor, uint8_t *MotorsIDs, int MotorsIds_size, int32_t *DxlTrapzProfParams, float *StpTrapzProfParams, int TrapzProfParams_size){
    
    /* I.
     * Dynamixel Initialization 
     * Here only Dxl Position change is considered for position status monitoring
     */
    unsigned long t1 = (600*DxlTrapzProfParams[2])/DxlTrapzProfParams[3];
    int32_t Dpos = abs(DxlTrapzProfParams[1]-DxlTrapzProfParams[0]);
    unsigned long t2 = (5.97701241*Dpos)/DxlTrapzProfParams[2];
    unsigned long DxlTimeExec = t1+t2;                      // Time Units [millisecs]
    Serial.println(DxlTimeExec);
    int32_t DxlPresentPosition = 0;                         // Dynamixel is previously Homed!
    bool result = false;
    const char *log;
    //int32_t get_data = 0;
    
    /* II.
     * Stepper Initialization
     * Checks for Stepper Trapezoidal profile
     * Based on trapzPreAssignedVelAcceleP2P.cpp
     */
    float a = 2*pi/1000;                                    // [rad], q angle @ spr=1000[steps/rev] 
    float h  = StpTrapzProfParams[1]-StpTrapzProfParams[0];
    //h = abs(h);                                           // Calculate displacement in [rad]
    float h_step1 = h/a;                                    // Calculate displacement in [steps]
    long h_step = round(h_step1);
    //Serial.println(h_step);    
    float hcond = pow(StpTrapzProfParams[2],2)/StpTrapzProfParams[3];
    float Ta;
    long  nmov_Ta;
    long  nmov_Td;
    bool segmentExists;
    long nmov_linseg;

    if(h>=hcond)                                            // Checks if linear segment exists Melchiorri
    {
      // equations 3.9 will be executed
      Serial.println("Trapezoidal Profile!");
      Ta = (1.5*StpTrapzProfParams[2])/StpTrapzProfParams[3];                                 // Calculate accelration time
      float T  = (h*StpTrapzProfParams[3]+pow(StpTrapzProfParams[2],2))/(StpTrapzProfParams[3]*StpTrapzProfParams[2]);          // Calculate total execution time
      segmentExists = true;
      // Calculate Intermediate phase steps
      float qmov_Ta = 0.5*StpTrapzProfParams[3]*pow(Ta,2);
      nmov_Ta = qmov_Ta/a;                                  // Steps of Acceleration Phase;
      nmov_linseg = h_step-2*nmov_Ta;                       // Steps of liner segment if Accel=Deccel
      nmov_Td = nmov_Ta;
    } 
    else
    {
      // equations 3.10 will be executed
      Serial.println("Triangular Profile! Vmax is recalculated!");
      Ta = sqrt(h/StpTrapzProfParams[3]);
      float T  = 2*Ta;
      Serial.println(T);
      float nVmax = StpTrapzProfParams[3]*Ta;
      Serial.printf("New maximum Velocity: %f \n",nVmax);
      segmentExists = false;
      // Calculate Intermediate phase steps
      float qmov_Ta = 0.5*StpTrapzProfParams[3]*pow(Ta,2);
      nmov_Ta = qmov_Ta/a;
      nmov_linseg = 0;
      nmov_Td = h_step-nmov_Ta;
    }

    long StpPresentPosition = 0;                                                // Moves motor until specified number of steps is reached
    float delta_t = 0.676*(1/Ta)*(2*a/StpTrapzProfParams[3]);                   // c0 with ignored inaccuracy factor
    float new_delta_t;
    unsigned long rest  = 0;
    long accel_count = 0; 
    long ctVel_count = 0;
    long decel_count = -nmov_Td;                                                // counter of steps executed for Acceleration Phase
    

    // III. Displays initial position of motors
    Serial.printf("[Stepper Motor  %d  ] Initial Position : %f \n",MotorsIDs[1],StpTrapzProfParams[0]);
    
    // Displays initial Dynamixel Position
    result = DxlMotor.itemRead(MotorsIDs[0], "Present_Position", &DxlTrapzProfParams[0] , &log);
    if (result == false)
    {
      Serial.println(log);
    }
    else
    {
      Serial.printf("[Dynamixel Motor %d ] Initial Position : %d \n",MotorsIDs[0], DxlTrapzProfParams[0]);
    }
    

    // IV. Executes P2P as long as q is not pressed
    while(1)                                                                                                              
    {
      Serial.print("Press any key to continue! (or press q to quit!)\n");
      if (getch() == 'q')                                                                                                 // If q is pressed: Turning Off torque in Dynamixel
      {
          result = dxl_wb.torqueOff(MotorsIDs[0], &log);
          if (result == false)
          {
            Serial.println(log);
            Serial.println("Failed to disable torque on Dynamixel:");
            Serial.print(MotorsIDs[0]);
            Serial.println(")");
          }
          else
          {
            Serial.println(log);
            Serial.printf("Torque Disabled on Dynamixel: %d \n",MotorsIDs[0]);
          }
          break;
      }
      
        
      // IV.a. Starts running Dynamixel using Trapezoidal Profile
      result = dxl_wb.jointMode(MotorsIDs[0], DxlTrapzProfParams[2], DxlTrapzProfParams[3], &log);

      if (result == false)
      {
        Serial.println(log);
        Serial.println("Failed to change joint mode");
      }
      else
      {
        DxlMotor.goalPosition(MotorsIDs[0], DxlTrapzProfParams[1]);
        Serial.println("Dynamixel started moving...");
      }

      time_now_millis = millis();
      // IV.b. Stepper Motor Position Control Loop
      do // executes until BOTH reach goal position
      { 

          // IV.b.1 Calls function for StepperTrapezoidal Velocity Profile  
          // =============================================================================================================================================
          StpPresentPosition++;
          // IV.b.1.I. Locate position in ramp

          if(segmentExists)   // Linear Segment exists
          {
                Serial.println("Segment exists");                               
                if(StpPresentPosition<nmov_Ta){
                  Serial.println("Acceleration Phase");
                  accel_count++;                              // Acceleration Phase: delta_t -> minimizes
                  new_delta_t =  delta_t - (2*delta_t+rest)/(4*accel_count+1);      // c_n
                  //new_rest = (2*delta_t + rest)*mod(4*accel_count+1);         // r_n
                }else if( StpPresentPosition>nmov_Ta && StpPresentPosition<(nmov_Ta+nmov_linseg) ){     // Linear Segment: delta_t -> constant
                  ctVel_count++;
                  Serial.printf("Accel Phase: %ld \n",accel_count);
                  Serial.println("Constant Velocity Phase");
                  new_delta_t = delta_t;  
                }
                else{
                  Serial.printf("CtVel Phase steps: %ld \n",ctVel_count);
                  Serial.println("Decelleration Phase");
                  decel_count++;                              // Negative Value!
                  Serial.println(decel_count);
                  new_delta_t =  delta_t - (2*delta_t+rest)/(4*decel_count+1) ;     // Deceleration Phase: delta_t -> maximizes 
                }                                                                         
          }
          else
          {                                       // Linear Segment doesn't exist
                Serial.println("Segment doesn't exist");
                if(StpPresentPosition<nmov_Ta)                            // Acceleration Phase: delta_t -> minimizes
                {
                  Serial.println("Acceleration Phase");
                  accel_count++;
                  new_delta_t = delta_t - (2*delta_t+rest)/(4*accel_count+1);       // c_n

                }                                   
                else{                                     // Deceleration Phase: delta_t -> maximizes
                  Serial.println("Decelleration Phase");
                  decel_count++;                              // Negative Value!
                  new_delta_t = delta_t - (2*delta_t+rest)/(4*decel_count+1);       // Deceleration Phase: delta_t -> maximizes 
                }                                                                       
          }
          Serial.printf("New step delay time[s]: %f \n",new_delta_t);

          
          // IV.b.1.II. Steps Motor with variable time delay step
          unsigned long new_delta_t_micros = new_delta_t*1000000;
          Serial.printf("New step delay time[micros]: %lu \n",new_delta_t_micros);
          singleStepVarDelay(new_delta_t_micros);       

          delta_t = new_delta_t;
            
          Serial.println(StpPresentPosition);

          // =============================================================================================================================================  
          // IV.b.2 Read present Dynamixel position
          result = DxlMotor.itemRead(MotorsIDs[0], "Present_Position", &DxlPresentPosition , &log);
          Serial.printf("[Dynamixel Motor %d ] Present Position : %d Goal Position : %d \n",MotorsIDs[0],DxlPresentPosition, DxlTrapzProfParams[1]);
            
          // IV.b.3 Read present Stepper position
          Serial.printf("[Stepper  Motor  %d ] Present Position : %ld Goal Position : %ld \n",MotorsIDs[1],StpPresentPosition, h_step);
          
          // IV.b.4 Print Motor status           
          Serial.println("stepper moving......");
          Serial.println("dynamixel moving....");

      //}while(   (millis() < time_now_millis + DxlTimeExec)  );
      }while( ( (abs(h_step - StpPresentPosition) != 0) ));
      
      // Change goal position
      if (pos_index == 0)
      {
        pos_index = 1;
      }
      else
      {
        pos_index = 0;
      }

    } //while(1)

} // END FUNCTION
