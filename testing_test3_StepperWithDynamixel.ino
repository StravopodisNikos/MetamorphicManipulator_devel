/* Synchronize Stepper Nema23 and Dynamixel Motors */
/* Reference for Stepper:   test1_SimpleStepperControl.ino
                 Dynamixel: k_Read_Write from OpenCR Examples 
Devel Update: Wed 18/12/19 Homing functions created, inserted millis-macros for timing
              Thu 19/12/19 Simple function for Synchrovize stp-dxl for P2P task */

#include <AccelStepper.h> 
#include <MultiStepper.h>
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

// P2P point
int pos_index = 0;
int32_t DxlGoalPosition[2] = {501000, -501000};                                                                           // +-90 deg // with no sync
long StpGoalPosition[2] = {500, -500};
//int32_t DxlGoalPosition[2] = {-251000, 251000};                                                                         // with sync
//long StpGoalPosition[2] = {250, -250};                                                                                


void setup() {
  // Serial Communication Baudrate:
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor

  // Variables intialization
  const char *log;
  bool result = false;
  uint8_t dxl_id = DXL_ID;
  uint8_t stp1_id = STP_ID;
  uint16_t model_number = 0;
  long current_stepper_position;
  int buttonState         =   0;   // variable for reading the pushbutton status
  int move_finished       =   1;   // Used to check if move is completed

  // stepper
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);
  pinMode(ledPin, OUTPUT);          // initialize the LED pin as an output:
  pinMode(buttonPin, INPUT_PULLUP); // initialize the pushbutton pin as an input: Button open(unpressed) => we read high(1)

  digitalWrite(stepPin, LOW);
  //digitalWrite(ledPin, LOW);
  digitalWrite(enblPin, LOW);
  digitalWrite(dirPin, LOW);
  
  // Dynamixel Check
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

  result = dxl_wb.itemWrite(dxl_id, "LED", 1, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to LED On");
  }
  else
  {
    Serial.println("Succeed to LED On");
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

/*
  result = dxl_wb.addSyncWriteHandler(dxl_id, "Goal_Position", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler");
  }

  result = dxl_wb.addSyncReadHandler(dxl_id, "Present_Position", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync read handler");
  }
  */
  
}

void loop() {

  uint8_t dxl_id = DXL_ID;
  uint8_t stp1_id = STP_ID;
  int32_t dxl_init_pos = 0;
  int32_t stp_init_pos = 0;
  
  // Simple Sync Stepper-Dynamixel motor for P2P motion:
  SimpleSyncP2P(dxl_wb, dxl_id, stepper1, stp1_id, dxl_init_pos, stp_init_pos);

  // Turning Off torque in Dynamixel
  /*result = dxl_wb.torqueOff(dxl_id, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to disable torque on Dynamixel:");
    Serial.print(dxl_id);
    Serial.println(")");
  }
  else
  {
    Serial.println(log);
    Serial.printf("Torque Disabled on Dynamixel: %d \n",dxl_id);
  }*/
}

// Custom Function Definitions

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

// HomingDynamixel
  bool HomingDynamixel(DynamixelWorkbench DxlMotor, uint8_t MOTOR_DXL_ID){
  time_now_millis = millis();
  const char *log;  
  bool result = false;
  int32_t dxl_homing_position = 0;
  //result = DxlMotor.itemWrite(MOTOR_DXL_ID, "Goal_Position", dxl_homing_position);

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
              //DxlMotor.goalVelocity(MOTOR_DXL_ID,(float) dxlVelocity);
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

// CustomStepping Function
void singleStep() {
    if (curMillis - prevStepMillis >= millisBetweenSteps) {
        prevStepMillis = curMillis;
        digitalWrite(stepPin,HIGH);
        time_now_micros = micros();
        while(micros() < time_now_micros + 100){} //wait approx. [stpWritePeriod] μs
        digitalWrite(stepPin,LOW);
        time_now_micros = micros();
        while(micros() < time_now_micros + 100){} //wait approx. [stpWritePeriod] μs
    }
} // END function singleStep

    
// SimpleSyncP2P
bool SimpleSyncP2P(DynamixelWorkbench DxlMotor, uint8_t MOTOR_DXL_ID, AccelStepper AccelStepperMotor, uint8_t stp_ID, int32_t DxlInitPos, int32_t StpInitPos){
    long StpPresentPosition = 0;
    int32_t DxlPresentPosition = 0;
    bool result = false;
    const char *log;
    //int32_t get_data = 0;
    
    // Displays initial position of motors
    StpInitPos = AccelStepperMotor.currentPosition();                                                                // Initial Stepper Position
      Serial.printf("[Stepper Motor  %d  ] Initial Position : %d \n",stp_ID,StpInitPos);
    AccelStepperMotor.setCurrentPosition(0);
    
    // Displays initial Dynamixel Position
    result = DxlMotor.itemRead(MOTOR_DXL_ID, "Present_Position", &DxlInitPos , &log);
    if (result == false)
    {
      Serial.println(log);
    }
    else
    {
      Serial.printf("[Dynamixel Motor %d ] Initial Position : %d \n",MOTOR_DXL_ID,DxlInitPos);
    }
    
    while(1)                                                                                                              // Executes P2P as long as q is not pressed
    {
      Serial.print("Press any key to continue! (or press q to quit!)\n");
      if (getch() == 'q')                                                                                                 // If q is pressed: Turning Off torque in Dynamixel
      {
          result = dxl_wb.torqueOff(MOTOR_DXL_ID, &log);
          if (result == false)
          {
            Serial.println(log);
            Serial.println("Failed to disable torque on Dynamixel:");
            Serial.print(MOTOR_DXL_ID);
            Serial.println(")");
          }
          else
          {
            Serial.println(log);
            Serial.printf("Torque Disabled on Dynamixel: %d \n",MOTOR_DXL_ID);
          }
          break;
      }
      
    // Write Goal to Dynamixel
    float dxlVelocity = 0.5*0.00199234;
    //result = DxlMotor.goalPosition(MOTOR_DXL_ID, DxlGoalPosition[pos_index]);
    /*result = DxlMotor.itemWrite(MOTOR_DXL_ID, "Goal_Position",DxlGoalPosition[pos_index], &log);
    if (result == false)
    {
    //Serial.println(log);
    Serial.println("Failed to write Dynamixel Goal Position");
    }
    else
    {
    Serial.println("Succeded to write Dynamixel Goal Position");
    }*/

    // Write Goal to Stepper
    //float movSpeed = 1000.0*0.000277778;
    AccelStepperMotor.setSpeed(4000);
    AccelStepperMotor.setAcceleration(1000);
    AccelStepperMotor.moveTo(StpGoalPosition[pos_index]);
    //AccelStepperMotor.setSpeed(movSpeed); 
    Serial.println("Succeded to  write  Stepper Goal Position");

        /*time_now_micros = micros(); 
        // Continue to run Stepper
        AccelStepperMotor.run();
        while(micros() < time_now_micros + stpWritePeriod){
        //wait approx. [stpWritePeriod] μs
        Serial.println("stepper moving...");
        } 
           
        time_now_millis = millis();
        // Continue to run Dynamixel
        result = DxlMotor.itemWrite(MOTOR_DXL_ID, "Goal_Position",DxlGoalPosition[pos_index], &log);//==================== runs?
        //result = DxlMotor.syncWrite(MOTOR_DXL_ID,&DxlGoalPosition[pos_index], &log);//==================== runs?
        while(millis() < time_now_millis + dxlWritePeriod){
        //wait approx. [dxlWritePeriod] ms
        //Serial.println("dynamixel moving...");
        if (result == false)
        {
        //Serial.println(log);
        Serial.println("Failed to write");
        }
        else
        {
        Serial.printf("Dynamixel Motor moves to position: %d. \n",DxlGoalPosition[pos_index]);
        }
        } */
        
        /*// Starts Dynamixel
        result = dxl_wb.jointMode(MOTOR_DXL_ID, 0, 0, &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to change joint mode");
        }
        else
        {
              DxlMotor.goalPosition(MOTOR_DXL_ID, DxlGoalPosition[pos_index]);
              Serial.println("dynamixel moving...");
              time_now_millis = millis();
              while(millis() < time_now_millis + dxlWritePeriod){
              //wait approx. [dxlWritePeriod] μs
              }
              //delay(dxlWritePeriod);
        }*/
        
        // Starts running Dynamixel
        result = dxl_wb.jointMode(MOTOR_DXL_ID, 1150, 10000, &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to change joint mode");
        }
        else
        {
              DxlMotor.goalPosition(MOTOR_DXL_ID, DxlGoalPosition[pos_index]);
              //DxlMotor.goalVelocity(MOTOR_DXL_ID,(float) dxlVelocity);
              Serial.println("dynamixel moving...");
              //==========================
              /*
              time_now_millis = millis();
              while(millis() < time_now_millis + 2){
              //wait approx. [dxlWritePeriod] ms
              }*/
              
              /*
              time_now_micros = micros();
              while(micros() < time_now_micros + 200){
              //wait approx. [stpWritePeriod] μs
              }*/
              // =============================
        }
        
      time_now_millis = millis();
      do // executes until BOTH reach goal position
      { 
        // Runs Stepper with AccelStepper
        //AccelStepperMotor.moveTo(StpGoalPosition[pos_index]);
        // -> // AccelStepperMotor.run();

        // Runs Stepper with CustomStepping
        curMillis = millis();
        singleStep();
        
        /*//allo monopati...
        digitalWrite(stepPin,HIGH);
        time_now_micros = micros();
        while(micros() < time_now_micros + 500){} //wait approx. [stpWritePeriod] μs
        digitalWrite(stepPin,LOW);
        time_now_micros = micros();
        while(micros() < time_now_micros + 100){} //wait approx. [stpWritePeriod] μs
        */

        Serial.println("stepper moving...");
        Serial.println("dynamixel moving...");

          
        // Read present Dynamixel position
        result = DxlMotor.itemRead(MOTOR_DXL_ID, "Present_Position", &DxlPresentPosition , &log);
        //result = DxlMotor.syncRead(handler_index, &log);
        //result = DxlMotor.getSyncReadData(handler_index, &DxlPresentPosition, &log);
        Serial.printf("[Dynamixel Motor %d ] Present Position : %ld Goal Position : %d \n",MOTOR_DXL_ID,DxlPresentPosition, DxlGoalPosition[pos_index]);
        
        // Read present Stepper position
        //StpPresentPosition = AccelStepperMotor.currentPosition();
        StpPresentPosition++;
        Serial.printf("[Stepper  Motor  %d ] Present Position : %d Goal Position : %d \n",stp_ID,StpPresentPosition, StpGoalPosition[pos_index]);
        
      //}while( ( (abs(DxlGoalPosition[pos_index] - DxlPresentPosition) > DXL_MOVING_STATUS_THRESHOLD) && (abs(StpGoalPosition[pos_index] - StpPresentPosition) > STP_MOVING_STATUS_THRESHOLD) ) );
      //}while( ( (abs(DxlGoalPosition[pos_index] - DxlPresentPosition) > DXL_MOVING_STATUS_THRESHOLD) && (AccelStepperMotor.distanceToGo() != 0) ) ); 
      //}while( (AccelStepperMotor.distanceToGo() != 0) );
      }while(   (millis() < time_now_millis + 2750)  );
      //}while( ( (abs(StpGoalPosition[pos_index] - StpPresentPosition) > STP_MOVING_STATUS_THRESHOLD) ));
      
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
} 
