 /*
  * CustomStepperMetamorphicManipulator.h - Library for controlling Steppers of a Metamorphic Manipulator
  * Created by N.A. Stravopodis, February , 2020.
  */

#ifndef CustomStepperMetamorphicManipulator_h
#define CustomStepperMetamorphicManipulator_h

#include "Arduino.h"
#include <Vector.h>
#include <Streaming.h>
//#include <vector>

//using namespace std;

const float pi              = 3.14159265359;

extern bool return_function_state;
extern Vector<double> TrajAssignedDuration;
extern Vector<double> StpTrapzProfParams;
extern Vector<unsigned long> PROFILE_STEPS;

class CustomStepperMetamorphicManipulator
{
    public:
        long currentAbsPos;
        long currentMoveRel;
        int currentDirStatus;
        bool segmentExists;

        CustomStepperMetamorphicManipulator(int stepID, int stepPin, int dirPin, int enblPin, int ledPin, int hallSwitchPin, int spr, int GEAR_FACTOR, int ft );
        
        // User gives Texec, hAbs

        // Returns hRel
        double setStepperGoalPositionAssignedDuration(double Texec, double hAbs);                                                       // -> ok

        // Returns: StpTrapzProfParams
        Vector<double> returnTrajAssignedDurationProperties(double Texec, double hRel);

            // Returns: segmentExists
            bool segmentExists_TrapzVelProfile(Vector<double> StpTrapzProfParams);
            
            // Returns: PROFILE_STEPS
            Vector<unsigned long> returnTrapzVelProfileSteps(Vector<double> StpTrapzProfParams, bool segmentExists);

            // Returns: delta_t
            double calculateInitialStepDelay(Vector<double> StpTrapzProfParams);

        bool setStepperHomePosition();

        

        // Executes Trajectory
        bool executeStepperTrapzProfile(bool segmentExists, Vector<unsigned long> PROFILE_STEPS, double Texec, double delta_t);

    private:
        int _stepID;
        int _stepPin;
        int _dirPin;
        int _enblPin;
        int _ledPin;
        int _hallSwitchPin;
        int _spr;
        int _GEAR_FACTOR;
        int _ft;
        float _a;
        float _ag;
        float _accel_width;
        
        void singleStepVarDelay(unsigned long delayTime);
};

 #endif