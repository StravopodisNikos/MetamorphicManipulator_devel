 /*
  * CustomStepperMetamorphicManipulator.h - Library for controlling Steppers of a Metamorphic Manipulator
  * Created by N.A. Stravopodis, February , 2020.
  */

#ifndef CustomStepperMetamorphicManipulator_h
#define CustomStepperMetamorphicManipulator_h

#include "Arduino.h"

const float pi              = 3.14159265359;

extern bool return_function_state;

class CustomStepperMetamorphicManipulator
{
    public:
        long currentAbsPos;
        long currentMoveRel;
        int currentDirStatus;

        CustomStepperMetamorphicManipulator(int stepID, int stepPin, int dirPin, int enblPin, int ledPin, int hallSwitchPin, int spr, int GEAR_FACTOR, int ft );

        void singleStepVarDelay(unsigned long delayTime);
        
        double * returnTrajAssignedDurationProperties(double Texec, double h);

        bool executeStepperTrapzProfile(bool segmentExists, unsigned long *PROFILE_STEPS, double Texec, double delta_t);

        bool syncTrajPreassignedAccelVel(double *StpTrapzProfParams);

        bool setStepperHomePosition();

        bool setStepperGoalPositionAssignedDuration(double Texec, double h);

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
        double *StepperGoalPositionProperties;
        unsigned long PROFILE_STEPS[4];
};

 #endif