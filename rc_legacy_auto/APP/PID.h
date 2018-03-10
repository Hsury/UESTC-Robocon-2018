#ifndef __PID_H
#define __PID_H

#include "stdbool.h"
#include "delay.h"

//Constants used in some of the functions below
#define PID_AUTOMATIC 1
#define PID_MANUAL 0
#define PID_DIRECT 0
#define PID_REVERSE 1
#define PID_P_ON_M 0
#define PID_P_ON_E 1

typedef struct
{
    double dispKp;                  // * we'll hold on to the tuning parameters in user-entered 
    double dispKi;                  //   format for display purposes
    double dispKd;
    
    double kp;                      // * (P)roportional Tuning Parameter
    double ki;                      // * (I)ntegral Tuning Parameter
    double kd;                      // * (D)erivative Tuning Parameter
    
    int controllerDirection;
    int pOn;
    
    double *myInput;                // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;               //   This creates a hard link between the variables and the 
    double *mySetpoint;             //   PID, freeing the user from having to constantly tell us
                                    //   what these values are.  with pointers we'll just know.
    
    unsigned long lastTime;
    double outputSum, lastInput;
    
    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto, pOnE;
}
PID_t;

//commonly used functions **************************************************************************
PID_t PID(double* Input, double* Output, double* Setpoint, double Kp,  // * constructor.  links the PID to the Input, Output, and
          double Ki, double Kd, int POn, int ControllerDirection);     //   Setpoint.  Initial tuning parameters are also set here.
                                                                       //   (overload for specifying proportional mode)

void PID_SetMode(PID_t* PIDStruct, int Mode);                          // * sets PID to either Manual (0) or Auto (non-0)

bool PID_Compute(PID_t* PIDStruct);                                    // * performs the PID calculation.  it should be
                                                                       //   called every time loop() cycles. ON/OFF and
                                                                       //   calculation frequency can be set using SetMode
                                                                       //   SetSampleTime respectively

void PID_SetOutputLimits(PID_t* PIDStruct, double Min, double Max);    // * clamps the output to a specific range. 0-255 by default, but
                                                                       //   it's likely the user will want to change this depending on
                                                                       //   the application

//available but not commonly used functions ********************************************************
void PID_SetTunings(PID_t* PIDStruct, double Kp, double Ki, double Kd, // * While most users will set the tunings once in the
                    int POn);                                          //   constructor, this function gives the user the option
                                                                       //   of changing tunings during runtime for Adaptive control

void PID_SetControllerDirection(PID_t* PIDStruct, int Direction);      // * Sets the Direction, or "Action" of the controller. DIRECT
                                                                       //   means the output will increase when error is positive. REVERSE
                                                                       //   means the opposite.  it's very unlikely that this will be needed
                                                                       //   once it is set in the constructor.

void PID_SetSampleTime(PID_t* PIDStruct, int NewSampleTime);           // * sets the frequency, in Milliseconds, with which
                                                                       //   the PID calculation is performed.  default is 100

//Display functions ****************************************************************
double PID_GetKp(PID_t* PIDStruct);                                    //   These functions query the pid for interal values.
double PID_GetKi(PID_t* PIDStruct);                                    //   they were created mainly for the pid front-end,
double PID_GetKd(PID_t* PIDStruct);                                    //   where it's important to know what is actually 
int PID_GetMode(PID_t* PIDStruct);                                     //   inside the PID.
int PID_GetDirection(PID_t* PIDStruct);

static void PID_Initialize(PID_t* PIDStruct);

#endif
