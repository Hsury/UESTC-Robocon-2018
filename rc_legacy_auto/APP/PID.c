#include "PID.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID_t PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    PID_t PIDInstance;
    PIDInstance.myOutput = Output;
    PIDInstance.myInput = Input;
    PIDInstance.mySetpoint = Setpoint;
    PIDInstance.inAuto = false;
    PID_SetOutputLimits(&PIDInstance, 0, 255);                        //default output limit corresponds to the arduino pwm limits
    PIDInstance.SampleTime = 100;                                     //default Controller Sample Time is 0.1 seconds
    PID_SetControllerDirection(&PIDInstance, ControllerDirection);
    PID_SetTunings(&PIDInstance, Kp, Ki, Kd, POn);
    PIDInstance.lastTime = millis() - PIDInstance.SampleTime;
    return PIDInstance;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID_Compute(PID_t* PIDStruct)
{
    if (!PIDStruct->inAuto) return false;
    unsigned long now = millis();
    unsigned long timeChange = (now - PIDStruct->lastTime);
    if (timeChange >= PIDStruct->SampleTime)
    {
        /*Compute all the working error variables*/
        double input = *PIDStruct->myInput;
        double error = *PIDStruct->mySetpoint - input;
        double dInput = (input - PIDStruct->lastInput);
        PIDStruct->outputSum += (PIDStruct->ki * error);
        /*Add Proportional on Measurement, if P_ON_M is specified*/
        if (!PIDStruct->pOnE) PIDStruct->outputSum -= PIDStruct->kp * dInput;
        if (PIDStruct->outputSum > PIDStruct->outMax) PIDStruct->outputSum = PIDStruct->outMax;
        else if (PIDStruct->outputSum < PIDStruct->outMin) PIDStruct->outputSum = PIDStruct->outMin;
        /*Add Proportional on Error, if P_ON_E is specified*/
        double output;
        if (PIDStruct->pOnE) output = PIDStruct->kp * error;
        else output = 0;
        /*Compute Rest of PID Output*/
        output += PIDStruct->outputSum - PIDStruct->kd * dInput;
        if (output > PIDStruct->outMax) output = PIDStruct->outMax;
        else if (output < PIDStruct->outMin) output = PIDStruct->outMin;
        *PIDStruct->myOutput = output;
        /*Remember some variables for next time*/
        PIDStruct->lastInput = input;
        PIDStruct->lastTime = now;
        return true;
    }
    else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID_SetTunings(PID_t* PIDStruct, double Kp, double Ki, double Kd, int POn)
{
    if (Kp < 0 || Ki < 0 || Kd < 0) return;
    PIDStruct->pOn = POn;
    PIDStruct->pOnE = (POn == PID_P_ON_E);
    PIDStruct->dispKp = Kp;
    PIDStruct->dispKi = Ki;
    PIDStruct->dispKd = Kd;
    double SampleTimeInSec = ((double)PIDStruct->SampleTime) / 1000;
    PIDStruct->kp = Kp;
    PIDStruct->ki = Ki * SampleTimeInSec;
    PIDStruct->kd = Kd / SampleTimeInSec;
    if (PIDStruct->controllerDirection == PID_REVERSE)
    {
        PIDStruct->kp = (0 - PIDStruct->kp);
        PIDStruct->ki = (0 - PIDStruct->ki);
        PIDStruct->kd = (0 - PIDStruct->kd);
    }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID_SetSampleTime(PID_t* PIDStruct, int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        double ratio  = (double)NewSampleTime / (double)PIDStruct->SampleTime;
        PIDStruct->ki *= ratio;
        PIDStruct->kd /= ratio;
        PIDStruct->SampleTime = (unsigned long)NewSampleTime;
    }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(PID_t* PIDStruct, double Min, double Max)
{
    if (Min >= Max) return;
    PIDStruct->outMin = Min;
    PIDStruct->outMax = Max;
    if (PIDStruct->inAuto)
    {
        if (*PIDStruct->myOutput > PIDStruct->outMax) *PIDStruct->myOutput = PIDStruct->outMax;
        else if (*PIDStruct->myOutput < PIDStruct->outMin) *PIDStruct->myOutput = PIDStruct->outMin;
        if (PIDStruct->outputSum > PIDStruct->outMax) PIDStruct->outputSum = PIDStruct->outMax;
        else if (PIDStruct->outputSum < PIDStruct->outMin) PIDStruct->outputSum = PIDStruct->outMin;
    }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID_SetMode(PID_t* PIDStruct, int Mode)
{
    bool newAuto = (Mode == PID_AUTOMATIC);
    if (newAuto && !PIDStruct->inAuto)
    {
        /*we just went from manual to auto*/
        PID_Initialize(PIDStruct);
    }
    PIDStruct->inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID_Initialize(PID_t* PIDStruct)
{
    PIDStruct->outputSum = *PIDStruct->myOutput;
    PIDStruct->lastInput = *PIDStruct->myInput;
    if (PIDStruct->outputSum > PIDStruct->outMax) PIDStruct->outputSum = PIDStruct->outMax;
    else if (PIDStruct->outputSum < PIDStruct->outMin) PIDStruct->outputSum = PIDStruct->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SetControllerDirection(PID_t* PIDStruct, int Direction)
{
    if (PIDStruct->inAuto && Direction != PIDStruct->controllerDirection)
    {
        PIDStruct->kp = (0 - PIDStruct->kp);
        PIDStruct->ki = (0 - PIDStruct->ki);
        PIDStruct->kd = (0 - PIDStruct->kd);
    }
    PIDStruct->controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID_GetKp(PID_t* PIDStruct) {return PIDStruct->dispKp;}
double PID_GetKi(PID_t* PIDStruct) {return PIDStruct->dispKi;}
double PID_GetKd(PID_t* PIDStruct) {return PIDStruct->dispKd;}
int PID_GetMode(PID_t* PIDStruct) {return PIDStruct->inAuto ? PID_AUTOMATIC : PID_MANUAL;}
int PID_GetDirection(PID_t* PIDStruct) {return PIDStruct->controllerDirection;}
