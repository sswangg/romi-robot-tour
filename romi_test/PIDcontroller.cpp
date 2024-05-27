#include "PIDcontroller.h"
#include <math.h>

/** \brief Used to calculate the effort from the error.
 * 
 * \param error The current error (which is calculated in the calling code).
 * */
float PIDController::calcEffort(float error)
{
    currError = error; //store in case we want it later
    sumError += currError;

    if(errorBound > 0) //cap error; errorBound == 0 means don't cap
    {
        if(fabs(sumError) > errorBound) 
        {
            sumError -= currError; //if we exceeded the limit, just subtract it off again
        }
    }

    float derivError = currError - prevError;
    prevError = currError;

    currEffort = Kp * currError + Ki * sumError + Kd * derivError;

    return currEffort;
}
