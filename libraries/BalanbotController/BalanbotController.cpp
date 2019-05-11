#include "BalanbotController.h"

PIDController::PIDController() :
	mKp (0),
	mKi (0),
	mKd (0),
	mReference (0)
{

}

void PIDController::SetPID(float kp, float ki, float kd)
{
	mKp = kp;
	mKi = ki;
	mKd = kd;
}

void SetBound(float up,float down){
	upper_bound = up;
	lower_bound = down;
}

void PIDController::SetReference(float reference)
{
	mReference = reference;
}

bool PIDController::GetIfSteady()
{
	if(mError < ERROR_TOLERANCE)
		return true;
	return false;
}

float PIDController::Update(float feedback)
{
	float pEffort = 0.0;
	float iEffort = 0.0;
	float dEffort = 0.0;
	float effort = 0.0;
	mError = mReference - feedback;
	
	pEffort = mKp * mError;
	iEffort = mKi * mIntegrator.integral(mError);
	dEffort = mKd * mDifferentiator.differential(mError);
	effort = pEffort + iEffort + dEffort;
	effort = (effort < upper_bound)? effort : upper_bound;
	effort = (effort > lower_bound)? effort : lower_bound;
 	return effort;
}