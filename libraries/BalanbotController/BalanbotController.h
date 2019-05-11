#ifndef BalanbotCONTROLLER_H
#define BalanbotCONTROLLER_H

#include <NumericalTool.h>

class PIDController{
	const float ERROR_TOLERANCE = 0.5;

	private:
		bool mSteady;
		float mReference;
		float mKp, mKi, mKd;
		float mError;
		float upper_bound;
		float lower_bound;
		Differentiator mDifferentiator;
    	Integrator mIntegrator;  

	public: 
		PIDController();
		void SetPID(float kp, float ki, float kd);
		void SetReference(float reference);
		void SetBound(float up,float down);
		bool GetIfSteady();
		float Update(float feedback);
};

#endif //BalanbotCONTROLLER_H