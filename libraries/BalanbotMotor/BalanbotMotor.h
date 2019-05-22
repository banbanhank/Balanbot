#ifndef BalanbotMOTOR_H
#define BalanbotMOTOR_H

#include "BalanbotEncoder.h"
#include "BalanbotController.h"
#include <NumericalTool.h>
#include <Arduino.h>

class BalanbotMotor{
  private:
    BalanbotEncoder mEncoder; 
    Differentiator mDifferentiator;
    float mDirectionCoefficient;
    int mPwmPin, mDirectionPinA, mDirectionPinB, mStandbyPin;
    int mControlMode;
    float mSpeed;
    float mAngle;
    void UpdateAngle();
    void UpdateSpeed();
    void UpdateControl();

  public:  
    BalanbotMotor();
    inline void SetPWMPin(const int pin);
    inline void SetDirectionPins( const int pinA, 
                                  const int pinB );
    inline void SetStandbyPin(const int pin);
    void SetMotorPins( const int pinPWM, 
                       const int directionPinA, 
                       const int directionPinB, 
                       const int standbyPin);
    void SetEncoderPins(const int interruptPin, 
                        const int directionPin);
    void SetControl(int mode, float reference, float kp, float ki, float kd);
    void SetControllerBound(float angUp,float angDown,float posUp,float posDown);
    void InverseRotationDirectionDefinition(bool ifInverse);
    int GetEncoderInterruptPin();
    float GetSpeed();
    float GetAngle();
    void Rotate(int voltage);
    void Brake();
    void UpdateEncoder();
    void UpdateControl(float phi);
    void Update(float phi);

    PIDController angleController;
    PIDController posController;
};



#endif /* BALANBOTMOTOR_H */
