#include "BalanbotMotor.h"


BalanbotMotor::BalanbotMotor() :
  mDirectionCoefficient (1.0),
  mSpeed (0.0),
  mAngle (0.0),
  mControlMode (0)
{

}

inline void BalanbotMotor::SetPWMPin(const int pin)
{
  mPwmPin = pin;
  pinMode(mPwmPin,OUTPUT);
}

inline void BalanbotMotor::SetDirectionPins( const int pinA, 
                                             const int pinB )
{
  mDirectionPinA = pinA;
  mDirectionPinB = pinB; 
  pinMode(mDirectionPinA,OUTPUT);
  pinMode(mDirectionPinB,OUTPUT);
}

inline void BalanbotMotor::SetStandbyPin(const int pin)
{
  mStandbyPin = pin;
  pinMode(mStandbyPin,OUTPUT);
  digitalWrite(mStandbyPin, HIGH);                     // disable standby(可動)

}

void BalanbotMotor::SetMotorPins( const int pwmPin, 
                                  const int directionPinA, 
                                  const int directionPinB, 
                                  const int standbyPin )
{
  SetPWMPin(pwmPin);
  SetDirectionPins(directionPinA, directionPinB);
  SetStandbyPin(standbyPin);
  
}

void BalanbotMotor::SetEncoderPins( const int interruptPin, 
                                    const int directionPin )
{
  mEncoder.SetInterruptPin(interruptPin);
  mEncoder.SetDirectionPin(directionPin); 
}

void BalanbotMotor::SetControl(int mode, float reference,float kp, float ki, float kd)
{
  if(mode==0){
    angleController.SetPID(kp,ki,kd);
    angleController.SetReference(reference);
  }
  else{
    posController.SetPID(kp,ki,kd);
    posController.SetReference(reference);
  }
}

void BalanbotMotor::SetControllerBound(float angUp,float angDown,float posUp,float posDown){
  angleController.SetBound(angUp,angDown);
  posController.SetBound(posUp,posDown);
}


void BalanbotMotor::InverseRotationDirectionDefinition(const bool ifInverse){
  if( ifInverse )
    mDirectionCoefficient = -1.0;
  else
    mDirectionCoefficient = 1.0;
}

int BalanbotMotor::GetEncoderInterruptPin() 
{ 
  return mEncoder.GetInterruptPin();
}

float BalanbotMotor::GetSpeed() 
{
  return mSpeed;
}

float BalanbotMotor::GetAngle() 
{
  return mAngle;
}

void BalanbotMotor::Rotate(const int voltage){
  boolean inPin1 = LOW;                         // 初始轉動方向為 clockwise
  boolean inPin2 = HIGH;
  if(mDirectionCoefficient*voltage > 0) {                          // 若 direction 指定為 1 時，轉動方向為 counterclockwise
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  
  digitalWrite(mDirectionPinA, inPin1);
  digitalWrite(mDirectionPinB, inPin2);
  analogWrite(mPwmPin, abs(voltage));
  
}

void BalanbotMotor::Brake(){
  //TODO
}

void BalanbotMotor::UpdateAngle(){
  int encoderPosition = mEncoder.GetPosition();
  mAngle =  mDirectionCoefficient 
            * (2*PI) 
            * ( static_cast<float>(encoderPosition) 
              / static_cast<float>(mEncoder.GetPPR()) );
}

void BalanbotMotor::UpdateSpeed(){
  mSpeed = mDifferentiator.differential(mAngle);
}

void BalanbotMotor::UpdateEncoder(){
  mEncoder.Update();
}

void BalanbotMotor::UpdateControl(float phi)
{
  float pos_out = posController.Update(mAngle);
  int effort = (int)angleController.Update(phi+pos_out);
  Rotate(effort);
}

void BalanbotMotor::Update(float phi){
  UpdateAngle();
  UpdateSpeed();
  UpdateControl(phi);
}
