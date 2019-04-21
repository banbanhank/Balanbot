#include "BalanbotEncoder.h"

BalanbotEncoder::BalanbotEncoder() :
  mInterruptPin (0),
  mDirectionPin (0),
  mPosition (0)
{
  v_pre = new float[5];
}

void BalanbotEncoder::SetInterruptPin(const int pin){
  mInterruptPin = pin;
}

void BalanbotEncoder::SetDirectionPin(const int pin){
  mDirectionPin = pin;
  pinMode(mDirectionPin,INPUT);
}

int BalanbotEncoder::GetInterruptPin(){
  return mInterruptPin;
}

int BalanbotEncoder::getDirectionPin(){
  return mDirectionPin;
}

int BalanbotEncoder::GetPosition(){
  return mPosition;
}

void BalanbotEncoder::setPosition(int pos){
  mPosition = pos;
}

int BalanbotEncoder::GetPPR(){
  return PPR;
}

void BalanbotEncoder::ClearPosition(){
  mPosition = 0;
}

void BalanbotEncoder::Update(){
  if (digitalRead(mDirectionPin) == HIGH)
    mPosition--;
  else
    mPosition++;
}

float BalanbotEncoder::getSpeed(float dt){
  float newAngle = 2*PI*((float)mPosition/(float)PPR)*RAD_TO_DEG;
  speed = (lastAngle - newAngle) / dt;
  lastAngle = newAngle;
  // v_pre[5]=v_pre[4];
  // v_pre[4]=v_pre[3];
  // v_pre[3]=v_pre[2];
  // v_pre[2]=v_pre[1];
  // v_pre[1]=speed;
  return speed;
}