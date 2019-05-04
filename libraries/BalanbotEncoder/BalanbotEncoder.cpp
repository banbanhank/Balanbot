#include "BalanbotEncoder.h"

BalanbotEncoder::BalanbotEncoder() :
  mInterruptPin (0),
  mDirectionPin (0),
  mPosition (0)
{

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

