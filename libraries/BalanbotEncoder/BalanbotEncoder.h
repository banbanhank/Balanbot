#ifndef BalanbotENCODER_H
#define BalanbotENCODER_H

#include <Arduino.h>
#define PPR 390

class BalanbotEncoder{
  private:
    int mInterruptPin;
    int mDirectionPin;
    int mPosition;
    float speed;
    int lastPosition;
    float lastAngle;
  public:
    BalanbotEncoder(); 
    void SetInterruptPin(const int pin);
    void SetDirectionPin(const int pin);
    void SetPins();
    void Update();
    int GetInterruptPin();
    int getDirectionPin();
    int GetPosition();
    void setPosition(int pos);
    int GetPPR();
    void ClearPosition();
};

#endif //BalanbotENCODER_H