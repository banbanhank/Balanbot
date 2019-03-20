#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>
#include <BalanbotMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

SoftwareSerial BT(12,13);

char val;  
String recieveData = "";   
bool startRecieve = false;  

BalanbotMotor motor1;
BalanbotMotor motor2;
BalanbotEncoder encoder1;
BalanbotEncoder encoder2;

float dT = 0.01;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
void timerInterrupt(){
    sei();
    float speed_right = encoder1.getSpeed(dT);
    float speed_left = encoder2.getSpeed(dT);
    
    Serial.print(speed_right);Serial.print("\t");
    Serial.print(speed_left);Serial.print("\t");
    Serial.println();
}

void encoder1Interrupt(){
    encoder1.Update();
}

void encoder2Interrupt(){
    encoder2.Update();
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

void setup(){
    Serial.begin(115200);   
    BT.begin(115200); 
    setupMotor();
    setupEncoder();
    setupMPU6050();
    MsTimer2::set(dT*1000, timerInterrupt);
    MsTimer2::start();
}

void loop(){
    updateBT();
    //getOrientation();
}
