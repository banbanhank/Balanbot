#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>
#include <BalanbotMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman.h>


//------------------------
//PID variable
int mode = 0;
float reference = -0.5;
float kp = 25;
float ki = 1.05;
float kd = 32;
float pkp = 0.5;
float pki = 0;
float pkd = 0;
//-------------------------
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
Kalman kalmanX; // Create the Kalman instances
double accX, accY, accZ;
double gyroX;
double kalAngleX; // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
//------------------------
SoftwareSerial BT(12,13);
char val;
String recieveData = "";   
bool startRecieve = false;  
//------------------------

BalanbotMotor motor1;
BalanbotMotor motor2;


float dT = 0.01;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
void timerInterrupt(){
    sei();
    double phi = getPhi();
    motor1.Update(phi);
    motor2.Update(phi);
    //float speed_right = motor1.GetSpeed();
    //float speed_left = motor2.GetSpeed();
    
    //BT.println(phi);
    //BT.println(speed_right);
    Serial.println(phi);
    //Serial.print(speed_right);Serial.print("\t");
    //Serial.print(speed_left);Serial.print("\t");
    //Serial.println();
}

void encoder1Interrupt(){
    motor1.UpdateEncoder();
}

void encoder2Interrupt(){
    motor2.UpdateEncoder();
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

void setup(){
    Serial.begin(115200);   
    BT.begin(115200); 
    setupMotor();
    setupMPU6050();
    MsTimer2::set(dT*1000, timerInterrupt);
    MsTimer2::start();
}

void loop(){

    //updateBT();
}
