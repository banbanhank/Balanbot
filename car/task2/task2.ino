#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>
#include <BalanbotMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman.h>


//------------------------
//PID variable(phi)
float reference = 0;
float kp = 19;
float ki = 200;
float kd = 0.3;
//PID variable(position)
float preference = 0;
float pkp = 0.09;
float pki = 0;
float pkd = 0.03;
//PID direction
float dreference = 0;
float dkp = 0.4;
float dki = 0;
float dkd = 0;
//-------------------------
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
Kalman kalmanX; // Create the Kalman instances
double accX, accY, accZ;
double gyroX;
double kalAngleX; // Calculated angle using a Kalman filter
uint32_t timer,btTimer;
uint8_t i2cData[14]; // Buffer for I2C data
//------------------------
SoftwareSerial BT(12,13);
char val;
String recieveData = "";   
bool startRecieve = false;  
//------------------------

BalanbotMotor motor1;
BalanbotMotor motor2;
PIDController directionController;

float dT = 0.008;
float lj = 0;
float rj = 0;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

int bound(int v,int u,int d){
  v = (v < u)? v : u;
  v = (v > d)? v : d;
  return v;
}

void timerInterrupt(){
    sei();
    double phi = getPhi();
    if(abs(phi) > 30){
      motor2.SetControl(0,reference,0,0,0);
    }
    motor2.Update();
    motor1.Update();
    
    if(lj!=0 || rj!=0){
      motor2.reset();
    }
    motor2.UpdateControl(phi-lj);
    float speed_d = motor1.GetSpeed() - motor2.GetSpeed();
    int speed_d_out = directionController.Update(speed_d);
    Serial.println(speed_d_out);
    int effort = motor2.getEffort();
    int effort1 = bound(effort+speed_d_out,255,-255);
    int effort2 = bound(effort-speed_d_out,255,-255);
    
    motor1.Rotate(effort1);
    motor2.Rotate(effort2);
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
    Serial.begin(57600);   
    BT.begin(57600);
    setupMotor();
    setupMPU6050();
    MsTimer2::set(dT*1000, timerInterrupt);
    MsTimer2::start();
    btTimer=micros();
}

void loop(){
    updateBT();
    
}
