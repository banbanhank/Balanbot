#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>
#include <BalanbotMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman.h>


//------------------------
//PID variable(phi)
float reference = 1.75;
float kp = 18;
float ki = 300;
float kd = 0.35;
//PID variable(position)
float preference = 0;
float pkp = 0.12;
float pki = 0;
float pkd = 0.03;
//PID direction
float dreference = 3.4;
float dkp = 1;
float dki = 0;
float dkd = 0;
//PID straight
float skp = 0.015;
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
float wheel_ang = 0;
//------------------------
float dT = 0.008;
float lj = 0;
float rj = 0;
//------------------------
typedef struct{
  float pos;//-3~3
  float ang;//-200~200
  float goal;
}command;

int state;
#define CMD_SIZE 3
command cmd[CMD_SIZE];
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
    motor2.Update();
    motor1.Update();
    motor2.UpdateControl(phi-lj);
    if(lj!=0 || rj!=0){
      motor2.reset();
    }
    
    float speed_d = motor1.GetSpeed() - motor2.GetSpeed();
    int speed_d_out = directionController.Update(speed_d);

    int effort = motor2.getEffort();
    int effort1 = bound(effort+speed_d_out,255,-255);
    int effort2 = bound(effort-speed_d_out,255,-255);
    
    motor1.Rotate(effort1);
    motor2.Rotate(effort2);

    wheel_ang = motor1.GetAngle();
    
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

    cmd[0] = (command){1.0,0,10};
    cmd[1] = (command){0,-90,10};
    cmd[2] = (command){1.0,0,10};
}

void loop(){
    //updateBT();

    
    //state machine
    if(state<CMD_SIZE){
      lj = cmd[state].pos;
      rj = cmd[state].ang * 0.3;
      directionController.SetReference(dreference + rj);
      if(abs(wheel_ang-cmd[state].goal) < 1){
        motor1.reset();
        wheel_ang = 0;
        state++;
        lj = 0;
        rj = 0;
        directionController.SetReference(dreference);
        delay(3000);
      }
    }
    else{
      lj = 0;
      rj = 0;
    }
    
    
    
    if((micros()-btTimer) > 100000){
      btTimer = micros();
      String info = String(state) + ",";
      info += String(lj) + ",";
      info += String(rj);
      BT.println(info);
    }
    
}
