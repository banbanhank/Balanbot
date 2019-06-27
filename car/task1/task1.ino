#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>
#include <BalanbotMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman.h>

//------------------------
//PID variable(phi)
float reference = 0.03;
float kp = 16;
float ki = 240;
float kd = 0.35;
//PID variable(position)
float preference = 0;
float pkp = 0.134;
float pki = 0;
float pkd = 0.034;
//PID direction
float dreference = 0;//3.4
float dkp = 1;
float dki = 0;
float dkd = 0;
//PID straight
float skp = 0.01;
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
PIDController angleController;
PIDController posController;
float wheel_ang1 = 0;
float wheel_ang2 = 0;
//------------------------
float dT = 0.008;
float rj = 0;
bool pos_ctl = true;
bool turn_ctl = false;
float nextstate_ctl = 1.0;
int delaytime;
//------------------------
typedef struct{
  bool pos;
  bool turn;
  float ang;
  float goal;
  float ang1;
  float goal1;
  float err;
  int set_delay;
}command;

int state;
#define CMD_SIZE 9
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
  Serial.println(phi);
    motor2.Update();
    motor1.Update();
    float pos_out = posController.Update(motor2.GetAngle());
    float ang_out = angleController.Update((phi-pos_out));
    int effort = (int)(ang_out);
    
    float speed_d = motor1.GetSpeed() - motor2.GetSpeed();
    int speed_d_out = directionController.Update(speed_d);
    
    int effort1 = bound(effort+speed_d_out,255,-255);
    int effort2 = bound(effort-speed_d_out,255,-255);
    
    motor1.Rotate(effort1);
    motor2.Rotate(effort2);

    wheel_ang1 = motor1.GetAngle();
    wheel_ang2 = motor2.GetAngle();
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
    int index = 0;
    cmd[index] = (command){true,false,1.3,33,0,0,5.5,1};//1~2
    index++;
    cmd[index] = (command){false,true,-65,8,0,0,1,1};//turn left
    index++;
    cmd[index] = (command){true,false,1.3,29,0,0,5.2,1};//2~3
    index++;
    cmd[index] = (command){false,true,-65,7.5,0,0,1,1};//turn left
    index++;
    cmd[index] = (command){true,false,1.3,33,0,0,5,1};//3~4
    index++;
    cmd[index] = (command){false,true,70,-18,0,0,1,1};//turn right 180
    index++;
    cmd[index] = (command){true,true,1.25,63,9.98,-9,15,1};//4~5
    index++;
    cmd[index] = (command){false,true,70,-10,0,0,1,1};//turn right 90
    index++;
    cmd[index] = (command){true,true,1.25,24,16,-7,1,1};//5~6
}

void loop(){
    //updateBT();
    //sendInfo();
    stateMachine();
}

void stateMachine(){
  if(state<CMD_SIZE){
      pos_ctl = cmd[state].pos;
      turn_ctl = cmd[state].turn;
      nextstate_ctl = cmd[state].err;//nextstate error
      delaytime = cmd[state].set_delay;
      if(pos_ctl && !turn_ctl){
        posController.SetReference(cmd[state].goal);
        float bound = cmd[state].ang;
        posController.SetBound(bound,-bound);
        directionController.SetPID(0,0,0);
      }
      else if(!pos_ctl && turn_ctl){
        posController.SetPID(0,0,0);
        rj = cmd[state].ang * 0.4;
        directionController.SetReference(dreference + rj);
      }
      else{
        posController.SetReference(cmd[state].goal);
        float bound = cmd[state].ang;
        posController.SetBound(bound,-bound);
        rj = cmd[state].ang1 * 0.4;
        directionController.SetReference(dreference + rj);
      }
      
      if(next_state()){
        pause();
        delay(delaytime);
        state++;
        
      }
  }
}

void pause(){
  motor1.reset();
  motor2.reset();
  posController.SetReference(0);
  posController.SetPID(pkp,pki,pkd);
  directionController.SetPID(dkp,dki,dkd);
  wheel_ang2 = 0;
  rj = 0;
  directionController.SetReference(dreference);
}

bool next_state(){
  bool next;
  if(pos_ctl && !turn_ctl){
    next = abs(wheel_ang2-cmd[state].goal) < nextstate_ctl;
  }
  else if(!pos_ctl && turn_ctl){
    next = abs(wheel_ang1-wheel_ang2-cmd[state].goal) < nextstate_ctl;
  }else{
    next = (abs(wheel_ang2-cmd[state].goal)<nextstate_ctl);
  }

  return next;
}
