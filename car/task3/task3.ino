#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>
#include <BalanbotMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman.h>

//------------------------
//PID variable(phi)
float reference = -0.04;
float kp = 24;
float ki = 300;
float kd = 0.35;
//PID variable(position)
float preference = 0;
float pkp = 0.2;
float pki = 0;
float pkd = 0.06;
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
int color=0;
int shape=0;
//------------------------
typedef struct{
  bool pos;
  bool turn;
  float ang;
  float goal;
  float ang1;
  float goal1;
  bool stop_car;
  int delaytime;
}command;

int state = 0;
int index = 0;
int set_delay;
bool stop_car;
#define CMD_SIZE 18
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

    cmd[0] = (command){true,false,0.6,3,0,0,false,8000};
    
}

void loop(){
    String cmd;
    while(Serial.available()){
      char c = Serial.read();
      cmd += c;
    }
    if(cmd.length()==4){
      shape = cmd[0]-48;
      color = cmd[2]-48;
    }
    
    //updateBT();
    //sendInfo();
    stateMachine();
}

void stateMachine(){
  if(state<CMD_SIZE){
      pos_ctl = cmd[state].pos;
      turn_ctl = cmd[state].turn;
      set_delay = cmd[state].delaytime;
      stop_car = cmd[state].stop_car;
      if(stop_car){
        angleController.SetPID(0,0,0);
      }
      if(pos_ctl && !turn_ctl){
        posController.SetReference(cmd[state].goal);
        float bound = cmd[state].ang;
        posController.SetBound(bound,-bound);
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
        delay(set_delay);
        state+=1;
        command next_cmd;
        switch(shape){
          case 0:
            next_cmd = (command){true,false,1.5,5,0,0,false,100};
            break;
          case 3:
            next_cmd = (command){false,true,-100,7,0,0,false,1000};
            break;
          case 4:
            next_cmd = (command){false,true,100,-10,0,0,false,1000};
            break;
          case 5:
            next_cmd = (command){false,false,0,0,0,0,true,5000};
            break;
        };
        cmd[state] = next_cmd;
        
     }
  }
}

void pause(){
  motor1.reset();
  motor2.reset();
  posController.SetReference(0);
  posController.SetPID(pkp,pki,pkd);
  wheel_ang2 = 0;
  rj = 0;
  directionController.SetReference(dreference);
}

bool next_state(){
  bool next;
  if(pos_ctl && !turn_ctl){
    next = abs(wheel_ang2-cmd[state].goal) < 1;
  }
  else if(!pos_ctl && turn_ctl){
    next = abs(wheel_ang1-wheel_ang2-cmd[state].goal) < 1;
  }else{
    next = (abs(wheel_ang2-cmd[state].goal) < 1);
  }

  return next;
}
