#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>
#include <BalanbotMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman.h>

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
//------------------

Kalman kalmanX; // Create the Kalman instances

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle; // Angle calculate using the gyro only
double compAngleX; // Calculated angle using a complementary filter
double kalAngleX; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


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
    double phi = getPhi();
    Serial.println(phi);
    Serial.println(speed_right);
    BT.println(phi);
    BT.println(speed_right);
    //Serial.print(speed_right);Serial.print("\t");
    //Serial.print(speed_left);Serial.print("\t");
    //Serial.println();
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
    //updateBT();
}
