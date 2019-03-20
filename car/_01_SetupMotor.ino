
void setupMotor(){
    int PWMA = 5, PWMB = 9;               // Speed control 
    int AIN1 = 6, BIN1 = 11;              // Direction +
    int AIN2 = 4, BIN2 = 10;              // Direction -
    int STBY = 7;                         // standby(停止)
    motor1.SetMotorPins(PWMA,AIN1,AIN2,STBY);
    motor2.SetMotorPins(PWMB,BIN1,BIN2,STBY);
}

void setupEncoder(){
    int C1_A = A3 , C2_A = 2;
    int C1_B = 8 , C2_B = 3;
    encoder1.SetInterruptPin(C2_A);
    encoder2.SetInterruptPin(C2_B);
    encoder1.SetDirectionPin(C1_A);
    encoder2.SetDirectionPin(C1_B);
    attachInterrupt(digitalPinToInterrupt(encoder1.GetInterruptPin())
                    ,encoder1Interrupt,RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2.GetInterruptPin())
                    ,encoder2Interrupt,RISING);
}