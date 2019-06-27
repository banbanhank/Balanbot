
void setupMotor(){
    int PWMA = 5, PWMB = 9;               // Speed control 
    int AIN1 = 6, BIN1 = 11;              // Direction +
    int AIN2 = 4, BIN2 = 10;              // Direction -
    int STBY = 7;                         // standby(停止)
    int C1_A = A3 , C2_A = 2;
    int C1_B = 8 , C2_B = 3;

    directionController.SetBound(100,-100);
    directionController.SetPID(dkp,dki,dkd);
    directionController.SetReference(dreference);

    angleController.SetBound(200,-200);
    angleController.SetPID(kp,ki,kd);
    angleController.SetReference(reference);
    
    posController.SetBound(0.7,-0.7);
    posController.SetPID(pkp,pki,pkd);
    posController.SetReference(preference);

    motor1.SetMotorPins(PWMA,AIN1,AIN2,STBY);
    motor2.SetMotorPins(PWMB,BIN1,BIN2,STBY);

    motor1.InverseRotationDirectionDefinition(false);
    motor2.InverseRotationDirectionDefinition(true);

    motor1.SetEncoderPins(C2_A,C1_A);
    motor2.SetEncoderPins(C2_B,C1_B);

    attachInterrupt(digitalPinToInterrupt(motor1.GetEncoderInterruptPin())
                    ,encoder1Interrupt,RISING);
    attachInterrupt(digitalPinToInterrupt(motor2.GetEncoderInterruptPin())
                    ,encoder2Interrupt,RISING);
}
