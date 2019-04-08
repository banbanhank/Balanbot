void setupMPU6050()
{
    Wire.begin();
    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

    i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68)
    { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }

    delay(100); // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while (i2cRead(0x3B, i2cData, 6));
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];

    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

    kalmanX.setAngle(roll); // Set starting angle
    gyroXangle = roll;
    compAngleX = roll;

    timer = micros();
}

void getRoll()
{
    /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14))
        ;
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double gyroXrate = gyroX / 131.0; // Convert to deg/s

    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;



    // Serial.print(roll); Serial.print("\t");
    //Serial.print(gyroXangle); Serial.print("\t");
    //Serial.print(compAngleX); Serial.print("\t");
    Serial.println(kalAngleX);
    BT.println(kalAngleX);
}
