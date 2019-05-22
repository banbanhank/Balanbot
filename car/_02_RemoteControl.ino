
void updateBT(){
    while(BT.available()){  
        startRecieve = true;  
        val=BT.read();
        recieveData += val;
    }  
    if(startRecieve){  
        startRecieve = false;  
       // Serial.println(recieveData);
        remoteControl(recieveData);
        recieveData = "";  
    }
}

void remoteControl(String data) {
    for(int i=0;i<data.length();i++){
        if(data[i]<48 || data[i]>57){
          if(data[i]!='r' && data[i]!='p' && data[i]!='i' && data[i]!='d' && data[i]!='e' &&  data[i]!='o' && data[i]!='u' && data[i]!='s' && data[i]!='.' && data[i]!='-')
            return;
        }
    }

    String data_PID="";
    for(int i=1;i<data.length();i++)
        data_PID += data[i];

    switch(data[0]){
      case 'p':
        kp = data_PID.toFloat();
        break;
      case 'i':
        ki = data_PID.toFloat();
        break;
      case 'd':
        kd = data_PID.toFloat();
        break;
      case 'r':
        reference = data_PID.toFloat();
        break;
      case 'o':
        pkp = data_PID.toFloat();
        break;
      case 'u':
        pki = data_PID.toFloat();
        break;
      case 's':
        pkd = data_PID.toFloat();
        break;
      case 'e':
        preference = data_PID.toFloat();
        break;
    }
    motor1.SetControl(0,reference,kp,ki,kd);
    motor2.SetControl(0,reference,kp,ki,kd);
    motor1.SetControl(1,preference,pkp,pki,pkd);
    motor2.SetControl(1,preference,pkp,pki,pkd); 
    
}
