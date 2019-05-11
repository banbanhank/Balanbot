
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
          //check reliability
      //if(data.length()!=6) return;
      for(int i=0;i<data.length();i++){
          if(data[i]<48 || data[i]>57){
            if(data[i]!='r' && data[i]!='p' && data[i]!='i' && data[i]!='d' && data[i]!='e' &&  data[i]!='o' && data[i]!='u' && data[i]!='s' && data[i]!='.' && data[i]!='-')
              return;
          }
      }    
    //Serial.println(data);
    int power1 = 0;
    int power2 = 0;
    int power_data1=0;
    int power_data2=0;

    String data_PID="" ;
    
    if(data[0]=='p'){
      mode=0;
      for(int i=1;i<data.length();i++)
        data_PID += data[i];
      kp = data_PID.toFloat();
      motor1.SetControl(mode,reference,kp,ki,kd);
      motor2.SetControl(mode,reference,kp,ki,kd);  

    }
    else if(data[0]=='i'){
      mode=0;
      for(int i=1;i<data.length();i++)
        data_PID += data[i];
      ki = data_PID.toFloat();
      motor1.SetControl(mode,reference,kp,ki,kd);
      motor2.SetControl(mode,reference,kp,ki,kd);          
    }
    else if(data[0]=='d'){
      mode=0;
      for(int i=1;i<data.length();i++)
        data_PID += data[i];
      kd = data_PID.toFloat();
      motor1.SetControl(mode,reference,kp,ki,kd);
      motor2.SetControl(mode,reference,kp,ki,kd);    
    }
    else if(data[0]=='r'){
      mode=0;
      for(int i=1;i<data.length();i++)
        data_PID += data[i];
      reference = data_PID.toFloat();
      motor1.SetControl(mode,reference,kp,ki,kd);
      motor2.SetControl(mode,reference,kp,ki,kd);   

    }
    else if(data[0]=='u'){
      mode=1;
      for(int i=1;i<data.length();i++)
        data_PID += data[i];
      pkp = data_PID.toFloat();
      motor1.SetControl(mode,preference,pkp,pki,pkd);
      motor2.SetControl(mode,preference,pkp,pki,pkd);          
    }
    else if(data[0]=='s'){
      mode=1;
      for(int i=1;i<data.length();i++)
        data_PID += data[i];
      pki = data_PID.toFloat();
      motor1.SetControl(mode,preference,pkp,pki,pkd);
      motor2.SetControl(mode,preference,pkp,pki,pkd);          
    }
    else if(data[0]=='e'){
      mode=1;
      for(int i=1;i<data.length();i++)
        data_PID += data[i];
      preference = data_PID.toFloat();
      motor1.SetControl(mode,preference,pkp,pki,pkd);
      motor2.SetControl(mode,preference,pkp,pki,pkd);          
    }            
    else{

      power1=(data[1]-48)*10+(data[2]-48);
      power2=(data[4]-48)*10+(data[5]-48);
      power_data1=power1*25.0/33.0; //0~75
      power_data2=power2*25.0/33.0; //0~75
    
      if(data[0]=='1')
          power_data1 *= -1;
    
      if(data[3]=='1')
          power_data2 *= -1;

      if(power1==0 && power2==0){
          motor1.Rotate(0); 
          motor2.Rotate(0);
      }
      else{
          motor1.Rotate(power_data1); 
          motor2.Rotate(power_data2);
      }
    }

    
}
