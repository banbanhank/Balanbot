
void updateBT(){
    while(BT.available()){  
        startRecieve = true;  
        val=BT.read();
        recieveData += val;
    }  
    if(startRecieve){  
        startRecieve = false;  
        //Serial.println(recieveData);
        remoteControl(recieveData);
        recieveData = "";  
    }
}

void remoteControl(String data) {
    //check reliability
    if(data.length()!=6) return;
    for(int i=0;i<data.length();i++){
        if(data[i]<48 || data[i]>57) return;
    }
    //Serial.println(data);
    int power1 = 0;
    int power2 = 0;
    int power_data1=0;
    int power_data2=0;
    if(data[0]=='1')
        motor1.InverseRotationDirectionDefinition(true);
    else
        motor1.InverseRotationDirectionDefinition(false);
    
    if(data[3]=='1')
        motor2.InverseRotationDirectionDefinition(true);
    else
        motor2.InverseRotationDirectionDefinition(false);

    power1=(data[1]-48)*10+(data[2]-48);
    power2=(data[4]-48)*10+(data[5]-48);
    power_data1=power1*25.0/33.0; //0~75
    power_data2=power2*25.0/33.0; //0~75
    
    if(power1==0 && power2==0){
        motor1.Rotate(0); 
        motor2.Rotate(0);
    }
    else{
        motor1.Rotate(power_data1); 
        motor2.Rotate(power_data2);
    }
}
