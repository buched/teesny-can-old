
//---Start Teensy CANBus Ports and Claim Addresses - If needed 

void CAN_setup (void) {

//V_Bus is CAN-3 and is the Steering BUS
  V_Bus.begin();
  V_Bus.setBaudRate(250000);
  V_Bus.enableFIFO();
  V_Bus.setFIFOFilter(REJECT_ALL);
if (Brand == 0){
  V_Bus.setFIFOFilter(0, 0x0CAC1E13, EXT);  //Claas Curve Data & Valve State Message
  V_Bus.setFIFOFilter(1, 0x18EF1CD2, EXT);  //Claas Engage Message
  V_Bus.setFIFOFilter(2, 0x1CFFE6D2, EXT);  //Claas Work Message (CEBIS Screen MR Models)
  }
if (Brand == 1){
  V_Bus.setFIFOFilter(0, 0x0CAC1C13, EXT);  //Valtra Curve Data & Valve State Message
  V_Bus.setFIFOFilter(1, 0x18EF1C32, EXT);  //Valtra Engage Message
  }  
if (Brand == 2){
  V_Bus.setFIFOFilter(0, 0x0CACAA08, EXT);  //CaseIH Curve Data & Valve State Message
  }   
if (Brand == 3){
  V_Bus.setFIFOFilter(0, 0x0CEF2CF0, EXT);  //Fendt Curve Data & Valve State Message
  }   

// Claim V_Bus Address 
if (Brand == 0){  
  CAN_message_t msgV;
  msgV.id = 0x18EEFF1E;
  msgV.flags.extended = true;
  msgV.len = 8;
  msgV.buf[0] = 0x00;
  msgV.buf[1] = 0x00;
  msgV.buf[2] = 0xC0;
  msgV.buf[3] = 0x0C;
  msgV.buf[4] = 0x00;
  msgV.buf[5] = 0x17;
  msgV.buf[6] = 0x02;
  msgV.buf[7] = 0x20;
  V_Bus.write(msgV);
}

if (Brand == 1){    
  CAN_message_t msgV;
  msgV.id = 0x18EEFF1C;
  msgV.flags.extended = true;
  msgV.len = 8;
  msgV.buf[0] = 0x00;
  msgV.buf[1] = 0x00;
  msgV.buf[2] = 0xC0;
  msgV.buf[3] = 0x0C;
  msgV.buf[4] = 0x00;
  msgV.buf[5] = 0x17;
  msgV.buf[6] = 0x02;
  msgV.buf[7] = 0x20;
  V_Bus.write(msgV); 
}

if (Brand == 3){    
  CAN_message_t msgV;
  msgV.id = 0x18EEFF2C;
  msgV.flags.extended = true;
  msgV.len = 8;
  msgV.buf[0] = 0x00;
  msgV.buf[1] = 0x00;
  msgV.buf[2] = 0xC0;
  msgV.buf[3] = 0x0C;
  msgV.buf[4] = 0x00;
  msgV.buf[5] = 0x17;
  msgV.buf[6] = 0x02;
  msgV.buf[7] = 0x20;
  V_Bus.write(msgV); 
}

delay(500);

//ISO_Bus is CAN-2 
  ISO_Bus.begin();
  ISO_Bus.setBaudRate(250000);
  ISO_Bus.enableFIFO();
  ISO_Bus.setFIFOFilter(REJECT_ALL);
//Put filters into here to let them through (All blocked by above line)
  ISO_Bus.setFIFOFilter(0,0x0CFE45F0, EXT);  //ISOBUS Rear Hitch Infomation
  
if (Brand == 3){
  ISO_Bus.setFIFOFilter(1,0x18EF2CF0, EXT);  //Fendt Engage Message
  } 

if (Brand == 1){    
  CAN_message_t msgISO;
  msgISO.id = 0x18EEFF1C;
  msgISO.flags.extended = true;
  msgISO.len = 8;
  msgISO.buf[0] = 0x00;
  msgISO.buf[1] = 0x00;
  msgISO.buf[2] = 0xC0;
  msgISO.buf[3] = 0x0C;
  msgISO.buf[4] = 0x00;
  msgISO.buf[5] = 0x17;
  msgISO.buf[6] = 0x02;
  msgISO.buf[7] = 0x20;
  ISO_Bus.write(msgISO); 
}

if (Brand == 3){    
  CAN_message_t msgISO;
  msgISO.id = 0x18EEFF2C;
  msgISO.flags.extended = true;
  msgISO.len = 8;
  msgISO.buf[0] = 0x00;
  msgISO.buf[1] = 0x00;
  msgISO.buf[2] = 0xC0;
  msgISO.buf[3] = 0x0C;
  msgISO.buf[4] = 0x00;
  msgISO.buf[5] = 0x17;
  msgISO.buf[6] = 0x02;
  msgISO.buf[7] = 0x20;
  ISO_Bus.write(msgISO); 
}

delay (500); 

//K_Bus is CAN-1 and is the Main Tractor Bus
  K_Bus.begin();
  K_Bus.setBaudRate(250000);
  K_Bus.enableFIFO();
  K_Bus.setFIFOFilter(REJECT_ALL);
//Put filters into here to let them through (All blocked by above line)
if (Brand == 3){
  K_Bus.setFIFOFilter(0, 0x613, STD);  //Fendt Arm Rest Buttons
  } 
  
  delay (500); 
} //End CAN SETUP


//---Send V_Bus message

void VBus_Send(){
    CAN_message_t VBusSendData;
if (Brand == 0){
    VBusSendData.id = 0x0CAD131E;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 8;
    VBusSendData.buf[0] = lowByte(setCurve);
    VBusSendData.buf[1] = highByte(setCurve);
    if (intendToSteer == 1)VBusSendData.buf[2] = 253;
    if (intendToSteer == 0)VBusSendData.buf[2] = 252;
    VBusSendData.buf[3] = 0;
    VBusSendData.buf[4] = 0;
    VBusSendData.buf[5] = 0;
    VBusSendData.buf[6] = 0;
    VBusSendData.buf[7] = 0;
    V_Bus.write(VBusSendData);
}
if (Brand == 1){
    VBusSendData.id = 0x0CAD131C;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 8;
    VBusSendData.buf[0] = lowByte(setCurve);
    VBusSendData.buf[1] = highByte(setCurve);
    if (intendToSteer == 1)VBusSendData.buf[2] = 253;
    if (intendToSteer == 0)VBusSendData.buf[2] = 252;
    VBusSendData.buf[3] = 255;
    VBusSendData.buf[4] = 255;
    VBusSendData.buf[5] = 255;
    VBusSendData.buf[6] = 255;
    VBusSendData.buf[7] = 255;
    V_Bus.write(VBusSendData);
}
if (Brand == 2){
    VBusSendData.id = 0x0CAD08AA;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 8;
    VBusSendData.buf[0] = lowByte(setCurve);
    VBusSendData.buf[1] = highByte(setCurve);
    if (intendToSteer == 1)VBusSendData.buf[2] = 253;
    if (intendToSteer == 0)VBusSendData.buf[2] = 252;
    VBusSendData.buf[3] = 255;
    VBusSendData.buf[4] = 255;
    VBusSendData.buf[5] = 255;
    VBusSendData.buf[6] = 255;
    VBusSendData.buf[7] = 255;
    V_Bus.write(VBusSendData);
}
if (Brand == 3){
    FendtSetCurve = setCurve - 32128;
    VBusSendData.id = 0x0CEFF02C;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 6;
    VBusSendData.buf[0] = 5;
    VBusSendData.buf[1] = 9;
    VBusSendData.buf[3] = 10;
    if (intendToSteer == 1){  
      VBusSendData.buf[2] = 1;
      VBusSendData.buf[4] = highByte(FendtSetCurve);
      VBusSendData.buf[5] = lowByte(FendtSetCurve);
    }
    else{
      VBusSendData.buf[2] = 0;
      VBusSendData.buf[4] = 0;
      VBusSendData.buf[5] = 0; 
    }
    V_Bus.write(VBusSendData);
}
}

//---Receive V_Bus message

void VBus_Receive(){
  CAN_message_t VBusReceiveData;
if (V_Bus.read(VBusReceiveData)) {

if (Brand == 0){
  //**Current Wheel Angle & Valve State**
  if (VBusReceiveData.id == 0x0CAC1E13){        
        estCurve = ((VBusReceiveData.buf[1] << 8) + VBusReceiveData.buf[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
        steeringValveReady = (VBusReceiveData.buf[2]); 
  } 
  
  //**Engage Message**
  if (VBusReceiveData.id == 0x18EF1CD2){
    if ((VBusReceiveData.buf[0])== 39 && (VBusReceiveData.buf[1])== 1 && (VBusReceiveData.buf[2])== 241){   //Ryan MR Models?
      Time = millis();
      digitalWrite(engageLED,HIGH); 
      engageCAN = 1;
      relayTime = ((millis() + 1000));
   }
   else if ((VBusReceiveData.buf[0])== 4 && (VBusReceiveData.buf[1])== 0 && (VBusReceiveData.buf[2])== 125){ //Tony Non MR Models?
      Time = millis();
      digitalWrite(engageLED,HIGH); 
      engageCAN = 1;
      relayTime = ((millis() + 1000));
   }
  } 

  //**Work Message**
  if (VBusReceiveData.id == 0x1CFFE6D2){
    if ((VBusReceiveData.buf[0])== 144){
     workCAN = bitRead(VBusReceiveData.buf[6],0);
    }
  }
}//End Brand == 0

 if (Brand == 1){
  //**Current Wheel Angle & Valve State**
  if (VBusReceiveData.id == 0x0CAC1C13){        
        estCurve = ((VBusReceiveData.buf[1] << 8) + VBusReceiveData.buf[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
        steeringValveReady = (VBusReceiveData.buf[2]); 
  } 
  
  //**Engage Message**
  if (VBusReceiveData.id == 0x18EF1C32){
    if ((VBusReceiveData.buf[0])== 15 && (VBusReceiveData.buf[1])== 96 && (VBusReceiveData.buf[2])== 1){   
      Time = millis();
      digitalWrite(engageLED,HIGH); 
      engageCAN = 1;
      relayTime = ((millis() + 1000));
   }
  } 
}//End Brand == 1   

 if (Brand == 2){
  //**Current Wheel Angle & Valve State**
  if (VBusReceiveData.id == 0x0CACAA08){        
        estCurve = ((VBusReceiveData.buf[1] << 8) + VBusReceiveData.buf[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
        steeringValveReady = (VBusReceiveData.buf[2]); 
  } 
  
}//End Brand == 2 

 if (Brand == 3){
//**Current Wheel Angle**
 if (VBusReceiveData.len == 8 && VBusReceiveData.buf[0] == 5 && VBusReceiveData.buf[1] == 10){
      FendtEstCurve = (((int8_t)VBusReceiveData.buf[4] << 8) + VBusReceiveData.buf[5]);
      estCurve = FendtEstCurve + 32128;
      }
      
//**Cutout CAN Message** 
 if (VBusReceiveData.len == 3 && VBusReceiveData.buf[2] == 0) steeringValveReady = 80;      // Fendt Stopped Steering So CAN Not Ready
    
}//End Brand == 3 

 }//End if message 
}//End Receive V-Bus Void


//---Receive ISO_Bus message
void ISO_Receive(){
    CAN_message_t ISOBusReceiveData;
if (ISO_Bus.read(ISOBusReceiveData)) { 
  //Put code here to sort a message out from ISO-Bus if needed 
  
  //**Work Message**
  if (ISOBusReceiveData.id == 0x0CFE45F0){
    ISORearHitch = (ISOBusReceiveData.buf[0]);   
    if (steerConfig.PressureSensor == 1 && ISORearHitch < steerConfig.PulseCountMax) workCAN = 1; 
    else workCAN = 0; 
  }
  
  if (Brand == 3){
  if (ISOBusReceiveData.id == 0x18EF2CF0){   //**Fendt Engage Message**  
    if ((ISOBusReceiveData.buf[0])== 0x0F && (ISOBusReceiveData.buf[1])== 0x60 && (ISOBusReceiveData.buf[2])== 0x01){   
      Time = millis();
      digitalWrite(engageLED,HIGH); 
      engageCAN = 1;
      relayTime = ((millis() + 1000));
    }
   }
  }
  
 }
}

//---Receive K_Bus message
void K_Receive(){
    CAN_message_t KBusReceiveData;
if (K_Bus.read(KBusReceiveData)) { 
  //Put code here to sort a message out from K-Bus if needed 
  
  if (Brand == 3){
    if (KBusReceiveData.buf[0]==0x15 and KBusReceiveData.buf[2]==0x06 and KBusReceiveData.buf[3]==0xCA){
       
      if(KBusReceiveData.buf[1]==0x8A and KBusReceiveData.buf[4]==0x80) steeringValveReady = 80;      // Fendt Auto Steer Active Pressed So CAN Not Ready
      
      if (KBusReceiveData.buf[1]==0x88 and KBusReceiveData.buf[4]==0x80){     // Fendt Auto Steer Go   
      Time = millis();
      digitalWrite(engageLED,HIGH); 
      engageCAN = 1;
      relayTime = ((millis() + 1000));
   }
  }                                                             
 }
   
 }
}
