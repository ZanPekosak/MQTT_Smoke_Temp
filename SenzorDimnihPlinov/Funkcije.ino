void lcdData() {  // Custom function to print data on display. Menus to follow.
  lcd.clearDisplay();
  lcd.setCursor(5, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" ");
  lcd.print((char)247);
  lcd.print("C");
  lcd.setCursor(5, 15);
  lcd.print("Cas cikla: ");
  lcd.print(cycleTime / 1000);
  lcd.print(" s");
  lcd.setCursor(5, 30);
  lcd.print("IP: ");
  lcd.print(myIP);
  lcd.setCursor(5, 45);
  lcd.print("Baterija: ");
  lcd.print(battery);
  lcd.display();
}

void serialData() { //  Custom function to print data on the serial interface for debugging and plotting
  Serial.print("Temperatura: ");
  Serial.println(temp);
  Serial.print("Status termoclena: ");
  Serial.println(sensDataTrans(status));
  Serial.print("Status termoclena (num): ");
  Serial.println(status);
  Serial.print("Cas cikla: ");
  Serial.println(cycleTime);
}

char* sensDataTrans(int status) { //  Custom function which translates the integer error code into strings
  static char sensState[30];
  switch (status) {
    case 0:
      strcpy(sensState, "OK");
      break;
    case 4:
      strcpy(sensState, "Thermocouple short to VCC");
      break;
    case 128:
      strcpy(sensState, "No read done yet");
      break;
    case 129:
      strcpy(sensState, "No communication");
  }
  return sensState;
}

int retMaxMsgLen(const char* message1, const char* message2, const char* message3,  const char* message4, const char* message5){
  size_t len[] = {strlen(message1), strlen(message2), strlen(message3), strlen(message4), strlen(message5)};
  byte num = 0;
  int length = 0;

  for(uint8_t x = 0; x < 5; x++){ //  count how many strings are input into the function
    if(len[x] != 0){
      num++;
    }
    else if(len[x] == 0){
      break;
    }
  }

  length = len[0];  //  set first string length as default

  for(uint8_t y = 1; y < num; y++){
    if(len[y] > length) length = len[y]; //  compare if the next string is longer than the saved string and overwrite
  }
  return (length + 1)*2;  //  double as it needs a bit of extra space
}

void clearBuffer(char* buffer, int length){
  for(int x = 0; x < length; x++){
    buffer[x] = '\0';
  }
}

int batVolt(int pin, int ADCresolution, float maxVoltDiv, float supVolt){
  if(maxVoltDiv > supVolt){
    Serial.println("Error! In function batVolt(), you have input a larger voltage from divider than supply voltage.");
    Serial.println("Calculation impossible! Will round scaling to 1");
    maxVoltDiv = supVolt;
  }

  int raw_adc = analogRead(pin);                    //  Reads battery voltage
  int maxADC = (maxVoltDiv/supVolt)*ADCresolution;  //  Converts max ADC output based on voltage divider specs
  int mapped = (int)map(raw_adc, 0, maxADC, 0, 100);//  Maps ADC reading to percentage
  return constrain(mapped, 0, 100);                 //  Constrains max ADC value to battery percentage and returns value
}

void formAndPubData(){
  char sendCycleTime[10];                     //  Temporary array to format cycle data
  char tempMsg[6];                            //  Temporary array to format temperature data
  char send[2];                               //  Temporary array to format running state data
  char battMsg[5];                            //  Temporary array to format battery status

  sprintf(send, "%d", run);                   //  Format running state to char array - String
  sprintf(tempMsg, "%.2f", temp);             //  Format temperature to char array - String
  sprintf(sendCycleTime, "%d", cycleTime);    //  Format cycle time to char array - String
  sprintf(battMsg, "%d", battery);            //  Format battery status to char array - String

  client.publish(cpuStateTop, send);                    //  Publish running status
  client.publish(tempTop, tempMsg);                     //  Publish temperature
  client.publish(sensStateTop, sensDataTrans(status));  //  Publish sensor status
  client.publish(cpuCyleTop, sendCycleTime);            //  Publish cycle time
  client.publish(batteryTop, battMsg);                  //  Publish battery data
}