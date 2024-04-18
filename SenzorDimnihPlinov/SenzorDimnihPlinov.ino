/*  Slovensko   */
/*  Program napisan dne 19. 4. 2024, za potrebe domače uporabe in branja MAX6675 pretvornika
termočlena. Program je zasnovan tako, da na MQTT teme pošilja izmerjeno temperaturo in status
MAX6675 pretvornika. Ker je bila okoli te kode zasnovana tudi tiskanina, se lahko tiskanino
uporablja za namene tetsiranja in razvijanja ostalih MQTT aplikacij. Edini trenutno podprti 
programirni način je preko OTA - Over The Air. Naslednje revizije bodo morda vključevale pretvornik
USB / UART. Tiskanina je primarno dimenzionirana da sede v električno razvodnico in za točno ta projekt
branja temperature dimnih plinov.
POTREBNO JE ŠE IMPLEMENTIRAT OLED PREKLOPNIK IN TIPKE!
Več podrobnosti na mojem Githubu: https://github.com/ZanPekosak 
Ali preko maila: pekosak.zan@gmail.com */

/*  English   */
/*This code was written on the 19th of April, 2024 for the means of home lab testing and measuring
temperature with a K-type thermocouple and a MAX6675 temperature converter. The basic functionality of
the code is reading and publishing the thermocouples temperature and converter's status on dedicated
MQTT topics. There was also a PCB designed around this code, which can be used for further testing
and commissioning other SPI devices over MQTT. Currently the only programming option is via OTA.
Further revisions may feature USB to UART bridges. The PCB was mainly dimensioned to fit in an electrical
junction box and the specific need to read the temperature of furnace smoke.
OLED JUMPER AND PUSHBUTTONS STILL NEED TO BE IMPLEMENTED!
More details at my Github page: https://github.com/ZanPekosak 
Or over mail at: pekosak.zan@gmail.com */

#include <Arduino.h>           //  Library to ensure arduino functitonality
#include "EspMQTTClient.h"     //  Library to enable ESP as MQTT Client
#include <Adafruit_SSD1306.h>  //  Library to handle OLED display
#include <Wire.h>              //  Library to enable SPI functionality
#include <MAX6675.h>           //  Library to handle thermocouple data

#define oledWakePin 4        // Display select pin
#define wakePin 17           // Display wake button
#define okButton  15         // First button pin
#define nextButton  16       // Second button pin

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

volatile bool flag0 = 0;  //  interrupt flag
bool run = 0;
float temp = 0;
int status = 0;

/// Pins for SPI connection od MAX6675  ///
const int dataPin = 27;
const int clockPin = 25;
const int selectPin = 26;
/// END of SPI pins ///

/// Diferent defined times for updating and sending data  ///
unsigned long cycleTime = 5000;                 //  Time between individual MQTT broadcasts
const unsigned long displayRefTime = 200;       //  Refresh rate of display data
const unsigned long serialTime = 1000;          //  Refresh rate of serial stream
const unsigned long displayOnTime = 30 * 1000;  //  Time of OLED activity after wakeup
/// END of times X) ///

/// Millis trackers for keeping track of elapsed time ///
unsigned long trackMillis = millis();          //  Elapsed time - MQTT transmit
unsigned long trackMillis2 = millis();         //  Elapsed time - OLED refresh
unsigned long trackMillis3 = millis();         //  Elapsed time - Serial transmit
unsigned long displayOnTimeMillis = millis();  //  Elapsed time - OLED Wake time

/// MQTT topics ///
const char* runTop = "esp/run/rx";              //  Status of operation can be enabled here (0 = not running || 1 = running)
const char* cycleTop = "esp/cycle/rx";          //  Cycle time can be sent here (min = 2000 || max = 20min = 1.200.000)
const char* cpuStateTop = "esp/run/tx";         //  Status of operation will be published here
const char* tempTop = "esp/sensor/temp";        //  Temperature reading will be published here
const char* sensStateTop = "esp/sensor/state";  //  Sensor state will be published here
const char* cpuCyleTop = "esp/cycle/tx";        //  Cycle time will be published here
const char* lastWillTop = "esp/lastWill";       //  Last will will be published here
/// END of MQTT topics

Adafruit_SSD1306 lcd(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  //  OLED init
MAX6675 thermoCouple(selectPin, dataPin, clockPin);                    //  Thermocouple init
IPAddress myIP;                                                        //  Required declaration of IP variable. IP gets updated in code later

///  Function declarations   ///
void IRAM_ATTR setWake() {  //Function to set interrupt flag and rudimentary debounce
  if (digitalRead(wakePin) == 0 && digitalRead(wakePin) == 0) {
    flag0 = true;
  }
}
void  lcdData();                   //  Function to clean up code by containing all lcd data and handling
void  serialData();                //  Function to clean up code by containing all serial data and handling
char* sensDataTrans(int status);   //  Function returning meaning of sensor error code
///   END of function declarations    ///

/// MQTT Client settings  ///
EspMQTTClient client(
  "Wokwi-GUEST",   // SSID
  "",              // Network password
  "3.73.141.152",  // MQTT Broker server ip
  "user",          // Can be omitted if not needed
  "pass",          // Can be omitted if not needed
  "T_Dim_Plin",    // Client name that uniquely identify your device
  1883             // The MQTT port, default to 1883. this line can be omitted
);
/// END of MQQT Client settings ///

void setup() {
  Serial.begin(115200);               //  Serial communication initialization
  SPI.begin();                        //  SPI communication initialization
  thermoCouple.begin();               //  TC -thermocouple communication initialization
  thermoCouple.setSPIspeed(4000000);  //  TC communication speed setting

  pinMode(wakePin, INPUT_PULLUP);     //  Pin for oled selection
  pinMode(oledWakePin, INPUT_PULLUP); //  Pin for display wake up - prevent OLED burn-in
  pinMode(okButton, INPUT_PULLUP);  //  Pin for first button
  pinMode(nextButton, INPUT_PULLUP);  //  Pin for second button

  /// OLED init and config  ///
  if (!lcd.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {  //  OLED initializaiton
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  lcd.display();            //  Display splash screen
  lcd.setTextColor(WHITE);  //  set text color to white
  lcd.setTextSize(1);       //  set text size to 1
  delay(1000);              //  Wait a little
  lcd.clearDisplay();       //  Clear display
  lcd.display();            //  Turn off display
  ///  END of OLED init and config

  /// Optional functionalities of EspMQTTClient ///
  client.enableDebuggingMessages();                                 // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater();                                    // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  client.enableOTA();                                               // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  client.enableLastWillMessage(lastWillTop, "Crknu bom - bemti!");  // You can activate the retain flag by setting the third parameter to true
                                                                    /// END of MQTT Client function list  ///

  attachInterrupt(digitalPinToInterrupt(wakePin), setWake, FALLING);

  trackMillis = millis();
  trackMillis2 = millis();
  trackMillis3 = millis();
  displayOnTimeMillis = millis();
}

// This function is called once everything is connected (Wifi and MQTT)
// # --> Wildcard --> all subtopics

void onConnectionEstablished() {
  //Subscribe to topics, take in parameters and display information on console
  client.subscribe(runTop, [](const String& payload) {
    Serial.println(payload);
    if (payload == "0") {
      run = 0;
    } else if (payload == "1") {
      run = 1;
    } else {
      Serial.printf("Napaka na sprejemu stanja - topic: %s", runTop);
    }
  });

  client.subscribe(cycleTop, [](const String& payload) {
    Serial.println(payload);
    int cikel = payload.toInt();

    if (cikel >= 2000 && cikel <= 1200000) {                      //  Fuse to protect against wrong values being input
      cycleTime = cikel;                                          //  If times match --> save value
      Serial.printf("Cas cikla nastavljen na: %d\n", cycleTime);  //  Print set time
    } else {
      Serial.printf("Nastavljen cas cikla: %d izven mej [2000 <= t <= 1200000 ms]!\n", cikel);
    }
  });
}

void loop() {
  do {    
    myIP = WiFi.localIP();                //  Get ESP's IP address at start
  } while (1 == 0);

  client.loop();

  if (run == 1) {
    status = thermoCouple.read();          //  Read the thermocouple's status
    temp = thermoCouple.getTemperature();  //  Read the thermocouple's temperature

    if ((millis() - trackMillis) >= cycleTime) {  //  Periodically send variables to MQTT
      char sendCycleTime[10];                     //  Temporary array to format cycle data into
      char tempMsg[6];                            //  Temporary array to format temperature data into
      char send[2];                               //  Temporary array to format running state data

      sprintf(send, "%d", run);                 //  Format running state to char array - String
      sprintf(tempMsg, "%.2f", temp);           //  Format temperature to char array - String
      sprintf(sendCycleTime, "%d", cycleTime);  //  Format cycle time to char array - String

      client.publish(cpuStateTop, send);                    //  Publish running status
      client.publish(tempTop, tempMsg);                     //  Publish temperature
      client.publish(sensStateTop, sensDataTrans(status));  //  Publish sensor status
      client.publish(cpuCyleTop, sendCycleTime);            //  Publish cycle time
      trackMillis = millis();                               //  Reset millis counter
    }
    if (flag0 == 1) {
      if ((millis() - trackMillis2) >= displayRefTime) {  //  OLED refresh rate
        lcdData();                                        // Function to resfresh OLED
        trackMillis2 = millis();                          //  Reset millis counter
      }
      if (((millis() - displayOnTimeMillis) >= displayOnTime) && flag0 == 1) {  //  Display on time
        displayOnTimeMillis = millis();                                         //  Reset millis counter
        lcd.clearDisplay();                                                     //  Clear display
        lcd.display();                                                          //  Turn off display
        flag0 = 0;                                                              //  Reset display flag
      }
    }

    if ((millis() - trackMillis3) >= serialTime) {  //  Send data to serial port after elapsed time
      serialData();                                 //  Function to send data
      trackMillis3 = millis();                      //  Reset millis counter
    }
  }
}

void lcdData() {
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
  lcd.display();
}

void serialData() {
  Serial.print("Temperatura: ");
  Serial.println(temp);
  Serial.print("Status termoclena: ");
  Serial.println(sensDataTrans(status));
  Serial.print("Status termoclena (num): ");
  Serial.println(status);
  Serial.print("Cas cikla: ");
  Serial.println(cycleTime);
}

char* sensDataTrans(int status) {
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