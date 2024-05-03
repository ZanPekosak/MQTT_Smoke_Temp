/*  Slovensko   */
/*  Program napisan dne 19. 4. 2024, za potrebe domače uporabe in branja MAX6675 pretvornika
termočlena. Program je zasnovan tako, da na MQTT teme pošilja izmerjeno temperaturo in status
MAX6675 pretvornika. Ker je bila okoli te kode zasnovana tudi tiskanina, se lahko tiskanino
uporablja za namene tetsiranja in razvijanja ostalih MQTT aplikacij. Edini trenutno podprti 
programirni način je preko OTA - Over The Air. Naslednje revizije bodo morda vključevale pretvornik
USB / UART. Tiskanina je primarno dimenzionirana da sede v električno razvodnico in za točno ta projekt
branja temperature dimnih plinov.
POTREBNO JE ŠE IMPLEMENTIRAT TIPKE!
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
PUSHBUTTONS STILL NEED TO BE IMPLEMENTED!
More details at my Github page: https://github.com/ZanPekosak 
Or over mail at: pekosak.zan@gmail.com */

// Last edit: 4. 5. 2024

#include <Arduino.h>           //  Library to ensure arduino functitonality
#include "EspMQTTClient.h"     //  Library to enable ESP as MQTT Client
#include <Adafruit_SSD1306.h>  //  Library to handle OLED display
#include <Wire.h>              //  Library to enable SPI functionality
#include <MAX6675.h>           //  Library to handle thermocouple data
#include <stdio.h>             //  Library to enable char array commands
#include <ArduinoJson.h>       //  Library to send intial JSON config

#define oledWakePin 4        // Display exists pin
#define wakePin 17           // Display wake pin
#define okButton  15         // First button pin
#define nextButton  16       // Second button pin
#define ADCpin 34            // ADC battery voltage monitoring

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D  // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

/// Required global variables ///
  volatile bool flag0 = 0;  //  Interrupt flag
  bool hasDisplay = 0;      //  Flag for enabling and disabling diplay  
  bool run = 0;             //  Running flag
  float temp = 0;           //  Temperature variable
  int battery = 0;          //  Battery percentage variable
  int status = 0;           //  Status return data from sensor for decoding
  char buffer[1000];        //  Buffer for sending init config to HA --> NOT USED
/// END of required global variables  ///

/// Pins for SPI connection od MAX6675  ///
  const int dataPin = 27;   //  Data pin
  const int clockPin = 25;  //  Clock pin
  const int selectPin = 26; //  Chip select pin
/// END of SPI pins ///

/// Diferent defined times for updating and sending data  ///
  unsigned long cycleTime = 5000;                 //  Time between individual MQTT broadcasts
  const unsigned long displayRefTime = 200;       //  Refresh rate of display data
  const unsigned long serialTime = 1000;          //  Refresh rate of serial stream
  const unsigned long displayOnTime = 30 * 1000;  //  Time of OLED activity after wakeup
  const unsigned long pollingRate = 250;          //  Refresh rate for reading parameters
/// END of times  ///

/// Millis trackers for keeping track of elapsed time ///
  unsigned long trackMillis = millis();          //  Elapsed time - MQTT transmit
  unsigned long trackMillis2 = millis();         //  Elapsed time - OLED refresh
  unsigned long trackMillis3 = millis();         //  Elapsed time - Serial transmit
  unsigned long trackMillis4 = millis();         //  Elapsed time - Data polling
  unsigned long displayOnTimeMillis = millis();  //  Elapsed time - OLED Wake time
/// END of millis trackers  ///

/// MQTT Topics ///
  /// Device parameter topics ///
  const char* runTop = "esp/T_DimPlin/cpu/run/set";             //  Status of operation can be enabled here (0 = not running || 1 = running)
  const char* cycleTop = "esp/T_DimPlin/cpu/cycleTime/set";     //  Cycle time can be sent here (min = 2000 || max = 20min = 1.200.000)

  /// MQTT operation topics ///
  const char* cpuStateTop = "esp/T_Dim_Plin/cpu/run/state";           //  Status of operation will be published here
  const char* tempTop = "esp/T_Dim_Plin/sensor/temp/state";           //  Temperature reading will be published here
  const char* sensStateTop = "esp/T_Dim_Plin/sensor/status/state";    //  Sensor state will be published here
  const char* cpuCyleTop = "esp/T_Dim_Plin/cpu/cycleTime/state";      //  Cycle time will be published here
  const char* batteryTop = "esp/T_DimPlin/battery/state";             //  Battery level will be published here
  const char* lastWillTop = "esp/T_Dim_Plin/cpu/lastWill";            //  Last will will be published here

  /// HA Related MQTT configs ///
  /*
  const char* HAstateConfigTop = "homeassistant/switch/T_Dim_Plin/config";  //  Config topic for state
  const char* HAtempConfigTop = "homeassistant/sensor/T_Dim_Plin/config";   //  Config topic for temperature
  const char* HAalarmConfigTop = "homeassistant/text/T_Dim_Plin/config";    //  Config topic for alarms
  const char* HAstatusTop = "homeassistant/status";                         //  Topic to post and receive the HA status message

  const char* HAstateConfigMessage = R"json(
    {
    "name": null,
    "command_topic": "homeassistant/switch/T_Dim_Plin/set",
    "state_topic": "homeassistant/switch/T_Dim_Plin/state",
    "unique_id": "state01sen",
    "device": {
      "identifiers": [
        "01sen"
      ], 
      "name": "T_Dim_Plin",
      "manufacturer": "Pekosak measurement d.o.o",
      "model": "TS01B",
      "serial_number": "1234",
      "hw_version": "1.0",
      "sw_version": "1.1"
    }
  })json";

  const char* HAtempConfigMessage = R"json(
    {
      "device_class": "temperature",
      "state_topic": "homeassistant/sensor/T_Dim_Plin/state",
      "unit_of_measurement: "°C",
      "value_template": "{{ value_json_temperature }}",
      "unique_id": "temp01sen",
      "device": {
        "identifiers": [
          "01sen"
          ]
      }
    }
  )json";

  const char* HAalarmConfigMessage = R"json(
    {
      "device_class": "text",
      "state_topic": "homeassistant/text/T_Dim_Plin/state",
      "unique_id": "alarm01sen",
      "device": {
        "identifiers": [
          "01sen"
          ]
      }
    }
  )json";
  */
/// END of MQTT topics

/// Constructor setup ///
  Adafruit_SSD1306 lcd(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  //  OLED init
  MAX6675 thermoCouple(selectPin, dataPin, clockPin);                    //  Thermocouple init
  IPAddress myIP;                                                        //  Required declaration of IP variable. IP gets updated in code later
/// END of constructor setup  ///

///  Function declarations   ///
  void IRAM_ATTR setWake() {  //Function to set interrupt flag and rudimentary debounce
    if (digitalRead(wakePin) == 0 && digitalRead(wakePin) == 0) {
      flag0 = true;
    }
  }
  void lcdData();                             //  Function to clean up code by containing all lcd data and handling
  void serialData();                          //  Function to clean up code by containing all serial data and handling
  char* sensDataTrans(int status);            //  Function returning meaning of sensor error code
  int retMaxMsgLen(const char* message1, const char* message2, const char* message3 = "\0", const char* message4 = "\0", const char* message5 = "\0");
                                              // Function returning the longest possible message (universal function --> 5 arguments)
  void clearBuffer(char* buffer, int length); //  Function to clear the buffer
  int batVolt(int pin, int ADCresolution, float maxVoltDiv, float supVolt);    //  Function to read battery voltage and convert to percentage
  void formAndPubData();                      //  Function to keep code in loop readable
///   END of function declarations    ///

/// MQTT Client settings  ///
EspMQTTClient client(
  "Pekosak_net",   // SSID
  "Zanovtelefon15",              // Network password
  "192.168.0.24",  // MQTT Broker server ip
  "mqtt",          // Can be omitted if not needed
  "zanovtelefon",          // Can be omitted if not needed
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
  pinMode(okButton, INPUT_PULLUP);    //  Pin for first button
  pinMode(nextButton, INPUT_PULLUP);  //  Pin for second button
  pinMode(ADCpin, INPUT);             //  Pin for ADC battery voltage reading

  hasDisplay = !digitalRead(oledWakePin); //  Test if board is configured for OLED display (solder bridge)

  /// OLED init and config  ///
  if(hasDisplay){
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
  }
  ///  END of OLED init and config
  
  /// Optional functionalities of EspMQTTClient ///
  client.enableDebuggingMessages();                                 // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater();                                    // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  client.enableOTA();                                               // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  client.enableLastWillMessage(lastWillTop, "Crknu bom - shit!");   // You can activate the retain flag by setting the third parameter to true
                                                                    /// END of MQTT Client function list  ///

  attachInterrupt(digitalPinToInterrupt(wakePin), setWake, FALLING);

  trackMillis = millis();
  trackMillis2 = millis();
  trackMillis3 = millis();
  trackMillis4 = millis();
  displayOnTimeMillis = millis();
}

void onConnectionEstablished() {
  //Subscribe to topics, take in parameters and display information on console
  /*
  Serial.print("Success in setting packet size?: ");
  Serial.println(client.setMaxPacketSize(retMaxMsgLen(HAstateConfigMessage, HAtempConfigMessage, HAalarmConfigMessage)));
  Serial.print("Packet max length: ");
  Serial.println(retMaxMsgLen(HAstateConfigMessage, HAtempConfigMessage, HAalarmConfigMessage));
  */
  client.subscribe(runTop, [](const String& payload) {
    Serial.println(payload);
    if (payload == "0" || payload == "false") {
      run = 0;
    } else if (payload == "1" || payload == "true") {
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

  /*
  client.publish(HAstateConfigTop, buffer);                   //  Config the state switch
  client.publish(HAtempConfigTop, buffer);                    //  Config the temperature sensor
  client.publish(HAalarmConfigTop, buffer);                   //  Config the alarm topic
  */
}

void loop() {   
  if((millis() - trackMillis4) >= pollingRate){
    status = thermoCouple.read();               //  Read the thermocouple's status
    temp = thermoCouple.getTemperature();       //  Read the thermocouple's temperature
    battery = batVolt(ADCpin, 4096, 3.15, 3.3); //  Reads battery voltage
    myIP = WiFi.localIP();                      //  Get ESP's IP address at start
    trackMillis4 = millis();                    //  Reset tick marker
  }

  if (run == 1) { //  Only send data to MQTT if switch is toggled in HA
    if ((millis() - trackMillis) >= cycleTime) {    //  Periodically send variables to MQTT
        formAndPubData();
        trackMillis = millis();                     //  Reset millis counter
      }
    } 
  if (flag0 && hasDisplay) {
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
  client.loop();
}