/**************************************************************

  Water top-off system written and designed @ Thug, LLC.

  Code by AJ Moon

  www.thugdesign.com

 **************************************************************

   NOTE: Blynk BLE support is in beta!

 **************************************************************/
//BLYNK DEBUG CONFIG
#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG
#define SERIALDEBUG

//BLYNK can connect using Wifi or BLE. BLE is in beta and unreliably connects and will randomly drop the connection.
//Uncomment the one you wish to use, but not both. 
#define BLYNK_WIFI //Use WIFI
//#define BLYNK_BLE //Use BLE

//Blynk connectivity Setup

bool isFirstConnect = true;

char auth[] = "AUTHCODE";


//Includes

#if !(defined(BLYNK_WIFI) ^ defined(BLYNK_BLE))
#error "To many options or none selcted. Please use BLE or WIFI."
#endif


#ifdef BLYNK_WIFI
#include <SPI.h>
#include <WiFi101.h>
#include <BlynkSimpleWiFiShield101.h>
#include <WidgetRTC.h>
#include <TimeLib.h>
#include <time.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "SSID";
char pass[] = "PASSWORD";
#pragma message "Blynk will connect using WIFI!"
int status = WL_IDLE_STATUS;     // the WiFi radio's status

#endif

#ifdef BLYNK_BLE
#include <BlynkSimpleCurieBLE.h>
#include <CurieBLE.h>
BLEPeripheral  blePeripheral;
#pragma message "Blynk will connect using BLE!"
#endif

//general includes
#include <cstring>
#include <SimpleTimer.h>

//Define Pins
//Hardware Pins
//WIFI
//ARDUINO WIFI SHEILD 101 USES D5, D6, D7, D10, D11, D12, and D13 for stuff and these pins should not be used.

//Pump Control Pins (L293D)
#define pumpEnablePin 3 //Motor LED also run off this pin
#define pumpControlPin1 4
#define pumpControlPin2 8

//Water Level Destination Sensor
#define levelPin A0

//Water Level Source Sensors
#define lowPin 9
#define fullPin 2

//Status hardware
#define buzzerPin 15 //Digital pins 15 is A1 on the 101
//#define errorLedPin 13

//Virtual Pins

WidgetRTC rtc;

//Status
#define currentLevelPin V3
#define sourceLevelPin V15
#define desiredFillPin V2 //push on set
#define triggerPointPin V1 //push on set
#define statusPin V4
#define statusDescriptionPin V5 //push
WidgetLCD lcd(statusDescriptionPin);

//Control
#define powerPin V0 //switch
#define setDesiredFillPin V6
#define getDesiredFillPin V7 // Push on set
#define setTriggerPointPin V8
#define getTriggerPointPin V9 //Push on set

//Alert control
#define errorAlertPin V11
#define fillNoticePin V12
#define lowWaterAlertPin V13

//History
#define notificationHistoryPin V14

//Global Variables
bool powerStatus = false;

//pump
int flowRate = 255;
bool pumpReverse = false;
bool pumpPower = false;

//destination water level
float desiredFillLevel = 0;
float triggerPointLevel = 0;
float waterLevelCurrent = 0;
float waterLevelLast = 0;


const int levelAverageNum = 250; // how many readings of the water level are being averaged
const unsigned int waterLevelCheckFrequency = 2000; // how often simpleTimer runs to get the level

//Gauge Property changes
String triggerGaugeColor;

//Water Sensor Calibration
  #define SERIES_RESISTOR     2000

/* The following are calibration values you can fill in to compute the depth of measured liquid.
* To find these values first start with no liquid present and record the resistance as the
* ZERO_DEPTH_RESISTANCE value.  Next fill the container with a known depth of liquid and record
* the sensor resistance (in ohms) as the CALIBRATION_RESISTANCE value, and the depth (which you've
* measured ahead of time) as CALIBRATION_DEPTH.
*/
  #define ZERO_DEPTH_RESISTANCE   2047.00    // Resistance value (in ohms) when no liquid is present.
  #define CALIBRATION_RESISTANCE    732.00    // Resistance value (in ohms) when liquid is at the CALIBRATION_DEPTH.
  #define CALIBRATION_DEPTH        25.3    // Depth (in any units) of liquid as measured by the eTape. I would recommend submerging at least half the eTape but theoretically any depth should work.

#define MAX_DEPTH 31.00 //max value of your eTape in CM
#define DISPLAY_SCALE 310.00 //scale of the sliders for choosing depth settings

//Variance Set
const float upperLimit = .50;
const float lowerLimit = 1.00;
const float accuracy = .1;

//source water level
int fullState = 0;
int lowState = 0;

//flowrate Variable
const int variometerNum = 6; // Number of water level checks to use to calculate if waterlevel is rising or falling
float variometerArr[variometerNum] = {0,0,0,0,0,0};
float variometer;
const unsigned int variometerCheckTime = 12000;
int variometerCount = 0;
unsigned long sinceStart;
unsigned const long variometerDelay = variometerNum * variometerCheckTime;


///BLYNK COLORS
#define GREEN     "#23C48E"
#define BLUE      "#04C0F8"
#define YELLOW    "#ED9D00"
#define RED       "#D3435C"
#define DARK_BLUE "#5F7CD8"


//Error Status Array
#define STATUS_DESCRIPTION 0
#define STATUS_INFO_TOP 1
#define STATUS_INFO_BOTTOM 2
#define STATUS_COLOR 3

//Error States
#define MONITORING 0
#define DISABLED 1
#define REVERSE_PUMPING 2
#define PUMPING 3
#define LOW_WATER 4
#define STARTING_UP 5
#define ERROR_HIGH_TRIGGER 6
#define ERROR_PUMPING 7
#define ERROR_REV_PUMPING 8
#define ERROR_REFILL 9
#define ERROR_SOURCE_FULL 10

//error state array, description, and alert color
const String statusArr[][4] = {
  {"Monitoring", "Device is ready.", "", GREEN},
  {"Powered off", "Power on", "on control tab.", RED},
  {"Evacuating", "Pumping from ", "target to source", YELLOW},
  {"Pumping", "Pumping to", "max fill.", GREEN},
  {"Low Water", "Source container", "low on water!", YELLOW},
  {"Starting up", "Getting going", "", GREEN},
  {"Error: Min above Max", "Fix settings", "on control tab", RED},
  {"Error: Pumping", "Unknown: Check", "setup & enable!", RED},
  {"Error: Evacuating", "Unknown: Check", "setup & enable!", RED},
  {"Error: Refill", "Source container", "empty. Refill!", RED},
  {"Error: Source Full", "Reverse pumping,", "empty container!", RED},

};

//Default Status set
int currentStatus = 5;
int oldStatus = 5;
int oldStatusPush = 5;
bool inError = false;

//table row id counter
int id = 0;

//how often the main status function runs to check and change state.
const unsigned int statusCheckDelay= 500;

bool errorAlert = true;
bool fillNotice = true;
bool waterLevelNotice = true;

const unsigned int notificationFrequency = 30000;

//SimpleTimer Setup
SimpleTimer timer;

///////////// SETUP FUNCTION /////////////

void setup() {
  //Simple timer setup
  timer.setInterval(waterLevelCheckFrequency, getCurrentWaterLevel);
  timer.setInterval(notificationFrequency, pushNotification);
  timer.setInterval(statusCheckDelay, checkStatus);
  timer.setInterval(variometerCheckTime, variometerCheck);

  //Interupts
  attachInterrupt(lowPin, lowWaterNotify, FALLING);

  //pinmode Setup
  pinMode(buzzerPin, OUTPUT);
  //pinMode(errorLedPin, OUTPUT);
  pinMode(levelPin, INPUT);
  pinMode(pumpEnablePin, OUTPUT);
  pinMode(pumpControlPin1, OUTPUT);
  pinMode(pumpControlPin2, OUTPUT);
  pinMode(lowPin, INPUT_PULLUP);
  pinMode(fullPin, INPUT_PULLUP);
  
  #ifdef BLYNK_BLE
  blePeripheral.setLocalName("Water");
  blePeripheral.setDeviceName("Water");
  blePeripheral.setAppearance(384);

  Blynk.begin(blePeripheral, auth);

  blePeripheral.begin();
  #ifdef SERIALDEBUG  
  Serial.println("Waiting for connections...");
  #endif
  #endif

  #ifdef SERIALDEBUG  
  Serial.begin(9600);
  #endif

  #ifdef BLYNK_WIFI
  /*while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }*/

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    #ifdef SERIALDEBUG  
    Serial.println("WiFi shield not present");
    #endif
    // don't continue:
    while (true);
  }

  Blynk.begin(auth, ssid, pass);

  // you're connected now, so print out the data:
  #ifdef SERIALDEBUG  
  Serial.print("You're connected to the network");
  #endif
  printCurrentNet();
  #endif

  //Start rtc
  rtc.begin();

  //clean table at start
  Blynk.virtualWrite(notificationHistoryPin, "clr");

}
/*====================================================
  /////////////----MAIN LOOP FUNCTION-----/////////////
  ======================================================*/
void loop() {

  Blynk.run();

#ifdef BLYNK_BLE
  blePeripheral.poll();
#endif

  timer.run();

}

///////////// BLYNK APP WRITES/////////////

BLYNK_CONNECTED() {
  if (isFirstConnect) {
    Blynk.syncAll();
    isFirstConnect = false;
  }
  #ifdef SERIALDEBUG
  Serial.println("Blynk Connected");
  #endif
}

//ENABLED/DISABLED STATUS
BLYNK_WRITE(powerPin) {
  powerStatus = param.asInt();

  if(powerStatus)
    setStatus(STARTING_UP);
  
  #ifdef SERIALDEBUG
  Serial.print("V0 Power Pin value is: ");
  Serial.println(powerStatus);
  #endif
}

//max Fill LEVEL SET
BLYNK_WRITE(setDesiredFillPin) {
  desiredFillLevel = scaleToMeasurement(param.asInt());
  #ifdef SERIALDEBUG
  Serial.print("V6 Slider value is: ");
  Serial.println(desiredFillLevel);
  #endif

  //float tempreading = scaleToMeasurement(desiredFillLevel);

  Blynk.virtualWrite(desiredFillPin, desiredFillLevel);
  Blynk.virtualWrite(getDesiredFillPin, desiredFillLevel);

  updateControlColors();
  checkSettings();
}

//minimum fill LEVEL SET
BLYNK_WRITE(setTriggerPointPin) {
  triggerPointLevel = scaleToMeasurement(param.asInt());
  #ifdef SERIALDEBUG
  Serial.print("V8 Slider value is: ");
  Serial.println(triggerPointLevel);
  #endif
  //float tempreading = scaleToMeasurement(triggerPointLevel);

  Blynk.virtualWrite(getTriggerPointPin, triggerPointLevel);
  Blynk.virtualWrite(triggerPointPin, triggerPointLevel);

  updateControlColors();
  checkSettings();
}

//ENABLED/DISABLED Error Alert
BLYNK_WRITE(errorAlertPin) {
  errorAlert = param.asInt();
  #ifdef SERIALDEBUG
  Serial.print("V11 Error Alert Pin value is: ");
  Serial.println(errorAlert);
  #endif
}

//ENABLED/DISABLEDFill Notice
BLYNK_WRITE(fillNoticePin) {
  fillNotice = param.asInt();
  #ifdef SERIALDEBUG
  Serial.print("V12 Fill Notice Pin value is: ");
  Serial.println(fillNotice);
  #endif
}

//ENABLED/DISABLED Low Water Warning
BLYNK_WRITE(lowWaterAlertPin) {
  waterLevelNotice = param.asInt();
  
  #ifdef SERIALDEBUG
  Serial.print("V13 Low Water Pin value is: ");
  Serial.println(waterLevelNotice);
  #endif
}

/////////////  BLYNK APP READS/////////////
  BLYNK_READ(currentLevelPin) {
    Blynk.virtualWrite(currentLevelPin, waterLevelCurrent);
    //Serial.print("Current Level Requested Data: ");
    //Serial.println(currentLevelPin);
  }

  BLYNK_READ(statusPin) {

    //Serial.print("old statatus ");
    //Serial.println(oldStatus);
    //Serial.print("new statatus ");
    //Serial.println(currentStatus);
    if(currentStatus != oldStatus) {

      oldStatus = currentStatus;


      Blynk.setProperty(statusPin, "color", statusArr[currentStatus][STATUS_COLOR]);

      Blynk.virtualWrite(statusPin, statusArr[currentStatus][STATUS_DESCRIPTION]);

      lcd.clear(); //Use it to clear the LCD Widget
      lcd.print(0, 0, statusArr[currentStatus][STATUS_INFO_TOP]); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
      lcd.print(0, 1, statusArr[currentStatus][STATUS_INFO_BOTTOM]);

      writeNotificationHistory(currentStatus);
    }

  }

  BLYNK_READ(sourceLevelPin){
    fullState = digitalRead(fullPin);
    lowState = digitalRead(lowPin);
    #ifdef SERIALDEBUG
    Serial.print("fullState: ");
    Serial.println(fullState);
    Serial.print("lowState: ");
    Serial.println(lowState);
    #endif
    Blynk.virtualWrite(sourceLevelPin, fullState + lowState);
  }

///////////// CUSTOM FUNCTIONS /////////////
void updatePump() {
  if (pumpReverse && pumpPower) {
    digitalWrite(pumpControlPin1, LOW); //Reverses Pump
    digitalWrite(pumpControlPin2, HIGH);
    digitalWrite(pumpEnablePin, HIGH); //Starts Pump
  } else if (pumpPower && !pumpReverse) {
    digitalWrite(pumpControlPin1, HIGH); // Sets pump to Pump Normally
    digitalWrite(pumpControlPin2, LOW);
    digitalWrite(pumpEnablePin, HIGH); //Starts Pump
  } else {
    digitalWrite(pumpEnablePin, LOW); //stops Pump
    digitalWrite(pumpControlPin1, LOW);
    digitalWrite(pumpControlPin1, LOW);
  }
}

void checkStatus() {

    if(inError){
      pumpPower = false;
        
    }else if (!powerStatus && currentStatus != 1) {
      pumpPower = false;
      setStatus(DISABLED);
    }else if (powerStatus) {

      waterLevelLast = waterLevelCurrent;

      switch(currentStatus){
        case MONITORING : {
                    if (waterLevelCurrent <= triggerPointLevel) {
                      pumpPower = true;
                      pumpReverse = false;
                      setStatus(PUMPING); //filling
                      sinceStart = millis();
                    }else if (waterLevelCurrent > desiredFillLevel + upperLimit) {
                      pumpPower = true; //turn on pumping
                      pumpReverse = true; //reverse pump direction
                      setStatus(REVERSE_PUMPING); //evacuating
                      sinceStart = millis();
                    }
                      break;
                  }
        case DISABLED : {
                    setStatus(MONITORING);
                  }
        case REVERSE_PUMPING : {
                    if (waterLevelCurrent <= desiredFillLevel){
                      pumpPower = false;
                      pumpReverse = false;
                      setStatus(MONITORING); //monitoring
                    }
                    if(fullState)
                      setStatus(ERROR_SOURCE_FULL);
                    break;
                  }
        case PUMPING : {
                    if ( waterLevelCurrent >= desiredFillLevel) {
                      pumpPower = false;
                      setStatus(MONITORING); //monitoring
                    }
                    break;
                  }
         case STARTING_UP : {
                    setStatus(MONITORING);
                    break;
                  }
      }
    }
    updatePump();
    checkError(); // Checks to confirm water level is rising or falling when pumping. If you're getting lots of unknow errors you can disable this but you risk . 
    checkSettings();
}

void getCurrentWaterLevel() {
  float levelSum(0);

  for (int i(0); i < levelAverageNum; i++)
    levelSum += round(analogRead(levelPin));

  // Get ADC value.
  float resistance = levelSum / levelAverageNum;

  // Convert ADC reading to resistance.
  resistance = (1023.0 / resistance) - 1.0;
  resistance = round(SERIES_RESISTOR / resistance);

  // Compute Depth using X = (b - Y) / m.
  float level = ((ZERO_DEPTH_RESISTANCE - resistance) / abs( (ZERO_DEPTH_RESISTANCE - CALIBRATION_RESISTANCE) / (2.54 - CALIBRATION_DEPTH))) + 2.54;

  // Round to nearest 10th and save to current water level
  waterLevelCurrent = (floor(level * 10 + .5)) / 10;

  Blynk.virtualWrite(currentLevelPin, waterLevelCurrent);
}

float scaleToMeasurement(int input){
  return input * (MAX_DEPTH / DISPLAY_SCALE);
}

void updateControlColors(){
  String newColor;
  if (triggerPointLevel >= desiredFillLevel) {
    newColor = RED;
  } else if (triggerPointLevel >= desiredFillLevel - lowerLimit) {
    newColor = YELLOW;
  } else {
    newColor = GREEN;
  }

  // Send only if changed
  if (newColor != triggerGaugeColor) {
    triggerGaugeColor = newColor;
    Blynk.setProperty(setTriggerPointPin, "color", triggerGaugeColor);
    Blynk.setProperty(triggerPointPin, "color", triggerGaugeColor);
    Blynk.setProperty(getTriggerPointPin, "color", triggerGaugeColor);
  }
}


void pushNotification(){
  if(currentStatus != oldStatusPush) {

    oldStatusPush = currentStatus;
    bool sendPush = false;

    if(errorAlert && inError){
      sendPush = true;
    }else if (fillNotice && currentStatus == PUMPING){
      sendPush = true;
    }else if (waterLevelNotice && currentStatus == LOW_WATER){
      sendPush = true;
    }

    if(sendPush){
      sendPushNotify(currentStatus);
    }
  }

}

void checkSettings(){
    if (triggerPointLevel >= desiredFillLevel){
      setStatus(ERROR_HIGH_TRIGGER);
    }else if (currentStatus == ERROR_HIGH_TRIGGER)
      setStatus(DISABLED);

}


void setStatus(int const status){
  
  currentStatus = status;
  
  if(status >= 0 && status <= 5){
      inError = false;
  } else if (status >= 6 && status <= 10){
      inError = true;
      Blynk.virtualWrite(powerPin, false);
      powerStatus = false;
  }
}

void variometerCheck(){

    float sum = 0;

    //Adds Current Water Level to array of waterlevels
    variometerArr[variometerCount] = waterLevelCurrent;
    #ifdef SERIALDEBUG
    Serial.print("current water level:");
    Serial.println(waterLevelCurrent);
    #endif
    //Sums all available water level measurements
    for(int i(0); i < variometerNum; i++){
      sum += variometerArr[i];
      #ifdef SERIALDEBUG
      Serial.print("array value:");
      Serial.println(variometerArr[i]);
      #endif
    }

    //removes current water level measurement and subtracts some extra for variance in reading
    sum -= (variometerArr[variometerCount]);
    #ifdef SERIALDEBUG
    Serial.print("sum:");
    Serial.println(sum);
    #endif
    //gets the average water level over all previous measurements
    sum  /= (variometerNum - 1);
    #ifdef SERIALDEBUG
    Serial.print("sum:");
    Serial.println(sum);
    #endif
    //subtract the current water level measurement again
    sum -= variometerArr[variometerCount];

    /*If "sum" is negative it means the current water level is greater 
    than the average of all previous measurements. Meaning the water is 
    rising and thus we return true. If the number is positive we know the 
    current level is lower than the average of all previous measurements and thus is going down*/
    
    variometer = sum ;

    #ifdef SERIALDEBUG
    Serial.print("variometer:");
    Serial.println(variometer);
    #endif

    if(variometerCount >= (variometerNum - 1))
      variometerCount = 0;
    else
      variometerCount++;
}

void checkError(){
  if(((millis() - sinceStart) > variometerDelay) & (waterLevelCurrent > 2.8)){
    if(variometer >= 0 && currentStatus == PUMPING){
      if(!lowState)
        setStatus(ERROR_REFILL);
      else
        setStatus(ERROR_PUMPING);
    }else if(variometer <= 0 && currentStatus == REVERSE_PUMPING){
        setStatus(ERROR_REV_PUMPING);
    }
  }
}

void writeNotificationHistory(const int intstatus){
  //Write History Table
  Blynk.virtualWrite(notificationHistoryPin, "add", id, statusArr[intstatus][STATUS_DESCRIPTION] , String(hour()) + ":" + minute() + " - " + month() + "/" + day() + "/" + (year() - 2000));
  //highlighting latest added row in table
  Blynk.virtualWrite(notificationHistoryPin, "pick", id);
  id++;
}

void sendPushNotify(const int intstatus){
  Blynk.notify(statusArr[intstatus][STATUS_DESCRIPTION] + ": " + statusArr[intstatus][STATUS_INFO_TOP] + " " + statusArr[intstatus][STATUS_INFO_BOTTOM]);
}

void lowWaterNotify(){
  if(waterLevelNotice)
    sendPushNotify(LOW_WATER);
  writeNotificationHistory(LOW_WATER);
}

#if defined(BLYNK_WIFI) && defined(SERIALDEBUG)
void printCurrentNet() {
  // print the SSID of the network you're attached to:

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());


  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  Serial.print(bssid[5], HEX);
  Serial.print(":");
  Serial.print(bssid[4], HEX);
  Serial.print(":");
  Serial.print(bssid[3], HEX);
  Serial.print(":");
  Serial.print(bssid[2], HEX);
  Serial.print(":");
  Serial.print(bssid[1], HEX);
  Serial.print(":");
  Serial.println(bssid[0], HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);

}

#endif
