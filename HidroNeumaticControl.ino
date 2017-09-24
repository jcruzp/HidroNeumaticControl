// HidroControl
// v3.3
// Validate withoutwater flag
//
// v4.0
// Used enums for status and level
//
// v4.1
// Remove some not used code
//
// v4.2
// Show at display wifi disconnected problem
//
// v5.0
// LCDmsgs MQ message to get LCD display info
//
// v5.1
// Support to Cayenne cloud
//
// v5.2
// Adjust some bugs
//
// Copyright (C)2017 Jose Cruz. All right reserved
// web: https://sites.google.com/view/raeiot/home
//
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <Adafruit_MCP23008.h>
#include <LiquidCrystal_I2C.h>

#define USE_CAYENNE
//#define isDEBUG

const char *ssid = "YOUR WIFI SSID"; // Wifi SSID
const char *pass = "YOUR WIFI PASSWORD";   // Wifi password

const String clientID = "YOUR CLIENT ID AT CAYENNE CLOUD";

#ifdef USE_CAYENNE
// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
const String username = "YOUR MQTT USERNAME AT CAYENNE CLOUD";
const char password[] = "YOUR MQTT PASSWORD AT CAYENNE CLOUD";
const char *mqtt_server = "mqtt.mydevices.com";
const int mqtt_port = 1883;

// Cayenne type of commands
const String prefixMsg = "v1/" + username + "/things/" + clientID + "/";
const String updateCmd = prefixMsg + "digital/";
const String responseCmd = prefixMsg + "response";
const String commandCmd = prefixMsg + "cmd/";

//Notify types
enum notifyMQ
{ 
  notifyCayenne,
  notifyMosquitto
};

notifyMQ notify = notifyCayenne; //Notify Cayenne cloud or HidroControl server

WiFiClient wclient2; // Cayenne cloud

PubSubClient client(wclient2, mqtt_server, mqtt_port);
#endif

// HidroControl server topic
#define mqTopic "HidroControl"

//Define I2C control pins data and clock
#define SDA 4
#define SCL 5

//Define gpio pins from mcp23008
#define hidroPump 0   // Hidroneumatic pump port
#define waterPump 1   // Water Pump port
#define hidroValve 2  // Hidroneumatic valve port
#define tankValve 3   // Tank valve port
#define waterValve 4  // Water from street valve port
#define fullSensor 5  // Tank is full sensor port
#define light 6       // Light rele port
#define emptySensor 7 // Tank is empty sensor port

//Define gpio pins from esp-8266
#define pirsensor 12 // PIR sensor
#define piezo 15     // Piezoelectric buzzer

Adafruit_MCP23008 mcp; // MCP23008 controller

//Define initial conditions
String msg = "";   // All messages from MQTT
uint8_t _on = LOW; // On or Off rele values
uint8_t _off = HIGH;
int pirState = 0;          // Status of PIR
bool pirIrq = false;       // Interrupt from PIR
bool sendWarningTF = true; // Only send tank full warning one time
bool sendWarningTE = true; // Only send tank empty warning one time
String LCDLine1="";
String LCDLine2="";

//All tank level status
enum levelEnum
{ 
  tankFull,
  tankWithWater,
  tankEmpty
};

//All state machine
enum statusEnum
{ 
  pumpOn,
  pumpOff,
  fillpumpOn,
  fillpumpOff,
  allOff
};

statusEnum statusHidro = allOff;     //Machine state control
levelEnum tankLevel = tankWithWater; //Level tank

//Define all objects and call constructors
IPAddress server(192, 168, 1, 149); // MQTT ip address

WiFiClient wclient; // HidroControl

PubSubClient hidroControl(wclient, server); // Suscribe HidroControl

LiquidCrystal_I2C lcd(0x27, 16, 2); //Set the LCD I2C address to 0x27 for a 16 chars and 2 line display

//*********************************************
// Print and store all msgs send to LCD display
//*********************************************
void printLCD(int line, String msg)
{
  lcd.setCursor(0, line);
  lcd.print(msg);
  if (line == 0)
    LCDLine1 = msg;
  else
    LCDLine2 = msg;
}


//****************************
// Activate hidroneumatic pump
//****************************
void ActivateHidro()
{
  if (tankLevel != tankEmpty)
  {
    printLCD(1, "Hidroneumatic On");
    mcp.digitalWrite(hidroValve, _on);
    mcp.digitalWrite(tankValve, _off);
    mcp.digitalWrite(waterValve, _off);
    mcp.digitalWrite(hidroPump, _on);
    mcp.digitalWrite(waterPump, _off);
    statusHidro = pumpOn;
  }
  else
  {
    statusHidro = pumpOff;
    printLCD(1, "WithOut Water   ");
  }
}

//***********************************
// Deactivate pump from hidroneumatic
//***********************************
void DeactivateHidro()
{
  statusHidro = pumpOff;
  FillTankWithStreetWater();
}

//********************
// Fill tank with pump
//********************
void FillTankWithPump()
{
  if (tankLevel != tankFull)
  {
    printLCD(1, "Fill Tank Pump  ");
    mcp.digitalWrite(hidroValve, _off);
    mcp.digitalWrite(tankValve, _off);
    mcp.digitalWrite(waterValve, _on);
    mcp.digitalWrite(hidroPump, _off);
    mcp.digitalWrite(waterPump, _on);
    statusHidro = fillpumpOn;
  }
  else
  {
    statusHidro = fillpumpOff;
    printLCD(0, "Tank is full - 1");
    mcp.digitalWrite(waterValve, _off);
    mcp.digitalWrite(waterPump, _off);
  }
}

//********************************
// Deactivate fill water tank pump
//********************************
void NotFillTankWithPump()
{
  statusHidro = fillpumpOff;
  FillTankWithStreetWater();
}

//*********************************
// Fill tank with water from street
//*********************************
void FillTankWithStreetWater()
{
  if (tankLevel != tankFull && statusHidro != fillpumpOn)
  {
    printLCD(1, "Fill from Street");
    mcp.digitalWrite(hidroValve, _off);
    mcp.digitalWrite(tankValve, _on);
    mcp.digitalWrite(waterValve, _off);
    mcp.digitalWrite(hidroPump, _off);
    mcp.digitalWrite(waterPump, _off);
  }
  else
  {
    if (tankLevel == tankFull)
      NotFillTankWithStreetWater();
  }
}

//*****************************************
// Deactivate all elements valves and pumps
//*****************************************
void NotFillTankWithStreetWater()
{
  printLCD(1, "Not Fill all off");
  mcp.digitalWrite(hidroValve, _off);
  mcp.digitalWrite(tankValve, _off);
  mcp.digitalWrite(waterValve, _off);
  mcp.digitalWrite(hidroPump, _off);
  mcp.digitalWrite(waterPump, _off);
  statusHidro = allOff;
  if (tankLevel == tankFull)
    printLCD(0, "Tank is full - 2");
}

//************************
// Check if tank is empty
//************************
bool isTankEmpty()
{
  bool _tankEmpty = mcp.digitalRead(emptySensor) == LOW;
  if (_tankEmpty)
  {
    tankLevel = tankEmpty;
    if (sendWarningTE)
    {
      hidroControl.publish(mqTopic, "TankEmpty");
      sendWarningTE = false;
    }
  }
  else
    sendWarningTE = true;
  return _tankEmpty;
}

//**********************
// Check if tank is full
//**********************
bool isTankFull()
{
  bool _tankFull = mcp.digitalRead(fullSensor) == LOW;
  if (_tankFull)
  {
    tankLevel = tankFull;
    if (sendWarningTF)
    {
      hidroControl.publish(mqTopic, "TankFull");
      sendWarningTF = false;
    }
  }
  else
    sendWarningTF = true;
  return _tankFull;
}

//***********************
// Check tank level water
//***********************
void CheckLevel()
{
  tankLevel = tankWithWater;
  if (isTankEmpty())
  { //Is Tank empty
    printLCD(0, "Tank is Empty   ");
    FillTankWithStreetWater(); //If tank is without water fill with street water
    tone(piezo, 100);          //Beep sound
  }
  else
  {
    noTone(piezo);
    if (isTankFull() && statusHidro != allOff && statusHidro != pumpOn) //Is Tank full
      NotFillTankWithStreetWater();
  }
}

//**************
// Turn Light On
//**************
void LightOn()
{
  mcp.digitalWrite(light, _on); //Turn light On
  pirIrq = false;               //Dont allow PIR activation
}

//***************
// Turn Light Off
//***************
void LightOff()
{
  mcp.digitalWrite(light, _off); //Turn light Off
  pirIrq = false;                //Dont allow PIR activation
}

//*********************************
// Detect movement with PIR sensor
//*********************************
void CheckPIR()
{
  pirState = digitalRead(pirsensor);
  pirIrq = true;
}

//********************************************************
// Activate or Deactivate light rele
//********************************************************
void TurnLight()
{
  if (pirIrq)
  {
#ifdef USE_CAYENNE
    notify = notifyCayenne;
#endif
    if (pirState)
    {
      LightOn();
      hidroControl.publish(mqTopic, "PIROn");
    }
    else
    {
      LightOff();
      hidroControl.publish(mqTopic, "PIROff");
    }
  }
}

//*********************************************
// Send two messages with all LCD display lines
//*********************************************
void LCDmsgs()
{
  hidroControl.publish(mqTopic, "LCDLine1:" + LCDLine1);
  hidroControl.publish(mqTopic, "LCDLine2:" + LCDLine2);
}

#ifdef USE_CAYENNE
//*****************************************
// Split and extract all chains from string
//*****************************************
int StringSplit(String sInput, char cDelim, String sParams[], int iMaxParams)
{
  int iParamCount = 0;
  int iPosDelim, iPosStart = 0;

  do
  {
    // Searching the delimiter using indexOf()
    iPosDelim = sInput.indexOf(cDelim, iPosStart);
    if (iPosDelim > (iPosStart + 1))
    {
      // Adding a new parameter using substring()
      sParams[iParamCount] = sInput.substring(iPosStart, iPosDelim - 1);
      iParamCount++;
      // Checking the number of parameters
      if (iParamCount >= iMaxParams)
      {
        return (iParamCount);
      }
      iPosStart = iPosDelim + 1;
    }
  } while (iPosDelim >= 0);
  if (iParamCount < iMaxParams)
  {
    // Adding the last parameter as the end of the line
    sParams[iParamCount] = sInput.substring(iPosStart);
    iParamCount++;
  }
  return (iParamCount);
}

//*****************************************************
// If msg come from Cayenne notify HidroControl server
//*****************************************************
void NotifyMosquitto(String channel, String value)
{
  notify = notifyMosquitto;
  if (channel == "1")
    hidroControl.publish(mqTopic, value == "1" ? "PumpOn" : "PumpOff");
  else if (channel == "2")
    hidroControl.publish(mqTopic, value == "1" ? "FillTankOn" : "FillTankOff");
  else if (channel == "3")
    hidroControl.publish(mqTopic, value == "1" ? "LoadWaterOn" : "LoadWaterOff");
  else if (channel == "5")
    hidroControl.publish(mqTopic, value == "1" ? "LightOn" : "LightOff");
  else if ((channel == "4") && (value == "1"))
      hidroControl.publish(mqTopic, "DeactivateAll");
}

//***********************************************************
// Callback function to interpret all cayenne cloud commands
//***********************************************************
void callbackCayenne(const MQTT::Publish &pub)
{
  msg = pub.payload_string();
  msg.trim();

#ifdef isDEBUG
  Serial.println(msg);
  Serial.println(pub.topic());
#endif

  if (pub.topic().indexOf("/cmd/") != -1)
  {
    String sParams[6];
    int iCount = StringSplit(pub.payload_string(), ',', sParams, 2);
    String value = sParams[1];
    String idmsg = sParams[0];

    iCount = StringSplit(pub.topic(), '/', sParams, 6);
    String channel = sParams[5];

    client.publish(updateCmd + channel, value); // Send update data
#ifdef isDEBUG
    Serial.println(updateCmd + channel);
    Serial.println(value);
#endif
    client.publish(responseCmd, "ok," + idmsg); // Send OK with the id to acnowledge
#ifdef isDEBUG
    Serial.println(responseCmd);
    Serial.println("ok," + idmsg);
#endif
    NotifyMosquitto(channel, value); // Message receive fron Cayenne update Mosquitto
  }
}

//*****************************************************
// If msg come from HidroControl server notify Cayenne
//*****************************************************
void NotifyCayenne()
{
  if (msg == "PumpOn")
    client.publish(updateCmd + "1", "1");
  else if (msg == "FillTankOn")
    client.publish(updateCmd + "2", "1");
  else if (msg == "LoadWaterOn")
    client.publish(updateCmd + "3", "1");
  else if (msg == "PumpOff")
    client.publish(updateCmd + "1", "0");
  else if (msg == "FillTankOff")
    client.publish(updateCmd + "2", "0");
  else if ((msg == "LoadWaterOff") || (msg == "DeactivateAll"))
    client.publish(updateCmd + "4", "1");
  else if (msg == "LightOn")
    client.publish(updateCmd + "5", "1");
  else if (msg == "LightOff")
    client.publish(updateCmd + "5", "0");
  else if (msg == "PIROn")
    client.publish(updateCmd + "6", "1");
  else if (msg == "PIROff")
    client.publish(updateCmd + "6", "0");
}
#endif

//*********************************************************************
// Callback function to interpret all accesories messages send to queue
//*********************************************************************
void callbackPump(const MQTT::Publish &pub)
{

  msg = pub.payload_string();
  msg.trim();
#ifdef isDEBUG
  Serial.println(msg);
  Serial.println(pub.topic());
#endif
  if (msg == "PumpOn")
    ActivateHidro();
  else if (msg == "FillTankOn")
    FillTankWithPump();
  else if (msg == "LoadWaterOn")
    FillTankWithStreetWater();
  else if (msg == "PumpOff")
    DeactivateHidro();
  else if (msg == "FillTankOff")
    NotFillTankWithPump();
  else if ((msg == "LoadWaterOff") || (msg == "DeactivateAll"))
    NotFillTankWithStreetWater();
  else if (msg == "LightOn")
    LightOn();
  else if ((msg == "LightOff"))
    LightOff();
  else if ((msg == "LCDmsgs"))
    LCDmsgs();

  if ((msg != "LCDmsgs") && (!msg.startsWith("LCDLine"))) {
    printLCD(0, "                ");
    printLCD(0, "MQ:" + msg);
  }

#ifdef USE_CAYENNE
  if (notify == notifyCayenne)
    NotifyCayenne();
  else
    notify = notifyCayenne;
#endif
}

void setup()
{

  lcd.begin(SDA, SCL);
  lcd.backlight(); //open the backlight

  mcp.begin(); //Initialize MCP23008

#ifdef isDEBUG
  Serial.begin(115200);
  Serial.println("Starting Control Pump Water Tank v1.0");
#endif
  printLCD(0, "Starting...v5.2");

  mcp.pinMode(hidroPump, OUTPUT);
  mcp.pinMode(waterPump, OUTPUT);
  mcp.pinMode(hidroValve, OUTPUT);
  mcp.pinMode(tankValve, OUTPUT);
  mcp.pinMode(waterValve, OUTPUT);
  mcp.pinMode(light, OUTPUT);
  mcp.pinMode(fullSensor, INPUT);
  mcp.pullUp(fullSensor, HIGH); // turn on a 100K pullup internally
  mcp.pinMode(emptySensor, INPUT);
  mcp.pullUp(emptySensor, HIGH); // turn on a 100K pullup internally
  pinMode(pirsensor, INPUT_PULLUP);
  pinMode(piezo, OUTPUT);

  mcp.digitalWrite(hidroValve, _off);
  mcp.digitalWrite(tankValve, _off);
  mcp.digitalWrite(waterValve, _off);
  mcp.digitalWrite(hidroPump, _off);
  mcp.digitalWrite(waterPump, _off);
  mcp.digitalWrite(light, _off);

  if (WiFi.status() != WL_CONNECTED)
  {
#ifdef isDEBUG
    Serial.print("Connecting to ");
    Serial.println(ssid);
#endif
    printLCD(1, "Connecting...   ");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
#ifdef isDEBUG
    Serial.println("WiFi connected");
#endif
    printLCD(1, "Wifi Connected  ");
  }

  // Attach interrupt function for PIR sensor
  attachInterrupt(digitalPinToInterrupt(pirsensor), CheckPIR, CHANGE);

  delay(2000);

  noInterrupts();            // Disable interrupts
  CheckLevel();              // Check water tank level
  FillTankWithStreetWater(); // Fill tank with street water
  interrupts();              // Enable interrupts
}

void loop()
{

  if (WiFi.status() == WL_CONNECTED)
  {
    noInterrupts();

    if (!hidroControl.connected()) // Suscribe PumpControl MQTT topic
    {
#ifdef isDEBUG
      Serial.println("Connecting to HidroControl server");
#endif
      if (hidroControl.connect(MQTT::Connect(clientID).set_auth("", "")))
      {
#ifdef isDEBUG
        Serial.println("Connected to HidroControl server");
#endif
        hidroControl.set_callback(callbackPump);
        hidroControl.subscribe("HidroControl");
      }
#ifdef isDEBUG
      else
        Serial.println("Could not connect to HidroControl server");
#endif
    }

    if (hidroControl.connected())
      hidroControl.loop();

#ifdef USE_CAYENNE
    if (!client.connected()) // Suscribe PumpControl MQTT topic
    {
#ifdef isDEBUG
      Serial.println("Connecting to Cayenne cloud");
#endif
      if (client.connect(MQTT::Connect(clientID).set_auth(username, password)))
      {
#ifdef isDEBUG
        Serial.println("Connected to Cayenne cloud");
#endif
        client.set_callback(callbackCayenne);
        client.subscribe(commandCmd + "1");
        client.subscribe(commandCmd + "2");
        client.subscribe(commandCmd + "4");
        client.subscribe(commandCmd + "5");
        client.publish(updateCmd + "3", "0");
        client.publish(updateCmd + "6", "0");
      }
#ifdef isDEBUG
      else
        Serial.println("Could not connect to Cayenne cloud");
#endif
    }

    if (client.connected())
      client.loop();
#endif

    CheckLevel(); // Check water tank level
    TurnLight();  // Turn Light On or Off with PIR status

    interrupts();
    delay(10);
  }
  else
  {
    printLCD(1, "Wifi disconn..");
    delay(10);
  }
}
