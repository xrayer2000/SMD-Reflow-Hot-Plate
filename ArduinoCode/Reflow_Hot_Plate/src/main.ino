#include <Arduino.h>
#include <Wire.h> //temperatur
#include <Adafruit_GFX.h> //oled
#include <Adafruit_SSD1306.h> //oled
#include <U8g2lib.h> //oled
#include <Adafruit_I2CDevice.h>
#include <PID_v1.h> //PID
#include <PressButton.h> //Interface
#include <RotaryEncoder.h>; //Interface
#include <EEPROM.h> //Save Settings
#include <WiFi.h> //Home assistant
#include <PubSubClient.h> //Home assistant
#include "Privates.h" //Homeassistant 
#include <math.h>

Privates privates; //Homeassistant
WiFiClient espClient; //Home assistant
PubSubClient client(espClient); //Home assistant
char messages[50]; //Home assistant
volatile  bool TopicArrived = false;
const     int mqttpayloadSize = 10;
char mqttpayload [mqttpayloadSize] = {'\0'};
String mqtttopic;
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long mqttReconnectInterval = 5000; // 5 seconds

#define SCREEN_WIDTH 128 // OLED display1 width, in pixels
#define SCREEN_HEIGHT 64 // OLED display1 height, in pixels

#define dutyCycleOutPin 26 //pin 26 ESP32 - Grön LED
#define confirmBtnPin 25 // pin 25 ESP32 - ok
#define ledOnPin 14 //pin 14 ESP32 - röd LED
#define VoutPin 35

#define DISP_ITEM_ROWS 4
#define DISP_CHAR_WIDTH 16
#define PACING_MC 30 //25
#define FLASH_RST_CNT 3 //30
#define SETTINGS_CHKVAL 3647 
#define CHAR_X SCREEN_WIDTH/DISP_CHAR_WIDTH // 240/16
#define CHAR_Y SCREEN_HEIGHT/DISP_ITEM_ROWS // 240/8
#define SHIFT_UP 0 //240/8

//rotary encoder
#define outputA 13 //pin 13 ESP32 - 
#define outputB 16 //pin 16 ESP32 - 
RotaryEncoder encoder(outputA,outputB,5,1,60000);

//Buttons
PressButton btnOk(confirmBtnPin);

//Menu structure
enum pageType{
  MENU_ROOT,
  MENU_TARGET_TEMP,
  MENU_TEMPERATURES,
  MENU_MISC,
  MENU_PID,
  MENU_TIME
};

enum pageType currPage = MENU_ROOT;
void page_MenuRoot();
void page_MENU_TARGET_TEMP();
void page_TEMPERATURES();
void page_MENU_MISC();
void page_MENU_PID();
void page_MENU_TIME();

//Menu internals
boolean updateAllItems;
boolean updateItemValue;
uint8_t itemCnt;
uint8_t pntrPos;
uint8_t dispOffset;
uint8_t root_pntrPos = 1;
uint8_t root_dispOffset = 0;
uint8_t flashCntr;
boolean flashIsOn;
void initMenuPage(String title, uint8_t itemCount);
void captureButtonDownStates();
void incrementDecrementFloat(float *v, float amount, float min, float max);
void incrementDecrementDouble(double *v, double amount, double min, double max);
void doPointerNavigation();
bool isFlashChanged();
void pacingWait();
bool menuItemPrintable(uint8_t xPos, uint8_t yPos);

//Print tools
void printPointer();
void printOnOff(bool val);
void printUint32_tAtWidth(uint32_t value, uint8_t width, char c);
void printDoubleAtWidth(double value, uint8_t width, char c);

//Settings
#pragma pack(1) //memory alignment
struct Mysettings{
  double Kp_element = 125.0;
  double Ki_element = 0.0;
  double targetTemp = 25;

  boolean manualMode = 1;
  float temp1 = 65.0;
  float time1 = 45.0;
  float temp2 = 70.0;
  float time2 = 30.0;
  float temp3 = 77.0;
  float time3 = 15.0;

  boolean power = true;
  float RawLow = 0.31;
  float RawHigh = 99.56;
  float maxElementTemp = 250.0; //126
  float marginalTemp = 0.0;

  uint16_t settingsCheckValue = SETTINGS_CHKVAL;
};

Mysettings settings;
void sets_SetDefaults();
void sets_Load();
void sets_Save();

float RawRange;
float ReferenceHigh = 100.0;
float ReferenceLow = 0.0;
float ReferenceRange = ReferenceHigh - ReferenceLow;

unsigned long previousTime = 0; 
unsigned long currentTime;
unsigned long loopTime;

//PID
double Output_elementTemp;
double current_elementTemp;
double previous_elementTemp;
double DutyCycle = 0;
double previousDutyCycle = 0;
PID PID_elementTemp(&current_elementTemp, &Output_elementTemp, &settings.targetTemp, settings.Kp_element, settings.Ki_element, 0, DIRECT);

double tS;
long passedTimeS;
long previousPassedTimeS;

// setting PWM properties
int freq = 1000; //5
const int ledChannel = 0;
const int resolution = 12;

//rotary encoder
int counter = 1; 
int aState = 0;
int bState = 0;
int aLastState = 0;  
 
// Declaration for an SSD1306 display1 connected to I2C (SDA, SCL pins)
//Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C display2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
long timeLastPressed = 0;
const long timeBeforeDisable = 10000;

const double Vref = 3.27; //power supply voltage (3.3 V rail) -STM32 ADC pin is NOT 5 V tolerant
double Vout; //Voltage divider output
double R_NTC; //NTC thermistor resistance in Ohms
const double R_ref = 98300.0; //10k resistor measured resistance in Ohms (other element in the voltage divider)
const double B_param = 3950.0; //B-coefficient of the thermistor
const double T_0 = 298.15; //25°C in Kelvin
double Temp_C; //Temperature measured by the thermistor (Celsius)

double filteredValue = 0.0f;

void setup() {//=================================================SETUP=======================================================
  Serial.begin(115200);

  setupWiFi(); //Home assistant
  client.setServer(privates.broker, 1883); //Home assistant
  client.setCallback(callback);

  currentTime = millis();
  loopTime = currentTime;

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(dutyCycleOutPin, ledChannel);

  pinMode(ledOnPin, OUTPUT);
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);

  Wire.setClock(3400000 );      //3.4 MHz
  display1.setBusClock(3400000 );  //3.4 MHz
  display2.setBusClock(3400000 ); //3.4 MHz

  display1.setI2CAddress(0x78); 
  display1.begin();  
  display1.setFont(u8g2_font_6x10_mf);	// choose a suitable font
  display1.clearBuffer();					// clear the internal memory
  display1.setCursor(0, 0);

  display2.setI2CAddress(0x7A);
  display2.begin();  
  display2.setFont(u8g2_font_6x10_mf);	// choose a suitable font  
  display2.clearBuffer();					// clear the internal memory
  display2.setCursor(0, 0);

  EEPROM.begin(sizeof(settings));
  sets_Load();

  PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, 0);
  //PID_elementTemp.SetOutputLimits(0,settings.maxElementTemp - settings.marginalTemp);
  PID_elementTemp.SetMode(AUTOMATIC);
  PID_elementTemp.Compute();

}

void loop() { //=================================================LOOP=======================================================

  currentTime = millis();
  
  RawRange = settings.RawHigh - settings.RawLow;
  switch (currPage)
  {
  case MENU_ROOT: page_MenuRoot(); break;
  case MENU_TARGET_TEMP: page_MENU_TARGET_TEMP(); break;
  case MENU_TEMPERATURES: page_TEMPERATURES(); break;
  case MENU_MISC: page_MENU_MISC(); break;
  case MENU_PID: page_MENU_PID(); break;
  case MENU_TIME: page_MENU_TIME(); break;
  }
}

void page_MenuRoot(){//=================================================ROOT_MENU============================================
  pntrPos = root_pntrPos;
  dispOffset = root_dispOffset;
  //Serial.println("root");
  initMenuPage(F("MAIN MENU"), 5);
  
  double passedTime,previousPassedTime = 0;

  while(true){
    
    passedTime = millis() / (1000.0);

   if(passedTime - previousPassedTime >= 0.8)
      updateSettings();

    if(updateAllItems)
    {
      previousPassedTime = passedTime;
      passedTime = 0;
      updateSensorValues(); 
      updateDisp2();

      display1.clearBuffer(); 
      if(menuItemPrintable(1,1)){display1.print(F("TARGET TEMP  "));}
      if(menuItemPrintable(1,2)){display1.print(F("LIVE VARIBLES"));} 
      if(menuItemPrintable(1,3)){display1.print(F("MISC         "));} 
      if(menuItemPrintable(1,4)){display1.print(F("PID          "));} 
      if(menuItemPrintable(1,5)){display1.print(F("MashProgram  "));} 
      printPointer();
      display1.sendBuffer();
    }

    updateAllItems = false;
    captureButtonDownStates();

    if(btnOk.PressReleased())
    {
      FlashPointer();
      root_pntrPos = pntrPos;
      root_dispOffset = dispOffset;
      switch (pntrPos)
      {
      case 1: currPage = MENU_TARGET_TEMP; return;
      case 2: currPage = MENU_TEMPERATURES; return;
      case 3: currPage = MENU_MISC; return;
      case 4: currPage = MENU_PID; return;
      case 5: currPage = MENU_TIME; return;
      }
    }
    doPointerNavigation();
    if(passedTime - previousTime > 1.0)
    {
      publishMessage();
      subscribeMessage();
      previousTime = millis() / 1000.0;
    }
    yield(); // Add this line at the end of the loop
  }
  }
void page_MENU_TARGET_TEMP(){//=================================================TARGET_TEMP============================================
  pntrPos = 1;
  dispOffset = 0;
  initMenuPage(F("TARGET_TEMP"), 2);
  
  double passedTime,previousPassedTime = 0;
  bool changeValue = false;
  while(true){
    passedTime = millis() / (1000.0);

    if(passedTime - previousPassedTime >= 0.8)
      updateSettings();

    if(changeValue)
      incrementDecrementDouble(&settings.targetTemp, 1.0, 15.0, 250.0);
    else 
     doPointerNavigation(); 

    if(updateAllItems)
    {
      previousPassedTime = passedTime;
      passedTime = 0;
      updateSensorValues();
      updateDisp2();
      display1.clearBuffer();
      if(menuItemPrintable(1,1)){display1.print(F("Target Temp =     "));}
      if(menuItemPrintable(1,2)){display1.print(F("Back              "));}
      printPointer();
    }

    if(updateAllItems || updateItemValue)
    {
      if(menuItemPrintable(12,1)){printUint32_tAtWidth(settings.targetTemp, 3, 'C');}
      display1.sendBuffer();
    }
    updateAllItems = false;
    updateItemValue = false;

    captureButtonDownStates();

    if(btnOk.PressReleased())
    {
      FlashPointer();
      switch (pntrPos)
      {
        case 1: changeValue = !changeValue; break;
        case 2: currPage = MENU_ROOT; sets_Save(); return;
      }
    }
    if(passedTime - previousTime > 1.0)
    {
      publishMessage();
      subscribeMessage();
      previousTime = millis() / 1000.0;
    }
    yield(); // Add this line at the end of the loop
  }
}

void page_TEMPERATURES(){//=================================================LIVE_VARIABLES============================================
  pntrPos = 1;
  dispOffset = 0;
  initMenuPage(F("LIVE_VARIABLES"), 4);
  double passedTime,previousPassedTime = 0;

  while(true){
    passedTime = millis() / (1000.0);

    if(passedTime - previousPassedTime >= 0.8)
      updateSettings();

    if(updateAllItems)
    {
      previousPassedTime = passedTime;
      passedTime = 0;
      updateSensorValues();
      updateDisp2();
      display1.clearBuffer();
      //display1.setTextColor(WHITE, BLACK);
      if(menuItemPrintable(1,1)){display1.print(F("TargetTemp =     "));}
      if(menuItemPrintable(1,2)){display1.print(F("ElementTemp =     "));}
      if(menuItemPrintable(1,3)){display1.print(F("DC       =      "));}
      if(menuItemPrintable(1,4)){display1.print(F("Back            "));}

      if(menuItemPrintable(11,1)){printDoubleAtWidth(settings.targetTemp, 4, 'C');}
      if(menuItemPrintable(11,2)){printDoubleAtWidth(current_elementTemp, 4, 'C');}
      if(menuItemPrintable(11,3)){printDoubleAtWidth(DutyCycle/4096.0 * 100.0, 4, '%');}
      printPointer();
      display1.sendBuffer();
    }
    updateAllItems = false;
    updateItemValue = false;
    captureButtonDownStates();
    if(btnOk.PressReleased())
    {
      FlashPointer();
      switch (pntrPos)
      {
        case 4: currPage = MENU_ROOT; sets_Save(); return;
      }
    }
    doPointerNavigation();
    if(passedTime - previousTime > 1.0)
    {
      publishMessage();
      subscribeMessage();
      previousTime = millis() / 1000.0;
    }
    yield(); // Add this line at the end of the loop
  }
}
void page_MENU_MISC(){//=================================================MISC==========================================================
  pntrPos = 1;
  dispOffset = 0;
  initMenuPage(F("MISC"), 6);
  double passedTime,previousPassedTime = 0;
  bool changeValues [10];

  while(true){
    
    passedTime = millis() / (1000.0);

    if(passedTime - previousPassedTime >= 0.8)
      updateSettings();
     
    if(updateAllItems)
    {
      previousPassedTime = passedTime;
      passedTime = 0;
      updateSensorValues();
      updateDisp2();
      display1.clearBuffer();
      if(menuItemPrintable(1,1)){display1.print(F("POWER        =     "));}
      if(menuItemPrintable(1,2)){display1.print(F("RawLow       =     "));}
      if(menuItemPrintable(1,3)){display1.print(F("RawHigh      =     "));}
      if(menuItemPrintable(1,4)){display1.print(F("max_Ele_temp =     "));}
      if(menuItemPrintable(1,5)){display1.print(F("marginalTemp =     "));}
      if(menuItemPrintable(1,6)){display1.print(F("Back               "));}
    }
    if(updateAllItems || updateItemValue)
    {
      if(menuItemPrintable(12,1)){printOnOff(settings.power);}
      if(menuItemPrintable(12,2)){printDoubleAtWidth(settings.RawLow, 3, ' ');}
      if(menuItemPrintable(12,3)){printDoubleAtWidth(settings.RawHigh, 3, ' ');}
      if(menuItemPrintable(12,4)){printUint32_tAtWidth(settings.maxElementTemp, 4, 'C');}
      if(menuItemPrintable(12,5)){printUint32_tAtWidth(settings.marginalTemp, 4, 'C');}
      printPointer();
      display1.sendBuffer();
    }

    updateAllItems = false;
    updateItemValue = false;
    captureButtonDownStates();
    if(btnOk.PressReleased())
    {
      FlashPointer();
      switch (pntrPos)
      {
        case 1: changeValues [0] = !changeValues [0]; break; 
        case 2: changeValues [1] = !changeValues [1]; break; 
        case 3: changeValues [2] = !changeValues [2]; break;
        case 4: changeValues [3] = !changeValues [3]; break; 
        case 5: changeValues [4] = !changeValues [4]; break;
        case 6: currPage = MENU_ROOT; sets_Save(); return;
      }
    }
    double amountConstant = 0.01;
    if(changeValues[0])
    {
      *&settings.power = !*&settings.power;
      changeValues [0] = false;
      updateItemValue = true; 
    }
    else if(changeValues[1])incrementDecrementFloat(&settings.RawLow, 0.1f, 0.0f, 5.0f);
    else if(changeValues[2])incrementDecrementFloat(&settings.RawHigh, 0.1f, 95.0f, 105.0f);
    else if(changeValues[3])incrementDecrementFloat(&settings.maxElementTemp, 1.0f, 15.0f, 250.0f);
    else if(changeValues[4])incrementDecrementFloat(&settings.marginalTemp, 1.0f, 0.0f, 20.0f);
    else 
      doPointerNavigation(); 

    // PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, 0);
    // //PID_elementTemp.SetOutputLimits(0,settings.maxElementTemp - settings.marginalTemp);
    // PID_elementTemp.SetMode(AUTOMATIC);
    // PID_elementTemp.Compute();
    if(passedTime - previousTime > 1.0)
    {
      publishMessage();
      subscribeMessage();
      previousTime = millis() / 1000.0;
    }
    yield(); // Add this line at the end of the loop
  }
}
void page_MENU_PID(){//=================================================PID===============================================================
  pntrPos = 1;
  dispOffset = 0;
  initMenuPage(F("PID"), 3);
  double passedTime,previousPassedTime = 0;
  bool changeValues [10]; 

  while(true){
  passedTime = millis() / (1000.0);

   if(passedTime - previousPassedTime >= 0.8)
      updateSettings();

    if(updateAllItems)
    {
      previousPassedTime = passedTime;
      passedTime = 0;
      updateSensorValues();
      updateDisp2();
      display1.clearBuffer();
      if(menuItemPrintable(1,1)){display1.print(F("Kp_ele   =           "));}
      if(menuItemPrintable(1,2)){display1.print(F("Ki_ele   =           "));}
      if(menuItemPrintable(1,3)){display1.print(F("Back                 "));}
    }

    if(updateAllItems || updateItemValue)
    {
      if(menuItemPrintable(13,1)){printDoubleAtWidth(settings.Kp_element, 4, ' ');}
      if(menuItemPrintable(13,2)){printDoubleAtWidth(settings.Ki_element, 4, ' ');}
      //display1.display();
      printPointer();
      display1.sendBuffer();
    }

    updateAllItems = false;
    updateItemValue = false;

    captureButtonDownStates();

    if(btnOk.PressReleased())
    {
      FlashPointer();
      switch (pntrPos)
      {
        case 1: changeValues [0] = !changeValues [0]; break;
        case 2: changeValues [1] = !changeValues [1]; break;
        case 3: currPage = MENU_ROOT; sets_Save(); return;
      }
    }
         if(changeValues[0])incrementDecrementDouble(&settings.Kp_element, 1.0, 0.0, 1000.0);
    else if(changeValues[1])incrementDecrementDouble(&settings.Ki_element, 0.1, 0.0, 200.0);
    else 
      doPointerNavigation(); 

    // PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, 0);
    // //PID_elementTemp.SetOutputLimits(0,settings.maxElementTemp - settings.marginalTemp);
    // PID_elementTemp.SetMode(AUTOMATIC);
    // PID_elementTemp.Compute();
    
    if(passedTime - previousTime > 1.0)
    {
      publishMessage();
      subscribeMessage();
      previousTime = millis() / 1000.0;
    } //Home assistant
    yield(); // Add this line at the end of the loop
  }
}

void page_MENU_TIME(){//=================================================REFLOW_PROGRAM====================================================
  pntrPos = 1;
  dispOffset = 0;
  initMenuPage(F("Reflow Program"), 9);
  double passedTime,previousPassedTime = 0;
  bool changeValues [10];

  while(true){
  passedTime = millis() / (1000.0);

  if(passedTime - previousPassedTime >= 0.8)
      updateSettings();

  if(updateAllItems)
  {
    previousPassedTime = passedTime;
    passedTime = 0;
    updateSensorValues();
    updateDisp2();
    display1.clearBuffer();
    if(menuItemPrintable(1,1)){display1.print(F("Manual =            "));}
    if(menuItemPrintable(1,2)){display1.print(F("Element:            "));}
    if(menuItemPrintable(1,3)){display1.print(F("temp_1 =            "));}
    if(menuItemPrintable(1,4)){display1.print(F("time_1 =            "));}
    if(menuItemPrintable(1,5)){display1.print(F("temp_2 =            "));}
    if(menuItemPrintable(1,6)){display1.print(F("time_2 =            "));}
    if(menuItemPrintable(1,7)){display1.print(F("temp_3 =            "));}
    if(menuItemPrintable(1,8)){display1.print(F("time_3 =            "));}
    if(menuItemPrintable(1,9)){display1.print(F("Back                "));}
  }

  if(updateAllItems || updateItemValue)
  {
    if(menuItemPrintable(11,1)){printOnOff(settings.manualMode);}
    if(menuItemPrintable(7,2)){printDoubleAtWidth(passedTimeS/60.0, 3, 'm');}
    if(menuItemPrintable(13,2)){printDoubleAtWidth(settings.targetTemp, 3, 'C');}
    if(menuItemPrintable(11,3)){printDoubleAtWidth(settings.temp1, 3, 'C');}
    if(menuItemPrintable(11,4)){printDoubleAtWidth(settings.time1, 3, 'm');}
    if(menuItemPrintable(11,5)){printDoubleAtWidth(settings.temp2, 3, 'C');}
    if(menuItemPrintable(11,6)){printDoubleAtWidth(settings.time2, 3, 'm');}
    if(menuItemPrintable(11,7)){printDoubleAtWidth(settings.temp3, 3, 'C');}
    if(menuItemPrintable(11,8)){printDoubleAtWidth(settings.time3, 3, 'm');}
    printPointer();
    display1.sendBuffer();
  }

  updateAllItems = false;
  updateItemValue = false;

  captureButtonDownStates();

  if(btnOk.PressReleased())
  {
    FlashPointer();
    switch (pntrPos)
    {
      case 1: changeValues [0] = !changeValues [0]; break;
      case 3: changeValues [1] = !changeValues [1]; break;
      case 4: changeValues [2] = !changeValues [2]; break;
      case 5: changeValues [3] = !changeValues [3]; break;
      case 6: changeValues [4] = !changeValues [4]; break;
      case 7: changeValues [5] = !changeValues [5]; break;
      case 8: changeValues [6] = !changeValues [6]; break;
      case 9: currPage = MENU_ROOT; sets_Save(); return; 
    }
  }

  if(changeValues[0])
  {
    *&settings.manualMode = !*&settings.manualMode;
    changeValues [0] = false;
    updateItemValue = true; 
  } 
  else if(changeValues[1])incrementDecrementFloat(&settings.temp1, 1.0f, 15.0f, 100.0f);
  else if(changeValues[2])incrementDecrementFloat(&settings.time1, 1.0f, 0.0f, 90.0f);
  else if(changeValues[3])incrementDecrementFloat(&settings.temp2, 1.0f, 15.0f, 100.0f);
  else if(changeValues[4])incrementDecrementFloat(&settings.time2, 1.0f, 0.0f, 90.0f);
  else if(changeValues[5])incrementDecrementFloat(&settings.temp3, 1.0f, 15.0f, 100.0f);
  else if(changeValues[6])incrementDecrementFloat(&settings.time3, 1.0f, 0.0f, 90.0f);
  else 
    doPointerNavigation();
  if(passedTime - previousTime > 1.0)
    {
      publishMessage();
      subscribeMessage();
      previousTime = millis() / 1000.0;
    }
    yield(); // Add this line at the end of the loop
  }
}

//======================================================TOOLS - menu Internals==================================================
void initMenuPage(String title, uint8_t itemCount){
  display1.clearBuffer();
  printPointer();
  uint8_t fillCnt = (DISP_CHAR_WIDTH - title.length()) / 2;

  btnOk.ClearWasDown();
 
  itemCnt = itemCount;
  flashCntr = 0;
  flashIsOn = false;
  updateAllItems = true;
}
void captureButtonDownStates(){
  btnOk.CaptureDownState();
}

void doPointerNavigation(){
  currentTime = millis();
  if (currentTime >= (loopTime + 1) ) {
    aState = digitalRead(outputA); 
   bState = digitalRead(outputB);
  if (aState > aLastState)
  {
    if (bState != aState){  
        if(pntrPos > 1)
        {
          flashIsOn = false; flashCntr = 0; 
          if(pntrPos - dispOffset == 1){updateAllItems = true; dispOffset--;}
          pntrPos--;
          printPointer();
        }
        counter ++;
      } 
    else{
        if(pntrPos < itemCnt)
        {
          flashIsOn = false; flashCntr = 0; 
          if(pntrPos - dispOffset == DISP_ITEM_ROWS){updateAllItems = true; dispOffset++;}
          pntrPos++;
          printPointer();
        }
        counter --;
    }
   }
   aLastState = aState; 
   loopTime = currentTime;
  }
}

void incrementDecrementFloat(float *v, float amount, float min, float max)
{
  int enc = encoder.readEncoder();
  if(enc != 0) {
    *v += (enc*amount);
    *v = constrain(*v,min,max);
    //Serial.println(enc*amount);
    updateItemValue = true;
  } 
  delayMicroseconds(5);
}
void incrementDecrementDouble(double *v, double amount, double min, double max)
{
  int enc = encoder.readEncoder();
  if(enc != 0) {
    *v += (enc*amount);
    *v = constrain(*v,min,max);
    //Serial.println(enc*amount);
    updateItemValue = true;
  } 
  delayMicroseconds(5);
}

bool isFlashChanged(){
  if(flashCntr == 0){
    flashIsOn = !flashIsOn;

    flashCntr = FLASH_RST_CNT;

    return true;
  }
  else{flashCntr--; return false;}
}

bool menuItemPrintable(uint8_t xPos, uint8_t yPos){
  if(!(updateAllItems || (updateItemValue && pntrPos == yPos))){return false;}
  uint8_t yMaxOffset = 0;
  if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
  if(dispOffset <= (yPos) && dispOffset >= yMaxOffset){display1.setCursor(CHAR_X*xPos, CHAR_Y*(yPos - dispOffset)); return true;}
  return false;
}

bool menuItemPrintableDisp2(uint8_t xPos, uint8_t yPos){ 
  if(!(updateAllItems || (updateItemValue && pntrPos == yPos))){return false;}
  uint8_t yMaxOffset = 0;
  if(yPos > DISP_ITEM_ROWS) {yMaxOffset = yPos - DISP_ITEM_ROWS;}
  if(0 <= (yPos) && 0 >= yMaxOffset){display2.setCursor(CHAR_X*xPos, CHAR_Y*(yPos)); return true;}
  return false;
}

//======================================================TOOLS_display========================================================
void printPointer(){
  //Serial.println("printPointer");
  display1.drawStr(0, 1*CHAR_Y, " ");
  display1.drawStr(0, 2*CHAR_Y, " ");
  display1.drawStr(0, 3*CHAR_Y, " ");
  display1.drawStr(0, 4*CHAR_Y, " ");
  display1.drawStr(0, (pntrPos - dispOffset)*CHAR_Y, "*");
  display1.sendBuffer();
}
void FlashPointer(){
  display1.drawStr(0, 1*CHAR_Y, " ");
  display1.drawStr(0, 2*CHAR_Y, " ");
  display1.drawStr(0, 3*CHAR_Y, " ");
  display1.drawStr(0, 4*CHAR_Y, " ");
  display1.sendBuffer();

  delay(50);
  //Serial.println("FlashPointer");
  display1.drawStr(0, (pntrPos - dispOffset)*CHAR_Y, "*");
  display1.sendBuffer();
}

void printOnOff(bool val){
  if(val){display1.print(F("ON    "));}
  else   {display1.print(F("OFF   "));}
}
void printChars(uint8_t cnt, char c){
  if(cnt > 0){
    char cc[] = " "; cc[0] = c;
    for(u_int8_t i = 1; i < cnt; i++){display1.print(cc);}
  }
}
uint8_t getUint32_tCharCnt(uint32_t value)
{
  if(value == 0){return 1;}
  uint32_t tensCalc = 10; int8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
uint8_t getDoubleCharCnt(double value)
{
  if(value == 0){return 1;}
  uint32_t tensCalc = 10; int8_t cnt = 1;
  while (tensCalc <= value && cnt < 20){tensCalc *= 10; cnt += 1;}
  return cnt;
}
void printUint32_tAtWidth(uint32_t value, uint8_t width, char c){
  display1.print(value);
  display1.print(c);
  printChars(width-getUint32_tCharCnt(value), ' ');
}
void printDoubleAtWidth(double value, uint8_t width, char c){
  char buf[10];
  dtostrf(value, width-getDoubleCharCnt(value), 1, buf); // 1 decimal
  display1.print(buf);
  display1.print(c);
}
//======================================================DISPLAY_2======================================================
void printCharsDisplay2(uint8_t cnt, char c){
  if(cnt > 0){
    char cc[] = " "; cc[0] = c;
    for(u_int8_t i = 1; i < cnt; i++){display2.print(cc);}
  }
}
void printUint32_tAtWidthDisplay2(uint32_t value, uint8_t width, char c){
  display2.print(value);
  display2.print(c);
  printCharsDisplay2(width-getUint32_tCharCnt(value), ' ');
}
void printDoubleAtWidthDisplay2(double value, uint8_t width, char c){
  char buf[10];
  dtostrf(value, width-getDoubleCharCnt(value), 1, buf); // 1 decimal
  display2.print(buf);
  display2.print(c);
}

//======================================================TOOLS_settings======================================================
void sets_SetDeafault()
{
  Mysettings tempSets;
  memcpy(&settings, &tempSets, sizeof settings);
}

void sets_Load()
{
  EEPROM.get(0,settings);
  if(settings.settingsCheckValue != SETTINGS_CHKVAL){sets_SetDeafault();}
}
void sets_Save()
{
  EEPROM.put(0, settings);
  EEPROM.commit();
}
void updateSettings()
{
  if(millis() - timeLastPressed > timeBeforeDisable)
  {
    display1.ssd1306_command(SSD1306_DISPLAYOFF);
    display2.ssd1306_command(SSD1306_DISPLAYOFF);
  }
  else
  {
    display1.ssd1306_command(SSD1306_DISPLAYON);
    display2.ssd1306_command(SSD1306_DISPLAYON);
  }

  if(settings.power)
  {
    digitalWrite(ledOnPin, HIGH);
    if (settings.manualMode == false)
    {
      previousPassedTimeS = passedTimeS;
      passedTimeS = (millis() - tS) / (1000);

      if (0 <= passedTimeS && passedTimeS < settings.time1 * 60) //60
        settings.targetTemp = settings.temp1;
      else if (settings.time1 * 60 <= passedTimeS && passedTimeS < (settings.time1 + settings.time2) * 60) //60
        settings.targetTemp = settings.temp2;     
      else if ((settings.time1 + settings.time2) * 60 <= passedTimeS && passedTimeS < (settings.time1 + settings.time2 + settings.time3) * 60) //60     
        settings.targetTemp = settings.temp3;     
      else
        settings.targetTemp = 15;
      
    }
    else
    {
      tS = millis();
      passedTimeS = 0;
    }

    PID_elementTemp.SetTunings(settings.Kp_element, settings.Ki_element, 0);
    PID_elementTemp.SetMode(AUTOMATIC);
    updateAllItems = PID_elementTemp.Compute();

    previousDutyCycle = DutyCycle;
    DutyCycle = Output_elementTemp;  
    ledcWrite(ledChannel, DutyCycle); 

    client.loop();
    if(TopicArrived)
    {
      // Serial.print("Message arrived: ");
      // Serial.println(mqttpayload);
      //settings.targetTemp = atof(mqttpayload);
      TopicArrived = false;
    }
  }
  else
  {
    digitalWrite(ledOnPin, LOW);
    ledcWrite(ledChannel, 0);
  }
  
}
void updateSensorValues()                        // långsamast i hela programet
{

  double alpha = 0.2f;
  double rawReading = analogRead(VoutPin);
  filteredValue = alpha * rawReading + (1 - alpha) * filteredValue;

  Vout = (filteredValue / 4095.0) * Vref;
  R_NTC = (Vout * R_ref) / (Vref - Vout);
  double logTerm = log(R_NTC / R_ref);
  Temp_C = ((T_0 * B_param) / (B_param + T_0 * logTerm)) - 273.15f;

  previous_elementTemp = current_elementTemp;
  current_elementTemp = Temp_C;

  // Serial.print(", Temp_C: ");
  // Serial.print(Temp_C, 1);
  // Serial.print(", filteredValue: ");
  // Serial.println(filteredValue);
}
void updateDisp2()
{
  display2.clearBuffer();
  if(menuItemPrintableDisp2(1,1)){display2.print(F("TargetTemp =      "));}
  if(menuItemPrintableDisp2(1,2)){display2.print(F("ElementTemp =     "));}
  if(menuItemPrintableDisp2(1,3)){display2.print(F("DC         =      "));}

  if(menuItemPrintableDisp2(11,1)){printDoubleAtWidthDisplay2(settings.targetTemp, 3, 'C');}
  if(menuItemPrintableDisp2(11,2)){printDoubleAtWidthDisplay2(current_elementTemp, 3, 'C');}
  if(menuItemPrintableDisp2(11,3)){printDoubleAtWidthDisplay2(DutyCycle/4095.0 * 100.0, 3, '%');}
  display2.sendBuffer();
}

void setupWiFi() //Homeassistant
{
  WiFi.begin(privates.ssid, privates.pass);
}

void reconnect() //Homeassistant
{
  // Only try to reconnect if enough time has passed
  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt > mqttReconnectInterval) {
    lastMqttReconnectAttempt = now;
    if (!client.connected()) {
      client.connect("boll", privates.brokerUser, privates.brokerPass);
    }
  }
}

void publishMessage() //Homeassistant
{
    if(!client.connected()){reconnect();}
    client.loop();
    snprintf(messages, 5, "%f", current_elementTemp);
    client.publish(privates.topicElementTemp, messages);
    snprintf(messages, 5, "%f", passedTimeS/60.0);
    client.publish(privates.topicTimePassed, messages);
    snprintf(messages, 5, "%f", settings.targetTemp);
    client.publish(privates.topicTargetTemp, messages);
    snprintf(messages, 5, "%f", DutyCycle/4096.0 * 100.0);
    client.publish(privates.topicDutyCycle, messages);
    snprintf(messages, 5, "%f", settings.Kp_element);
    client.publish(privates.topicKp, messages);
    snprintf(messages, 5, "%f", settings.Ki_element);
    client.publish(privates.topicKi, messages);
}

void subscribeMessage() //Homeassistant
{
    if(!client.connected()){reconnect();}
    client.loop();
    client.subscribe(privates.topicTargetTemp); 
}

void callback(char* topic, byte* payload, unsigned int length) //Homeassistant
{
  if ( !TopicArrived )
  {
    memset( mqttpayload, '\0', mqttpayloadSize ); // clear payload char buffer
    mqtttopic = ""; //clear topic string buffer
    mqtttopic = topic; //store new topic
    memcpy( mqttpayload, payload, length );
    TopicArrived = true;
  }
}