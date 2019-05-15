/*
 * Firework Launcher 
 * Cortland Ratliff 
 * 8 - 2018
 * This code made for ESP32 Uno style board or Uno 
 * Select Define before compiling
 * Select Wemos Lion32 for board configuration
 * Thanks to the Arduino Community for all the great libraries
 * 
 * 
 * 
 * Description:
 * This is a 12 channel firework launcher with SerialBT Interface
 * Notes on the SerialBT protocol can be found at the Github Link below
 * 
 * Code Link: 
 * https://github.com/testtech13/Firework-Launcher-ESP32
 * 
 * Circuit Board design:
 * https://workspace.circuitmaker.com/Projects/Details/Cortland-Ratliff-3/Launcher
 * 
 * ESPDUINO Reference:
 * http://www.raspberrypiwiki.com/images/e/ee/ESPDUINO-32-Guide-CN.pdf
 * 
 * 
 * Features:
 * 12 channels
 * Over Current/Short Circuit protection
 * Open igniter feedback
 * battery voltage monitor
 * 2-3S operation
 * 
 * Sequentail firing
 * Heart Beat
 * PWM
 * 
 *** To Do List
 * Define UNO VS ESP32 - Uno maybe a lost cause as using some of the speical feature of the ESP32
 * Low Battery - Should announce occasionally that the battery is low and then sleep to save battery
 * Error Codes - open ignitor, low battery, over heat, lost connection, 
 * Open ignitor detect - 
 * Armmed Indicator - done
 * Battery voltage accuracy needs to be improved 
 * Sounds\tones for armmed and such
 * Heart Beat time out
 *  
 ***
 * 4/15/19 - Added Serial Support along with bluetooth so either can work -  They should be used one at a time I believe
 *            Should allow for simple connection to RPI.  RPI(Raspberry Pi) to be used as user interface.  
 *            
 * 4/18/19 - Added Battery Voltage Percentage           
 * 
 * 4/19/19 - Add Tones - Not working correctly
 * 4/23/19 - Add Struct for battery !!!!!!!!
 */

#include <CircularBuffer.h>
#include <EasyBuzzer.h>


//Select one
#define UseESP32
//#define UseUNO

//#define Debug

//Define IO lines

//Battery Voltage Measurement setup
#ifdef UseESP32
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#define ADCRef 3.47
#define BatteryScale 6*ADCRef/4095  // voltage divider R1 10K R2 2.0K
const int BatADC = 39; //Analog Pin number
#endif

#ifdef UseUNO
#define ADCRef 3.47
#define BatteryScale 3.197*ADCRef/1023  // voltage divider R1 10K R2 4.7K
const int BatADC = 5; //Analog Pin number
#endif



String BatteryVoltage = "No Value";
String BattPercent = "No Value";
const int LowBatVolt = 8*100;  //Default is 7 volts scaled by 100 to make the math simple to use map function
const int HighBatVolt = 13*100; //Default is 13 volts scaled by 100 to make the math simple to use map function

const int TwoSBatLow  = 650; //6.5v
const int TwoSBatHigh = 840;
const int ThreeSBatLow  = 975;
const int ThreeSBatHigh = 1260;
const int FourSBatLow  = 1300;
const int FourSBatHigh = 1680;

int LowVolt = 0;
int HighVolt = 0;

//PWM Settings
int LaunchOnTime = 120; //1200ms Default
int LaunchPWMTime = 150; //1500ms Default
int LaunchPWMPercentInt = 255;

const float HalfPower = 0.5; //50% Default
uint16_t LowPower = 0;

const int PWMFreq = 1000;
const int PWMPrecision = 8; //8 bit


int time1second, firetime = 0;
bool Arm = false;
bool Launching = false;
bool FlipFlop = false;

// Start by defining the relationship between 
//       note, period, &  frequency. 
#define  c     3830    // 261 Hz 
#define  d     3400    // 294 Hz 
#define  e     3038    // 329 Hz 
#define  f     2864    // 349 Hz 
#define  g     2550    // 392 Hz 
#define  a     2272    // 440 Hz 
#define  b     2028    // 493 Hz 
#define  C     1912    // 523 Hz 
// Define a special note, 'R', to represent a rest
#define  R     0

int speakerOut = 21;

//Other Variables
String ReadBuffer;
CircularBuffer<int, 20> queue;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//index of the Command input to input pin
#ifdef UseUNO
uint8_t pinInputArray[] = {0,12,12,9,9,6,6,3,3,A4,A4,A4,A4};
#endif
#ifdef UseESP32
uint8_t pinInputArray[] = {0,19,19,13,13,27,27,25,25,36,36,36,36};
#endif
//index of the command output to output pin
#ifdef UseUno
uint8_t pinOutputArray[] = {0,13,11,10,8,7,5,4,2,A5,A3,A2,A1};

#endif
#ifdef UseESP32
uint8_t pinOutputArray[] = {0,18,23,5,12,14,16,17,26,02,34,35,4};
uint8_t ArmedLED = 22;
#endif

int CSRef = 0;
int pinOut = 0;
uint8_t pinRef = 0;
#define debug

//Interrupt ever 10ms
void IRAM_ATTR GetBatteryVoltage(){
  portENTER_CRITICAL_ISR(&timerMux);
  //SerialBTBT.println(analogRead(BatADC) * BatteryScale); //Debug
  //Two Stage igniter Power Level independed on Battery Voltage
  //Start igniter if armed and channel selected
  if(!queue.isEmpty()&& firetime ==0&&Arm){
    firetime++;
    pinOut = queue.shift();
    SerialBT.println(pinOut);
    Serial.println(pinOut);
    ledcWrite(pinOut, LaunchPWMPercentInt);
  }  
  //Clear Buffer if disarmed
  else if(!queue.isEmpty()&&!Arm){
    queue.clear();
    firetime =0;
  }
  //Turn Channel Off after timer expires
  else if(firetime >= LaunchPWMTime&&Arm){
    firetime = 0;
    ledcWrite(pinOut, 0);
    pinRef = 0;
  }
  //Set Hold PWM level after Launch On Time
  else if(firetime == LaunchOnTime&&Arm){
    ledcWrite(pinOut, LowPower);
    firetime++;
  }
  //Increment Timer
  else if (firetime > 0&&Arm){
    firetime++;
  }
  //If all else clear output pin
  else{
    ledcWrite(pinOut, 0);
    pinRef = 0;
  }

  
  //Timer for one second Items
  if(time1second >= 100){
  //Read Battery Voltage 
  BatteryVoltage = String(analogRead(BatADC) * BatteryScale);
  //Calculate Battery Percentage
  //Scale based on number of cells detected
  BattPercent = String(constrain(map(int(analogRead(BatADC) * BatteryScale*100), LowVolt, HighVolt, 0, 100),0,100));
  
  //Send Battery Percent
  SerialBT.print("BP ");
  SerialBT.println(BattPercent);
  Serial.print("BP ");
  Serial.println(BattPercent);

  //Shut down if battery voltage too low for number of cells.  
  //Send warning battery level low
  
  time1second = 0;
  //Use the battery voltage to determine what the PWM Duty cycle to be
  //Keeps from burning out the igniter when you plug in a fresh battery
  LaunchPWMPercentInt = constrain(map(int(analogRead(BatADC) * BatteryScale*100), LowBatVolt, HighBatVolt, 255, 128),128,255);
  LowPower = LaunchPWMPercentInt/2;
  
  //SerialBT.println(LaunchPWMPercentInt);

 
  //Visual Indicator for if the device is armed
  if(Arm){
  digitalWrite(ArmedLED, !digitalRead(ArmedLED));
  }
  else {
    digitalWrite(ArmedLED, false);
  }
  
  }
  else{
    time1second++;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

int ReadBatteryVoltage(){
  return int(analogRead(BatADC) * BatteryScale * 100);
}
void setup() {
  SerialBT.begin("ESP32test"); //Need to Update Nmae in Test App
  SerialBT.setTimeout(100);
  Serial.begin(115200);
  Serial.println("Starting Up");
  //timer setup
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &GetBatteryVoltage, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);

  pinMode(ArmedLED,OUTPUT);

  for (int i = 1; i < sizeof(pinOutputArray); i++){
    ledcAttachPin(pinOutputArray[i],i);
    ledcSetup(i, PWMFreq, PWMPrecision);
  }
  
  for (int i = 1; i < sizeof(pinInputArray); i++){
    pinMode(pinInputArray[i],INPUT_PULLUP);
  }
  adcAttachPin(BatADC);
  analogSetAttenuation(ADC_11db);

  //Figure out the number of cells in the battery
  if (ReadBatteryVoltage() >= FourSBatLow){
    LowVolt = FourSBatLow;
    HighVolt = FourSBatHigh;
  }
  else if (ReadBatteryVoltage() >= ThreeSBatLow){
    LowVolt = ThreeSBatLow;
    HighVolt = ThreeSBatHigh;
  }
  else{
    LowVolt = TwoSBatLow;
    HighVolt = TwoSBatHigh;
  }

  EasyBuzzer.setPin(speakerOut);
  EasyBuzzer.update();
  EasyBuzzer.beep(c);
  //delay(5000);
  //EasyBuzzer.stopBeep();
 
} 


//Function to Launch the Channel commanded
void Launch(){
  //SerialBT.print("Launching ");  //Debug
  pinRef = atoi(&ReadBuffer[2]);
  SerialBT.print("FC");
  Serial.print("FC");
  SerialBT.println(pinRef);
  Serial.println(pinRef);
  queue.push(pinRef);

}

//Check what Channel is doing
void ChannelStatus(){
  CSRef = atoi(&ReadBuffer[2]);
  if (!digitalRead(pinInputArray[CSRef])){
    SerialBT.println("2");
    Serial.println("2");
  }
  else if(CSRef==pinRef){
    SerialBT.println("1");
    Serial.println("1");
  }
  else{
    SerialBT.println("0");
    Serial.println("0");
  }
}

void loop() {
  //Check to see if we got any new commands
  if(SerialBT.available()|Serial.available()){
    //if we got a new command make sure we have the whole command by looking for the newline
    if (SerialBT.available()){ReadBuffer = SerialBT.readStringUntil('\n');}
    else {ReadBuffer = Serial.readStringUntil('\n');}
    //Heart Beat
    if (ReadBuffer == "HB"){
        SerialBT.println("HB");
        Serial.println("HB");
        //Reset Timer
    }
    //Read Battery Voltage
    else if (ReadBuffer == "BV"){
      SerialBT.print("BV");
      SerialBT.println(BatteryVoltage);
      Serial.print("BV");
      Serial.println(BatteryVoltage);
    }
    //Arm Launcher
    else if (ReadBuffer == "EA"){
      SerialBT.println("EA");
      Serial.println("EA");
      Arm = true;
    }
    //Disarm Launcher
    else if (ReadBuffer == "DA"){
      SerialBT.println("DA");
      Serial.println("DA");
      Arm = false;
    }
    //Arm Status - Good for showing if the unit is armed on the GUI
    else if (ReadBuffer == "AS"){
      SerialBT.print("AS");
      SerialBT.println(Arm);
      Serial.print("AS");
      Serial.println(Arm);
    }
    //Command to launch a channel
    else if (ReadBuffer.substring(0,2) == "LC"){
      //SerialBT.println(ReadBuffer);
      //Serial.println(ReadBuffer);
      Launch();
    }
    //Status of the channel
    else if (ReadBuffer.substring(0,2) == "CS"){
      SerialBT.print(ReadBuffer.substring(0,2));
      Serial.print(ReadBuffer.substring(0,2));
      ChannelStatus();
    }
    //Error trapping if command did not match
    else{
      SerialBT.println("ER");
      Serial.println("ER");
      
    }
  }
}
