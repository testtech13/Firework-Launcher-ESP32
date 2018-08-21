/*
 * Firework Launcher 
 * Cortland Ratliff 
 * 8 - 2018
 * This code made for ESP32 Uno style board or Uno 
 * Select Define before compiling
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
 * Define UNO VS ESP32
 * Low Battery
 * Error Codes
 * Armmed Indicator
 * 
 * 
 */

#include <CircularBuffer.h>

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
#define BatteryScale 3.197*ADCRef/4095  // voltage divider R1 10K R2 4.7K
const int BatADC = 32; //Analog Pin number
#endif

#ifdef UseUNO
#define ADCRef 3.47
#define BatteryScale 3.197*ADCRef/1023  // voltage divider R1 10K R2 4.7K
const int BatADC = 5; //Analog Pin number
#endif

String BatteryVoltage;
const int LowBatVolt = 8*100;  //Default is 7 volts scaled by 100 to make the math simple to use map function
const int HighBatVolt = 13*100; //Default is 13 volts scaled by 100 to make the math simple to use map function

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

//Other Variables
String ReadBuffer;
CircularBuffer<int, 12> queue;

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
uint8_t pinOutputArray[] = {0,18,23,5,12,14,16,17,26,39,34,35,4};
uint8_t VisualArmed = 33;
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
    pinOut = queue.pop();
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
   
  BatteryVoltage = String(analogRead(BatADC) * BatteryScale);
  time1second = 0;
  //Use the battery voltage to determine what the PWM Duty cycle to be
  //Keeps from burning out the igniter when you plug in a fresh battery
  LaunchPWMPercentInt = constrain(map(int(analogRead(BatADC) * BatteryScale*100), LowBatVolt, HighBatVolt, 255, 128),128,255);
  LowPower = LaunchPWMPercentInt/2;
  
  //SerialBT.println(LaunchPWMPercentInt);

 
  //Visual Indicator for if the device is armed
  if(Arm){
  digitalWrite(VisualArmed, !digitalRead(VisualArmed));
  }
  else {
    digitalWrite(VisualArmed, false);
  }
  
  }
  else{
    time1second++;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup() {
  SerialBT.begin("ESP32test");
  SerialBT.setTimeout(100);
  //timer setup
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &GetBatteryVoltage, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);

  pinMode(VisualArmed,OUTPUT);

  for (int i = 1; i < sizeof(pinOutputArray); i++){
    ledcAttachPin(pinOutputArray[i],i);
    ledcSetup(i, PWMFreq, PWMPrecision);
  }
  
  for (int i = 1; i < sizeof(pinInputArray); i++){
    pinMode(pinInputArray[i],INPUT_PULLUP);
  }
} 

//Function to Launch the Channel commanded
void Launch(){
  //SerialBT.print("Launching ");  //Debug
  pinRef = atoi(&ReadBuffer[2]);
  queue.push(pinRef);

}

//Check what Channel is doing
void ChannelStatus(){
  CSRef = atoi(&ReadBuffer[2]);
  if (!digitalRead(pinInputArray[CSRef])){
    SerialBT.println("2");
  }
  else if(CSRef==pinRef){
    SerialBT.println("1");
  }
  else{
    SerialBT.println("0");
  }
}

void loop() {
  //Check to see if we got any new commands
  if(SerialBT.available()){
    //if we got a new command make sure we have the whole command by looking for the newline
    ReadBuffer = SerialBT.readStringUntil('\n');
    //Heart Beat
    if (ReadBuffer == "HB"){
        SerialBT.println("HB");
        //Reset Timer
    }
    //Read Battery Voltage
    else if (ReadBuffer == "BV"){
      SerialBT.print("BV");
      SerialBT.println(BatteryVoltage);
    }
    //Arm Launcher
    else if (ReadBuffer == "EA"){
      SerialBT.println("EA");
      Arm = true;
    }
    //Disarm Launcher
    else if (ReadBuffer == "DA"){
      SerialBT.println("DA");
      Arm = false;
    }
    //Arm Status - Good for showing if the unit is armed on the GUI
    else if (ReadBuffer == "AS"){
      SerialBT.print("AS");
      SerialBT.println(Arm);
    }
    //Command to launch a channel
    else if (ReadBuffer.substring(0,2) == "LC"){
      SerialBT.println(ReadBuffer);
      Launch();
    }
    //Status of the channel
    else if (ReadBuffer.substring(0,2) == "CS"){
      SerialBT.print(ReadBuffer.substring(0,2));
      ChannelStatus();
    }
    //Error trapping if command did not match
    else{
      SerialBT.println("ER");
      
    }
  }
}
