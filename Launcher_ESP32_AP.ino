/*
 * Firework Launcher 
 * Cortland Ratliff 
 * 2018
 * This code made for ESP32 Uno style board
 * Started with https://randomnerdtutorials.com/esp32-web-server-arduino-ide/
 * Great tutorial on how the code works!
 * 
 * 
 * Description:
 * This is supposed to be a simple 12 channel firework launcher with webpage interface
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
 * Open ignitor feedback
 * battery voltage monitor
 * 2-3S operation
 * 
 * 
 */


// Load Wi-Fi library
#include <WiFi.h>

// Replace with your network credentials
const char* ssid     = "Firework"; // Must be Set
const char* password = "Launcher"; //Must be Set

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output1State = "off";
String output2State = "off";

#define ADCRef 3.2
#define BatteryScale 3.197*ADCRef/4095  // voltage divider R1 10K R2 4.7K
float BatteryVoltage;

// Assign output variables to GPIO pins
const int output1 = 18;
const int output2 = 19;

// Assing Input IO
// Using analog input 32 as ADC2 pins 2 and 4 are used when WIFI mode is on
const int BatADC = 32;

void setup() {
  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(output1, OUTPUT);
  pinMode(output2, OUTPUT);
  pinMode(BatADC, INPUT);
  adcAttachPin(BatADC);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  // Set outputs to LOW
  digitalWrite(output1, LOW);
  digitalWrite(output2, LOW);

  // Start Wi-Fi network with SSID and password
  //Serial.print("Connecting to ");
  //Serial.println(ssid);
  WiFi.softAP(ssid, password);
  // Print local IP address and start web server
  Serial.println("");
  //Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    BatteryVoltage = analogRead(BatADC) * BatteryScale;
    Serial.print("Battery Voltage = ");
    Serial.println(BatteryVoltage);
    
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /1/on") >= 0) {
              Serial.println("Launch 1 on");
              output1State = "on";
              digitalWrite(output1, HIGH);
            } else if (header.indexOf("GET /1/off") >= 0) {
              Serial.println("Launch 1 off");
              output1State = "off";
              digitalWrite(output1, LOW);
            } else if (header.indexOf("GET /2/on") >= 0) {
              Serial.println("Launch 2 on");
              output2State = "on";
              digitalWrite(output2, HIGH);
            } else if (header.indexOf("GET /2/off") >= 0) {
              Serial.println("Launch 2 off");
              output2State = "off";
              digitalWrite(output2, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            //Auto Reload
            client.println("<script>");
            client.println("<!--");
            client.println("function timedRefresh(timeoutPeriod) {");
            client.println("  setTimeout(\"window.location.reload(true);\",timeoutPeriod);");
            client.println("}");
            client.println("window.onload = timedRefresh(5000);");
            client.println("</script>");
                
            // Web Page Heading
            client.println("<body><h1>The Best Firework Launcher Ever!!!</h1>");

            // Display Battery Voltage
            client.println("Battery Voltage: ");
            client.println(BatteryVoltage);
            
            // Display current state, and ON/OFF buttons for Launch 1  
            client.println("<p>Launch 1 - State " + output1State + "</p>");
            // If the output1State is off, it displays the ON button       
            if (output1State=="off") {
              client.println("<p><a href=\"/1/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/1/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for Launch 2  
            client.println("<p>Launch 2 - State " + output2State + "</p>");
            // If the output2State is off, it displays the ON button       
            if (output2State=="off") {
              client.println("<p><a href=\"/2/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/2/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}