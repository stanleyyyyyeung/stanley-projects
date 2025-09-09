#include <HCSR04.h>
#include <Arduino.h>
#include <Wire.h>
//#include <ArtronShop_BH1750.h>
#include <BH1750.h>
//ArtronShop_BH1750 bh1750(0x23, &Wire); // Non Jump ADDR: 0x23, Jump ADDR: 0x5C
 
//ultrasonic sensor 1 configuration (each pins are referred directly, i.e. 23 = IO23)
byte triggerPin1 = 23;
byte echoPin1 = 22;
byte motorPin1 = 27;
//ultrasonic sensor 2
byte triggerPin2 = 21;
byte echoPin2 = 19;
byte motorPin2 = 26;
//ultrasonic sensor 3
byte triggerPin3 = 23;
byte echoPin3 = 22;
byte motorPin3 = 25;
 
//light sensor
BH1750 lightMeter;
byte sda = 14;
byte scl = 12;
 
 
byte LEDpin = 33;
 
 
 
 
// Setting up the sensors (initialising the whole system) 
void setup () {
  Serial.begin(9600);
//initialise ultrasonic sensors (do this first as the distance sensing subsystem is always given the highest priority)
  HCSR04.begin(triggerPin1, echoPin1); // First sensor
  HCSR04.begin(triggerPin2, echoPin2); // Second sensor
  HCSR04.begin(triggerPin3, echoPin3); // Third sensor
 
//echo (configured as input)
  pinMode(echoPin1, INPUT); // Used to calculate distance from obstacles by calculating the difference in time between the trigger and echo signals
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
//trigger
  pinMode(triggerPin1, OUTPUT); // Indicates the start of a transmission
  pinMode(triggerPin2, OUTPUT);
  pinMode(triggerPin3, OUTPUT);
//motor
  pinMode(motorPin1, OUTPUT); // GPIO of the MCU connected to the motors for haptic feedback generation
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
 
//LED
  pinMode(LEDpin, OUTPUT);
 
 
 
//light sensor
  pinMode(sda, INPUT);
  pinMode(scl, OUTPUT);
 
//initialise light sensor
  Wire.begin(sda, scl);
  lightMeter.begin();
  if (lightMeter.begin()) {
    Serial.println("BH1750 initialised");
  }
  Serial.println(F("BH1750 Test begin"));
 
 
 
}
 
// Function for measuring the distance of obstacles in each direction
void loop () {
 
  //light sensor
  light_sensor(LEDpin);
 
  //obstacle distance measurement
  int pingTravelTime;
  float pingTravelDistance;
  float distanceToTarget;
  
  // Calls the measure_distance function to calculate the distance from obstacles for each sensor
  measure_distance(triggerPin1, echoPin1, motorPin1);
  measure_distance(triggerPin2, echoPin2, motorPin2);
  measure_distance(triggerPin3, echoPin3, motorPin3);
 
}
 
// Function for turning on the LEDs in low lighting conditions 
void light_sensor(int LEDPin){
  // Reads the brightness and transmits to MCU using I2C protocol
  // returns -2.0 if device is unconfigured, by default = -1.0
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  if (lux < 100) {
    digitalWrite(LEDPin, HIGH);
    delay(1000);
    digitalWrite(LEDPin, LOW);
  }
  delay(50);
 
}

// Function for calculating the distance of obstacles
void measure_distance(int triggerPin, int echoPin, int motorPin){
  int pingTravelTime;
  float pingTravelDistance;
  float distanceToTarget;
 
  //send trigger
  digitalWrite(triggerPin, LOW); // Initialise the trigger signal to 0
  delayMicroseconds(10);  // Delay for 10 microseconds
  digitalWrite(triggerPin, HIGH); // Apply a trigger to start sensing distancxe
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
 
  //read echo 
  // Arduino function: Starts timing when rising edge is detected from the echo pin, stops when falling edge is detected
  pingTravelTime = pulseIn(echoPin, HIGH); 
  delay(50);
 
  //calculate distance
 distanceToTarget = pingTravelTime/59.;
 
 
  Serial.print("Distance to Target is: ");
  Serial.print(distanceToTarget);
  Serial.println(" cm.");
 
  if (distanceToTarget >= 0 && distanceToTarget < 50) {
    digitalWrite(motorPin, HIGH);
    delay(2000);
    digitalWrite(motorPin, LOW);
    Serial.println("too close");
  } else if (distanceToTarget >= 50 && distanceToTarget < 100) {
    digitalWrite(motorPin, HIGH);
    delay(1000);
    digitalWrite(motorPin, LOW);
    Serial.println("close");
  } else if (distanceToTarget >= 100 && distanceToTarget < 200) {
    digitalWrite(motorPin, HIGH);
    delay(500);
    digitalWrite(motorPin, LOW);
    Serial.print("relatively close");
  } else {
    Serial.println("far");
  }
 
 
}
 
//Web #include <WiFiClient.h> #include <Web... by Shengrong Zhang (teammate)
// This section of code is used for generating the website to alert family members of the users in case of emergencies
Shengrong Zhang
6:27 pm
//Web
#include <WiFiClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <ESPmDNS.h>
//GPS
#include <TinyGPS++.h>
#include <Arduino.h>
#include <Wire.h>
#define GPS_BAUDRATE 9600  // The default baudrate of NEO-6M is 9600
#define RXD2 3   //Rx pin for the GPS module
#define TXD2 1   // Tx pin for the GPS module
HardwareSerial neogps(1);
TinyGPSPlus       gps;
 
//GPS
float latitute = 0;
float longitude = 0;
 
//emergency button
byte EmergencyPin = 18;
 
//web
/*
const char *ssid = "fortest";
const char *password = "123456789";
*/
const char *ssid = "JU";
const char *password = "ELEC31051417";
 
 
 
 
 
//web
WebServer server(80); // Create an object named WebServer with 80 as port number (default port number to connect user to insecure network)
String header; // Variable to store the HTTP request
String PIN = "off"; // Variables to store the current LED states
const int pin = 12; // Output variable to GPIO pins
 
// Sets the appearence of the website
void handleRoot() {
    char msg[1500];
    snprintf(msg, 1500,
        "<html>\
        <head>\
            <meta http-equiv='refresh' content='4'/>\
            <meta name='viewporpt' content='width=device-width, initial-scale=1'>\
            <title>Travel Buddy</title>\
            <style>\
                html { font-family: Arial; text-align: left;}\
                .container { margin-left: 50px; }\
                h2 { font-size: 6rem; }\
                p { font-size: 3rem; }\
                .dht-labels { font-size: 1.5rem; vertical-align:middle; padding-bottom: 15px; }\
                button { background-color: yellowgreen; border: none; color: white; padding: 20px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer; }\
            </style>\
        </head>\
        <body>\
            <div class='container'>\
                <h2>Guidance Tool Terminal</h2>\
                <p><span class='dht-labels'>Latitute:&nbsp;&nbsp;</span><span>%.6f</span></p>\
                <p><span class='dht-labels'>Lingitude:&nbsp;&nbsp;</span><span>%.6f</span></p>\
                <p><span class='dht-labels'>Condition:&nbsp;&nbsp;</span><span>%s</span></p>\
            </div>\
        </body>\
        </html>", latitute, longitude, Condition()
    );
    server.send(200, "text/html", msg);
}
 
void setup() {
    Serial.begin(115200);
 
    pinMode(EmergencyPin, INPUT);
 
    WiFi.mode(WIFI_STA);  // station mode: the ESP32 will connect to the access point
    WiFi.begin(ssid, password);
    Serial.println("");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    if (MDNS.begin("esp32")) {
        Serial.println("MDNS responder started");
    }
    server.on("/", handleRoot);
   
 
    neogps.begin(GPS_BAUDRATE, SERIAL_8N1, RXD2, TXD2);  // Initialise the GPS module
 
 
    pinMode(12, OUTPUT); 
    digitalWrite(12, LOW); 
    server.on("/toggle12", handleTogglePin12); 
    server.begin();
    Serial.println("HTTP server started");
}
 
void loop() {
  server.handleClient();
  delay(20);
 
  GPS_function();
 
  delay(20); //allow the cpu to switch to other tasks
}
 
 
void GPS_function(){
  // GPS message
  while (neogps.available()) {
    char c = neogps.read();
    if (gps.encode(c)) {  // Process GPS data when a complete sentence is parsed
      if (gps.location.isValid()) {  // Check if location data is valid
        latitute = gps.location.lat();
        longitude = gps.location.lng();
        Serial.print(F("Latitude: "));
        Serial.println(latitute, 6);  // Print latitude with 6 decimal places for precision
        Serial.print(F("Longitude: "));
        Serial.println(longitude, 6);  // Print longitude with 6 decimal places for precision
      } else {
        latitute = 0; // Set latitude to 0 by default
        longitude = 0;
        Serial.println(F("Location: INVALID"));
      }
    } else {
      Serial.println("bad"); // Bad connection
    }
    //upload to web
  }
 
  delay(20);
}
 
 
 
char* Condition() {
    char* h = "safe";  // By default the user is safe
    if (digitalRead(EmergencyPin) == HIGH) {
        h = "warning";
    }
    return h;
}
 
void handleTogglePin12() {
    static bool pinState = false;
    pinState = !pinState;
    digitalWrite(pin, pinState ? HIGH : LOW);
    server.sendHeader("Location", "/");
    server.send(303);
}
has context menu


has context menu