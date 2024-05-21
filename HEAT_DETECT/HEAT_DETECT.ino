// BLYNK DEFINE
#define BLYNK_TEMPLATE_ID "TMP******gJt"
#define BLYNK_TEMPLATE_NAME "heat detection"
#define BLYNK_AUTH_TOKEN "UPgS6gmX4i-*********-V4CdSp6fil"
#define BLYNK_PRINT Serial

// LIBRARIES USED
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// BLYNK START
BlynkTimer timer; 
char auth[] = BLYNK_AUTH_TOKEN; 
char ssid[] = "SSID";
char pass[] = "PASSWORD";


#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0 
#define lm35Pin 34 //LM345 PIN 
const int fanRelayPin = 4;        //RELAY PIN CONNECTED TO MACHINE 
const int fanPin = 15;            //RELAY PIN CONNECTED TO FAN FOR COOLING



LiquidCrystal_I2C lcd(0x27, 16, 2); // Address for the LCD display
float alertThreshold = 45.0;        // Set your desired threshold temperature
float normalThreshold = 35.0;       // Set the temperature at which the fan turns off

//BLYNK NOTIFICATION SETUP
int flag=0; 
void sendSensor(){ 
  int adcVal = analogRead(lm35Pin);
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  float temperature = milliVolt;
  Blynk.virtualWrite(V0, temperature);
   int analogValue = analogRead(lm35Pin);
  if (temperature > alertThreshold) {
    Blynk.logEvent("heat_detected","Heat detected");
    flag=1;
  }
  else
  {
    flag=0;   
  } 
}

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.print("Machine Temp:");
  pinMode(fanRelayPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, sendSensor);
  Serial.begin(115200);
}

void loop() {
  Blynk.run();
  timer.run();
  Serial.println("temp");
  int adcVal = analogRead(lm35Pin);
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  float temperature = milliVolt;
  // float temperature = analogRead(lm35Pin) * 0.48828125; // Convert analog reading to temperature in Celsius
  lcd.setCursor(0, 1);
  lcd.print(temperature);
  Serial.println(temperature);
  // RELAY TRIGGER  
  if (temperature > alertThreshold) {
    Serial.println("heat detected");
    // Turn on the fan
    digitalWrite(fanRelayPin, LOW);
    digitalWrite(fanPin, HIGH);
    delay(10000);
  } else if (temperature < normalThreshold) {
    Serial.println("normal");
    
    // Turn off the fan
    digitalWrite(fanRelayPin, HIGH);
    digitalWrite(fanPin, LOW);
    delay(500);
  } 
  delay(500); // Delay to avoid continuous alerts
}
