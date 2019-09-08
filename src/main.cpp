#define TINY_GSM_MODEM_A6

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <elapsedMillis.h>
#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include "TinyGsmClientA9G.h"

#define VIBRATION_PIN D1
#define AT_RX D2
#define AT_TX D3

SoftwareSerial AT(AT_RX, AT_TX);
elapsedMillis publishElapsed;
elapsedMillis gprsElapsed;
elapsedMillis vibrationElapsed;
TinyGPSPlus gps;
TinyGsmA6 modem(AT);
String apn = "www.dtac.co.th";
String topic = "Payload";
String hostIP = "203.172.40.152";
String port = "1883";
String cilentID = "55555";
String lat = "Null";
String lon = "Null";
String macAddress = "";
uint8_t vibrate = 0;
String satPayload = "";

void initGSM(){
  bool gprsState = false;
  bool gpsState = false;
  bool mqttState = false;
  bool simState = false;

  while (!(simState && gprsState && gpsState && mqttState))
  {    
    delay(3000);
    Serial.println("Initializing modem...");
    Serial.println(modem.initTest()? "OK" : "FAIL");
    delay(1000);
    
    simState = modem.getSimStatus();
    Serial.println(simState? "READY": "Fail");
    delay(1000);    

    Serial.println("Connecting to " + apn);
    gprsState = modem.gprsConnect(apn.c_str());  
    Serial.println(gprsState? "OK": "Fail");    
    delay(1000);
    
    Serial.println("Connecting to MQTT");
    mqttState = modem.mqttConnect(hostIP.c_str(),port.c_str(),cilentID.c_str());
    Serial.println(mqttState? "OK": "Fail");    
    delay(1000);

    Serial.println("Connecting to GPS");  
    gpsState = modem.gpsConnect();
    Serial.println(gpsState? "OK": "Fail");
    delay(1000);
  }
}

void setup() {
  Serial.begin(9600);
  AT.begin(115200);
  delay(1000);

  macAddress = WiFi.macAddress();
  pinMode(VIBRATION_PIN, INPUT);

  initGSM();
}

void loop() {
    while (AT.available() > 0)
    {
      char c = AT.read();
      // Serial.print(c);
      if (gps.encode(c))
      {
          if(gps.location.isValid())
          {
            lat = String(gps.location.lat(), 6);
            lon = String(gps.location.lng(), 6);
          }
          else
          {
            lat = lon = "Invalid data";
          }          
      }
    }
    if(publishElapsed > 2000)
    {    
      String payload =  macAddress + "," + \
                        lat + "," + \
                        lon + "," + \
                        vibrate;
      modem.mqttPublish(topic.c_str(),payload.c_str());      
      Serial.println("Topic = " + topic + ":" + "Payload = " + payload);
      publishElapsed = 0;
      
    }
    if(vibrationElapsed > 200)
    {
      vibrate = digitalRead(VIBRATION_PIN);
      vibrationElapsed = 0;
    }

    if(gprsElapsed > 30000L)
    {    
      delay(500);
      if(!modem.isGprsConnected()){
      initGSM();
      }
      gprsElapsed = 0;
    }

    
}