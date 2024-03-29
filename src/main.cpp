#include <Arduino.h>
#include <elapsedMillis.h>
#include <TinyGPS++.h>
#include "TinyGsmClientA9G.h"
#include <BluetoothSerial.h>
#include <esp_bt_device.h>

#define VIBRATION_PIN 4

elapsedMillis publishElapsed;
elapsedMillis vibrationElapsed;
TinyGPSPlus gps;
TinyGsmA6 modem(Serial2);
BluetoothSerial bt;

String apn = "internet";
String topic = "Payload";
String hostIP = "203.172.40.152";
String port = "1883";
String cilentID = "55555";
String lat = "lost";
String lon = "lost";
uint8_t vibrate = 0;
int sumError = 0;
char deviceAddr[100];

void initGSM(){
  bool gprsState = false;
  bool gpsState = false;
  bool mqttState = false;

  while (!(gprsState && gpsState && mqttState))
  {    
    delay(3000);
    Serial.println("Initializing modem...");
    Serial.println(modem.restart()? "OK" : "FAIL");
    delay(1000);
    
    Serial.println(modem.getSimStatus()? "READY": "Fail");
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

void getDeviceAddress(char* deviceAddr){
  const uint8_t* point = esp_bt_dev_get_address();
  snprintf(deviceAddr, 100 ,"%02X:%02X:%02X:%02X:%02X:%02X", point[0],point[1],point[2],point[3],point[4],point[5]);
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  delay(1000);

  pinMode(VIBRATION_PIN, INPUT);

  bt.begin("ESP32");
  getDeviceAddress(deviceAddr);

  initGSM();
}

void loop() {

    while (Serial2.available() > 0)
    {
      char c = Serial2.read();
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

    if(publishElapsed > 5000)
    {    
      String payload =  String(deviceAddr) + "," + \
                        lat + "," + \
                        lon + "," + \
                        vibrate;
      if(modem.mqttPublish(topic.c_str(),payload.c_str()))
      {
        sumError = 0;
        Serial.println("Topic = " + topic + ":" + "Payload = " + payload);

      }
      else
      {
        sumError++;
        Serial.println("Publish lost");  
      }
      
      if(sumError == 3){
        Serial.println("MQTT Lost Connection ");
        lat = lon = "lost";
        initGSM();
        sumError = 0;
      }        
      publishElapsed = 0;      
    }

    if(vibrationElapsed > 200)
    {
      vibrate = digitalRead(VIBRATION_PIN);
      vibrationElapsed = 0;
    }    
}