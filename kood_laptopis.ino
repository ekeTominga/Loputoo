
/*
 * USB charger with current control according to electricity price
 * Author: Eke Tominga  
 * Date: Started 15.04.2022
 * 
 */
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <Wire.h> //Library for I2C communication
#include <WiFi.h>
#include <HTTPClient.h>
#include <iostream>
#include <cstring>
#include "time.h"
//#include <Arduino_JSON.h>

using namespace std;

//variables for Wifi connection
const char* ssid = "TalTech";


//Defining Pins
#define ledPin 2 //debug LED

#define voltagePinUSB1 4 //USB port 1 voltage Measurement
#define voltagePinUSB2 5 //USB port 2 voltage Measurement
#define voltagePin5V 12 //5V Bus voltage Measurement
#define voltagePinMicroUSB 13 //Programming Micro USB port voltage Measurement
#define voltagePin3V3 14 //3V3 bus voltage Measurement

#define currentLimitSelection1 15 //send output LOW for digital potentiometer current limit set, and output HIGH so that external resistor current limit set (22k1 - 2A setting)
#define currentLimitSelection2 16 //send output LOW for digital potentiometer current limit set, and output HIGH so that external resistor current limit set (22k1 - 2A setting)

#define enableUSBportPin1 17 //enable USB port 1, output HIGH enables output
#define enableUSBportPin2 18 //enable USB port 2, output HIGH enables output

#define SDAPin 21  //I2C data 
#define SCLPin 22  //I2C clock

#define buttonInput1 23 //currently not used, default input LOW, in case of button press it goes HIGH
#define buttonInput2 25 //currently not used, default input LOW, in case of button press it goes HIGH

#define currentMeasurePinUSB1 26 //measures output current of USB Port 1 through current sense. Current sense 6 mOhm and amplifier has a gain of 200
#define currentMeasurePinUSB2 27 //measures output current of USB Port 1 through current sense. Current sense 6 mOhm and amplifier has a gain of 200
#define currentMeasurePin3V3 32 //measures output current of USB Port 1 through current sense. Current sense 6 mOhm and amplifier has a gain of 200
#define currentMeasureNotUsedYet 33 //measures output current of USB Port 1 through current sense. Current sense 6 mOhm and amplifier has a gain of 200

#define potentiometerPin1 34 //currently not used
#define potentiometerPin2 35 //currently not used

float currentSense = 0.006;
int gainCurrentAmplifier = 200;
int ADCmaxVoltage = 3600; //maximum measurable voltage is 2450 mV in ESP32 WROOM
float ADCresolution = 4095.00; //12 bit ADC
float voltageDividerVoltageMeasurement = 22.1/(22.1+33);

#define PotentiometerI2Caddress 45 
#define SDA 21
#define SCL 22

unsigned long last_time = 0;
unsigned long timer_delay = 20000;
String json_array;

int pricesNordPool[23];

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200; //GMT+2 normal time
const int   daylightOffset_sec = 3600; //Summer time +1 more
int currentHour; 
float priceAverage;

void setup() {  
pinInitalize(); //initalizes pins as either input or output

//ADC setup
//ADC CALIBTRATION TO BE DONE! - https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/adc.html
analogSetAttenuation(ADC_11db); // sets ADC to maximum 2450 mV reading
digitalWrite (enableUSBportPin1, LOW); //Enables USB port 1 
digitalWrite (enableUSBportPin2, LOW); // Enables USB port 2 
digitalWrite (currentLimitSelection1, HIGH); // 2A current limit to USB port 1
digitalWrite (currentLimitSelection2, HIGH); // 2A current limit to USB port 1 

//I2C transmission, try to make it 100kohm)
Wire.setClock(100000); //Sets I2C clock speed to 100 khz
//if (!Wire.isEnabled()){; //Checks if I2C is already enabled and only start I2C when it is not 
Wire.begin(SDA, SCL); //Begins I2C in MASTER mode, pins of SDA and SCL (21 and 22)
Wire.beginTransmission(PotentiometerI2Caddress);  //Starts transmit process to aadress 45 (potentiometer)
Wire.write(41); //(DDDDCCDD) - CC = 00 write data, 0000 - volatile wiper 0, 0001 volatile wiper 1; DD is bit 9 and 8 of digital potentiometer wiper setting  01000001 11111111
Wire.write(255); //maybe 100kohm - needs to be tested
Wire.endTransmission();
//Transmission ends

Serial.begin(115200); //serial monitor UP
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, NULL);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Yhendamine WiFI-ga...");
  }
  Serial.println("Yhendatud!");


Serial.println("");  Serial.print("WiFi connected to: ");Serial.println(ssid);  Serial.println("IP address: ");  Serial.println(WiFi.localIP());
//  client.setCACert(root_ca);
  delay(2000);


  
    if(WiFi.status()== WL_CONNECTED){
      String server = "https://dashboard.elering.ee/api/nps/price/csv?start=2022-05-05T00%3A00%3A00.000Z&end=2022-05-05T23%3A00%3A00.000Z&fields=ee";
      
      json_array = GET_Request(server.c_str());
 //     Serial.println("CSV fail:");

//Serial.println(json_array[45]);

int csv_length = json_array.length();
char csv_char_array[csv_length +1];
strcpy(csv_char_array, json_array.c_str());
for (int i = 0; i < csv_length; i++)
  cout << csv_char_array[i];

//Serial.print("char array on:");
//Serial.println(csv_char_array);
char * token = strtok(csv_char_array, "\"");
int j = 0;
   while( token != NULL ) {
  //   printf( " %s\n", token ); //printing each token
     
      j++;
      switch(j){
          case 11:
          {
          char * hind00 = token;
        Serial.print("Hind00 on:");
       Serial.println(hind00);
          int temp0 = atoi(hind00);
         pricesNordPool[0] = temp0;
          break;
           }
        case 17:
          {
          char * hind01 = token;
            int temp1 = atoi(hind01);
           pricesNordPool[1] = temp1;
          break;
           }
        case 23:
          {
           char * hind02 = token;
           int temp2 = atoi(hind02);
          pricesNordPool[2] = temp2;
           break;
          }
             case 29:
          {
           char * hind03 = token;
            int temp3 = atoi(hind03);
          pricesNordPool[3] = temp3;
           break;
          }
             case 35:
          {
           char * hind04 = token;
            int temp4 = atoi(hind04);
           pricesNordPool[4] = temp4;
           break;
          }
             case 41:
          {
           char * hind05 = token;
            int temp5 = atoi(hind05);
          pricesNordPool[5] = temp5;
           break;
          }
           case 47:
          {
          char * hind06 = token;
            int temp6 = atoi(hind06);
          pricesNordPool[6] = temp6;
          break;
           }
        case 53:
          {
          char * hind07 = token;
           int temp7 = atoi(hind07);
          pricesNordPool[7] = temp7;
          break;
           }
        case 59:
          {
           char * hind08 = token;
            int temp8 = atoi(hind08);
         pricesNordPool[8] = temp8;
           break;
          }
             case 65:
          {
           char * hind09 = token;
           int temp9 = atoi(hind09);
          pricesNordPool[9] = temp9;
           break;
          }
             case 71:
          {
           char * hind10 = token;
            int temp10 = atoi(hind10);
          pricesNordPool[10] = temp10;
           break;
          }
             case 77:
          {
           char * hind11 = token;
            int temp11 = atoi(hind11);
           pricesNordPool[11] = temp11;
           break;
          }
           case 83:
          {
          char * hind12 = token;
            int temp12 = atoi(hind12);
          pricesNordPool[12] = temp12;
          break;
           }
        case 89:
          {
          char * hind13 = token;
            int temp13 = atoi(hind13);
          pricesNordPool[13] = temp13;
          break;
           }
        case 95:
          {
           char * hind14 = token;
            int temp14 = atoi(hind14);
           pricesNordPool[14] = temp14;
           break;
          }
             case 101:
          {
           char * hind15 = token;
           int temp15 = atoi(hind15);
          pricesNordPool[15] = temp15;
           break;
          }
             case 107:
          {
           char * hind16 = token;
            int temp16 = atoi(hind16);
           pricesNordPool[16] = temp16;
           break;
          }
             case 113:
          {
           char * hind17 = token;
           int temp17 = atoi(hind17);
          pricesNordPool[17] = temp17;
           break;
          }
           case 119:
          {
          char * hind18 = token;
            int temp18 = atoi(hind18);
           pricesNordPool[18] = temp18;
          break;
           }
        case 125:
          {
          char * hind19 = token;
           int temp19 = atoi(hind19);
          pricesNordPool[19] = temp19;
          break;
           }
        case 131:
          {
           char * hind20 = token;
            int temp20 = atoi(hind20);
           pricesNordPool[20] = temp20;
           break;
          }
             case 137:
          {
           char * hind21 = token;
           int temp21= atoi(hind21);
           pricesNordPool[21] = temp21;
           break;
          }
             case 143:
          {
           char * hind22 = token;
            int temp22 = atoi(hind22);
           pricesNordPool[22] = temp22;
           break;
          }
             case 149:
          {
           char * hind23 = token;
            int temp23 = atoi(hind23);
          pricesNordPool[23] = temp23;
           break;
          }
          }
          
      
      token = strtok(NULL, "\"");

   }
 /*     JSONVar my_obj = JSON.parse(json_array);
  
      if (JSON.typeof(my_obj) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
     Serial.println("JSON on object is:");
      Serial.print("JSON object = ");
      Serial.println(my_obj);
       Serial.print("Proov ");
      Serial.println(my_obj["data"]["ee"]);
       Serial.println(my_obj[1]);
        Serial.println(my_obj["data"][3]);
*/
    }
    else {
      Serial.println("WiFi Disconnected");
    }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();


}








  




void loop() {

Wire.beginTransmission(PotentiometerI2Caddress); //I2C General Call
Wire.write(byte(0b10010000)); // 9(b1001) Wiper 1 , 8(b1000) Decrement Wiper Register
Wire.write(byte(0b11101111));
Wire.endTransmission();
delay(1000);

Wire.beginTransmission(PotentiometerI2Caddress); //I2C General Call
Wire.write(byte(0b10000100)); // 8(b1000) Wiper 0 , 4(b0100) Increment Wiper + last bit 0 to form 8 bit
Wire.write(byte(0b11101111));
Wire.endTransmission();
delay(1000);

for (int i = 0; i < 24; i++){
  Serial.print("Hind algusega kell xx.00:");
  Serial.println(pricesNordPool[i]);
   delay (1);

  Serial.print("CURRENTHOUR ");
  Serial.println(currentHour);

priceAverage = getAverage();
 Serial.print("Average Price of the Day:");
 Serial.println(priceAverage);
  

}


 /* 
float voltageUSB1 = measurementVoltageUSB1();
Serial.print("USB1 Voltage:");
Serial.println(voltageUSB1);


float voltage5V = measurementVoltage5V();
Serial.print("5V Voltage:");
Serial.println(voltage5V);

float voltageMicroUSB = measurementVoltageMicroUSB();
Serial.print("Micro USB Voltage:");
Serial.println(voltageMicroUSB);



float currentUSB1 = measurementCurrentUSB1();
Serial.print("USB1 Current:");
Serial.println(currentUSB1);

float currentUSB2 = measurementCurrentUSB2();
Serial.print("USB2 Current:");
Serial.println(currentUSB2);

float current3V3 = measurementCurrent3V3();
Serial.print("3V3 Current:");
Serial.println(current3V3);




Wire.beginTransmission(PotentiometerI2Caddress);  //Starts transmit process to aadress 45 (potentiometer)
Wire.write(0); //(DDDDCCDD) - CC = 00 write data, 0000 - volatile wiper 0, 0001 volatile wiper 1; DD is bit 9 and 8 of digital potentiometer wiper setting 
Wire.write(255); //maybe 100kohm - needs to be tested
Wire.endTransmission();
digitalWrite (ledPin, HIGH);
delay(2000);


Wire.beginTransmission(PotentiometerI2Caddress);  //Starts transmit process to aadress 45 (potentiometer)
Wire.write(4); //(DDDDCCDD) - CC = 00 write data, 0000 - volatile wiper 0, 0001 volatile wiper 1; DD is bit 9 and 8 of digital potentiometer wiper setting 
Wire.write(255); //maybe 100kohm - needs to be tested
Wire.endTransmission();
digitalWrite (ledPin, LOW);
delay(2000);

Wire.beginTransmission(PotentiometerI2Caddress);  //Starts transmit process to aadress 45 (potentiometer)
Wire.write(0); //(DDDDCCDD) - CC = 00 write data, 0000 - volatile wiper 0, 0001 volatile wiper 1; DD is bit 9 and 8 of digital potentiometer wiper setting 
Wire.write(0); //maybe 100kohm - needs to be tested
Wire.endTransmission();
digitalWrite (ledPin, HIGH);
delay(2000);

Wire.beginTransmission(PotentiometerI2Caddress);  //Starts transmit process to aadress 45 (potentiometer)
Wire.write(4); //(DDDDCCDD) - CC = 00 write data, 0000 - volatile wiper 0, 0001 volatile wiper 1; DD is bit 9 and 8 of digital potentiometer wiper setting 
Wire.write(1); //maybe 100kohm - needs to be tested
Wire.endTransmission();
digitalWrite (ledPin, LOW);
delay(2000);

*/
}

float getAverage(){
  float sumAverage = 0;
  for (int i = 0; i < 24; i++){
  sumAverage += pricesNordPool[i];
  }
  float priceAverage = sumAverage / 24; 
  return priceAverage;
}

float getPowerPercentage(){
  int minimum = pricesNordPool[0];
  int maximum = pricesNordPool[0];
    for (int = 0; i < 23; i++){
      if (pricesNordPool[i] < minimum ){
        minimum = pricesNordPool[i]; 
      }
      if (maximum < pricesNordPool[i]){
        maximum = pricesNordPool[i]; 
      }
    }  
}



void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  currentHour = timeinfo.tm_hour;

}


String GET_Request(const char* server) {
  HTTPClient http;    
  http.begin(server);
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  http.end();
  return payload;
}


float measurementVoltageUSB1(){
  //voltage on USB1
  int voltageRAWUSB1 = analogRead(voltagePinUSB1);
  float voltageUSB1 = ((voltageRAWUSB1/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
return voltageUSB1; 
}


float measurementVoltage5V(){
  //voltage on 5V bus
  int voltageRAW5V = analogRead(voltagePin5V);
  float voltage5V = ((voltageRAW5V/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
return voltage5V; 
}

float measurementVoltageMicroUSB(){
  //voltage on Programming bus
  int voltageRAWMicroUSB = analogRead(voltagePinMicroUSB);
  float voltageMicroUSB = ((voltageRAWMicroUSB/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
return voltageMicroUSB; 
}







float measurementCurrentUSB1(){
  //current USB1  
  int currentRAWUSB1 = analogRead(currentMeasurePinUSB1);
  float currentUSB1 = ((currentRAWUSB1/ADCresolution)*ADCmaxVoltage/1000)/(currentSense*gainCurrentAmplifier);
  return currentUSB1;
}

float measurementCurrentUSB2(){
  //current USB2
  int currentRAWUSB2 = analogRead(currentMeasurePinUSB2); 
  float currentUSB2 = ((currentRAWUSB2/ADCresolution)*ADCmaxVoltage/1000)/(currentSense*gainCurrentAmplifier);
  return currentUSB2;
}

float measurementCurrent3V3(){
  //current 3V3
  int currentRAW3V3 = analogRead(currentMeasurePin3V3); 
  float current3V3 = ((currentRAW3V3/ADCresolution)*ADCmaxVoltage/1000)/(currentSense*gainCurrentAmplifier);
  return currentRAW3V3;
}
//Potentiometer MCP4652T-104E/UN
//8 bit (257 taps)
// 100 kohm
// 100 khz, 400 khz and 3.4 mhz support
// A0 = 1, A1 = 0
// datasheet  https://ww1.microchip.com/downloads/en/DeviceDoc/22096b.pdf
// ID 0101101
// 256 series resistances, 100kohm total. One series equals 100/256 = 0.4kohm => minimum step is 0.4 kohm when adjusting current limit 
// all wire.h functions available: https://docs.particle.io/cards/firmware/wire-i2c



// RTC DS1339C-33#T&R
// 100 khz or 400 khz
// datasheet https://datasheets.maximintegrated.com/en/ds/DS1339-DS1339U.pdf
// ID 1101000



void pinInitalize() {
  //OUTPUTS  
  pinMode (ledPin, OUTPUT);
  pinMode (currentLimitSelection1, OUTPUT);
  pinMode (currentLimitSelection2, OUTPUT);
  pinMode (enableUSBportPin1, OUTPUT);
  pinMode (enableUSBportPin2, OUTPUT);
  //INPUTS
  pinMode (voltagePinUSB1, INPUT);
  pinMode (voltagePinUSB2, INPUT);
  pinMode (voltagePin5V, INPUT);
  pinMode (voltagePinMicroUSB, INPUT);
  pinMode (voltagePin3V3, INPUT);
  pinMode (currentMeasurePinUSB1, INPUT);
  pinMode (currentMeasurePinUSB2, INPUT);
  pinMode (currentMeasurePin3V3, INPUT);
  pinMode (currentMeasureNotUsedYet, INPUT);
  pinMode (potentiometerPin1, INPUT);
  pinMode (potentiometerPin2, INPUT);
  pinMode (buttonInput1, INPUT);
  pinMode (buttonInput2, INPUT);
}
