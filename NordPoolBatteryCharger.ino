
/*
 * USB charger with current control according to electricity price
 * Author: Eke Tominga  
 * Date: Started 15.04.2022
 * Finished 13.05.2022
 */
 
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <Wire.h> // For I2C communication
#include <WiFi.h> // For WiFI connection
#include <HTTPClient.h> //For Get Request
#include <iostream>
#include <cstring>
#include "time.h" //For Time Request
//#include <Arduino_JSON.h> //Not used, is only needed when wanting to download data from Elering in JSON format

using namespace std;

//variables for Wifi connection
const char* ssid = "TalTech"; //TalTech WiFi - there is no password


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

#define potentiometerPin1 34 //currently not used, potentiometer connected to pin 34
#define potentiometerPin2 35 //currently not used, potentiometer connected to pin 35

float currentSense = 0.006; // current sense resistance is 0.006 Ohm = 6 mOhm
int gainCurrentAmplifier = 200; // gain of current sense amplifier 
int ADCmaxVoltage = 3600; // when using analogRead, it translates everything to 3600, but ESP32 still is able to only read to 2450 mV
float ADCresolution = 4095.00; //12 bit ADC, values 0...4095 
float voltageDividerVoltageMeasurement = 22.1/(22.1+33); //voltage divider resistors are 22.3 kohm and 33 kohm

#define PotentiometerI2Caddress 45 //I2C ID of rheostat
#define SDA 21 //IO pin for Serial Data of I2C
#define SCL 22 //IO pin for Serial Clock of I2C

String csv_file; //String where CSV file from Elering with NPS data is stored

int pricesNordPool[24]; //array of prices per hour - int is used because function is not able to read places after comma - maybe something that Elering could improve
float powerPercentage[24]; // power percentage per hour - maximum power (100%) usable is 1.5 A and minimum power (0%) is 0.5A
float priceAverage; // average power of the day - needed for calculations
byte resistance8bit[24]; // what resistance value is sent to rheostat and with what current limit is set per hour
float approxCurrent[24]; // approximates current - not very accurate as resistance and current limit don't have linear relation

const char* ntpServer = "pool.ntp.org"; //server for time
const long gmtOffset_sec = 7200; //GMT+2 normal time
const int daylightOffset_sec = 3600; //Summer time +1 more
int currentHour; //current time, if time is 15.23 it gives int = 15;
int currentMinute; //current time, if  time is 15.23 it gives int = 23;
int currentDay; //current day, if it is 22.12.1997 then it gives int = 22;
int currentMonth; //current day, if it is 22.12.1997 then it gives int = 12;
int currentYear; //current day, if it is 22.12.1997 then it gives int = 1997;


void setup() {  
pinInitalize(); //initalizes pins as either input or output

//ADC setup
//ADC CALIBTRATION TO BE DONE! - https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/adc.html
analogSetAttenuation(ADC_11db); // sets ADC to maximum 2450 mV reading
digitalWrite (enableUSBportPin1, HIGH); //Enables USB port 1 
digitalWrite (enableUSBportPin2, HIGH); // Enables USB port 2 
digitalWrite (currentLimitSelection1, HIGH); // 2A current limit to USB port 1
digitalWrite (currentLimitSelection2, HIGH); // 2A current limit to USB port 1 

//I2C TCON register configuration 
Wire.setClock(100000); //Sets I2C clock speed to 100 khz
//if (!Wire.isEnabled()){; //Checks if I2C is already enabled and only start I2C when it is not 
Wire.begin(SDA, SCL); //Begins I2C in MASTER mode, pins of SDA and SCL (21 and 22)
Wire.beginTransmission(PotentiometerI2Caddress);  //Starts transmit process to aadress 45 (potentiometer)
Wire.write(0b01000001); //(DDDDCCDD) - CC = 00 write data, 0100 TCON register; DD is bit 9 and 8 of data - 01000001
Wire.write(0b11111111); //Allow all editing in rheostat values
Wire.endTransmission(); //Transmission ends


Serial.begin(115200); //Jadaandmeside arvuti ja kontrolleri vahel
WiFi.mode(WIFI_STA); //Kasutusel WiFi station'na mitte Access pointina
WiFi.begin(ssid, NULL); //SSID on TalTech, parooli ei ole 
  while (WiFi.status() != WL_CONNECTED) { //Seniks kuni WiFi staatus EI OLE ühendatud
    delay(500); 
    Serial.println("Yhendamine WiFI-ga...");  //Kirjuta serial monitori et ühendamine käib
  }
  Serial.println("Yhendatud!"); // Kui ühendatud siis prindi välja ja mine edasi 
  Serial.println("");  Serial.print("WiFi connected to: ");Serial.println(ssid);  Serial.println("IP address: ");  Serial.println(WiFi.localIP());
  delay(500); 

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //insert my local time zones int time function
  printLocalTime(); //this functions also includes getting int values for time variables
  
  //For GET request, available time variable (int) needs to be converted into a string
  //int YEAR to string YEAR
  char currentYearChar[4];  //4 chars long array
  sprintf(currentYearChar, "%d", currentYear); //transform currentYear int into this char array
  String currentYearString = currentYearChar; //convert it into string (/0 added to the end)
  
   // int MONTH to string MONTH
   char currentMonthChar[2]; //2 chars long array for month
   sprintf(currentMonthChar, "%d", currentMonth); //transform currentMonth int into char array
   String currentMonthStringRAW = currentMonthChar; //make a temporary string 
   //Reason why it is needed - when it is may, then int currentMonth gives out value 5, but for GET request, string 05 is needed.  
   String currentMonthString; 
   if (currentMonth < 10){ //When currentMonth int is under 10
      currentMonthString = "0"+currentMonthStringRAW; //Then add "0" in front of it and write it into main string
    }
   else{
      currentMonthString = currentMonthStringRAW; //but if it is 10,11 or 12 then just use temporary string
    }

   // int DAY to string DAY
   char currentDayChar[2]; //2 chars long array for day
   sprintf(currentDayChar, "%d", currentDay); //transform currentDay in into char array
   String currentDayStringRAW = currentDayChar; //make a  temporary string - reason same as in month
   String currentDayString; 
   if (currentDay < 10){ //when currentDay is under 10
      currentDayString = "0"+currentDayStringRAW; //add "0" in front of it and write it into main string
    }
   else{
      currentDayString = currentDayStringRAW; //else just use temporary string
    }

  
   if(WiFi.status()== WL_CONNECTED){
      // For GET request, the right URL is needed. Elering API has specific way for GET requests. I use strings of current year and month and day to always ask for the right data
      String server = "https://dashboard.elering.ee/api/nps/price/csv?start="+currentYearString+"-"+currentMonthString+"-"+currentDayString+"T00%3A00%3A00.000%2B03%3A00&end="+currentYearString+"-"+currentMonthString+"-"+currentDayString+"T23%3A59%3A59.999%2B03%3A00&fields=ee";
      csv_file = GET_Request(server.c_str()); //write data to csv_file string

      //First make requested string into char array 
      int csv_length = csv_file.length(); //get length of requested csv file
      char csv_char_array[csv_length +1]; //make char array out of it with the length of csv + 1 because of ending null terminator
      strcpy(csv_char_array, csv_file.c_str()); 
      for (int i = 0; i < csv_length; i++)
      cout << csv_char_array[i];

      //next, parse char array into pieces and filter out only the price
      char * token = strtok(csv_char_array, "\""); //use " as limiting 
      int j = 0; //temporary variable
      while( token != NULL ) { //broke char array into pieces until NULL token received (char array ends)

     // While loop breaks char array into pieces and for every loop, token equals the last broken up parat of char arrray 
      j++; // every loop add j+1
      switch(j){ // and depending of the value of j
          // First price of csv file is the 11th price of char array that is broken up
          case 11: 
          {
          char * hind00 = token; // write the token value inte a new pointer of char array
          int temp0 = atoi(hind00); // and use it with atoi function. Atoi takes char pointer and gives out integer
          // atof could also be used, but as the value is xx,xx not xx.xx (comma used for cents) then none of these
          // functions are able to understand how to read them 
          pricesNordPool[0] = temp0; //then write this temporary int into pricesNordPool array. 
          break;
           }        
          case 17: // every sixth char array is price array
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
   }
    else {
      Serial.println("WiFi Disconnected");
  }
}

void loop() { //In a forever loop
  printLocalTime(); //Get new time 
  Serial.println(); //Two empty rows so it would be clearer when new loop starts
  Serial.println(); 
  Serial.print("Kell on: ");   
  Serial.print(currentHour); //Print current hour
  
  if (currentMinute < 10){
  Serial.print(".0");
  Serial.println(currentMinute); //if minute is under 10, then print out as e.g. 07 instead of 7
  }
  else {
  Serial.print(".");
  Serial.println(currentMinute);
  }
  
  Serial.print("Kuupäev: ");  
  Serial.print(currentDay);
  Serial.print(".");
  Serial.print(currentMonth);
  Serial.print(".");
  Serial.println(currentYear);
 priceAverage = getAverage(); //calculate average price of the day
 Serial.print("Päeva keskmine hind on:");
 Serial.print(priceAverage);
 Serial.println(" €");
  getPowerPercentage(); //calculate what power should system give out
  priceToResistanceAndCurrent(); //and calculate what byte should be written to digital rheostat and what is apporximate current 

Wire.beginTransmission(PotentiometerI2Caddress);  //Starts transmit process to aadress 45 (potentiometer)
Wire.write(0); //(DDDDCCDD) - CC = 00 write data, 0000 - volatile wiper 0, 00010volatile wiper 0; DD is bit 9 and 8 of digital potentiometer wiper setting 
Wire.write(resistance8bit[currentHour]); //resistance value for current hour
Wire.endTransmission();

 Serial.print("Praegune hind on:"); 
 Serial.print(pricesNordPool[currentHour]);
 Serial.print(" €... Eeldatav väljundvool on:");
 Serial.print(approxCurrent[currentHour]);
 Serial.println(" A");

for (int i = 0; i < 24; i++){  //print out a table of the day
  Serial.print(i);
  Serial.print(".00 - ");
  Serial.print(i+1);
  Serial.print(".00.... Elektri hind: ");
  Serial.print(pricesNordPool[i]);
  Serial.print(" €/MWh... Võimsus: ");
  Serial.print(powerPercentage[i]);
  Serial.print(" % ... Eeldatav väljundvool: ");
  Serial.print(approxCurrent[i]);
  Serial.println(" A");
   delay (1000);   
  }
}



// Function that calculates average price for the day
float getAverage(){ 
  float sumAverage = 0;
  for (int i = 0; i < 24; i++){
  sumAverage += pricesNordPool[i];
  }
  float priceAverage = sumAverage / 24; 
  return priceAverage;
}

// Function that finds the minimum and maximum prices and then calculates 
// the power needed in percentage that is used for hour
void getPowerPercentage(){ 
  int minimum = pricesNordPool[0];
  int maximum = pricesNordPool[0];
    for (int i = 0; i < 24; i++){
      if (pricesNordPool[i] < minimum ){
        minimum = pricesNordPool[i]; 
      }
      if (maximum < pricesNordPool[i]){
        maximum = pricesNordPool[i]; 
      }
    }  
    Serial.print("Maksimaalne hind on:");
    Serial.print(maximum);
    Serial.println(" €");
    Serial.print("Minimaalne hind on:");
    Serial.print(minimum);
    Serial.println(" €");
    float m_low, b_low;
    float m_high, b_high;
    m_low = -50/(priceAverage - minimum); // (0-50)
    b_low = 100-(minimum*m_low); 
    m_high = 50/(priceAverage - maximum);
    b_high = 0-(maximum*m_high);
 for (int i = 0; i < 24; i++){
      if (pricesNordPool[i]<priceAverage) { 
        powerPercentage[i] = m_low*pricesNordPool[i]+b_low;
      }
      if (pricesNordPool[i]>priceAverage){
        powerPercentage[i] = m_high*pricesNordPool[i]+b_high;
      }
    }
return;
}


// Function that takes power in percents and calculates
// what resistance is used for current limiting resistor 
// and approximate current that is gonna flow 
void priceToResistanceAndCurrent(){
    float m_resistance, b_resistance;
    float m_current, b_current;
    // 1.5A is maximum for DCP. To limit it on 1.5A, resistor should be about 30 kohm
    m_resistance = (255.00-76.00)/(0.00-100.00); //255 - 100 kohm, 76 - 30kohm. Divide it with 0% to 100%
    b_resistance = 255;  
    m_current = (0.5-1.5)/(255-76);
    b_current = (0.5-(m_current*255));
    for (int i = 0; i < 24; i++){
       resistance8bit[i] = m_resistance * powerPercentage[i] + b_resistance;
       approxCurrent[i] = m_current * resistance8bit[i] + b_current;
   }
return;
}  


// Functions that sends a request for NTP and gets the time 
void printLocalTime(){
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
 // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S"); 
  currentHour = timeinfo.tm_hour;
  currentMinute = timeinfo.tm_min;
  currentDay = timeinfo.tm_mday;
  currentMonth = timeinfo.tm_mon + 1; //Need to add +1 as tm_mon is array [11] and e.g january is 0th member in array
  currentYear = timeinfo.tm_year + 1900; //Need + 1900 as it shows years since year 1900
}


//Function for GET request from HTTPClient.h
String GET_Request(const char* server){
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
  return payload; //String from GET request 
}


// Voltage measurement on USB1
float measurementVoltageUSB1(){
  int voltageRAWUSB1 = analogRead(voltagePinUSB1);
  float voltageUSB1 = ((voltageRAWUSB1/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
return voltageUSB1; 
}

// Voltage measurement on 5V bus
float measurementVoltage5V(){
  int voltageRAW5V = analogRead(voltagePin5V);
  float voltage5V = ((voltageRAW5V/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
return voltage5V; 
}

// Voltage measurement of MicroUSB input
float measurementVoltageMicroUSB(){
  int voltageRAWMicroUSB = analogRead(voltagePinMicroUSB);
  float voltageMicroUSB = ((voltageRAWMicroUSB/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
return voltageMicroUSB; 
}


  // Current measurement of USB1  
float measurementCurrentUSB1(){
  int currentRAWUSB1 = analogRead(currentMeasurePinUSB1);
  float currentUSB1 = ((currentRAWUSB1/ADCresolution)*ADCmaxVoltage/1000)/(currentSense*gainCurrentAmplifier);
  return currentUSB1;
}

  // Current measurement of USB2  
float measurementCurrentUSB2(){
  int currentRAWUSB2 = analogRead(currentMeasurePinUSB2); 
  float currentUSB2 = ((currentRAWUSB2/ADCresolution)*ADCmaxVoltage/1000)/(currentSense*gainCurrentAmplifier);
  return currentUSB2;
}

  // Current measurement of  3V3 bus
float measurementCurrent3V3(){
  int currentRAW3V3 = analogRead(currentMeasurePin3V3); 
  float current3V3 = ((currentRAW3V3/ADCresolution)*ADCmaxVoltage/1000)/(currentSense*gainCurrentAmplifier);
  return currentRAW3V3;
}

//Initialize all pins, needs to be done in void(setup)
void pinInitalize() {
  //OUTPUTS  
  pinMode (ledPin, OUTPUT); // Debug LED
  pinMode (currentLimitSelection1, OUTPUT); //HIGH for rheostat,  LOW for 22.1k
  pinMode (currentLimitSelection2, OUTPUT); //HIGH for rheostat,  LOW for 22.1k
  pinMode (enableUSBportPin1, OUTPUT); //HIGH to enable USB port 1
  pinMode (enableUSBportPin2, OUTPUT); //HIGH fto enable USB port 2 
  //INPUTS
  pinMode (voltagePinUSB1, INPUT); 
  pinMode (voltagePinUSB2, INPUT);
  pinMode (voltagePin5V, INPUT);
  pinMode (voltagePinMicroUSB, INPUT);
  pinMode (voltagePin3V3, INPUT);
  pinMode (currentMeasurePinUSB1, INPUT);
  pinMode (currentMeasurePinUSB2, INPUT);
  pinMode (currentMeasurePin3V3, INPUT);
  pinMode (currentMeasureNotUsedYet, INPUT); // From current sense, but no sense resistor connected to current sense
  pinMode (potentiometerPin1, INPUT); // Not yet used, could be used for configuration
  pinMode (potentiometerPin2, INPUT); // Not yet used, could be used for configuration
  pinMode (buttonInput1, INPUT); // Not yet used, could be used for configuration
  pinMode (buttonInput2, INPUT); // Not yet used, could be used for configuration
}


//Examples on how to get value and print it out for voltages and currents
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
*/


// I2C components 
// Potentiometer MCP4652T-104E/UN
// 8 bit (257 taps)
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
