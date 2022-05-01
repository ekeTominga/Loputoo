/*
 * USB charger with current control according to electricity price
 * Author: Eke Tominga  
 * Date: Started 15.04.2022
 * 
 */

#include <Wire.h> //Library for I2C communication
#include <WiFi.h>

//variables for Wifi connection
const char* ssid = "TalTech";
//const char* password =  "yourNetworkPass"; //not needed right now as TalTech is without a passowrd

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
int ADCmaxVoltage = 3300; //maximum measurable voltage is 2450 mV in ESP32 WROOM
int ADCresolution = 4095; //12 bit ADC
float voltageDividerVoltageMeasurement = 22.1/(22.1+33);

#define PotentiometerI2Caddress 45 
#define SDA 21
#define SCL 22



void setup() {  
pinInitalize(); //initalizes pins as either input or output

//ADC setup
//ADC CALIBTRATION TO BE DONE! - https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/adc.html
analogSetAttenuation(ADC_11db); // sets ADC to maximum 2450 mV reading
digitalWrite (enableUSBportPin1, HIGH); //Enables USB port 1 
digitalWrite (enableUSBportPin2, HIGH); // Enables USB port 2 
digitalWrite (currentLimitSelection1, HIGH); // 2A current limit to USB port 1
digitalWrite (currentLimitSelection2, HIGH); // 2A current limit to USB port 1 

//I2C transmission, try to make it 100kohm)
Wire.setClock(100000); //Sets I2C clock speed to 100 khz
//if (!Wire.isEnabled()){; //Checks if I2C is already enabled and only start I2C when it is not 
Wire.begin(SDA, SCL); //Begins I2C in MASTER mode, pins of SDA and SCL (21 and 22)
Wire.beginTransmission(PotentiometerI2Caddress);  //Starts transmit process to aadress 45 (potentiometer)
Wire.write(0); //(DDDDCCDD) - CC = 00 write data, 0000 - volatile wiper 0, 0001 volatile wiper 1; DD is bit 9 and 8 of digital potentiometer wiper setting 
Wire.write(255); //maybe 100kohm - needs to be tested
Wire.endTransmission();
//Transmission ends

Serial.begin(115200); //serial monitor UP
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, NULL);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

}

void loop() {
  
float voltageRead = measurementVoltage();
Serial.println(voltageRead);
float currentRead = measurementCurrent();
Serial.println(currentRead);
float proov = 5;
Serial.println(proov);

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
}




float measurementVoltage(){
  //voltage on USB1
  int voltageRAWUSB1 = analogRead(voltagePinUSB1);
  float voltageUSB1 = ((voltageRAWUSB1/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
  //voltage on USB2
  int voltageRAWUSB2 = analogRead(voltagePinUSB2);
  float voltageUSB2 = ((voltageRAWUSB2/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
  //voltage on 5V bus
  int voltageRAW5V = analogRead(voltagePin5V);
  float voltage5V = ((voltageRAW5V/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
  //voltage on Programming bus
  int voltageRAWMicroUSB = analogRead(voltagePinMicroUSB);
  float voltageProgramming = ((voltageRAWMicroUSB/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
  //voltage on 3V3 bus
  int voltageRAW3V3 = analogRead(voltagePin3V3);
  float voltage3V3 = ((voltageRAW3V3/ADCresolution)*ADCmaxVoltage/1000)/voltageDividerVoltageMeasurement;
return voltage5V; 
}



float measurementCurrent(){
  //current USB1  
  int currentRAWUSB1 = analogRead(currentMeasurePinUSB1);
  float currentUSB1 = ((currentRAWUSB1/ADCresolution)*ADCmaxVoltage/1000)/(currentSense*gainCurrentAmplifier);
  //current USB2
  int currentRAWUSB2 = analogRead(currentMeasurePinUSB2); 
  float currentUSB2 = ((currentRAWUSB2/ADCresolution)*ADCmaxVoltage/1000)/(currentSense*gainCurrentAmplifier);
  //current 3V3
  int currentRAW3V3 = analogRead(currentMeasurePin3V3); 
  float current3V3 = ((currentRAW3V3/ADCresolution)*ADCmaxVoltage/1000)/(currentSense*gainCurrentAmplifier);
  return currentUSB2;
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
