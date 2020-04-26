/*
 Name:		AmsTempToRFM95.ino
 Created:	4/26/2020 18:26:28 PM
 Author:	aslemann
*/

#include <DallasTemperature.h>
#include <OneWire.h>
#include <SPI.h>
#include <LoRa.h>
#include "LowPower.h"
#include "TimeLib.h"

// Setting up DS1820
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
float Temp = 0;
long supplyV;

// Config RFM95 module
int ss = 6;
int rfm_reset = 5;
int dio0 = 3;
int RFMfreq = 868E6;

// Define objects to contain data
unsigned long P;
char meterID[16];
// TimeLib elements
tmElements_t my_time;  // time elements structure
time_t unix_timestamp; // a timestamp

// Number of pulses counted
volatile int powerPulses = 0;
int counter =0;
byte sender = 0xFF;

//// The HAN Port reader, used to read serial data and decode DLMS
int numByte = 511;
byte messageLength=0;
byte hanData[512]; // an array to store the received data
boolean readyData = false;
int debug = 0; // Integer to debug function

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(2400);
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,HIGH);
  // Start up the library
  sensors.begin();
  // Start up LoRa
  setupLoRa();
  // Read Vcc
  long supplyV = readVcc(); // Get supply voltage
  while ( supplyV < 3200 ){
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    supplyV = readVcc(); // Get supply voltage
  }
}

// the loop function runs over and over again until power down or reset
void loop()
{

// Read Vcc
  supplyV = readVcc(); // Get supply voltage

// Read data from the HAN port
  if ( readHanData() ){
  if ( dataKaifa(messageLength) ){
// Read temperature
  sensors.requestTemperatures(); // Send the command to get temperatures
  Temp = sensors.getTempCByIndex(0);  
  
// Transmit Data
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print("\"t\":");
  LoRa.print(unix_timestamp);
  LoRa.print(",\"T\":");
  LoRa.print(Temp);
  LoRa.print(",\"Vcc\":");
  LoRa.print(supplyV);  
  LoRa.print(",\"P\":");
  LoRa.print(P);
//  if(messageLength>40){
//  LoRa.print(",\"mID\":\"");
//  for (int i=0;i<16;i++){
//  LoRa.print(meterID[i]);
//  }
//  LoRa.print("\"");
//  }
  LoRa.endPacket();
  LoRa.sleep();
  // Debug message
  Serial.print("hanData:");
  for (int i=0;i<=messageLength+1;i++){
  Serial.print(hanData[i],HEX);
  Serial.print(" ");
  }
  Serial.println();
  messageLength = 0;
  }
  }
  
  if (supplyV < 3200){
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  else{
  LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_ON, TWI_OFF);
  }
}

void countPulse() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 100ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 100)
  {
    powerPulses++;
  }
  last_interrupt_time = interrupt_time;
}

void setupLoRa() {
  LoRa.setPins(ss, rfm_reset, dio0);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.sleep();
}

bool readHanData() {
    static boolean recvInProgress = false;
    static int position = 0;
    byte Marker = 0x7E; // Start and end marker of data Package
    byte rc;
    readyData = false;
 
    while ( Serial.available() > 0 && readyData == false ) {
      rc = Serial.read();
      if (position == 0 && rc != Marker){
      // we haven't started yet, wait for the start flag (no need to capture any data yet)
      position = 0;
      messageLength = 0;
      return false;
      }
      else if (position == 0 && rc == Marker){
      // This is a start flag!! (Or maybe an end Flag?)
      hanData[0]=rc;
      position++;
      debug = position;
      }
      
      else{
        // We have started collecting data, position is > 0
         // Check if this is a second start flag, which indicates the previous one was a stop from the last package
        if (position == 1 && rc == Marker){
            // do nothing, this was a second flag, we just carry on.
        }
        else if( position > 1 && rc == Marker ){
          // We are done
          hanData[position] = rc;
          readyData = true;
          messageLength = position-1;
          position = 0;
          return true;
        }
        else{
        // We have started, so capture every byte
        hanData[position] = rc;
        position++;
        if(position >= numByte){
          position = numByte-1;
          readyData = true;
          messageLength = position;
          position = 0;
          return false;
        }
        }
      }
    }
    return false;
}

bool dataKaifa(int ListSize){
  int pos;
  // Check if valid data
  if (
      hanData[9] != 0xE6 || 
      hanData[10] != 0xE7 ||
      hanData[11] != 0x00 ||
      hanData[12] != 0x0F )
      {
        return false; // Invalid HAN data
      }
    
    if (ListSize == 39){ // List 1
    // Get time
    getTimeUNIX();
    
    pos = 34;
    // Find Power
    P=getLong(pos,4);
    return true;
    }
    
    if (ListSize == 120){ // List 2
    // Get time
    getTimeUNIX();
    
    // Get Meter ID
    getMeterID();

    // Find Power
    P=getLong(70,4);
    
    return true;
    }
}

int getInt(int startPos, int numBytes){
  int result;
  int i=0;
  for ( i=0; i<numBytes;i++){
        result = result << 8 | hanData[startPos+i];
  }
  return result;
}

long getLong(int startPos, int numBytes){
  long result;
  int i=0;
  for ( i=0; i<numBytes;i++){
        result = result << 8 | hanData[startPos+i];
  }
  return result;
}

bool getMeterID(){
  int pos = 44;
  for (int i=0;i<15;i++){
    meterID[i]=hanData[pos+i];
  }
    return true;
}

bool getTimeUNIX(){
    // Get time
    int pos = 19; // Start position of date
    my_time.Year = hanData[pos] << 8 |
           hanData[pos + 1];
    my_time.Year = my_time.Year - 1970;
    my_time.Month = hanData[pos + 2];
    my_time.Day = hanData[pos + 3];
    my_time.Hour = hanData[pos + 5];
    my_time.Minute = hanData[pos + 6];
    my_time.Second = hanData[pos + 7];
    unix_timestamp =  makeTime(my_time)-7200;
}

//--------------------------------------------------------------------------------------------------
// Read current supply voltage
//--------------------------------------------------------------------------------------------------
 long readVcc() {
   bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
   long result;
   // Read 1.1V reference against Vcc
   ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega328
   delay(2); // Wait for Vref to settle
   ADCSRA |= _BV(ADSC); // Convert
   while (bit_is_set(ADCSRA,ADSC));
   result = ADCL;
   result |= ADCH<<8;
   result = 1126400L / result; // Back-calculate Vcc in mV
   ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
   return result;
} 
