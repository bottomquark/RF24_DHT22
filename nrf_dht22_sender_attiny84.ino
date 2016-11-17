/*
 * TRANSMITTER NODE
 * Every 60 seconds, send temp, humidity, voltage, and the ID of the node to the receiver node.
 *
 * ATTINY84 must run on 8MHz (burn bootloader) or DHT22 would only output errors (-995 values)!
 */

//Pins: see pins_arduino.c
// ATMEL ATTINY84 / ARDUINO
//
//                           +-\/-+
//                     VCC  1|    |14  GND
//             (D  0)  PB0  2|    |13  AREF (D 10)
//             (D  1)  PB1  3|    |12  PA1  (D  9) 
//                     PB3  4|    |11  PA2  (D  8) 
//  PWM  INT0  (D  2)  PB2  5|    |10  PA3  (D  7) 
//  PWM        (D  3)  PA7  6|    |9   PA4  (D  6) 
//  PWM        (D  4)  PA6  7|    |8   PA5  (D  5)        PWM
//                           +----+

//                              +-\/-+                              
//nRF24L01  VCC, pin2 --- VCC  1|o   |14 GND --- nRF24L01  GND, pin1
//DHT22 Data              PB0  2|    |13 AREF --- 3.3V
//                        PB1  3|    |12 PA1
//                        PB3  4|    |11 PA2 --- nRF24L01   CE, pin3
//DHT22 VCC               PB2  5|    |10 PA3 --- nRF24L01  CSN, pin4
//                        PA7  6|    |9  PA4 --- nRF24L01  SCK, pin5
//nRF24L01 MOSI, pin7 --- PA6  7|    |8  PA5 --- nRF24L01 MISO, pin6
//                              +----+

//save some memory:
#define DHT22_NO_FLOAT
//#define ONEWIRE_CRC8_TABLE 0
//#define TinyDebugSerial_h

#include <JeeLib.h>
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <DHT22.h>
#include <OneWire.h>

// Pins for RF24 module
#define CE       8
#define CS       10

// Pin 2 on ATTINY84
#define DHT22_PIN 0 
// OneWire pin: Pin 3 on ATTINY84
#define OW_PIN 1
//#define DS18S20_ID 0x10
//#define DS18B20_ID 0x28

//**************** power supply pins **************
// Pin 5 on ATTINY84
#define VCC_DHT22 2
// Pin 6 on ATTINY84
#define VCC_DS1820 3
// Pin 10 on ATTINY84
//#define VCC_RF24 7
//analog pins are different! Physical pin 10 is digital pin 7 and analog pin 3
#define REF_33V 3 
// Pin 12 on ATTINY84
#define VCC_LED 9 
//**************** power supply pins **************

// Address of our node and the other
const uint16_t this_node = 3;
const uint16_t other_node = 0;

//*************** Structure of our payload ********
struct payload
{

  //temperature in deci-degrees C (233 means 23.3)
  int temp;
  //humidity in .1 increments (245 means 24.5)
  int hum;
  //voltage of the battery in millivolt
  int voltage;

  unsigned char error;
  //address of OneWire sensor
  unsigned char address[8];
};
typedef struct payload Payload;

//*************** Structure of our payload ********

//prototypes:
void readDht22();
long readVcc();
void radioOn();
void radioOff();
void readAndSendDs1820();
void flashLed();

//*************** setup necessary stuff *********
RF24 radio(CE, CS);
RF24Network network(radio);

DHT22 myDHT22(DHT22_PIN);
OneWire ds(OW_PIN);

ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup the watchdog

Payload payload;

bool dht22_attached = true;
bool ds1820_attached = true;
unsigned char header_type = 0;
//header type constants
#define HEADER_HUM_TEMP 0
#define HEADER_TEMP 1
#define HEADER_LEDS 2

//globally used counter variable
byte i;
//*************** setup necessary stuff *********

void setup(void)
{
  SPI.begin();

  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ this_node);

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);

  flashLed();

  pinMode(REF_33V, INPUT); // set to input, so no power is used
  analogReference(DEFAULT);

  readDht22();
  if(payload.error)
  {
    dht22_attached = false;
  }
  readAndSendDs1820();
  if(payload.error)
  {
    ds1820_attached = false;
  }
}

void loop(void)
{
  payload.voltage = readVcc();
  //set error value which will be sent if no sensor is attached. readDht22 and readAndSend1820 will set the error value correctly if a sensor is attached.
  payload.error = 11;

  if(dht22_attached)
  {
    readDht22();
    header_type = HEADER_HUM_TEMP;
  }

  RF24NetworkHeader header(/*to node*/ other_node, header_type);
  radioOn();
  bool writeOk = network.write(header,&payload,sizeof(payload));
  if(!writeOk) payload.error = 13;

  if(ds1820_attached)
  {
    readAndSendDs1820();
  }

  if(!ds1820_attached && !dht22_attached)
  {
    pinMode(VCC_LED, OUTPUT);
    if(writeOk)
    {
      //light up if sending was successful
      digitalWrite(VCC_LED, HIGH);
    }
    else
    {
      digitalWrite(VCC_LED, LOW);
    }
    Sleepy::loseSomeTime(500);
  }
  else
  {
    radioOff();
    //sleep about 60 seconds
    Sleepy::loseSomeTime(60000);
  }
}


void flashLed()
{
  pinMode(VCC_LED, OUTPUT);
  digitalWrite(VCC_LED, HIGH);
  Sleepy::loseSomeTime(500);
  //digitalWrite(VCC_LED, LOW);
  pinMode(VCC_LED, INPUT);
}

void radioOn()
{
  // Pump the network regularly
  network.update();
}

void radioOff()
{
  //power down, would wake up with the next write() call
  radio.powerDown();
}

/**
 *fill the given Payload with sensor data
 */
void readDht22() {
  //switch on DHT22
  pinMode(VCC_DHT22, OUTPUT); // set to output to activate power 
  digitalWrite(VCC_DHT22, HIGH);
  //wait 2s for DHT22 to warm up
  //delay(2000);
  Sleepy::loseSomeTime(2000);
  //DHT22_ERROR_t errorCode;
  
  // The sensor can only be read from every 1-2s, and requires a minimum
  // 2s warm-up after power-on.
  //delay(2000);
  payload.error = myDHT22.readData();  
  //errorCode = myDHT22.readData();
  /*
  switch(myDHT22.readData())
  {
    case DHT_ERROR_NONE:
      payload.error = 0;
      break;
    case DHT_ERROR_CHECKSUM:
      payload.error = 1;
      break;
    case DHT_BUS_HUNG:
      payload.error = 2;
      break;
    case DHT_ERROR_NOT_PRESENT:
      dht22_attached = false;
      payload.error = 3;
      break;
    case DHT_ERROR_ACK_TOO_LONG:
      //NOT_PRESENT does not work somehow, so assume that no DHT22 is attached:
      dht22_attached = false;
      payload.error = 4;
      break;
    case DHT_ERROR_SYNC_TIMEOUT:
      payload.error = 5;
      break;
    case DHT_ERROR_DATA_TIMEOUT:
      payload.error = 6;
      break;
    case DHT_ERROR_TOOQUICK:
      payload.error = 7;
      break;
  }
  */

  payload.temp = myDHT22.getTemperatureCInt();
  payload.hum =  myDHT22.getHumidityInt();

  pinMode(VCC_DHT22, INPUT); // set to input, so no power is used

  memset(payload.address,0,sizeof(payload.address));
  /*
  for(i = 0; i < 8; i++)
  {
    payload.address[i] = 0; 
  }
  */
}

// readVcc()-Lib von Nathan Chantrell
// Gibt die Spannung der Stromversorgung in mV aus
// https://github.com/nathanchantrell/TinyTX/blob/master/TinyTX_DS18B20/TinyTX_DS18B20.ino
long readVcc() {
  int shouldBe33v = analogRead(REF_33V);
  delay(2);
  shouldBe33v = analogRead(REF_33V);
  //1023 is VIN
  //shouldBe33v = 3,3v
  // schouldBe33V entspricht 3,3V
  // 1023 entspricht ?
  // => 1023*3.3/shouldBe33V
  // => 3375.9/shouldBe33V
  // => 3375900/shouldBe33V = in Millivolt

  return 3375900L/shouldBe33v;
  //return shouldBe33v;
/*
  bitClear(PRR, PRADC);
  ADCSRA |= bit(ADEN); // Enable the ADC
  long result;
  // Read 1.1V reference against Vcc
  //#if defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
  //#else
  //  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // For ATmega328
  //#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
 
  // Kalibrierung der VCC-Anzeige
  // http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // Abschnitt: Improving Accuracy
  // scale_constant = internal1.1Ref * 1023 * 1000
  // internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
  // Default: 1125300L
  // Meine Konstante: 1070860L, errechnet mit 3x1,5V Batterien als VCC
  //FIXME: messen und korrekten Wert eintragen: 
  result = 1125300L / result; // Back-calculate Vcc in mV; Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  ADCSRA &= ~ bit(ADEN);
  bitSet(PRR, PRADC); // Disable the ADC to save power
  return result;
*/
}

/**
 * reads and sends all values of all found sensors
 */
void readAndSendDs1820(){

  //switch on sensor 
  pinMode(VCC_DS1820, OUTPUT); // set to output to activate power 
  digitalWrite(VCC_DS1820, HIGH);

  ds1820_attached = false;
  byte data[12];
  byte addr[8];
  //find a device
  ds.reset_search();
  for(; ds.search(addr); )
  {
    payload.error = 0;
    ds1820_attached = true;
     /*
   if (!ds.search(addr)) {
     ds1820_attached = false;
     payload->error = 1;
     return;
   }
   */
    if (OneWire::crc8( addr, 7) != addr[7]) {
      payload.error = 8;
    }
/* 
    if (addr[0] != DS18S20_ID && addr[0] != DS18B20_ID) {
      ds1820_attached = false;
      payload.error = 9;
    }
*/
    ds.reset();
    ds.select(addr);
    // Start conversion
    ds.write(0x44, 1);
    // Wait some time...
    delay(850);
    ds.reset();
    ds.select(addr);
    // Issue Read scratchpad command
    ds.write(0xBE);
    // Receive 9 bytes
    for ( i = 0; i < 9; i++) {
      data[i] = ds.read();
    }
    // Calculate temperature value
    //payload->temp = ( ( (data[1] << 8) + data[0] )*0.0625 );
    payload.temp = (((data[1] << 8) + data[0])*10)/16;
    payload.hum = -1; 
    for(i = 0; i < 8; i++)
    {
      payload.address[i] = addr[i]; 
    }

    RF24NetworkHeader header(/*to node*/ other_node, HEADER_TEMP);
    bool writeok = network.write(header,&payload,sizeof(payload));
    if(!writeok) payload.error = 12;
  }

  pinMode(VCC_DS1820, INPUT); // set to input, so no power is used
}

// vim:ai:cin:sts=2 sw=2 ft=cpp
