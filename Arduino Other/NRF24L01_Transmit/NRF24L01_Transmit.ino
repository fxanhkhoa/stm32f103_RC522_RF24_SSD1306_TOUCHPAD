#include <SPI.h>
#include "RF24.h"

#include "SimpleDHT.h"
#include <Adafruit_BMP085.h>


 
const uint64_t pipe = 0xE8E8F0F0E1LL; // địa chỉ để phát
RF24 radio(9,10); //thay 10 thành 53 với mega
byte msg[3];
const int sensor = A0;
int value = 0;

// DHT22 variables
int pinDHT22 = 2;
SimpleDHT22 dht22;
Adafruit_BMP085 bmp;

float temperature = 0;
float humidity = 0;
byte pressure = 0;
 
void setup(){ 
  //============================================================Module NRF24
  radio.begin();                     
  radio.setAutoAck(1);               
  radio.setRetries(1,1);             
  radio.setDataRate(RF24_1MBPS);    // Tốc độ truyền
  radio.setPALevel(RF24_PA_MAX);      // Dung lượng tối đa
  radio.setChannel(10);               // Đặt kênh
  radio.openWritingPipe(pipe);        // mở kênh
  //pinMode(sensor, INPUT);
  //============================================================Module DHT22
  //============================================================Serial Port
  Serial.begin(9600);
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
}
 
void loop(){
  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read2(pinDHT22, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.println(err);delay(2000);
    return;
  }
  
  Serial.print("Sample OK: ");
  Serial.print((float)temperature); Serial.print(" *C, ");
  Serial.print((float)humidity); Serial.println(" RH%");

   Serial.print("Temperature = ");
   Serial.print(bmp.readTemperature());
   Serial.println(" *C");

   pressure = (byte)(bmp.readPressure() / 10325);
   Serial.print("Pressure = ");
   Serial.print(pressure);
   Serial.println(" atm");
    
   // Calculate altitude assuming 'standard' barometric
   // pressure of 1013.25 millibar = 101325 Pascal
   Serial.print("Altitude = ");
   Serial.print(bmp.readAltitude());
   Serial.println(" meters");

   Serial.print("Pressure at sealevel (calculated) = ");
   Serial.print(bmp.readSealevelPressure());
   Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
   Serial.print("Real altitude = ");
   Serial.print(bmp.readAltitude(101500));
   Serial.println(" meters");
   
   Serial.println();
  
  // DHT22 sampling rate is 0.5HZ.
  delay(2500);
  
  msg[0] = (byte) temperature;
  msg[1] = (byte) humidity;
  msg[2] = (byte) pressure;
  radio.write(&msg, sizeof(msg));
  
  delay(50);
}
