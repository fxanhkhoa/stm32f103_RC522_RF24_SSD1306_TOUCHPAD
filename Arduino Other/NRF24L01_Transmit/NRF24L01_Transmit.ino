#include <SPI.h>
#include "RF24.h"

#include "SimpleDHT.h"


 
const uint64_t pipe = 0xE8E8F0F0E1LL; // địa chỉ để phát
RF24 radio(9,10); //thay 10 thành 53 với mega
byte msg[3];
const int sensor = A0;
int value = 0;

// DHT22 variables
int pinDHT22 = 2;
SimpleDHT22 dht22;

float temperature = 0;
  float humidity = 0;
 
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
  
  // DHT22 sampling rate is 0.5HZ.
  delay(2500);
  
  msg[0] = (byte) temperature;
  msg[1] = (byte) humidity;
  radio.write(&msg, sizeof(msg));
  
  delay(50);
}
