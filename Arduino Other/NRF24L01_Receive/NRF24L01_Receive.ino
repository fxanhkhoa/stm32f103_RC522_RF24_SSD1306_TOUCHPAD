#include <SPI.h>
#include <Wire.h>
#include "RF24.h"
#include <U8g2lib.h>

#define REPORTING_PERIOD_MS     500
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);
 
const uint64_t pipe = 0xE8E8F0F0E1LL; // địa chỉ phát
RF24 radio(9,10);//thay 10 thành 53 với mega
byte msg[3];
const int led = 3;
int led_st = 0;
 
void setup(){
  Serial.begin(9600);
//  radio.begin();                     
//  radio.setAutoAck(1);              
//  radio.setDataRate(RF24_1MBPS);    // Tốc độ dữ liệu
//  radio.setChannel(10);               // Đặt kênh
//  radio.openReadingPipe(1,pipe);     
//  radio.startListening();            
  pinMode(led, OUTPUT);
  // Init Oled
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_te);
  
  u8g2.setCursor(0,8);
  u8g2.print("temperature:");
  

  u8g2.setCursor(70,8);
  u8g2.print("a");

  u8g2.setCursor(0,18);
  u8g2.print("humidity:");

  u8g2.setCursor(0,28);
  u8g2.print("another:");
  u8g2.sendBuffer();
}

void displayall(byte msg[])
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_te);
  
  u8g2.setCursor(0,8);
  u8g2.print("temperature:");
  u8g2.print(msg[0]);

  //u8g2.setCursor(70,8);
  //u8g2.print("a");

  u8g2.setCursor(0,18);
  u8g2.print("humidity:");
  u8g2.print(msg[1]);

  u8g2.setCursor(0,28);
  u8g2.print("another:");
  u8g2.sendBuffer();
}
 
void loop(){
  if (radio.available()){
    while (radio.available()){
      radio.read(&msg, sizeof(msg));
      Serial.println(msg[0]);
      analogWrite(led, msg[0]);
      displayall(msg);
      delay(2000);
    }
  }
  
}
