/*
* Arduino Wireless Communication Tutorial
*     Example 2 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(7, 8); // CE, CSN

struct DataToSend{
  byte manual_auto;
  int xJoy_none;
  int yJoy_none;
  int fi_xTarget;
  int ro_yTarget;
  byte strike_start;
  byte load_none;
  byte none_giveFedbackPositon;
} payload;


const byte addresses[][6] = {"00001", "00002"};
int tempW = 1;
int tempR = 0;
unsigned long timeForUpload = 0;
unsigned long gapInFeedback = 0;
//bool cant strike = false;
bool startFlag = false;

void receiveData();
void sendData();
void action();
void moveCar();
void moveRifle();

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
      receiveData();
      action();
}

void receiveData(){
  radio.startListening();
    while (radio.available()) {   // if też działał
      radio.read(&payload, sizeof(payload)); //..read jak przeczyta to ustawia .available na false
      Serial.println(payload.xJoy_none);
      //Serial.println(tempR);
    }
}
void sendData(){
  radio.stopListening();
  radio.write(&tempW, sizeof(tempW));
  Serial.println(tempW);
  tempW++;
}
void action(){
  if(payload.manual_auto == 0){ // manualny
    moveCar();
    moveRifle();
    if(payload.strike_start == 1){
      // rozpocznij proceduję strzału
    }
    if(payload.load_none == 1){
      // rozpocznij precedurę załadunku
    }
  }
  else{ // automatyczny
    if(payload.strike_start == 1 || startFlag){
      trackToTarget(); 

    }
    if(payload.none_giveFedbackPositon == 1){
      //delay(50);
      if(millis() - gapInFeedback >= 500){
        gapInFeedback = millis();
        sendData();
      }
    }
  }
}
void moveCar(){

}
void moveRifle(){

}
void trackToTarget(){

}