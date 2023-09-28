#include <SPI.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define pJoyButt 2  // pin joystick button
#define pPotX A0    // pin potentiometer X
#define pPotY A1    // pin joystick X
#define pJoyX A2    // pin joystick Y
#define pJoyY A3    // pin joystick X
#define pButt0 3
#define pButt1 4
#define pButt2 5
#define pButt3 6

// OBIEKTY
LiquidCrystal_I2C lcd(0x27, 16, 2);
RF24 radio(7, 8);  // CE, CSN
const byte addresses[][6] = { "00001", "00002" };
int tempW = 1;
int tempR = 0;

// ZMIENNE
struct Potentiometer {
  int X;
  int Y;
};  //pot;
struct Joystick {
  int X;
  int Y;
  byte Click;
};  //joy;
struct Pad {
  Potentiometer pot;
  Joystick joy;
  byte button[4];  // przyciski
} pad;
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

int panelNumber;  // numer wyświetlanego panelu
unsigned long now = 0;
unsigned long delayJoy = 0;
unsigned long delayButt0 = 0;
unsigned long delayButt1 = 0;
unsigned long delayButt2 = 0;
unsigned long delayButt3 = 0;
unsigned long delayPrint = 0;
unsigned long timeForDownload = 0;
unsigned long timeForUpload = 0;
unsigned long gapInDownload = 0;
const long minJoy = 0;
const long maxJoy = 100;
const long minPot = 0;
const long maxPot = 100;
byte lastButt;


// DEFINICJE FUNKCJI
void print(int x);
byte changeCondition(byte x, int num);
void prepareData();
void sendData();
void receiveData();

// PROGRAM
void setup() {
  pinMode(pJoyButt, INPUT_PULLUP);
  pinMode(pButt0, INPUT_PULLUP);
  pinMode(pButt1, INPUT_PULLUP);
  pinMode(pButt2, INPUT_PULLUP);
  pinMode(pButt3, INPUT_PULLUP);

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  radio.begin();
  radio.openWritingPipe(addresses[1]);     // 00002
  radio.openReadingPipe(1, addresses[0]);  // 00001
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
  now = millis();

  if ((digitalRead(pButt0) == 0) && (now - delayButt0 >= 500)) {  // wciśnięcie przycisku nr1 -> pButt0 = 0
    delayButt0 = now;
    pad.button[0] = changeCondition(pad.button[0], 2);  // 2 tryby ->  manualny-0 | automatyczny-1
    print(0);
    prepareData();
    sendData();
  }
  if ((digitalRead(pButt1) == 0) && (now - delayButt1 >= 500)) {  // wciśnięcie przycisku nr2 -> pButt1 = 0
    delayButt1 = now;
    pad.button[1] = changeCondition(pad.button[1], 2);  // menu dla trybów auto i manual
    print(1);
    prepareData();
    sendData();
  }
  if ((digitalRead(pButt2) == 0) && (now - delayButt2 >= 500)) {  // wciśnięcie przycisku nr3 -> pButt2 = 0
    delayButt2 = now;
    pad.button[2] = changeCondition(pad.button[2], 1);  // strzał/start
    print(2);
    prepareData();
    sendData();
  }
  if ((digitalRead(pButt3) == 0) && (now - delayButt3 >= 500)) {  // wciśnięcie przycisku nr4 -> pButt3 = 0
    delayButt3 = now;
    pad.button[3] = changeCondition(pad.button[3], 1);  // potwierdzenie (załadowania kulki)
    print(3);
    prepareData();
    sendData();
  }
  if ((abs(pad.joy.X - map(analogRead(pJoyX), 0, 1023, minJoy, maxJoy)) > 1) || (abs(pad.joy.Y - map(analogRead(pJoyY), 0, 1023, minJoy, maxJoy)) > 1)) {  // obsługa joystick'a
    pad.joy.X = map(analogRead(pJoyX), 0, 1023, minJoy, maxJoy);
    pad.joy.Y = map(analogRead(pJoyY), 0, 1023, minJoy, maxJoy);
    print(lastButt);
    prepareData();
    sendData();
  }
  if ((abs(pad.pot.X - map(analogRead(pPotX), 0, 1023, minPot, maxPot)) > 1) || (abs(pad.pot.Y - map(analogRead(pPotY), 0, 1023, minPot, maxPot)) > 1)) {  // obsługa joystick'a
    pad.pot.X = map(analogRead(pPotX), 0, 1023, minPot, maxPot);
    pad.pot.Y = map(analogRead(pPotY), 0, 1023, minPot, maxPot);
    print(lastButt);
    prepareData();
    sendData();
  }

  //prepareData();
  //sendData();
  //receiveData();
}


// FUNKCJE
void print(int x) {
  switch (x) {
    case 0:  // przycisk 1 -> wybór trybu manualny/autonomiczny
      lcd.clear();
      lcd.setCursor(0, 0);
      if (pad.button[0] == 0)
        lcd.print("Manulany");
      else
        lcd.print("Automatyczny");
      break;
    case 1:  // przycisk 2
      lcd.clear();
      if (pad.button[0] == 0) {  // dla trybu manulanego
        if (pad.button[1] == 0) {
          lcd.setCursor(0, 0);
          lcd.print("Katy dziala");
          lcd.setCursor(0, 1);
          lcd.print(pad.pot.X);
          lcd.setCursor(5, 1);
          lcd.print(pad.pot.Y);
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Ladowanie kuli");
          lcd.setCursor(0, 1);
          lcd.print("(potwierdz)");
        }
      } else {  // dla trybu autonomicznego
        if (pad.button[1] == 0) {
          lcd.setCursor(0, 0);
          lcd.print("Polozenie celu");
          lcd.setCursor(0, 1);
          lcd.print(pad.pot.X);
          lcd.setCursor(5, 1);
          lcd.print(pad.pot.Y);
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Polozenie czolgu");
        }
      }
      break;
    case 2:
      lcd.clear();
      if (pad.button[0] == 0) {
        lcd.setCursor(0, 0);
        lcd.print("Strzal");
      } else {
        lcd.setCursor(0, 0);
        lcd.print("Start");
      }
      break;
    case 3:
      if (pad.button[0] == 0 && lastButt == 1) {
        if (pad.button[1] == 1) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Potwierdzono");
        }
      }
      break;
  }
  lastButt = x;
}
byte changeCondition(byte x, int num) {
  x++;
  if (x == num) {  // num trybów
    x = 0;
  }
  return x;
}
void prepareData(){
  payload.manual_auto = pad.button[0];
  payload.xJoy_none = pad.joy.X;
  payload.yJoy_none = pad.joy.Y;
  payload.fi_xTarget = pad.pot.X;
  payload.ro_yTarget = pad.pot.Y;
  payload.strike_start = pad.button[2];
  payload.load_none = pad.button[3];
  payload.none_giveFedbackPositon = 0;
}
void sendData() {
  //if (now - timeForUpload >= 70) {
    //timeForUpload = now;
    radio.stopListening();
    radio.write(&payload, sizeof(payload));
  //}
}
void receiveData() {
  if (now - gapInDownload >= 2000) {
    gapInDownload = millis();
    radio.startListening();
    timeForDownload = millis();
    while (!radio.available()) {
      if ((millis() - timeForDownload) >= 100) {
        break;
      }
    }
    if (radio.available()) {  // jeśli będzie zacinało to tutaj może warto dać while, ale to może opóźnić program
      radio.read(&tempR, sizeof(tempR));
      Serial.println(tempR);
    }
  }
}