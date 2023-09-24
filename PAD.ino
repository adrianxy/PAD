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
#define pButt1 3
#define pButt2 4
#define pButt3 5
#define pButt4 6

// OBIEKTY
LiquidCrystal_I2C lcd(0x27, 16, 2);
RF24 radio(9, 10);  // CE, CSN


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
  byte manual0autonomic1;  // czy włączono tryb autonomiczny
  Potentiometer pot;
  Joystick joy;
  byte button[4];  // przyciski
} pad;

int panelNumber;  // numer wyświetlanego panelu
unsigned long now = 0;
unsigned long delayJoy = 0;
unsigned long delayButt1 = 0;
unsigned long delayButt2 = 0;
unsigned long delayButt3 = 0;
unsigned long delayButt4 = 0;
unsigned long delayPrint = 0;
const long minJoy = -50;
const long maxJoy = 50;
const long minPot = -50;
const long maxPot = 50;
const byte address[6] = "00001";  // adresy połączeń
int temp;
byte lastButt;


// DEFINICJE FUNKCJI
void print();
void print(int x);
byte changeCondition(byte x, int num);

// PROGRAM
void setup() {
  pinMode(pJoyButt, INPUT_PULLUP);
  pinMode(pButt1, INPUT_PULLUP);
  pinMode(pButt2, INPUT_PULLUP);
  pinMode(pButt3, INPUT_PULLUP);
  pinMode(pButt4, INPUT_PULLUP);

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  now = millis();

  if ((digitalRead(pButt1) == 0) && (now - delayButt1 >= 500)) {  // wciśnięcie przycisku nr1 -> pButt1 = 0
    delayButt1 = now;
    pad.manual0autonomic1 = changeCondition(pad.manual0autonomic1, 2);  // 2 tryby ->  manualny-0 | automatyczny-1
    print(0);
    Serial.println(pad.manual0autonomic1);
  }
  if ((digitalRead(pButt2) == 0) && (now - delayButt2 >= 500)) {  // wciśnięcie przycisku nr2 -> pButt2 = 0
    delayButt2 = now;
    pad.button[1] = changeCondition(pad.button[1], 1);  // 2 tryby ->  manualny-0 | automatyczny-1
    print(1);
    Serial.println("2");
  }
  if ((digitalRead(pButt3) == 0) && (now - delayButt3 >= 500)) {  // wciśnięcie przycisku nr3 -> pButt3 = 0
    delayButt3 = now;
    Serial.println("3");
  }
  if ((digitalRead(pButt4) == 0) && (now - delayButt4 >= 500)) {  // wciśnięcie przycisku nr4 -> pButt4 = 0
    delayButt4 = now;
    Serial.println("4");
  }

  if ((digitalRead(pJoyButt) == 0) && (now - delayJoy >= 500)) {  // wciśnięcie joysticka -> pJoyButt = 0
    delayJoy = now;
    panelNumber++;
    if (panelNumber == 2) {  // 2 panele
      panelNumber = 0;
    }
    print();
  }

  if ((abs(pad.joy.X - map(analogRead(pJoyX), 0, 1023, minJoy, maxJoy)) > 1) || (abs(pad.joy.Y - map(analogRead(pJoyY), 0, 1023, minJoy, maxJoy)) > 1)) {  // obsługa joystick'a
    pad.joy.X = map(analogRead(pJoyX), 0, 1023, minJoy, maxJoy);
    pad.joy.Y = map(analogRead(pJoyY), 0, 1023, minJoy, maxJoy);
    print();
  }
  if ((abs(pad.pot.X - map(analogRead(pPotX), 0, 1023, minPot, maxPot)) > 1) || (abs(pad.pot.Y - map(analogRead(pPotY), 0, 1023, minPot, maxPot)) > 1)) {  // obsługa joystick'a
    pad.pot.X = map(analogRead(pPotX), 0, 1023, minPot, maxPot);
    pad.pot.Y = map(analogRead(pPotY), 0, 1023, minPot, maxPot);
    print(lastButt);
  }
  //Serial.println(analogRead(1));
}


// FUNKCJE
void print() {
  /*Tryb Automatyczny{
    panel 1: współrzędnę celu z potencjometrów
    panel 2: położenie robota wzglęgam startu
  }
  Tryb Ręczny{
    panel 1: kąt obrotu działka
    panel 2: kąt nachylenia działka
  }*/
  //lcd.clear();

  /*if (now - delayPrint >= 30) {
    delayPrint = now;
    lcd.setCursor(0, 1);
    lcd.print("                ");

    switch (panelNumber) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("Cel");
        lcd.setCursor(0, 1);
        lcd.print("[dm]");

        lcd.setCursor(5, 0);
        lcd.print("PotX:");

        lcd.setCursor(5, 1);
        lcd.print(pad.pot.X);

        lcd.setCursor(11, 0);
        lcd.print("PotY:");

        lcd.setCursor(11, 1);
        lcd.print(pad.pot.Y);
        break;

      case 1:
        lcd.setCursor(0, 0);
        lcd.print("Joy");

        lcd.setCursor(5, 0);
        lcd.print("JoyX:");

        lcd.setCursor(5, 1);
        lcd.print(pad.joy.X);

        lcd.setCursor(11, 0);
        lcd.print("JoyY:");

        lcd.setCursor(11, 1);
        lcd.print(pad.joy.Y);
        break;
    }
  }*/
}
void print(int x){
  lcd.clear();
  switch(x){
    case 0:
      lcd.setCursor(0, 0);
      if (pad.manual0autonomic1 == 0)
        lcd.print("Manulany");
      else
        lcd.print("Automatyczny");
    break;
    case 1:
      if(pad.manual0autonomic1 == 0){
        lcd.setCursor(0, 0);
        lcd.print("Działo - kąty");
        lcd.setCursor(0, 1);
        lcd.print(pad.pot.X);
      }
    break;
  }
  lastButt = x;
}

byte changeCondition(byte x, int num){
  x++;
  if (x == num) {  // num trybów 
    x = 0;
  }
  return x;
}