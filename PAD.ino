#include <SPI.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>


// DEFINICJE
#define pPotX A0    // pin potentiometr X
#define pPotY A1    // pin potencjometr Y
#define pJoyX A2    // pin joystick X
#define pJoyY A3    // pin joystick Y
#define pButt0 3
#define pButt1 4
#define pButt2 5
#define pButt3 6


// OBIEKTY
LiquidCrystal_I2C lcd(0x27, 16, 2);
RF24 radio(7, 8);  // CE, CSN


// STRUKTURY
struct Potentiometer {
  int X;
  int Y;
};
struct Joystick {
  int X;
  int Y;
};
struct Pad {
  Potentiometer pot;  // potencjometr
  Joystick joy;       // joystick
  byte button[4];     // przyciski
};
struct DataToSend{    // pakiet wysyłanych danych
  byte manual_auto;
  int xJoy_none;
  int yJoy_none;
  int fi_xTarget;
  int ro_yTarget;
  byte strike_start;
  byte load_none;
  byte none_giveFedbackPositon;
};


// ZMIENNE
Pad pad;
DataToSend payload;
const byte addresses[][6] = { "00001", "00002" };
const long minJoy = 0;
const long maxJoy = 100;
const long minPot = 0;
const long maxPot = 100;
unsigned long now = 0;
unsigned long delayButt0 = 0;
unsigned long delayButt1 = 0;
unsigned long delayButt2 = 0;
unsigned long delayButt3 = 0;
unsigned long timeForDownload = 0;
unsigned long timeForUpload = 0;
unsigned long gapInDownload = 0;
unsigned long gapInFeedback = 0;
byte lastButt;
int tempW = 1;
int tempR = 0;


// DEFINICJE FUNKCJI
void ifPress(int pButt, unsigned long & delayButt, byte num, byte cCnum);
void ifShift();
void giveMeCarPosition();
void print(int x);
byte changeCondition(byte x, int num);
void prepareData();
void sendData();
void receiveData();

// PROGRAM
void setup() {
  pinMode(pButt0, INPUT_PULLUP);    // podciągania pinów do +5V
  pinMode(pButt1, INPUT_PULLUP);
  pinMode(pButt2, INPUT_PULLUP);
  pinMode(pButt3, INPUT_PULLUP);

  Serial.begin(9600);               // rozpoczęcie komunikacji z komputerem

  lcd.init();                       // ustawienia początkowe wyświetlacza
  lcd.backlight();

  radio.begin();                            // ustawienia transmisji bezprzewodowej
  radio.openWritingPipe(addresses[1]);      // 00002
  radio.openReadingPipe(1, addresses[0]);   // 00001
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
  now = millis();

  ifPress(pButt0, delayButt0, 0, 2);    // gdy wciśnięto jakiś przycisk
  ifPress(pButt1, delayButt1, 1, 2);
  ifPress(pButt2, delayButt2, 2, 1);
  ifPress(pButt3, delayButt3, 3, 1);
  ifShift();                            // gdy ruszono potencjometrem lub joystickiem
  giveMeCarPosition();                  // co 1 sekundę otrzymuje pozycję pojazdu
}


// FUNKCJE
void ifPress(int pButt, unsigned long & delayButt, byte num, byte cCnum){
  if ((digitalRead(pButt) == 0) && (now - delayButt >= 500)) {  // wciśnięcie przycisku nr1 -> pButt0 = 0
    delayButt = now;
    pad.button[num] = changeCondition(pad.button[num], cCnum);
    if (pButt == pButt3 && pad.button[0] == 0 && lastButt == 1 && pad.button[1] == 1){  // gdy włączono M-ładowanie
      payload.load_none = 1;
      sendData();             // wysyła '1' jako load_none
    }
    else if (pButt == pButt2){  // gdy włączono M-ładowanie
      payload.strike_start = 1;
      sendData();             // wysyła '1' jako load_none
    }
    else{
      prepareData();
      sendData();
    }
    print(num);
    
  }
}
void ifShift(){
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
}
void giveMeCarPosition(){
  if(payload.manual_auto == 1 && now - gapInFeedback >= 1000){
    gapInFeedback = now;
    prepareData();
    payload.none_giveFedbackPositon = 1;
    sendData();
    receiveData();
    payload.none_giveFedbackPositon = 0;
  }
}
void print(int x) {
  switch (x) {
    case 0:  // przycisk 0 -> wybór trybu manualny/autonomiczny
      lcd.clear();
      lcd.setCursor(0, 0);
      if (pad.button[0] == 0)
        lcd.print("Manulany");
      else
        lcd.print("Automatyczny");
      break;
    case 1:  // przycisk 1 -> menu 
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
      } else {                  // dla trybu autonomicznego
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
    case 2:   // przycisk 2 -> strzał lub start
      lcd.clear();
      if (pad.button[0] == 0) {
        lcd.setCursor(0, 0);
        lcd.print("Strzal");
      } else {
        lcd.setCursor(0, 0);
        lcd.print("Start");
      }
      break;
    case 3:   // przycisk 3 -> potwierdzanie (jeśli wymagane) [wymagane w M-ładowanie]
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
  payload.strike_start = 0; // w ifPress() może przyjąć '1'
  payload.load_none = 0;    // w ifPress() może przyjąć '1'
  payload.none_giveFedbackPositon = 0;
}
void sendData() {
    radio.stopListening();
    radio.write(&payload, sizeof(payload));
}
void receiveData() {
    timeForDownload = millis();
    radio.startListening();
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