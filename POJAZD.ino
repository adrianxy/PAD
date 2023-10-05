#include <BMI160.h>
#include <BMI160Gen.h>
#include <CurieIMU.h>
#include <math.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define pTrig 3
#define pEcho 2

RF24 radio(7, 8);  // CE, CSN

const int i2c_addr = 0x68;

struct DataToReceive {
  byte manual_auto;
  int xJoy_none;
  int yJoy_none;
  int fi_xTarget;
  int ro_yTarget;
  byte strike_start;
  byte load_none;
  byte none_giveFedbackPositon;
};

struct Motors {
  long motorDC1;
  long motorDC2;
  long motorRifleRotation;
  long motorRifleRaise;
};

struct CarCoords{
  double X;
  double Y;

};

DataToReceive payload;
DataToReceive startsPayload; // po wciśnięciu start zapamietuje nastawy
Motors motors;
CarCoords carCoords;
const byte addresses[][6] = { "00001", "00002" };
unsigned long gapInFeedback = 0;
unsigned long mes = 0;
unsigned long time = 0;
bool startFlag = false;
bool reachedPositionFlag = false;
bool lockedOnTargetFlag = false;
//bool cant strike = false;
int gx, gy, gz;  // wartości z żyroskopu
int ax, ay, az;  // wartości z akcelerometru (nieużywane)
int i = 0;
int gzSum = 0;
int timeSum = 0;
float gzAve = 0;
float actualRotation = 0;
float distance = 10;


void receiveData();
void sendData();
void getAccGyro();
void action();
void driveToTarget(DataToReceive startsPayload);
void moveCarManual();
void moveRifle();
void calculateCarPos();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void lockOnTarget();
void lastFunction();

void setup() {
  pinMode(pTrig, OUTPUT);
  pinMode(pEcho, INPUT);

  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[0]);     // 00001
  radio.openReadingPipe(1, addresses[1]);  // 00002
  radio.setPALevel(RF24_PA_MIN);

  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
}

void loop() {
  receiveData();
  getAccGyro();
  calculateCarPos();
  action();

  lastFunction();
}

void receiveData() {
  radio.startListening();
  while (radio.available()) {               // if też działał
    radio.read(&payload, sizeof(payload));  //..read jak przeczyta to ustawia .available na false
  }
  Serial.print("joy: ");
  Serial.print(payload.xJoy_none);
  Serial.print("\t");
  Serial.print(payload.yJoy_none);
  Serial.print("\t");
  Serial.print("pot: ");
  Serial.print(payload.fi_xTarget);
  Serial.print("\t");
  Serial.print(payload.ro_yTarget);
  Serial.print("\t");
  Serial.print("manual_auto: ");
  Serial.print(payload.manual_auto);
  Serial.print("\t");
  Serial.print("strike_start: ");
  Serial.print(payload.strike_start);
  Serial.print("\t");
  Serial.print("load_none: ");
  Serial.print(payload.load_none);
  Serial.print("\t");
  Serial.print("none_giveFedbackPositon: ");
  Serial.print(payload.none_giveFedbackPositon);
  Serial.println();
}
void getAccGyro() {
  if(payload.strike_start == 1){actualRotation = 0;}

  time = millis() - mes;  // pomiar czasu między pomiarami
  mes = millis();
  BMI160.readGyro(gx, gy, gz);

  if (abs(map(gz, -32768, 32768, 250, -250)) > 3) { // zbieranie informacji z żyroskopu 
    gz = map(gz, -32768, 32768, 250, -250);
  } else {
    gz = 0;
  }

  if (i < 8) {    // obliczanie obrotu
    gzSum = gzSum + gz;
    timeSum = timeSum + time;
    i++;
  } else {
    gzAve = gzSum;  
    gzAve = gzAve / 8 / 1000; // średnia prędkość obrotu z 5 próbek i zamiana na [deg/ms]
    actualRotation = actualRotation + timeSum * gzAve;
    if(actualRotation > 360 || actualRotation <-360){ actualRotation = 0;}
    gzSum = 0;
    timeSum = 0;
    i = 0;

    /*Serial.print("actualRotation: ");
    Serial.print(actualRotation);
    Serial.print("\t\t");

    Serial.print("gz: ");
    Serial.print(gz);
    Serial.println();*/
  }
}
void sendData() {
  radio.stopListening();
  radio.write(&carCoords, sizeof(carCoords));
}
void action() {
  if (payload.manual_auto == 0) {  // manualny
    moveCarManual();
    moveRifle();
    if (payload.strike_start == 1) {
      // rozpocznij proceduję strzału
    }
    if (payload.load_none == 1) {
      // rozpocznij precedurę załadunku
    }
  } 
  else {  // automatyczny
    if (payload.strike_start == 1 || startFlag) {
      if(!startFlag){
        startsPayload = payload;  // gdy dopiero wystartował to zapisuje ustawienia na których będzie działał automat
      }
      startFlag = true;
      driveToTarget(startsPayload);
      if (reachedPositionFlag){
        lockOnTarget();
      }
      else if(lockedOnTargetFlag){
        // rozpocznij procedurę strzału
        // rozpocznij procedurę załadunku
      }
    }
    if (payload.none_giveFedbackPositon == 1) {
      if (millis() - gapInFeedback >= 500) {
        gapInFeedback = millis();
        sendData();
      }
    }
  }
}
void moveCarManual() {
  if (payload.xJoy_none >= 0 && payload.yJoy_none >= 0) {
    motors.motorDC1 = max(payload.xJoy_none, payload.yJoy_none);
    motors.motorDC2 = map(payload.xJoy_none, 0, 20, motors.motorDC1, -motors.motorDC1);
    if (payload.yJoy_none == 0) { motors.motorDC2 = -motors.motorDC1; }
  } else if (payload.xJoy_none <= 0 && payload.yJoy_none >= 0) {
    motors.motorDC2 = max(abs(payload.xJoy_none), payload.yJoy_none);
    motors.motorDC1 = map(payload.xJoy_none, 0, -20, motors.motorDC2, -motors.motorDC2);
    if (payload.yJoy_none == 0) { motors.motorDC1 = -motors.motorDC2; }
  } else if (payload.xJoy_none <= 0 && payload.yJoy_none <= 0) {
    motors.motorDC2 = min(payload.xJoy_none, payload.yJoy_none);
    motors.motorDC1 = map(payload.xJoy_none, 0, -20, motors.motorDC2, -motors.motorDC2);
    if (payload.yJoy_none == 0) { motors.motorDC1 = motors.motorDC2; }
  } else {
    motors.motorDC1 = min(-payload.xJoy_none, payload.yJoy_none);
    motors.motorDC2 = map(payload.xJoy_none, 0, 20, motors.motorDC1, -motors.motorDC1);
    if (payload.yJoy_none == 0) { motors.motorDC2 = motors.motorDC1; }
  }
  if (abs(motors.motorDC1) <= 4 || abs(motors.motorDC2) <= 4){
    motors.motorDC1 = 0;
    motors.motorDC2 = 0;
  }
  
  /*Serial.print("joy:");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(motors.motorDC1);
  Serial.print("\t");
  Serial.print(motors.motorDC2);
  Serial.println();*/
}
void moveRifle() {
  motors.motorRifleRotation = payload.fi_xTarget;
  motors.motorRifleRaise = payload.ro_yTarget;
}
void driveToTarget(DataToReceive startsPayload) {
  //Serial.println(180/3.14*atan2((double)startsPayload.fi_xTarget,(double)startsPayload.ro_yTarget));
  double finalRotation = 180/3.14*atan2((double)startsPayload.fi_xTarget,(double)startsPayload.ro_yTarget);

  // silnikiDC(20,-20) albo silnikiDC(-20,20)

  if (actualRotation = finalRotation){ reachedPositionFlag = true; }
}
void calculateCarPos(){
  carCoords.X = sin(mapfloat(actualRotation, -360, 360, -6.28, 6.28)) * distance; // idstance będzie zliczał obroty kół z długości PWM
  carCoords.Y = cos(mapfloat(actualRotation, -360, 360, -6.28, 6.28)) * distance;
}
void lockOnTarget(){  // namierz cel
  // procedura namierzania celu
  lockedOnTargetFlag = true;
  reachedPositionFlag = false;
}
void lastFunction(){
  payload.load_none = 0;
  payload.strike_start = 0;
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}