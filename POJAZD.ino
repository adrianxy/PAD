#include <BMI160.h>
#include <BMI160Gen.h>
#include <CurieIMU.h>

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
};

struct CarCoords{
  double X;
  double Y;
};

DataToReceive payload;
Motors motors;
CarCoords carCoords;
const byte addresses[][6] = { "00001", "00002" };
unsigned long gapInFeedback = 0;
unsigned long mes = 0;
unsigned long time = 0;
bool startFlag = false;
//bool cant strike = false;
int gx, gy, gz;  // wartości z żyroskopu
int ax, ay, az;  // wartości z akcelerometru (nieużywane)
int i = 0;
int gzSum = 0;
int timeSum = 0;
float gzAve = 0;
float rotation = 0;
float distance = 10;



void receiveData();
void sendData();
void getAccGyro();
void action();
void moveCar();
void moveRifle();
void calculateCarPos();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

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



  /*Serial.print("joy:");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(payload.xJoy_none);
  Serial.print("\t");
  Serial.print(payload.yJoy_none);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(payload.fi_xTarget);
  Serial.print("\t");
  Serial.print(payload.ro_yTarget);
  Serial.println();*/
}

void receiveData() {
  radio.startListening();
  while (radio.available()) {               // if też działał
    radio.read(&payload, sizeof(payload));  //..read jak przeczyta to ustawia .available na false
    //Serial.println(payload.strike_start);
  }
}
void getAccGyro() {
  if(payload.strike_start == 1){rotation = 0;}

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
    rotation = rotation + timeSum * gzAve;
    if(rotation > 360 || rotation <-360){ rotation = 0;}
    gzSum = 0;
    timeSum = 0;
    i = 0;

    Serial.print("rotation: ");
    Serial.print(rotation);
    Serial.print("\t\t");

    Serial.print("gz: ");
    Serial.print(gz);
    Serial.println();
  }
}
void sendData() {
  radio.stopListening();
  radio.write(&carCoords, sizeof(carCoords));
}
void action() {
  if (payload.manual_auto == 0) {  // manualny
    moveCar();
    moveRifle();
    if (payload.strike_start == 1) {
      // rozpocznij proceduję strzału
    }
    if (payload.load_none == 1) {
      // rozpocznij precedurę załadunku
    }
  } else {  // automatyczny
    if (payload.strike_start == 1 || startFlag) {
      trackToTarget();
    }
    if (payload.none_giveFedbackPositon == 1) {
      //delay(50);
      if (millis() - gapInFeedback >= 500) {
        gapInFeedback = millis();
        sendData();
      }
    }
  }
}
void moveCar() {
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
  /*
  Serial.print("joy:");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(motors.motorDC1);
  Serial.print("\t");
  Serial.print(motors.motorDC2);
  Serial.println();*/
}
void moveRifle() {

}
void trackToTarget() {

}
void calculateCarPos(){
  carCoords.X = sin(mapfloat(rotation, -360, 360, -6.28, 6.28)) * distance; // idstance będzie zliczał obroty kół z długości PWM
  carCoords.Y = cos(mapfloat(rotation, -360, 360, -6.28, 6.28)) * distance;
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}