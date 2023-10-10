#include <BMI160.h>
#include <BMI160Gen.h>
#include <CurieIMU.h>
#include <math.h>
#include <printf.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>


#define in1_L298N 2 // H
#define in2_L298N 3 // L -> obrót ze wskazówkami zegara silnika DC1
#define in3_L298N 4
#define in4_L298N 5


RF24 radio(7, 8);  // CE, CSN
VL53L0X sensor;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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
const int maxPWMserwo180 = 650; // skrajna pozycja 2 (650 z 4096)
const int minPWMserwo180 = 125; // skrajna pozycja 1 (125 z 4096)
const int midPWMservo360 = 374; // nie rusza się (374 z 4096)
unsigned long gapInFeedback = 0;
unsigned long mes = 0;
unsigned long time = 0;
bool startFlag = false;
bool canMoveFlag = false;
bool reachedPositionFlag = false;
bool lockedOnTargetFlag = false;
//bool cant strike = false;
int gx, gy, gz;  // wartości z żyroskopu
int ax, ay, az;  // wartości z akcelerometru (nieużywane)
int i = 0;
int gzSum = 0;
int timeSum = 0;
float gzAve = 0;
float currentRotation = 0;
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
void setPWM();

void setup() {
  pinMode(in1_L298N, OUTPUT);
  pinMode(in2_L298N, OUTPUT);
  pinMode(in3_L298N, OUTPUT);
  pinMode(in4_L298N, OUTPUT);

  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[0]);     // 00001
  radio.openReadingPipe(1, addresses[1]);  // 00002
  radio.setPALevel(RF24_PA_MIN);

  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  //sensor.setAddress(0x29);
  //sensor.writeReg(VL53L0X::SYSRANGE_START, 0x29);
  sensor.setTimeout(1000);
  while (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    //while (1) {}
  }
  sensor.startContinuous();

  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() {
  receiveData();
  getAccGyro();
  calculateCarPos();
  action();
  //Serial.println(sensor.getAddress());
  //lockOnTarget();
  lastFunction();
}

void receiveData() {
  radio.startListening();
  while (radio.available()) {               // if też działał
    radio.read(&payload, sizeof(payload));  //..read jak przeczyta to ustawia .available na false
  }
  
  /*Serial.print("joy: ");
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
  Serial.println();*/
}
void getAccGyro() {
  if(payload.strike_start == 1){currentRotation = 0;}

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
    currentRotation = currentRotation + timeSum * gzAve;
    if(currentRotation > 360 || currentRotation <-360){ currentRotation = 0;}
    gzSum = 0;
    timeSum = 0;
    i = 0;

    /*Serial.print("currentRotation: ");
    Serial.print(currentRotation);
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
      if (canMoveFlag){ driveToTarget(startsPayload); }
      if (reachedPositionFlag){ lockOnTarget(); }
      if (lockedOnTargetFlag){
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
  setPWM();
  Serial.print("joy:");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(motors.motorDC1);
  Serial.print("\t");
  Serial.print(motors.motorDC2);
  Serial.println();
}
void moveRifle() {
  motors.motorRifleRotation = payload.fi_xTarget;
  motors.motorRifleRaise = payload.ro_yTarget;
}
void driveToTarget(DataToReceive startsPayload) {
  Serial.println(180/3.14*atan2((double)startsPayload.fi_xTarget,(double)startsPayload.ro_yTarget));
  double finalRotation = 180/3.14*atan2((double)startsPayload.fi_xTarget,(double)startsPayload.ro_yTarget);

  if (currentRotation >= (finalRotation - 2) && currentRotation <= (finalRotation + 2)){  // tolerancja 2 deg
    motors.motorDC1 = 0;
    motors.motorDC2 = 0;
    reachedPositionFlag = true;
    canMoveFlag = false;
  }
  else if ((finalRotation - 2) > currentRotation){
    motors.motorDC1 = 10;
    motors.motorDC2 = -10;
  }
  else if((finalRotation + 2) < currentRotation){
    motors.motorDC1 = -10;
    motors.motorDC2 = 10;
  }
}
void calculateCarPos(){
  carCoords.X = sin(mapfloat(currentRotation, -360, 360, -6.28, 6.28)) * distance; // distance będzie zliczał obroty kół z długości PWM
  carCoords.Y = cos(mapfloat(currentRotation, -360, 360, -6.28, 6.28)) * distance;

  /*Serial.print("carCoords.X \t");
  Serial.print(carCoords.X);
  Serial.print("\t carCoords.Y \t");
  Serial.print(carCoords.Y);
  Serial.println();*/
}
void lockOnTarget(){  // namierz cel
  // procedura namierzania celu (obroty czujnikiem)
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  //delay(100);
  Serial.println();
  lockedOnTargetFlag = true;
  reachedPositionFlag = false;
}
void setPWM(){
  int temp;
  if(motors.motorDC1 == 0){
    digitalWrite(in1_L298N, LOW);
    digitalWrite(in2_L298N, LOW);
  }
  else if(motors.motorDC1 < 0){
    temp = map(motors.motorDC1, -4, -20, 0, 4000);
    digitalWrite(in1_L298N, LOW);
    digitalWrite(in2_L298N, HIGH);
    pwm.setPWM(0, 0, temp);
  }
  else{
    temp = map(motors.motorDC1, 4, 20, 0, 4000);
    digitalWrite(in1_L298N, HIGH);
    digitalWrite(in2_L298N, LOW);
    pwm.setPWM(0, 0, temp);
  }

  if(motors.motorDC2 == 0){
    digitalWrite(in3_L298N, LOW);
    digitalWrite(in4_L298N, LOW);
  }
  else if(motors.motorDC2 < 0){
    temp = map(motors.motorDC2, -20, 0, 0, 4000);
    digitalWrite(in3_L298N, LOW);
    digitalWrite(in4_L298N, HIGH);
    pwm.setPWM(1, 0, temp);
  }
  else{
    temp = map(motors.motorDC2, 0, 20, 0, 4000);
    digitalWrite(in3_L298N, HIGH);
    digitalWrite(in4_L298N, LOW);
    pwm.setPWM(1, 0, temp);
  }
  //Serial.println(temp);
}
void lastFunction(){
  payload.load_none = 0;
  payload.strike_start = 0;
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}