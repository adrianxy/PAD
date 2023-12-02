/*
Autor: Adrian Nowogrodzki
Data: 11.11.2023
Temat: Zdalnie sterowana platforma gąsienicowa strzelająca do zadanego celu
Wersja środowiska: Arduino IDE 2.1.1
*/

#include <math.h>
#include <printf.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>


// DEFINICJE
#define in1_L298N 2              // | 10 - 13 w leonardo
#define in2_L298N 3             // |
#define in3_L298N 4              // |
#define in4_L298N 5              // | kontrola obrotów silników DC 1 i 2
#define CE_NRF24 7               // |
#define CSN_NRF24 8              // |
#define MOSI_NRF24 11            // |
#define MISO_NRF24 12            // |
#define CSK_NRF24 13             // | obsługa NRF24 dla UNO
#define interruptLeftEncoder 0   // | enkoder 1 - 400 impulsów na obrót
#define interruptRightEncoder 1  // | enkoder 2

// DEFINICJE FUNCKJI
 float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

// OBIEKTY Z INNYCH PLIKÓW
 RF24 radio(CE_NRF24, CSN_NRF24); // NRF24
 VL53L0X sensor;   // czujnik 
 Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// STRUKTURY
 struct ReceivedData {
  byte manual_auto;
  int xJoy_none;
  int yJoy_none;
  int fi_xTarget;
  int ro_yTarget;
  byte strike_start;
  byte load_none;
  byte none_giveFedbackPositon;
 };

 struct CarCoords {
  float X;
  float Y;
 };

 struct Time {
  unsigned long gapInFeedback = 0;          // timer - co jaki czas można wysłać dane o położeniu
 };

  struct Obstacle{
    int firstEdgeAngle;
    int secondEdgeAngle;
    int centerAngle;
    float distance;
  };
// KLASY
class Wheel {
  public:
    Wheel(float radius, int in1, int in2, int pwmPinNum);
    void setPower(int percent);  // [-100% - 100%]
    int getTurningDirection();
    float getDistance();
    float getVelocity();
    void increaseOrDecreaseEncoder(int plusOrMinus);
    void clearStoragedData();
  private:
    float _radius;  //[cm]
    int _in1;
    int _in2;
    int _speedPercent;
    float _distance;  // [cm]
    float _velocity;  // [cm/ms]
    long _encoderPulsesCount; // może zbędne
    unsigned long _dTimeOfMeasurement;
    unsigned long _timeOfMeasurement;
    int _pulsesPerOneRotation = 400;
    int _pwmPinNum;
 };
 Wheel::Wheel(float radius, int in1, int in2, int pwmPinNum) {
  _radius = radius;
  _in1 = in1;
  _in2 = in2;
  _pwmPinNum = pwmPinNum;
 }
 void Wheel::setPower(int percent) {
   _speedPercent = percent;
  if (percent == 0) {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    pwm.setPWM(_pwmPinNum, 0, 0);
  } else if (percent < 0) {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    pwm.setPWM(_pwmPinNum, 0, map(percent, 0, -100, 0, 3800));
  } else {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
    pwm.setPWM(_pwmPinNum, 0, map(percent, 0, 100, 0, 3800));
  }
 }
 int Wheel::getTurningDirection() {
  if (_speedPercent < 0) {
    return (-1);
  } else if (_speedPercent > 0) {
    return (1);
  };
 }
 float Wheel::getDistance() {
  return (_distance);
 }
 float Wheel::getVelocity() {
  if (_dTimeOfMeasurement != 0) {
    _velocity = 2 * M_PI * _radius / _dTimeOfMeasurement;
  } else {
    _velocity = 0;
  }
  return (_velocity);
 }
 void Wheel::increaseOrDecreaseEncoder(int plusOrMinus) {
  _dTimeOfMeasurement = millis() - _timeOfMeasurement;
  _timeOfMeasurement = millis();
  _encoderPulsesCount = _encoderPulsesCount + plusOrMinus;
  _distance = _distance + (float)plusOrMinus * 2 * M_PI * _radius / (float)_pulsesPerOneRotation;
 }
 void Wheel::clearStoragedData() {
  _dTimeOfMeasurement = 0;
  _timeOfMeasurement = 0;
  _encoderPulsesCount = 0;
  _distance = 0;
  _velocity = 0;
}

class Rifle {
 public:
  Rifle();
  void setYaw(int angle);
  void setPitch(int angle);
  void shoot();
  void reload();
 private: 
 };
 Rifle::Rifle() {
 }
 void Rifle::setYaw(int angle) {
  pwm.setPWM(2, 0, map(angle, 0, 180, 125, 650)); // wartości "angle" od 0 - 180 deg
 }
 void Rifle::setPitch(int angle) {
  pwm.setPWM(3, 0, map(angle, 0, 180, 125, 650)); // wartości "angle" od 0 - 90 deg
 }
 void Rifle::shoot() {
  /* rozpocznij proceduję strzału*/
 }
 void Rifle::reload() {
  /* rozpocznij precedurę załadunku*/
}

class Radar {
  public:
    Radar();
    void rotate(int angle);
    float takeMeasurement();
    void clearData();
  private:
    float _distance;
    int _angle;
 };
 Radar::Radar(){
 }
 void Radar::rotate(int angle){
   pwm.setPWM(7, 0,  map(angle, 0, 180, 125, 650));
 }
 float Radar::takeMeasurement(){
   _distance = sensor.readRangeSingleMillimeters()/10;
   if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
   return(_distance);
 }
 void Radar::clearData(){
   _distance =0;
   _angle = 0;
}


// OBIEKTY Z TEGO PLIKU
 Wheel leftWheel(3, in1_L298N, in2_L298N, 0);
 Wheel rightWheel(3, in3_L298N, in4_L298N, 1);
 Rifle rifle;
 Radar radar; 


// ZMIENNE
 const byte pipe_address[][6] = { "00001", "00002" };  // adresy kanałów przesyłowych NRF24
 const int i2c_addr = 0x68;                            // adres żyroskopu
 ReceivedData receivedPayload;
 ReceivedData startsPayload;
 CarCoords carCoords;
 Time time;
 Obstacle obstacle;
 bool startFlag;
 bool currentEqualsFinalRotationFlag = false;
 bool driveToTargetFlag = false; // flaga - czy można jechać do celu
 bool lockOnTargetFlag = false; // flaga - czy można namierzyć cel
 bool moveRifleFlag = false;
 bool strikeFlag = false; // flaga - czy można wystrzelić pocisk
 bool reloadFlag = false; // flaga - czy można załadować pocisk
 float finalRotation;
 float finalDistance;
 int tempAngleInLOT;
 float tempDistanceInLOT, tempBeforeDistanceInLOT;
 bool searchingFlag = false;

// PROGRAM
void setup() {
  //attachInterrupt(digitalPinToInterrupt(interruptLeftEncoder), leftEncoderInteruptFunction, RISING);
  //attachInterrupt(digitalPinToInterrupt(interruptRightEncoder), rightEncoderInteruptFunction, RISING);

  pinMode(in1_L298N, OUTPUT);
  pinMode(in2_L298N, OUTPUT);
  pinMode(in3_L298N, OUTPUT);
  pinMode(in4_L298N, OUTPUT);

  Serial.begin(9600);
  Wire.begin();
  radio.begin();
  radio.openWritingPipe(pipe_address[0]);
  radio.openReadingPipe(1, pipe_address[1]);
  radio.setPALevel(RF24_PA_MIN);

  /*sensor.setTimeout(500);
  if (!sensor.init()){
    Serial.println("sensor cant init");
  }*/

  pwm.begin();
  pwm.setPWMFreq(60);
}
void loop() {
  receiveData();
  int serw = map(receivedPayload.fi_xTarget, 0, 180, 2, 8);
  int powe = map(receivedPayload.ro_yTarget, 0, 90, 125, 650);
  pwm.setPWM(serw, 0, powe);
  Serial.print("serwo: ");
  Serial.print(serw);
  Serial.print(" power: ");
  Serial.print(powe);
  Serial.println();
  //action();
  //lastFunction();
  
  //Serial.println(radar.takeMeasurement());
}

void receiveData() {
  radio.startListening();                                   // nasłuchuj
  while (radio.available()) {                               // dopóki jest co czytać, to
    radio.read(&receivedPayload, sizeof(receivedPayload));  // czytaj, jak przeczytasz to ustaw radio.available na false
    Serial.println(1);
  }
 /*
  Serial.print("joy: ");
  Serial.print(receivedPayload.xJoy_none);
  Serial.print("\t");
  Serial.print(receivedPayload.yJoy_none);
  Serial.print("\t");
  Serial.print("pot: ");
  Serial.print(receivedPayload.fi_xTarget);
  Serial.print("\t");
  Serial.print(receivedPayload.ro_yTarget);
  Serial.print("\t");
  Serial.print("manual_auto: ");
  Serial.print(receivedPayload.manual_auto);
  Serial.print("\t");
  Serial.print("strike_start: ");
  Serial.print(receivedPayload.strike_start);
  Serial.print("\t");
  Serial.print("load_none: ");
  Serial.print(receivedPayload.load_none);
  Serial.print("\t");
  Serial.print("none_giveFedbackPositon: ");
  Serial.print(receivedPayload.none_giveFedbackPositon);
  Serial.println();*/
}
void action() {
  switch (receivedPayload.manual_auto) {
    // MANUALNY
    case 0:
      leftWheel.clearStoragedData();
      rightWheel.clearStoragedData();
      moveCarManual();
      moveRifleManual();
      if (receivedPayload.load_none == 1) { rifle.reload(); }      
      if (receivedPayload.strike_start == 1) { rifle.shoot(); }

      startFlag = false;  // resetuje tryb autonomiczny
      break;
    // AUTONOMICZNY
    case 1:
      if (receivedPayload.strike_start == 1 || startFlag) {
        if (!startFlag) {
          startsPayload = receivedPayload;  // gdy dopiero wystartował to zapisuje ustawienia na których będzie działał automat
          startFlag = true;
          driveToTargetFlag = true;
        }
        if (receivedPayload.none_giveFedbackPositon == 1 && (millis() - time.gapInFeedback >= 500)) {
          time.gapInFeedback = millis();
          calculateCarPosition();
          giveFeedback();
        }
        if (driveToTargetFlag){ moveCarAutonomical(startsPayload); }
        //if (lockOnTargetFlag){ lockOnTarget(); }
        //if (moveRifleFlag) { moveRifleAutonomical(); }
        //if (reloadFlag){ /* rozpocznij procedurę załadunku + startFlag=false */ }
        //if (strikeFlag){ rifle.shoot(); }

      }
      break;
  }
}
void moveCarManual() {
  // przeliczanie pozycji joysticka na napędy silników DC 1 i 2
  int _leftWheelSpeed, _rightWheelSpeed;

  if (receivedPayload.xJoy_none >= 0 && receivedPayload.yJoy_none >= 0) {
    _leftWheelSpeed = max(receivedPayload.xJoy_none, receivedPayload.yJoy_none);
    _rightWheelSpeed = map(receivedPayload.xJoy_none, 0, 100, _leftWheelSpeed, -_leftWheelSpeed);
    if (abs(receivedPayload.yJoy_none) < 2) { _rightWheelSpeed = -_leftWheelSpeed; }
  } 
  else if (receivedPayload.xJoy_none <= 0 && receivedPayload.yJoy_none >= 0) {
    _rightWheelSpeed = max(abs(receivedPayload.xJoy_none), receivedPayload.yJoy_none);
    _leftWheelSpeed = map(receivedPayload.xJoy_none, 0, -100, _rightWheelSpeed, -_rightWheelSpeed);
    if (abs(receivedPayload.yJoy_none) < 2) { _leftWheelSpeed = -_rightWheelSpeed; }
  } 
  else if (receivedPayload.xJoy_none <= 0 && receivedPayload.yJoy_none <= 0) {
    _rightWheelSpeed = min(receivedPayload.xJoy_none, receivedPayload.yJoy_none);
    _leftWheelSpeed = map(receivedPayload.xJoy_none, 0, -100, _rightWheelSpeed, -_rightWheelSpeed);
    if (abs(receivedPayload.yJoy_none) < 2) { _leftWheelSpeed = _rightWheelSpeed; }
  } 
  else {
    _leftWheelSpeed = min(-receivedPayload.xJoy_none, receivedPayload.yJoy_none);
    _rightWheelSpeed = map(receivedPayload.xJoy_none, 0, 100, _leftWheelSpeed, -_leftWheelSpeed);
    if (abs(receivedPayload.yJoy_none) < 2) { _rightWheelSpeed = _leftWheelSpeed; }
  }

  if (abs(_leftWheelSpeed) <= 10) {  // niewrażliwość na wartości joysticka mniejsze od 10%
    _leftWheelSpeed = 0;
  } 
  if (abs(_rightWheelSpeed) <= 10) {
    _rightWheelSpeed = 0;
  }

  // Wysterowanie sterownika i róch kół
  leftWheel.setPower(_leftWheelSpeed);
  rightWheel.setPower(_rightWheelSpeed);

  /*Serial.print("wheel speed:");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(_leftWheelSpeed);
  Serial.print("\t");
  Serial.print(_rightWheelSpeed);
  Serial.println();*/
}
void moveRifleManual(){
  rifle.setYaw(receivedPayload.fi_xTarget);
  rifle.setPitch(receivedPayload.ro_yTarget);
}
void calculateCarPosition(){
  if(currentEqualsFinalRotationFlag){
    carCoords.X = sin(mapfloat(finalRotation, -360, 360, -6.28, 6.28)) * leftWheel.getDistance(); // distance będzie zliczał obroty kół z długości PWM
    carCoords.Y = cos(mapfloat(finalRotation, -360, 360, -6.28, 6.28)) * rightWheel.getDistance();
  }
  else{
    carCoords.X = 0;
    carCoords.Y = 0;
  }
  
  /*Serial.print("carCoords.X \t");
  Serial.print(carCoords.X);
  Serial.print("\t carCoords.Y \t");
  Serial.print(carCoords.Y);
  Serial.println();*/
}
void giveFeedback(){
  radio.stopListening();                      // sprzestań nasłuchiwać
  //carCoords.X = 21; // dodatek
  //Serial.println("wysłano"); // dodatek
  radio.write(&carCoords, sizeof(carCoords)); // wyślij współrzędne pojazdu
}
void moveCarAutonomical(ReceivedData startsPayload) {
  finalRotation = 180/M_PI*(float)atan2((double)startsPayload.fi_xTarget,(double)startsPayload.ro_yTarget); // [deg] kąt obrotu o jaki trzeba się obrócić żeby być na lini z celem 
  finalDistance = (float)sqrt(pow((double)receivedPayload.fi_xTarget,2) + pow((double)receivedPayload.ro_yTarget,2)); // [cm]
  
  /*Serial.print("finalRotation: \t");
  Serial.print(finalRotation);
  Serial.print("\t finalDistance: \t");
  Serial.print(finalDistance);
  Serial.println();*/

  if(currentEqualsFinalRotationFlag){
    if (leftWheel.getDistance() < (finalDistance - 100)){ // "-10" bo zatrzymanie ma być w odległości 1 metra od celu
      leftWheel.setPower(70);
      rightWheel.setPower(70);
    }
    else{
      leftWheel.setPower(0);
      rightWheel.setPower(0);
      lockOnTargetFlag = true;
      driveToTargetFlag = false;
      currentEqualsFinalRotationFlag = false;
      searchingFlag = true;
    }
  }
  else{
    float c = 0.33;  // współczynnik obrotu
    if(finalRotation > 0){  // zgodnie ze wskazówkami zegara
      if(leftWheel.getDistance() >= finalRotation * c){
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        leftWheel.clearStoragedData();
        rightWheel.clearStoragedData();
        currentEqualsFinalRotationFlag = true;
      }
      else{
        leftWheel.setPower(100);
        rightWheel.setPower(-100);
      }    
    }
    else if(finalRotation < 0){ // przeciwnie do wskazówek zegara
      if(leftWheel.getDistance() <= finalRotation * c){
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        leftWheel.clearStoragedData();
        rightWheel.clearStoragedData();
        currentEqualsFinalRotationFlag = true;
      }
      else{
        leftWheel.setPower(-100);
        rightWheel.setPower(100);
      }
    }
  }    
}
void lockOnTarget(){
 // namierzanie
  if(searchingFlag && tempAngleInLOT >= 60 && tempAngleInLOT <= 120){
    radar.rotate(tempAngleInLOT);
    delay(100);
    tempDistanceInLOT = radar.takeMeasurement();
    if(tempBeforeDistanceInLOT - tempDistanceInLOT > 20){
      obstacle.firstEdgeAngle = tempAngleInLOT - 3;
    }
    else if(tempDistanceInLOT - tempBeforeDistanceInLOT > 20){
      obstacle.secondEdgeAngle = tempAngleInLOT - 3;
      searchingFlag = false;
    }
    tempBeforeDistanceInLOT = tempDistanceInLOT;
    tempAngleInLOT = tempAngleInLOT + 6;
  }
  /*else{
    tempAngleInLOT = 60;
  }*/

 // namierzono
  if(!searchingFlag){
    obstacle.centerAngle = (obstacle.secondEdgeAngle - obstacle.firstEdgeAngle)/2 + obstacle.firstEdgeAngle; 
    radar.rotate(obstacle.centerAngle);
    delay(500);
    obstacle.distance = radar.takeMeasurement();
    searchingFlag = true;
    tempAngleInLOT = 60;
    tempBeforeDistanceInLOT = 0;
    lockOnTargetFlag = false;
    moveRifleFlag = true;
  }
}
void moveRifleAutonomical(){
  // obliczanie kątów z tw. kosinusów
  //rifle.setYaw();
  //rifle.setPitch();
  //po sekundzie ustawić flage reloadFlag = true oraz moveRifleFlag = false;
}
void leftEncoderInteruptFunction() {
  Serial.println("1");
  if (startsPayload.manual_auto == 1) {
    if (leftWheel.getTurningDirection() < 0) {
      leftWheel.increaseOrDecreaseEncoder(-1);
    } 
    else if (leftWheel.getTurningDirection() > 0) {
      leftWheel.increaseOrDecreaseEncoder(1);
    }
  }
  //Serial.println("przerwanie koło lewe");
}
void rightEncoderInteruptFunction() {
  Serial.println("2");
  if (startsPayload.manual_auto == 1) {
    if (rightWheel.getTurningDirection() < 0) {
      rightWheel.increaseOrDecreaseEncoder(-1);
    } 
    else if (rightWheel.getTurningDirection() > 0) {
      rightWheel.increaseOrDecreaseEncoder(1);
    }
  }
}
void lastFunction(){
  receivedPayload.load_none = 0;
  receivedPayload.strike_start = 0;
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}