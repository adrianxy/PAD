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


// DEFINICJE
#define in1_L298N 9              // |
#define in2_L298N 10             // |
#define in3_L298N 4              // |
#define in4_L298N 5              // | kontrola obrotów silników DC 1 i 2
#define CE_NRF24 7               // |
#define CSN_NRF24 8              // |
#define MOSI_NRF24 11            // |
#define MISO_NRF24 12            // |
#define CSK_NRF24 13             // | obsługa NRF24
#define interruptLeftEncoder 2   // | enkoder 400 impulsów na obrót
#define interruptRightEncoder 3  // | obsługa NRF24


// OBIEKTY Z INNYCH PLIKÓW
RF24 radio(CE_NRF24, CSN_NRF24);
VL53L0X sensor;
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
  double X;
  double Y;
};

struct Time {
  unsigned long gapInFeedback = 0;          // timer - co jaki czas można wysłać dane o położeniu
  unsigned long timeOfGyroMeasurement = 0;  // czas wykonania pomiaru żyroskopem
  unsigned long dTimeGyroMeasurement = 0;   // czas pamiędzy pomiarami żyroskopem
  unsigned long timeOfDriving = 0;
  unsigned long timeOf_soft_stop = 0;
};
// KLASY
class Wheel {
public:
  Wheel(float radius, int in1, int in2);
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
  const int _pulsesPerOneRotation = 400;
};
Wheel::Wheel(float radius, int in1, int in2) {
  _radius = radius;
  _in1 = in1;
  _in2 = in2;
}
void Wheel::setPower(int percent) {
  if (percent == 0) {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
  } else if (percent < 0) {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    pwm.setPWM(0, 0, map(percent, 0, -100, 0, 4095));
  } else {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
    pwm.setPWM(0, 0, map(percent, 0, 100, 0, 4095));
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
  _distance = _distance + plusOrMinus * 2 * M_PI * _radius / (float)_pulsesPerOneRotation;
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
  pwm.setPWM(2, 0, map(angle, 0, 180, 125, 650)); // wartości "angle" od 0 - 90 deg
}
void Rifle::shoot() {
  /* rozpocznij proceduję strzału*/
}
void Rifle::reload() {
  /* rozpocznij precedurę załadunku*/
}


// OBIEKTY Z TEGO PLIKU
Wheel leftWheel(3, in1_L298N, in2_L298N);
Wheel rightWheel(3, in3_L298N, in4_L298N);
Rifle rifle;

// ZMIENNE
const byte pipe_address[][6] = { "00001", "00002" };  // adresy kanałów przesyłowych NRF24
const int i2c_addr = 0x68;                            // adres żyroskopu
ReceivedData receivedPayload;
ReceivedData startsPayload;
CarCoords carCoords;
Time time;
bool startFlag;
 bool currentEqualsFinalRotationFlag = false;
 bool driveToTargetFlag = true; // flaga - czy można jechać do celu
 bool lockOnTargetFlag = false; // flaga - czy można namierzyć cel
 bool strikeFlag = false; // flaga - czy można wystrzelić pocisk
 bool reloadFlag = false; // flaga - czy można załadować pocisk

void setup() {
  attachInterrupt(digitalPinToInterrupt(interruptLeftEncoder), leftEncoderInteruptFunction, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptRightEncoder), rightEncoderInteruptFunction, RISING);

  pinMode(in1_L298N, OUTPUT);
  pinMode(in2_L298N, OUTPUT);
  pinMode(in3_L298N, OUTPUT);
  pinMode(in4_L298N, OUTPUT);

  Serial.begin(9600);

  radio.begin();
  radio.openWritingPipe(pipe_address[0]);
  radio.openReadingPipe(1, pipe_address[1]);
  radio.setPALevel(RF24_PA_MIN);

  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  sensor.setTimeout(1000);
  while (!sensor.init()) { Serial.println("Failed to detect and initialize sensor!"); }
  sensor.startContinuous();

  pwm.begin();
  pwm.setPWMFreq(60);
}
void loop() {
  receiveData();
  action();
  //calculateCarPosition();
  lastFunction();
}

void receiveData() {
  radio.startListening();                                   // nasłuchuj
  while (radio.available()) {                               // dopóki jest co czytać, to
    radio.read(&receivedPayload, sizeof(receivedPayload));  // czytaj, jak przeczytasz to ustaw radio.available na false
  }

  /*Serial.print("joy: ");
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
      moveCarManual();
      moveRifleManual();
      if (receivedPayload.strike_start == 1) { rifle.shoot(); }
      if (receivedPayload.load_none == 1) { rifle.reload(); }
      startFlag = false;  // resetuje tryb autonomiczny
      break;
    // AUTONOMICZNY
    case 1:
      if (receivedPayload.strike_start == 1 || startFlag) {
        if (!startFlag) {
          startsPayload = receivedPayload;  // gdy dopiero wystartował to zapisuje ustawienia na których będzie działał automat
          startFlag = true;
        }
        if (receivedPayload.none_giveFedbackPositon == 1 && (millis() - time.gapInFeedback >= 500)) {
          time.gapInFeedback = millis();
          giveFeedback();
        }
        if (driveToTargetFlag){ moveCarAutonomical(startsPayload); }
        //if (lockOnTargetFlag){ lockOnTarget(); }
        //if (strikeFlag){ /* rozpocznij procedurę strzału */ }
        //if (reloadFlag){ /* rozpocznij procedurę załadunku + startFlag=false */ }
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
    if (receivedPayload.yJoy_none == 0) { _rightWheelSpeed = -_leftWheelSpeed; }
  } 
  else if (receivedPayload.xJoy_none <= 0 && receivedPayload.yJoy_none >= 0) {
    _rightWheelSpeed = max(abs(receivedPayload.xJoy_none), receivedPayload.yJoy_none);
    _leftWheelSpeed = map(receivedPayload.xJoy_none, 0, -100, _rightWheelSpeed, -_rightWheelSpeed);
    if (receivedPayload.yJoy_none == 0) { _leftWheelSpeed = -_rightWheelSpeed; }
  } 
  else if (receivedPayload.xJoy_none <= 0 && receivedPayload.yJoy_none <= 0) {
    _rightWheelSpeed = min(receivedPayload.xJoy_none, receivedPayload.yJoy_none);
    _leftWheelSpeed = map(receivedPayload.xJoy_none, 0, -100, _rightWheelSpeed, -_rightWheelSpeed);
    if (receivedPayload.yJoy_none == 0) { _leftWheelSpeed = _rightWheelSpeed; }
  } 
  else {
    _leftWheelSpeed = min(-receivedPayload.xJoy_none, receivedPayload.yJoy_none);
    _rightWheelSpeed = map(receivedPayload.xJoy_none, 0, 100, _leftWheelSpeed, -_leftWheelSpeed);
    if (receivedPayload.yJoy_none == 0) { _rightWheelSpeed = _leftWheelSpeed; }
  }

  if (abs(_leftWheelSpeed) <= 10) {  // niewrażliwość na wartości joysticka mniejsze od 10%
    _leftWheelSpeed = 0;
  } 
  else if (abs(_rightWheelSpeed) <= 10) {
    _rightWheelSpeed = 0;
  }

  // Wysterowanie sterownika i róch kół
  leftWheel.setPower(_leftWheelSpeed);
  rightWheel.setPower(_rightWheelSpeed);

  /*Serial.print("joy:");
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
void moveCarAutonomical(ReceivedData startsPayload) {
  float finalRotation = 180/M_PI*atan2((float)startsPayload.fi_xTarget,(float)startsPayload.ro_yTarget); // [deg] kąt obrotu o jaki trzeba się obrócić żeby być na lini z celem 
  float finalDistance = sqrt(pow((float)receivedPayload.fi_xTarget,2) + pow((float)receivedPayload.fi_xTarget,2)); // [cm]
  
  /*Serial.print("finalRotation: \t");
  Serial.print(finalRotation);
  Serial.print("\t finalDistance: \t");
  Serial.print(finalDistance);
  Serial.println();*/

  if(currentEqualsFinalRotationFlag){
    if (leftWheel.getDistance() < (finalDistance /*- 10*/)){ // "-10" bo zatrzymanie ma być w odległości 1 metra od celu
      leftWheel.setPower(70);
      rightWheel.setPower(70);
    }
    else{
      lockOnTargetFlag = true;
      driveToTargetFlag = false;
      currentEqualsFinalRotationFlag = false;
    }
  }
  else{
    float c = 0.5;  // współczynnik obrotu
    if(finalRotation > 0){  // zgodnie ze wskazówkami zegare
      if(leftWheel.getDistance() >= finalRotation * c){
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        leftWheel.clearStoragedData();
        rightWheel.clearStoragedData();
        currentEqualsFinalRotationFlag = true;
      }
      else{
        leftWheel.setPower(70);
        rightWheel.setPower(-70);
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
        leftWheel.setPower(-70);
        rightWheel.setPower(70);
      }
    }
  }    
}
void giveFeedback(){
  radio.stopListening();                      // sprzestań nasłuchiwać
  radio.write(&carCoords, sizeof(carCoords)); // wyślij współrzędne pojazdu
}
void calculateCarPos() {

  //carCoords.X = sin(mapfloat(currentRotation, -360, 360, -6.28, 6.28)) * distance; // distance będzie zliczał obroty kół z długości PWM
  //carCoords.Y = cos(mapfloat(currentRotation, -360, 360, -6.28, 6.28)) * distance;

  /*Serial.print("carCoords.X \t");
  Serial.print(carCoords.X);
  Serial.print("\t carCoords.Y \t");
  Serial.print(carCoords.Y);
  Serial.println();*/
}
void leftEncoderInteruptFunction() {
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