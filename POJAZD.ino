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

struct DataToSend {
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


DataToSend payload;
Motors motors;
const byte addresses[][6] = { "00001", "00002" };
int tempW = 1;
int tempR = 0;
int gx, gy, gz;    // raw gyro values
int ax, ay, az;    // raw gyro values
float gX, gY, gZ;  // raw gyro values
long aX, aY, aZ;   // raw gyro values
unsigned long timeForUpload = 0;
unsigned long gapInFeedback = 0;
//bool cant strike = false;
bool startFlag = false;

float obrot = 0;
unsigned long mes = 0;
unsigned long time = 0;
int i = 0;
int gzSum = 0;
int timeSum = 0;

void receiveData();
void sendData();
void getAccGyro();
void action();
void moveCar();
void moveRifle();

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
  //if(payload.strike_start == 1){obrot = 0;}
  //getAccGyro();
  action();
  //Serial.println(map(payload.yJoy_none, 0, 100, -10, 10));
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
    Serial.println(payload.strike_start);
    //Serial.println(tempR);
  }
}
void getAccGyro() {
  /*//delay(10);
  long gz2;
  gz2 = gz;
  time = millis() - mes;
  mes = millis();
  BMI160.readGyro(gx, gy, gz);
  BMI160.readAccelerometer(ax, ay, az);
  
  
  ax = map(ax, -32768, 32768, -19600, 19600);
  ay = map(ay, -32768, 32768, -19600, 19600);
  az = map(az, -32768, 32768, -19600, 19600);
  gx = map(gx, -32768, 32768, -125, 125);
  gy = map(gy, -32768, 32768, -125, 125);
  
  if(abs(gz2 - map(gz, -32768, 32768, -250, 250)) > 2){
    gz = map(gz, -32768, 32768, -250, 250);
  }
  else{
    gz = 0;
  }
  
  float gz1 = gz;
  gz1 = gz1/1000;
  obrot = obrot + time * gz1;
  Serial.print("obrot:");
  Serial.print(obrot);
  Serial.print("\t");

  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.println();
  */

  long gz2;
  gz2 = gz;
  time = millis() - mes;
  mes = millis();
  BMI160.readGyro(gx, gy, gz);
  if (abs(map(gz, -32768, 32768, -250, 250)) > 3) {
    gz = map(gz, -32768, 32768, -250, 250);
  } else {
    gz = 0;
  }
  if (i < 5) {
    gzSum = gzSum + gz;
    timeSum = timeSum + time;
    i++;
  } else {
    gzSum = gzSum / 5;
    float gz1 = gzSum;
    gz1 = gz1 / 1000;
    obrot = obrot + timeSum * gz1;
    gzSum = 0;
    timeSum = 0;
    i = 0;
    Serial.print("obrot:");
    Serial.print(obrot);
    Serial.print("\t");

    Serial.print("g:\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.println();
  }
}
void sendData() {
  radio.stopListening();
  radio.write(&tempW, sizeof(tempW));
  Serial.println(tempW);
  tempW++;
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
  
  Serial.print("joy:");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(motors.motorDC1);
  Serial.print("\t");
  Serial.print(motors.motorDC2);
  Serial.println();
}
void moveRifle() {
}
void trackToTarget() {
}