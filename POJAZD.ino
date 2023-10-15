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
#define in1_L298N 2 // | 
#define in2_L298N 3 // |
#define in3_L298N 4 // |
#define in4_L298N 5 // | kontrola obrotów silników DC 1 i 2
#define CE_NRF24 7    // |
#define CSN_NRF24 8   // |
#define MOSI_NRF24 11 // |
#define MISO_NRF24 12 // | 
#define CSK_NRF24 13  // | obsługa NRF24 


// OBIEKTY
 RF24 radio(CE_NRF24, CSN_NRF24);
 VL53L0X sensor;
 Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// STRUKTURY
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
  long motorDC1;  // zakres (-20;20)
  long motorDC2;  // zakres (-20;20)
  long motorRifleRotation; // zakres (0;180)
  long motorRifleRaise; // zakres (0;90)
 };

 struct CarCoords{
  double X;
  double Y;
 };


// ZMIENNE
 DataToReceive payload;  // paczka danych otrzymana od kontrolera
 DataToReceive startsPayload; // paczka danych zapamiętana dla automatu po wciśnięciu start
 Motors motors;  // nastawy silników (zakresy wewnątrz struktury)
 CarCoords carCoords;  // położenie pojazdu względem początku układu współrzędnych

 const byte pipe_address[][6] = { "00001", "00002" }; // adresy kanałów przesyłowych NRF24
 const int i2c_addr = 0x68; // adres żyroskopu
 const int maxPWMserwo180 = 650; // skrajna pozycja 2 (D = 650/4096)
 const int minPWMserwo180 = 125; // skrajna pozycja 1 (D = 125/4096)
 const int midPWMservo360 = 374; // nie rusza się (D = 374/4096)

 unsigned long gapInFeedback = 0; // timer - co jaki czas można wysłać dane o położeniu
 unsigned long timeOfGyroMeasurement = 0;  // czas wykonania pomiaru żyroskopem
 unsigned long dTimeGyroMeasurement = 0; // czas pamiędzy pomiarami żyroskopem
 unsigned long timeOfDriving = 0;

 bool startFlag = false; // flaga - czy rozpoczęto program autonomiczny (start)
 bool driveToTargetFlag = false; // flaga - czy można jechać do celu
 bool lockOnTargetFlag = false; // flaga - czy można namierzyć cel
 bool strikeFlag = false; // flaga - czy można wystrzelić pocisk
 bool reloadFlag = false; // flaga - czy można załadować pocisk
 bool currentEqualsFinalRotationFlag = false;

 int gx, gy, gz;  // wartości z żyroskopu
 int ax, ay, az;  // wartości z akcelerometru (nieużywane)
 int i_gyro = 0;  // zliczanie ilości próbek
 int gzSum = 0;   // suma i_gyro próbek
 int gzTimeSum = 0; // czas wykonania i_gyro próbek

 float gzAve = 0; // średni kąt obrotu w czasie gzTimeSum
 float currentRotation = 0; // obecny kąt obrotu
 float distance = 10;  // przejechany dystans


// DEFINICJE FUNKCJI
 void receiveData(); // odczytywanie danych do "payload"
 void giveFeedback(); // sprawdza czy ma wysłać dane do kontrolera
 void sendData();    // wysyłanie "carCoords" do kontrolera
 void getAccGyro();  // wpisywanie kątu obrotu do "currentRotation" 
 void action();      // zarząda ruchem części pojazdu
 void driveToTarget(DataToReceive startsPayload);  // jazda autonomiczna w okolice celu, ustawanie zmiennch motors.[]
 void moveCarManual();   // jazda manualna pojazdem, ustawianie zmiennych motors.[] 
 void moveRifleManual(); // ustawianie manualne pozycji działa, ustawianie zmiennych motors.[]
 void calculateCarPos(); // wpisywanie pozycji pojazdu do "carCoords" 
 void lockOnTarget();    // dokładne namierzanie celu poprzez czujnik laserowy  
 void lastFunction();    // ustawianie warunków dla prawidłowego działania programu
 void setPWM();          // interpretacja zmiennych motors.[] i przekazywanie ich do silników
 float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);  // zmiennoprzecinkowe mapowanie

// PROGRAM
 void setup() {
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
  while (!sensor.init()){ Serial.println("Failed to detect and initialize sensor!"); }
  sensor.startContinuous();

  pwm.begin();
  pwm.setPWMFreq(60);
 }

void loop() {
  receiveData();
  calculateCarPos();
  giveFeedback();
  action();
  lastFunction();
}

void receiveData() {
  radio.startListening();                   // nasłuchuj 
  while (radio.available()) {               // dopóki jest co czytać, to
    radio.read(&payload, sizeof(payload));  // czytaj, jak przeczytasz to ustaw radio.available na false
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
  if(payload.strike_start == 1){currentRotation = 0;} // zerowanie rotacji na starcie

  dTimeGyroMeasurement = millis() - timeOfGyroMeasurement;  // pomiar czasu między pomiarami
  timeOfGyroMeasurement = millis();
  BMI160.readGyro(gx, gy, gz);  // czytanie z żyroskopu

  if (abs(map(gz, -32768, 32768, 250, -250)) > 3) { // mapowanie wartości na kąty w stopniach
    gz = map(gz, -32768, 32768, 250, -250);
  } 
  else {
    gz = 0;
  }

  if (i_gyro < 8) {    // obliczanie obrotu
    gzSum = gzSum + gz;
    gzTimeSum = gzTimeSum + dTimeGyroMeasurement;
    i_gyro++;
  } else {
    gzAve = gzSum;  
    gzAve = gzAve / 8 / 1000; // średnia prędkość obrotu z 8 próbek i zamiana na [deg/ms]
    currentRotation = currentRotation + gzTimeSum * gzAve;
    if(currentRotation > 360 || currentRotation <-360){ currentRotation = 0;} // zerowanie pełnego obrotu
    gzSum = 0;
    gzTimeSum = 0;
    i_gyro = 0;

    /*Serial.print("currentRotation: ");
    Serial.print(currentRotation);
    Serial.print("\t\t");
    Serial.print("gz: ");
    Serial.print(gz);
    Serial.println();*/
  }
}
void giveFeedback(){
  if (payload.manual_auto == 1 && payload.none_giveFedbackPositon == 1) {
    if (millis() - gapInFeedback >= 500) {
      gapInFeedback = millis();
      sendData();
    }
  }
}
void sendData() {
  radio.stopListening();                      // sprzestań nasłuchiwać
  radio.write(&carCoords, sizeof(carCoords)); // wyślij współrzędne pojazdu
  
  /*Serial.print("carCoords.X: \t");
  Serial.print(carCoords.X);
  Serial.print("\t carCoords.Y: \t");
  Serial.print(carCoords.Y);
  Serial.println();*/
}
void action() {
 // MANUALNY
  if (payload.manual_auto == 0) {  
    moveCarManual();
    moveRifleManual();
    setPWM();
    if (payload.strike_start == 1) { /* rozpocznij proceduję strzału*/ }
    if (payload.load_none == 1) { /* rozpocznij precedurę załadunku*/ }
  } 
 // AUTONOMINCZY
  else { 
    if (payload.strike_start == 1 || startFlag) {
      if(!startFlag){
        startsPayload = payload;  // gdy dopiero wystartował to zapisuje ustawienia na których będzie działał automat
      }
      startFlag = true;
      if (driveToTargetFlag){ driveToTarget(startsPayload); }
      if (lockOnTargetFlag){ lockOnTarget(); }
      if (strikeFlag){ /* rozpocznij procedurę strzału */ }
      if (reloadFlag){ /* rozpocznij procedurę załadunku + startFlag=false */ }
    }
  }
}
void moveCarManual() {
  if (payload.xJoy_none >= 0 && payload.yJoy_none >= 0) {
    motors.motorDC1 = max(payload.xJoy_none, payload.yJoy_none);  // przeliczanie pozycji joysticka na napędy silników DC 1 i 2
    motors.motorDC2 = map(payload.xJoy_none, 0, 20, motors.motorDC1, -motors.motorDC1);
    if (payload.yJoy_none == 0) { motors.motorDC2 = -motors.motorDC1; }
  } 
  else if (payload.xJoy_none <= 0 && payload.yJoy_none >= 0) {
    motors.motorDC2 = max(abs(payload.xJoy_none), payload.yJoy_none);
    motors.motorDC1 = map(payload.xJoy_none, 0, -20, motors.motorDC2, -motors.motorDC2);
    if (payload.yJoy_none == 0) { motors.motorDC1 = -motors.motorDC2; }
  } 
  else if (payload.xJoy_none <= 0 && payload.yJoy_none <= 0) {
    motors.motorDC2 = min(payload.xJoy_none, payload.yJoy_none);
    motors.motorDC1 = map(payload.xJoy_none, 0, -20, motors.motorDC2, -motors.motorDC2);
    if (payload.yJoy_none == 0) { motors.motorDC1 = motors.motorDC2; }
  } 
  else {
    motors.motorDC1 = min(-payload.xJoy_none, payload.yJoy_none);
    motors.motorDC2 = map(payload.xJoy_none, 0, 20, motors.motorDC1, -motors.motorDC1);
    if (payload.yJoy_none == 0) { motors.motorDC2 = motors.motorDC1; }
  }

  if (abs(motors.motorDC1) <= 2){ // niewrażliwość na wartości joysticka mniejsze od 2
    motors.motorDC1 = 0;
  }
  else if(abs(motors.motorDC2) <= 2){
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
void moveRifleManual() {
  motors.motorRifleRotation = payload.fi_xTarget; // ustawianie pozycji serw według nastaw potencjometrów
  motors.motorRifleRaise = payload.ro_yTarget;
}
void driveToTarget(DataToReceive startsPayload) {
  double finalRotation = 180/3.14*atan2((double)startsPayload.fi_xTarget,(double)startsPayload.ro_yTarget); // kąt obrotu o jaki trzeba się obrócić żeby być na lini z celem 
  double finalDistance = sqrt(pow(payload.fi_xTarget,2) + pow(payload.fi_xTarget,2));
  
  /*Serial.print("finalRotation: \t");
  Serial.print(finalRotation);
  Serial.print("\t finalDistance: \t");
  Serial.print(finalDistance);
  Serial.println();*/

  if(currentEqualsFinalRotationFlag){
    if (distance < (finalDistance - 10)){ // "-10" bo zatrzymanie ma być w odległości 1 metra od celu
      motors.motorDC1 = 15;
      motors.motorDC2 = 15;
    }
    else{
      lockOnTargetFlag = true;
      driveToTargetFlag = false;
    }
    distance = millis() - (float)timeOfDriving;
  }
  else if (currentRotation >= (finalRotation - 2) && currentRotation <= (finalRotation + 2)){  // kąt obrotu jest uzyskany
    motors.motorDC1 = 0;
    motors.motorDC2 = 0;
    timeOfDriving = millis();
    currentEqualsFinalRotationFlag = true; // teoretycznie kod do jazdy (powyższy IF) może być tutaj o ile żyroskop podczas jazdy nie zgłupieje
  }
  else if ((finalRotation - 2) > currentRotation){  // kąt obrotu jest za mały -> obr. zgodnie z zegarem
    motors.motorDC1 = 10;
    motors.motorDC2 = -10;
  }
  else if((finalRotation + 2) < currentRotation){  // kąt obrotu jest za duży -> obr. przeciwnie do zegara
    motors.motorDC1 = -10;
    motors.motorDC2 = 10;
  }
}
void calculateCarPos(){
  getAccGyro();
  carCoords.X = sin(mapfloat(currentRotation, -360, 360, -6.28, 6.28)) * distance; // distance będzie zliczał obroty kół z długości PWM
  carCoords.Y = cos(mapfloat(currentRotation, -360, 360, -6.28, 6.28)) * distance;

  /*Serial.print("carCoords.X \t");
  Serial.print(carCoords.X);
  Serial.print("\t carCoords.Y \t");
  Serial.print(carCoords.Y);
  Serial.println();*/
}
void lockOnTarget(){ 
  // procedura namierzania celu (obroty czujnikiem)
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
  strikeFlag = true;
  lockOnTargetFlag = false;
}
void setPWM(){
  int temp;
  // sterowanie silnikiem DC1 w zależności od motors.motorDC1
  if(motors.motorDC1 == 0){
    digitalWrite(in1_L298N, LOW);
    digitalWrite(in2_L298N, LOW);
  }
  else if(motors.motorDC1 < 0){ 
    temp = map(motors.motorDC1, -2, -20, 0, 4000);
    digitalWrite(in1_L298N, LOW);
    digitalWrite(in2_L298N, HIGH);
    pwm.setPWM(0, 0, temp);
  }
  else{
    temp = map(motors.motorDC1, 2, 20, 0, 4000);
    digitalWrite(in1_L298N, HIGH);
    digitalWrite(in2_L298N, LOW);
    pwm.setPWM(0, 0, temp);
  }

  // sterowanie silnikiem DC2 w zależności od motors.motorDC2
  if(motors.motorDC2 == 0){
    digitalWrite(in3_L298N, LOW);
    digitalWrite(in4_L298N, LOW);
  }
  else if(motors.motorDC2 < 0){
    temp = map(motors.motorDC2, -2, -20, 0, 4000);
    digitalWrite(in3_L298N, LOW);
    digitalWrite(in4_L298N, HIGH);
    pwm.setPWM(1, 0, temp);
  }
  else{
    temp = map(motors.motorDC2, 2, 20, 0, 4000);
    digitalWrite(in3_L298N, HIGH);
    digitalWrite(in4_L298N, LOW);
    pwm.setPWM(1, 0, temp);
  }

  // sterowanie serwem obrotu działka w zależności od motors.motorRifleRotation
  temp = map(motors.motorRifleRotation, 0, 180, 125, 650); // motors.motorRifleRotation przyjmuje wartości (0 - 180)
  pwm.setPWM(2, 0, temp); 

  // sterowanie serwem podnośnika działka w zależności od motors.motorRifleRaise
  temp = map(motors.motorRifleRaise, 0, 180, 125, 650); // motors.motorRifleRaise przyjmuje wartości (0 - 90)
  pwm.setPWM(3, 0, temp); 
}
void lastFunction(){
  payload.load_none = 0;
  payload.strike_start = 0;
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}