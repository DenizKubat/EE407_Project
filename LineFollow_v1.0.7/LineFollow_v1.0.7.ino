#include   <QTRSensors.h> //Make sure to install the library
#include <SD.h>
#include <SPI.h>

QTRSensors   qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
File myFile;

float   Kp = 0.0065; //related to the proportional control term;
//change the   value by trial-and-error (ex: 0.07).
float Ki = 0; //related to the integral   control term;
//change the value by trial-and-error (ex: 0.0008).
float   Kd = 0; //related to the derivative control term;
//change the   value by trial-and-error (ex: 0.6).
int P;
int I;
int D;

int   lastError = 0;
boolean onoff = false;
int pinCS = A1;

const   uint8_t maxspeeda = 200;
const uint8_t maxspeedb = 200;
const uint8_t basespeeda   = 165;
const uint8_t basespeedb = 165;


int ENA = 3;
int IN1 = A2;
int IN2 = A3;
int IN3 = A4;
int IN4 = A5;
int ENB = 5;

//int   buttoncalibrate = 13; //or pin A3
//int buttonstart = 2;

void   setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    2, 4, 6, 7, 8, 9,
  }, SensorCount);
  qtr.setEmitterPin(10);//LEDON PIN
  pinMode(pinCS, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
// SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
  
 boolean Ok = false;
  while (Ok == false) { // the main function won't start   until the robot is calibrate
      calibration(); //calibrate the robot for 10 seconds
      Ok = true;
  }
}

void   calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0;   i < 160; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN,   LOW);
}


void   loop() {
  
   PID_control();
  }


void   forward_brake(int posa, int posb) {
  //set the appropriate values for aphase   and bphase so that the robot goes straight
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,   HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,   HIGH);
  analogWrite(ENA, posa);
  analogWrite(ENB, posb);
}


void   PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); //read   the current position
  int error = 2500 - position; //3500 is the ideal position   (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd; //calculate the correction
  //needed to be applied to the speed

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb -   motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 130) {
    motorspeeda = 130;
  }
  if (motorspeedb < 130)   {
    motorspeedb = 130;
  }
  forward_brake(motorspeedb, motorspeeda);
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {    
    myFile.print(position);
    myFile.print(',');
    myFile.print(motorspeeda);
    myFile.print(',');
    myFile.print(motorspeedb);    
    myFile.print("\n");
    myFile.close(); //close the file
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening test.txt");
  }
  
}
