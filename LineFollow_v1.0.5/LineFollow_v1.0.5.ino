#include   <QTRSensors.h> //Make sure to install the library
#include <SD.h>
#include <SPI.h>

QTRSensors   qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
File myFile;

float   Kp = 0.015; //related to the proportional control term;
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
int pinCS = 10;

const   uint8_t maxspeeda = 200;
const uint8_t maxspeedb = 200;
const uint8_t basespeeda   = 150;
const uint8_t basespeedb = 150;


int ENA = A5;
int IN1 = A4;
int IN2 = A3;
int IN3 = A2;
int IN4 = A1;
int ENB = A0;

//int   buttoncalibrate = 13; //or pin A3
//int buttonstart = 2;

void   setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    2, 3, 4, 5, 6, 7,
  }, SensorCount);
  qtr.setEmitterPin(8);//LEDON PIN
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
  for (uint16_t i = 0;   i < 400; i++)
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
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,   LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4,   LOW);
  analogWrite(ENA, posa);
  analogWrite(ENB, posb);
}

/*************************************************************************
    Function Name: PID_control
**************************************************************************
    Summary:
  This is the function of the PID control system. The distinguishing
  feature of the PID controller is the ability to use the three control
    terms of proportional, integral and derivative influence on the controller
    output to apply accurate and optimal control. This correction is applied to
    the speed of the motors, which should be in range of the interval [0, max_speed],
    max_speed <= 255.
  Parameters:
  none
  Returns:
   none
*************************************************************************/
void   PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); //read   the current position
  int error = 3500 - position; //3500 is the ideal position   (the centre)

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
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0)   {
    motorspeedb = 0;
  }
  //forward_brake(motorspeeda, motorspeedb);
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {    
    myFile.print(position);
    myFile.print("\n");
    myFile.close(); // close the file
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening test.txt");
  }
}
