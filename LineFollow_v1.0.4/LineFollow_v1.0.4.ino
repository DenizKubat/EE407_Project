#include   <QTRSensors.h> 
QTRSensors   qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

float Kp = 0.015; 
float Ki = 0; 
float Kd = 0;
int P;
int I;
int D;

int lastError = 0;
int lastposition = 250;
boolean onoff = false;
const uint8_t maxspeeda = 230;
const uint8_t maxspeedb = 230;
const uint8_t basespeeda   = 170;
const uint8_t basespeedb = 170;

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

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

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
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,   LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4,   LOW);
  analogWrite(ENA, posa);
  analogWrite(ENB, posb);
}

void   PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); //read   the current position
  
  if (lastposition==0 && position== 5000){
    position = 0;
  }

  if (lastposition==5000 && position== 0){
    position = 5000;
  }
  
  
  int error = 2500 - position; //3500 is the ideal position   (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;
  
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

  lastposition = position;
  forward_brake(motorspeeda, motorspeedb);
  Serial.print(motorspeeda);
  Serial.print('\t');
  Serial.print(motorspeedb);
  Serial.print('\t');
  Serial.print("Position:");
  Serial.print(position);
  Serial.print('\n');
  delay(100);
}
