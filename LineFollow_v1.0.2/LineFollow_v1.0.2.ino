#include   <QTRSensors.h> 
QTRSensors   qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

float Kp = 0.001; 
float Ki = 0; 
float Kd = 0; 
int P;
int I;
int D;

int lastError = 0;

const uint8_t maxspeeda  = 150;
const uint8_t maxspeedb  = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

int ENA = A5;
int IN1 = A4;
int IN2 = A3;
int IN3 = A2;
int IN4 = A1;
int ENB = A0;
int buttoncalibrate = 13; 
int buttonstart = 2;
int motorspeeda, motorspeedb;
uint16_t position;

void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    2, 3, 4, 5, 6, 7, 8, 10 }, SensorCount);
    // Far left sensor-> 2
    // Far right sensor-> 10
  //qtr.setEmitterPin(7);//LEDON PIN

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);
  
  boolean Ok = false;
  while (Ok == false) { // the main function won't start   until the robot is calibrated
    if (digitalRead(buttoncalibrate) == LOW) {
      Serial.print("Calibration starts");
      calibration(); //calibrate the robot for 10 seconds
      Ok = true;
      Serial.print("Calibration ends");
    }
  }
  drive(0, 0); //stop the motors 
}


void loop() {
    PID_control();
    drive(motorspeeda,motorspeeda);
    //drive(130,130);
    delay(20);
  /*
    position = qtr.readLineBlack(sensorValues);
    //Serial.println(position);
    if (digitalRead(buttoncalibrate)==HIGH){
      Serial.print("Pushed");
    }
    else {
      Serial.print("Not Pushed");
    }
    
    Serial.print('\t');  
    Serial.print('\n'); 
    delay(300);
  
    /*Serial.print(motorspeeda);
    Serial.print('\t');
    Serial.print(motorspeedb);  
    Serial.print('\n');

    
    /*Serial.print(motorspeeda); 
    Serial.print('\t'); 
    Serial.print(motorspeedb);
    Serial.print('\n');
    drive(motorspeeda, motorspeedb);*/
  
 /* qtr.read(sensorValues);
    for (int i = 0; i < 8; i++)
   {
       Serial.print(sensorValues[i]);
       Serial.print('\t'); 
   }
    Serial.print('\n'); */
    

  }


/************************ FUNCTIONS **************************/
void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0;   i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN,   LOW);
}

void drive(int posa, int posb) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, posa);
  analogWrite(ENB, posb);
}

void PID_control() {
  position = qtr.readLineBlack(sensorValues); //read   the current position
  int error = 3500 - position; //3500 is the ideal position   (the centre)
  
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;
  motorspeeda = basespeeda + motorspeed;
  motorspeedb = basespeedb -   motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 128) {
    motorspeeda = 128;
  }
  if (motorspeedb < 128)   {
    motorspeedb = 128;
  }
  
  return(motorspeeda,motorspeeda);
}
