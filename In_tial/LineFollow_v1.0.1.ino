/*
   File name: PID_LF_example

   Hardware requirements:   an Arduino Pro Mini
                          a QTR-8RC Reflectance Sensor Array
                            a DRV8835 Dual Motor Driver Carrier

     Description: The basic PID control system implemented with
                the   line follower with the specified hardware.
                The robot can follow   a black line on a white surface
                (or vice versa).
   Related   Document: See the written documentation or the LF video from
                     Bot   Reboot.

   Author: Bot Reboot
*/

#include   <QTRSensors.h> //Make sure to install the library

/*************************************************************************
    Sensor Array object initialisation
*************************************************************************/
QTRSensors   qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*************************************************************************
    PID control system variables
*************************************************************************/
float   Kp = 0.05; //related to the proportional control term;
//change the   value by trial-and-error (ex: 0.07).
float Ki = 0; //related to the integral   control term;
//change the value by trial-and-error (ex: 0.0008).
float   Kd = 0; //related to the derivative control term;
//change the   value by trial-and-error (ex: 0.6).
int P;
int I;
int D;

/*************************************************************************
    Global variables
*************************************************************************/
int   lastError = 0;
boolean onoff = false;

/*************************************************************************
    Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const   uint8_t maxspeeda = 200;
const uint8_t maxspeedb = 200;
const uint8_t basespeeda   = 150;
const uint8_t basespeedb = 150;

/*************************************************************************
    L298n GPIO pins declaration
*************************************************************************/
int ENA = A5;
int IN1 = A4;
int IN2 = A3;
int IN3 = A2;
int IN4 = A1;
int ENB = A0;
/*************************************************************************
    Buttons pins declaration
*************************************************************************/
int   buttoncalibrate = 13; //or pin A3
int buttonstart = 2;

/*************************************************************************
    Function Name: setup
**************************************************************************
    Summary:
  This is the setup function for the Arduino board. It first sets up   the
  pins for the sensor array and the motor driver. Then the user needs to
  slide the sensors across the line for 10 seconds as they need to be
    calibrated.

  Parameters:
   none

  Returns:
   none
*************************************************************************/
void   setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    2, 3, 4, 5, 6, 7, 8, 10
  }, SensorCount);
  //qtr.setEmitterPin(7);//LEDON PIN

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
 /* digitalWrite(ENA,   HIGH); //one of the two control interfaces
  digitalWrite(ENB, HIGH);*/
  //(simplified   drive/brake operation)
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  boolean Ok = false;
  while (Ok == false) { // the main function won't start   until the robot is calibrated
    if (digitalRead(buttoncalibrate) == LOW) {
      calibration(); //calibrate the robot for 10 seconds
      Ok = true;
    }
  }
  forward_brake(0, 0); //stop the motors
}

/*************************************************************************
    Function Name: calibration
**************************************************************************
    Summary:
  This is the calibration function for the QTR-8RC Reflectance Sensor   Array.
  The function calls the method 'qtr.calibrate()' offered by the imported
  library. For approx. 10 seconds, each of the 8 sensors will calibrate with
    readings from the track.

  Parameters:
   none

  Returns:
    none
*************************************************************************/
void   calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0;   i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN,   LOW);
}

/*************************************************************************
    Function Name: loop
**************************************************************************
    Summary:
  This is the main function of this application. When the start button   is
  pressed, the robot will toggle between following the track and stopping.
    When following the track, the function calls the PID control method.

    Parameters:
   none

  Returns:
   none
*************************************************************************/
void   loop() {
    PID_control();
  }


/*************************************************************************
    Function Name: forward_brake
**************************************************************************
    Summary:
  This is the control interface function of the motor driver. As shown   in
  the Pololu's documentation of the DRV8835 motor driver, when the MODE is
  equal to 1 (the pin is set to output HIGH), the robot will go forward at
    the given speed specified by the parameters. The phase pins control the
  direction   of the spin, and the enbl pins control the speed of the motor.

  A warning   though, depending on the wiring, you might need to change the
  aphase and bphase   from LOW to HIGH, in order for the robot to spin forward.

  Parameters:
    int posa: int value from 0 to 255; controls the speed of the motor A.
   int   posb: int value from 0 to 255; controls the speed of the motor B.

  Returns:
    none
*************************************************************************/
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
  forward_brake(motorspeeda, motorspeedb);
}