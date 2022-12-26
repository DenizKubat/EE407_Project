#define Sensor_Left ?
#define Sensor_Mid ?
#define Sensor_Right ?

#define Motor_R1 ?
#define Motor_R2 ?
#define Motor_RE ?

#define Motor_L1 ?
#define Motor_L2 ?
#define Motor_LE ?


void setup() {
  pinMode(Sensor_Left,INPUT);
  pinMode(Sensor_Mid,INPUT);
  pinMode(Sensor_Right,INPUT);

  pinMode(Motor_R1,OUTPUT);
  pinMode(Motor_R2,OUTPUT);
  pinMode(Motor_L1,OUTPUT);
  pinMode(Motor_L2,OUTPUT);


}

void loop() {
                             //Sensor:  White:0    Black:1
                             
  if (digitalRead(Sensor_Left)==0 && digitalRead(Sensor_Mid)==1 && digitalRead(Sensor_Right)==0) {
    move_forward()        
    }    
  if (digitalRead(Sensor_Left)==0 && digitalRead(Sensor_Mid)==0 && digitalRead(Sensor_Right)==1) {
    move_right();    
    }   
  if (digitalRead(Sensor_Left)==1 && digitalRead(Sensor_Mid)==0 && digitalRead(Sensor_Right)==0) {
    move_left();    
    }


  
  if (digitalRead(Sensor_Left)==0 && digitalRead(Sensor_Mid)==0 && digitalRead(Sensor_Right)==0) {
    ??????????   
    }

}



void move_forward(){
  digitalWrite(Motor_R1,HIGH);
  digitalWrite(Motor_R2,LOW);
  analogWrite(Motor_RE,255);

  digitalWrite(Motor_L1,HIGH);
  digitalWrite(Motor_L2,LOW);
  analogWrite(Motor_LE,255);
}

void move_right(){
  digitalWrite(Motor_R1,LOW);
  digitalWrite(Motor_R2,LOW);
  analogWrite(Motor_RE,0);

  digitalWrite(Motor_L1,HIGH);
  digitalWrite(Motor_L2,LOW);
  analogWrite(Motor_LE,255);
}

void move_left(){
  digitalWrite(Motor_R1,HIGH);
  digitalWrite(Motor_R2,LOW);
  analogWrite(Motor_RE,255);

  digitalWrite(Motor_L1,LOW);
  digitalWrite(Motor_L2,LOW);
  analogWrite(Motor_LE,0);
}

void Stop(){
  digitalWrite(Motor_R1,LOW);
  digitalWrite(Motor_R2,LOW);
  analogWrite(Motor_RE,0);

  digitalWrite(Motor_L1,LOW);
  digitalWrite(Motor_L2,LOW);
  analogWrite(Motor_LE,0);
}
