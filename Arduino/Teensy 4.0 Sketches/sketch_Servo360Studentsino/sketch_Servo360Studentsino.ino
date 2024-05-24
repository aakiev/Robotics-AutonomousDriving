#include <Servo.h>

Servo servoRight;
Servo servoLeft;

int pos = 0;
byte pinServoLeft = 10;
byte pinServoRight = 11;
byte pinFeedbackLeft = 7;
byte pinFeedbackRight = 8;

unsigned long highRight = 0, highLeft = 0;
unsigned long lowRight = 0, lowLeft = 0;
char keyCommand;
double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;
unsigned long cycleTimeRight, cycleTimeLeft;
double totalCycle = 1098.901;
double thetaRight = 0.0, thetaLeft = 0.0;
double thetaRightPrev = 0.0, thetaLeftPrev = 0.0;
int turnsRight = 0, turnsLeft = 0;
double deltaThetaRight = 0, deltaThetaLeft = 0;
double totalThetaRight = 0, totalThetaLeft = 0;
int PcontrolRight = 0, PcontrolLeft = 0;

void calculateThetaRight();
void calculateThetaLeft();
void Pcontroller(int dir);
void printInfos();

void setup(){
  Serial.begin(9600);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
  } 

void loop(){
  if (Serial.available() > 0){
    keyCommand = Serial.read();
    
    switch(keyCommand){
      case 'f': 
        servoRight.attach(pinServoRight);
        servoLeft.attach(pinServoLeft);
          servoRight.write(89 + PcontrolRight);
          servoLeft.write(98 + PcontrolLeft);
          delay(500);
          servoRight.detach();
          servoLeft.detach();
          Pcontroller(1);
        break;
      case 'b': 
        servoRight.attach(5);
        servoLeft.attach(6);
        servoRight.write(98 + PcontrolRight);
        servoLeft.write(89 + PcontrolLeft);
        delay(500);
        servoRight.detach();
        servoLeft.detach();
        Pcontroller(-1);
        break;
      case 's': 
        servoRight.attach(5);
        servoLeft.attach(6);
        servoRight.write(94);
        servoLeft.write(94);
        delay(500);
        servoRight.detach();
        servoLeft.detach();
        break;
      default: break;
    }   
  }
}

void Pcontroller(int dir){
   printInfos();
}

void printInfos(){
  Serial.print("ThetaRight: ");
  Serial.print(thetaRight);
  Serial.print(" , ThetaLeft: ");
  Serial.println(thetaLeft);
  Serial.print("deltaThetaRight: ");       
  Serial.print(deltaThetaRight);
  Serial.print(" , deltaThetaLeft: ");       
  Serial.println(deltaThetaLeft);
  Serial.print("totalThetaRight: ");         
  Serial.print(totalThetaRight);
  Serial.print(" , totalThetaLeft: ");       
  Serial.println(totalThetaLeft);
  Serial.print("PcontrolRight: ");         
  Serial.print(PcontrolRight);
  Serial.print(" , PcontrolLeft: ");       
  Serial.println(PcontrolLeft);
}

void calculateThetaRight(){
  //Code is available in the technical Data Sheet of the servo
}

void calculateThetaLeft(){
  //Code is available in the technical Data Sheet of the servo
}

