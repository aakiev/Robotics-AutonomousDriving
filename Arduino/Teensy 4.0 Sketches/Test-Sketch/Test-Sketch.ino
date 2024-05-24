#include <Servo.h>

Servo servoRight;
Servo servoLeft;

int pos = 0;
byte pinServoLeft = 11;
byte pinServoRight = 10;
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
int laufVariable = 0 ; 
int distance = 0 ; 
double distanceRight = 0 ; 
double distanceLeft = 0 ; 
double errorLeft = 0 ; 
double errorRight = 0 ; 
double targetDistance = 200; 

void calculateThetaRight();
void calculateThetaLeft();
void Pcontroller();
void printInfos();
void two_meters_drive(); 

void setup(){
  Serial.begin(9600);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
}

void loop(){
  if (Serial.available() > 0){
    keyCommand = Serial.read();
    
    switch(keyCommand){
      case 's': 
        servoRight.attach(pinServoRight);
        servoLeft.attach(pinServoLeft);
        servoRight.write(90 );
        servoLeft.write(90);
        delay(500);
        servoRight.detach();
        servoLeft.detach();
        break;
      case 'w': 
        servoRight.attach(pinServoRight);
        servoLeft.attach(pinServoLeft);
        servoRight.write(91);
        servoLeft.write(91);
        delay(500);
        servoRight.detach();
        servoLeft.detach();
        break;
      case 'a': 
        servoRight.attach(pinServoRight);
        servoLeft.attach(pinServoLeft);
        servoRight.write(92);
        servoLeft.write(92);
        delay(500);
        servoRight.detach();
        servoLeft.detach();
        break;
      case 'd': 
        servoRight.attach(pinServoRight);
        servoLeft.attach(pinServoLeft);
        servoRight.write(93);
        servoLeft.write(93);
        delay(500);
        servoRight.detach();
        servoLeft.detach();
        break;
      default: break;
    }
    
    calculateThetaRight();
    calculateThetaLeft();
    Pcontroller();
  }
}

void Pcontroller(){
  double avgDistance = (distanceRight + distanceLeft) / 2.0;
    errorRight = targetDistance - distanceRight;
    errorLeft = targetDistance - distanceLeft;
    integralRight += errorRight;
    integralLeft += errorLeft;
    double controlSignalRight = Kp * errorRight + Ki * integralRight;
    double controlSignalLeft = Kp * errorLeft + Ki * integralLeft;

    // Steuerung der Motoren basierend auf dem PI-Regler
    PcontrolRight = controlSignalRight;
    PcontrolLeft = controlSignalLeft;
    
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    servoRight.write(90 + PcontrolRight);
    servoLeft.write(90 - PcontrolLeft);
    delay(50);
    servoRight.detach();
    servoLeft.detach();
}

void printInfos(){
  Serial.print("ThetaRight: ");
  Serial.println(thetaRight); 
  Serial.print("ThetaLeft: ");
  Serial.println(thetaLeft);
}
void two_meters_drive(){
thetaRight = 0 ; 
thetaLeft = 0 ; 

while(distance <= 2){
  servoRight.attach(pinServoRight);
        servoLeft.attach(pinServoLeft);
        servoRight.write(90 );
        servoLeft.write(90);
        delay(500);
        servoRight.detach();
        servoLeft.detach();
        calculateThetaRight(); 
        calculateThetaLeft(); 
        if(thetaRight < thetaRightPrev && thetaLeft < thetaLeftPrev){
          laufVariable += 1; 
        }
}


}
void calculateThetaRight(){
  while(1){
    highRight = pulseIn(pinFeedbackRight, HIGH); 
    lowRight = pulseIn(pinFeedbackRight, LOW); 
    cycleTimeRight = highRight + lowRight; 
    if((cycleTimeRight > 1000) && (cycleTimeRight < 1200)){
      break; 
    }
  }
  dutyCycleRight = (100 * highRight) / cycleTimeRight; 
  thetaRightPrev = thetaRight; 
  thetaRight = 359 - ((dutyCycleRight - 2.9) * 360) / (97.1 - 2.9 + 1);  
}

void calculateThetaLeft(){
  while(1){
    highLeft = pulseIn(pinFeedbackLeft, HIGH); 
    lowLeft = pulseIn(pinFeedbackLeft, LOW); 
    cycleTimeLeft = highLeft + lowLeft; 
    if((cycleTimeLeft > 1000) && (cycleTimeLeft < 1200)){
      break; 
    }
  }
  dutyCycleLeft = (100 * highLeft) / cycleTimeLeft; 
  thetaLeftPrev = thetaLeft; 
  thetaLeft = 359 - ((dutyCycleLeft - 2.9) * 360) / (97.1 - 2.9 + 1);  
}
