
#include <Servo.h>
#include <cmath> // Für die Funktion fabs

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
int PcontrolRight = 0;
double distanceRight = 0.0, distanceLeft = 0.0;

// Definiere den Durchmesser des Rades in cm (anpassen je nach deinem Rad)
double wheelDiameter = 6.5;
double wheelCircumference = PI * wheelDiameter;

// PID Regler Parameter
double targetDistance = 200.0; // Zielentfernung in cm (2 Meter)
double Kp = 1.0; // Proportionaler Koeffizient
double Ki = 0.2; // Integraler Koeffizient
double Kd = 0.025; // Differenzierer Koeffizient
double errorRight = 0.0;
double previousErrorRight = 0.0;
double integralRight = 0.0;
double derivativeRight = 0.0;
double previousTime = 0.0;
double currentTime = 0.0;

// Begrenzung für den Integrationswert (Anti-Windup)
double integralLimit = 100.0; // Anpassen nach Bedarf
double controlSignalLimit = 50.0; // Begrenzung des Steuerungssignals

void calculateThetaRight();
void calculateThetaLeft();
void Pcontroller();
void printInfos();
void calculateDistance();
void driveForward();
void driveBackward();
void resetDistance();
void resetPID();

void setup(){
  Serial.begin(9600);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
}

void loop(){
  // Fahrbefehl für 2 Meter vorwärts und zurück
  if (Serial.available() > 0){
    keyCommand = Serial.read();
    if (keyCommand == 'f') {
      driveForward();
    } else if (keyCommand == 'b') {
      driveBackward();
    }
  }
}

void driveForward(){
  resetDistance();
  resetPID();
  previousTime = millis();
  // Linken Servo konstant halten
  servoLeft.attach(pinServoLeft);
  //servoLeft.write(90 + 10);
  while (fabs(distanceRight) < targetDistance) {
    currentTime = millis();
    double elapsedTime = (currentTime - previousTime) / 1000.0; // in Sekunden
    previousTime = currentTime;

    calculateThetaRight();
    calculateThetaLeft();
    calculateDistance();

    // PID-Regler Berechnung
    errorRight = fabs(distanceLeft) - fabs(distanceRight);
    integralRight += errorRight * elapsedTime;
    derivativeRight = (errorRight - previousErrorRight) / elapsedTime;
    previousErrorRight = errorRight;

    // Begrenze den Integrationswert (Anti-Windup)
    if (integralRight > integralLimit) {
      integralRight = integralLimit;
    } else if (integralRight < -integralLimit) {
      integralRight = -integralLimit;
    }

    double controlSignalRight = Kp * errorRight + Ki * integralRight + Kd * derivativeRight;

    // Begrenze das Steuerungssignal
    if (controlSignalRight > controlSignalLimit) {
      controlSignalRight = controlSignalLimit;
    } else if (controlSignalRight < -controlSignalLimit) {
      controlSignalRight = -controlSignalLimit;
    }

    // Steuerung des rechten Motors basierend auf dem PID-Regler
    PcontrolRight = controlSignalRight;
    
    servoRight.attach(pinServoRight);
    
    servoLeft.write(90 + 8); 
    servoRight.write(90 + 1.0*PcontrolRight);
    delay(100); // 150 perfekte zahl 
    servoRight.detach();
    
    printInfos();
  }
  servoLeft.detach();
}

void driveBackward(){
  resetDistance();
  resetPID();
  previousTime = millis();
  // Linken Servo konstant halten
  servoLeft.attach(pinServoLeft);
  servoLeft.write(90);
  while (distanceRight < targetDistance) {
    currentTime = millis();
    double elapsedTime = (currentTime - previousTime) / 1000.0; // in Sekunden
    previousTime = currentTime;

    calculateThetaRight();
    calculateThetaLeft();
    calculateDistance();

    // PID-Regler Berechnung
    errorRight = targetDistance - distanceRight;
    integralRight += errorRight * elapsedTime;
    derivativeRight = (errorRight - previousErrorRight) / elapsedTime;
    previousErrorRight = errorRight;

    // Begrenze den Integrationswert (Anti-Windup)
    if (integralRight > integralLimit) {
      integralRight = integralLimit;
    } else if (integralRight < -integralLimit) {
      integralRight = -integralLimit;
    }

    double controlSignalRight = Kp * errorRight + Ki * integralRight + Kd * derivativeRight;

    // Begrenze das Steuerungssignal
    if (controlSignalRight > controlSignalLimit) {
      controlSignalRight = controlSignalLimit;
    } else if (controlSignalRight < -controlSignalLimit) {
      controlSignalRight = -controlSignalLimit;
    }

    // Steuerung des rechten Motors basierend auf dem PID-Regler
    PcontrolRight = -controlSignalRight;
    
    servoRight.attach(pinServoRight);
    servoRight.write(90 - PcontrolRight);
    delay(50);
    servoRight.detach();
    
    printInfos();
  }
  servoLeft.detach();
}

void resetDistance() {
  distanceRight = 0.0;
  distanceLeft = 0.0;
}

void resetPID() {
  integralRight = 0.0;
  errorRight = 0.0;
  previousErrorRight = 0.0;
}

void Pcontroller(){
  // PID-Regler könnte hier verwendet werden, aber wir berechnen ihn bereits in driveForward() und driveBackward()
}

void printInfos(){
  Serial.print("ThetaRight: ");
  Serial.println(thetaRight); 
  Serial.print("ThetaLeft: ");
  Serial.println(thetaLeft);
  Serial.print("DistanceRight: ");
  Serial.println(distanceRight);
  Serial.print("DistanceLeft: ");
  Serial.println(distanceLeft);
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

void calculateDistance(){
  // Berechne die Änderung des Winkels
  deltaThetaRight = thetaRight - thetaRightPrev;
  deltaThetaLeft = thetaLeft - thetaLeftPrev;
  
  // Korrigiere den Winkel bei Überlauf
  if (deltaThetaRight < -180) {
    deltaThetaRight += 360;
  } else if (deltaThetaRight > 180) {
    deltaThetaRight -= 360;
  }

  if (deltaThetaLeft < -180) {
    deltaThetaLeft += 360;
  } else if (deltaThetaLeft > 180) {
    deltaThetaLeft -= 360;
  }
  
  // Konvertiere den Winkel in Umdrehungen
  double revolutionsRight = deltaThetaRight / 360.0;
  double revolutionsLeft = deltaThetaLeft / 360.0;
  
  // Berechne die Distanz
  distanceRight += revolutionsRight * wheelCircumference;
  distanceLeft += revolutionsLeft * wheelCircumference;
}
