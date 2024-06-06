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
int Pcontrol = 0;
double distanceRight = 0.0, distanceLeft = 0.0;

// Definiere den Durchmesser des Rades in cm (anpassen je nach deinem Rad)
double wheelDiameter = 6.5;
double wheelCircumference = PI * wheelDiameter;

//Kalibrierungsfaktoren für die Servomotoren
float calibrationFactorLeft = 0.0;
float calibrationFactorRight = 0.0;

// PID Regler Parameter
double targetDistance = 37.0; // Zielentfernung in cm (2 Meter)
double Kp = 25.0; // Proptionaler Koeffizient
double Ki = 0.4; // Integraler Koeffizient
double error = 0.0;
double previousError = 0.0;
double integral = 0.0;
double controlSignal = 0.0;
int lauf = 0 ;

void calculateThetaRight();
void calculateThetaLeft();
void Pcontroller();
void printInfos();
void calculateDistance();
void driveForward();
void driveBackward();
void resetDistance();
void resetPI();
void initializeTheta();
void turn45DegreesRight();
void turn90DegreesRight();
void turn45DegreesLeft();
void driveSchraeg();
void turn45DegreesRightEnde(); 


void setup(){
Serial.begin(9600);
pinMode(pinFeedbackRight, INPUT);
pinMode(pinFeedbackLeft, INPUT);
//initializeTheta();

calibrationFactorLeft = 0.98;
calibrationFactorRight = 1.0;

Serial.print("Kalibrierungsfaktor Links: ");
Serial.println(calibrationFactorLeft);
Serial.print("Kalibrierungsfaktor Rechts: ");
Serial.println(calibrationFactorRight);
if(lauf == 0){
for(int i =0 ;i < 3 ;i++){
  
servoLeft.attach(pinServoLeft);
servoRight.attach(pinServoRight);
servoLeft.writeMicroseconds(1600);

servoRight.writeMicroseconds(1410);

delay(100); }

} lauf = 1 ;
servoRight.detach();
servoLeft.detach();}

void loop(){
// Fahrbefehl für 2 Meter vorwärts und zurück
 
if (Serial.available() > 0){
keyCommand = Serial.read();

if (keyCommand == 'f') {
driveForward();
delay(500); 
turn45DegreesRight();
delay(500); 
driveSchraeg();
delay(500); 
turn45DegreesLeft();
delay(500); 
driveForward();
delay(500); 
turn45DegreesLeft();
delay(500); 
driveSchraeg();
delay(500); 
turn45DegreesRightEnde(); 
delay(5000);
driveForward();
delay(500); 
turn90DegreesRight();
delay(500); 
driveForward();
delay(500); 
turn90DegreesRight(); 
delay(500); 
driveForward();
delay(500); 
turn90DegreesRight();
delay(500); 
driveForward();
delay(500); 
turn90DegreesRight();




} else if (keyCommand == 'b') {
driveBackward();
}


switch (keyCommand){
case 'a':

Serial.print("Malik in case A");
servoLeft.attach(pinServoLeft);
servoRight.attach(pinServoRight);

servoLeft.writeMicroseconds(1600);
servoRight.writeMicroseconds(1400);
delay(1000);
servoRight.detach();
servoLeft.detach();

}
}
}

void driveForward(){
resetDistance();
resetPI();

while ((fabs(distanceRight) + fabs(distanceLeft)) / 2 < targetDistance) {

calculateThetaRight();
calculateThetaLeft();
calculateDistance();

// PI-Regler Berechnung
error = (fabs(distanceLeft) - fabs(distanceRight));
//float adjustment = Kp * error;

servoLeft.attach(pinServoLeft);
servoRight.attach(pinServoRight);

servoLeft.writeMicroseconds(1541);
servoRight.writeMicroseconds(1455);

printInfos();
}

servoRight.detach();
servoLeft.detach();
//turn90DegreesRight();
 

}

void driveBackward(){
resetDistance();
resetPI();

while (fabs(distanceRight) < targetDistance) {

calculateThetaRight();
calculateThetaLeft();
calculateDistance();

// PID-Regler Berechnung
error = fabs(distanceRight) - fabs(distanceLeft);

controlSignal = Kp * error;
servoLeft.attach(pinServoLeft);
servoRight.attach(pinServoRight);
servoLeft.writeMicroseconds(1463);
servoRight.writeMicroseconds(1534); //1529
if(fabs(distanceRight) >= 150 ){
servoRight.writeMicroseconds(1530);
}else{
servoRight.writeMicroseconds(1534);
}
delay(50); // 150 perfekte zahl
servoRight.detach();
servoLeft.detach();

printInfos();
}

}

void resetDistance() {
distanceRight = 0.0;
distanceLeft = 0.0;
}

void resetPI() {
integral = 0.0;
error = 0.0;
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
Serial.print("Error: ");
Serial.println(error);
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
distanceRight += revolutionsRight * wheelCircumference*calibrationFactorRight;
distanceLeft += revolutionsLeft * wheelCircumference*calibrationFactorLeft;
}

void initializeTheta() {
    // Proportionalitätsfaktor für die Drehung zur Nullstellung
    float Kp = 1.0;

    // Maximale Anzahl von Iterationen, um eine Endlosschleife zu vermeiden
    int maxIterations = 1000;
    int iterations = 0;

    while (iterations < maxIterations) {
        calculateThetaRight();
        calculateThetaLeft();

        // Fehlerberechnung
        float errorRight = -thetaRight;  // Zielwert ist 0, also -thetaRight
        float errorLeft = -thetaLeft;    // Zielwert ist 0, also -thetaLeft

        // Stellgrößenberechnung
        float adjustmentRight = Kp * errorRight;
        float adjustmentLeft = Kp * errorLeft;

        servoRight.attach(pinServoRight);
        servoLeft.attach(pinServoLeft);

        // Anpassen der Servosignale
        int baseSpeed = 1500;  // Neutralstellung der Servos
        servoRight.writeMicroseconds(baseSpeed + adjustmentRight);
        servoLeft.writeMicroseconds(baseSpeed + adjustmentLeft);

        // Kurze Wartezeit, um die Servos sich bewegen zu lassen
        delay(50);

        // Überprüfung, ob beide Winkel nahe genug bei 0 sind
        if (fabs(thetaRight) < 10.0 && fabs(thetaLeft) < 10.0) {
            break;
        }

        iterations++;
    }
}

void driveSchraeg(){
  resetDistance();
  resetPI();
  // Linken Servo konstant halten
  while (fabs(distanceRight) < 51.00) {
    calculateThetaRight();
    calculateThetaLeft();
    calculateDistance();

    // PID-Regler Berechnung
    error = fabs(distanceLeft) - fabs(distanceRight);
    integral += error;
    if (integral > 1){ integral = 1;}
    if (integral < -1){ integral = -1;}
    previousError = error;

    double controlSignal = Kp * error + Ki * integral;
    if (controlSignal > 2) controlSignal = 2;
    if (controlSignal < -2) controlSignal = -2;
    // Steuerung des rechten Motors basierend auf dem PID-Regler
    Pcontrol = controlSignal;

    servoLeft.attach(pinServoLeft);
    servoRight.attach(pinServoRight);
   
    if(controlSignal >0){
      servoLeft.writeMicroseconds(1531+0.5*Pcontrol);
      servoRight.writeMicroseconds(1453);
    }
    else if(controlSignal < 0){
      servoLeft.writeMicroseconds(1531);
      servoRight.writeMicroseconds(1453+0.5*Pcontrol);
    }else if (controlSignal == 0 ){
      servoLeft.writeMicroseconds(1531);
      servoRight.writeMicroseconds(1453);
    }
    delay(50); // 150 perfekte zahl
    printInfos();
  }
  servoLeft.detach();
  servoRight.detach();
}

void turn90DegreesLeft() {
  // Hier müssen Sie die richtigen Werte für Ihre Motoren finden, um eine 90-Grad-Drehung zu erreichen.
  resetDistance();
  resetPI();

  // Zielwinkel für 90 Grad Drehung
  double targetAngle = 130.0;
  double currentAngle = 0.0;

  servoLeft.attach(pinServoLeft);
  servoRight.attach(pinServoRight);

  // Starte die Drehung
  servoLeft.writeMicroseconds(1440);
  servoRight.writeMicroseconds(1440);

  while (fabs(currentAngle) < targetAngle) {
    calculateThetaRight();
    calculateThetaLeft();
    calculateDistance();

    // Berechne den aktuellen Drehwinkel basierend auf der Distanz
    currentAngle = (distanceRight + distanceLeft) / 2.0 / (wheelCircumference / 360.0);
  }

  // Stoppe die Motoren
  servoLeft.writeMicroseconds(1500); // Stop
  servoRight.writeMicroseconds(1500); // Stop

  servoLeft.detach();
  servoRight.detach();
}

void turn90DegreesRight() {
  // Hier müssen Sie die richtigen Werte für Ihre Motoren finden, um eine 90-Grad-Drehung zu erreichen.
  resetDistance();
  resetPI();

  // Zielwinkel für 90 Grad Drehung
  double targetAngle = 139.0;
  double currentAngle = 0.0;

  servoLeft.attach(pinServoLeft);
  servoRight.attach(pinServoRight);

  // Starte die Drehung
  servoLeft.writeMicroseconds(1540);
  servoRight.writeMicroseconds(1540);

  while (fabs(currentAngle) < targetAngle) {
    calculateThetaRight();
    calculateThetaLeft();
    calculateDistance();

    // Berechne den aktuellen Drehwinkel basierend auf der Distanz
    currentAngle = (distanceRight + distanceLeft) / 2.0 / (wheelCircumference / 360.0);
  }

  // Stoppe die Motoren
  servoLeft.writeMicroseconds(1500); // Stop
  servoRight.writeMicroseconds(1500); // Stop

  servoLeft.detach();
  servoRight.detach();
}

void turn45DegreesRight(){

 // Hier müssen Sie die richtigen Werte für Ihre Motoren finden, um eine 90-Grad-Drehung zu erreichen.
  resetDistance();
  resetPI();

  // Zielwinkel für 90 Grad Drehung
  double targetAngle = 211.0;
  double currentAngle = 0.0;

  servoLeft.attach(pinServoLeft);
  servoRight.attach(pinServoRight);

  // Starte die Drehung
  servoLeft.writeMicroseconds(1540);
  servoRight.writeMicroseconds(1540);

  while (fabs(currentAngle) < targetAngle) {
    calculateThetaRight();
    calculateThetaLeft();
    calculateDistance();

    // Berechne den aktuellen Drehwinkel basierend auf der Distanz
    currentAngle = (distanceRight + distanceLeft) / 2.0 / (wheelCircumference / 360.0);
  }

  // Stoppe die Motoren
  servoLeft.writeMicroseconds(1500); // Stop
  servoRight.writeMicroseconds(1500); // Stop

  servoLeft.detach();
  servoRight.detach();


}

void turn45DegreesLeft(){

 // Hier müssen Sie die richtigen Werte für Ihre Motoren finden, um eine 90-Grad-Drehung zu erreichen.
  resetDistance();
  resetPI();

  // Zielwinkel für 90 Grad Drehung
  double targetAngle = 197.0;
  double currentAngle = 0.0;

  servoLeft.attach(pinServoLeft);
  servoRight.attach(pinServoRight);

  // Starte die Drehung
  servoLeft.writeMicroseconds(1440);
  servoRight.writeMicroseconds(1440);

  while (fabs(currentAngle) < targetAngle) {
    calculateThetaRight();
    calculateThetaLeft();
    calculateDistance();

    // Berechne den aktuellen Drehwinkel basierend auf der Distanz
    currentAngle = (distanceRight + distanceLeft) / 2.0 / (wheelCircumference / 360.0);
  }

  // Stoppe die Motoren
  servoLeft.writeMicroseconds(1500); // Stop
  servoRight.writeMicroseconds(1500); // Stop

  servoLeft.detach();
  servoRight.detach();


}
void turn45DegreesRightEnde(){

 // Hier müssen Sie die richtigen Werte für Ihre Motoren finden, um eine 90-Grad-Drehung zu erreichen.
  resetDistance();
  resetPI();

  // Zielwinkel für 90 Grad Drehung
  double targetAngle = 206.0;
  double currentAngle = 0.0;

  servoLeft.attach(pinServoLeft);
  servoRight.attach(pinServoRight);

  // Starte die Drehung
  servoLeft.writeMicroseconds(1540);
  servoRight.writeMicroseconds(1540);

  while (fabs(currentAngle) < targetAngle) {
    calculateThetaRight();
    calculateThetaLeft();
    calculateDistance();

    // Berechne den aktuellen Drehwinkel basierend auf der Distanz
    currentAngle = (distanceRight + distanceLeft) / 2.0 / (wheelCircumference / 360.0);
  }

  // Stoppe die Motoren
  servoLeft.writeMicroseconds(1500); // Stop
  servoRight.writeMicroseconds(1500); // Stop

  servoLeft.detach();
  servoRight.detach();


}
