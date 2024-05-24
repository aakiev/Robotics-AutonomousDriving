#include <Servo.h>
Servo myservo;
int pos = 0;    //Servomotor auf 0 Grad


long duration;  //Daten für Ultraschallsensor
float distancecm;
int trigPin = 7;
int echoPin = 8;

void setup() {

  pinMode(trigPin, OUTPUT); //Setup welcher Pin Input und welcher Output ist
  pinMode(echoPin, INPUT);
  Serial.begin(9600);       //Geschwindigkeit der Ausgabe


  myservo.attach(3);        //3 Pins am Servo (Wo ich den anspreche)
  myservo.write(0);         //positioniere dich auf 0 grad
  delay(1000);

}

void loop() {

 // for(pos=0; pos <= 180; pos+=2){
 //   myservo.write(pos);
 //   delay(15);
 // }

//  for(pos=180; pos >= 0; pos-=2){
//    myservo.write(pos);
//    delay(15);
//  }


  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); 

  //Trigger the sensor with a 10 micrseconds HIGH pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //Obtain lenght of HIGH pulse in
  duration = pulseIn(echoPin, HIGH);

  //Ping sensor returns HIGH output pulse that gives the time required for the burst echo to return back to sensor
  //speed of sound travels at 340 m/s or 0.034cm/us
  //Must divide by 2 since time received is time for sound wave to travel to and from object
  distancecm = duration*0.034/2.0;

//  if(distancecm <= 5){
//    myservo.write(0);
//  } else if(distancecm > 5 && distancecm <= 30){
//    myservo.write((distancecm-5)*(180.0/(30-5)));
//  } else {
//    myservo.write(180);
//  }

  Serial.write("Distance: ");
  Serial.println(distancecm);
  delay(100);

}
