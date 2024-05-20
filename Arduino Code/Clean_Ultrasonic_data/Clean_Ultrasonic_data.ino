#include <Servo.h>
Servo myservo;
int pos = 0;    // Servomotor auf 0 Grad

long duration;  // Daten für Ultraschallsensor
float distancecm;
int trigPin = 7;
int echoPin = 8;

// Moving average filter parameters
const int numMeasurements = 5; // Number of measurements to smooth
float measurements[numMeasurements];
int currentMeasurement = 0;

// Function to calculate the moving average
float calculateMovingAverage(float* data, int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum / size;
}

void setup() {
    pinMode(trigPin, OUTPUT); // Setup welcher Pin Input und welcher Output ist
    pinMode(echoPin, INPUT);
    Serial.begin(9600);       // Geschwindigkeit der Ausgabe

    myservo.attach(3);        // 3 Pins am Servo (Wo ich den anspreche)
    myservo.write(0);         // positioniere dich auf 0 grad
    delay(1000);

    // Initialize the measurements array with zeros
    for (int i = 0; i < numMeasurements; i++) {
        measurements[i] = 0;
    }
}

void loop() {
    // Take a measurement
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Trigger the sensor with a 10 microseconds HIGH pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Obtain length of HIGH pulse in
    duration = pulseIn(echoPin, HIGH);

    // Calculate distance in cm
    distancecm = duration * 0.034 / 2.0;

    // Store the measurement in the array
    measurements[currentMeasurement] = distancecm;
    currentMeasurement = (currentMeasurement + 1) % numMeasurements;

    // Calculate the moving average
    float smoothedDistance = calculateMovingAverage(measurements, numMeasurements);

    // Control the servo based on the smoothed distance
    if (smoothedDistance <= 5) {
        myservo.write(0);
    } else if (smoothedDistance > 5 && smoothedDistance <= 30) {
        myservo.write((smoothedDistance - 5) * (180.0 / (30 - 5)));
    } else {
        myservo.write(180);
    }

    // Output the smoothed distance
    Serial.print("Smoothed Distance: ");
    Serial.println(smoothedDistance);
    delay(100);
}