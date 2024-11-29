#include <Arduino.h>

// Declare pins as constant variables
const int decreaseButtonPin = 5;
const int increaseButtonPin = 3;
const int greenLedPin = 1;
const int redLedPin = 12;
const int ultrasonicTrigPin = 10;
const int ultrasonicEchoPin = 11;
const int temperaturePin = A0;
const int photoresistorPin = A1;

// Variables for tracking sampling frequency and outputting data
const int measuringFrequency = 500;
int serialOutputFrequency = 10;
int currentIndex = 0;
unsigned long lastSavedTime = 0;

// Variable to store status for LED
bool isMore30 = false;

// Variables for storing the sums of each value
int temperatureSum = 0;
int photoresistorSum = 0;
int ultrasonicSum = 0;

float read_ultrasonic() {
  // Ensure a clean pulse
  digitalWrite(ultrasonicTrigPin, LOW);
  delayMicroseconds(2);

  // Start ultrasonic measurement
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);

  // Get pulse which received by the echo pin
  long duration = pulseIn(ultrasonicEchoPin, HIGH);

  // Calculate the duration in centimeters (speed of sound: 0.034 cm/μs)
  return duration * 0.034 / 2;
}

float read_temperature() {
  // Calculate the temperature in Celsius
  return analogRead(temperaturePin) * (5.0 / 1023.0) * 100.0;
}

float read_photoresistor() {
  // Convert analog value to a percentage
  return map(analogRead(photoresistorPin), 0, 1023, 0, 100);
}

void setup() {
  // Init serial at 9600 bps
  Serial.begin(9600);

  // Indicate pins for input
  pinMode(decreaseButtonPin, INPUT);
  pinMode(increaseButtonPin, INPUT);

  // Indicate pins for output
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
}

void loop() {
  // Save current time
  unsigned long currentTime = millis();

  // Read values from buttons
  int decreaseButtonValue = digitalRead(decreaseButtonPin);
  int increaseButtonValue = digitalRead(increaseButtonPin);

  // Checking button presses
  if(decreaseButtonValue == LOW) {
    // Remove two measurements (total -1 second)
    serialOutputFrequency -= 2;
    delay(200);
  }
  if(increaseButtonValue == LOW) {
    // Add two measurements (total +1 second)
    serialOutputFrequency += 2;
    delay(200);
  }
  // Checks that the value does not go beyond 20 measurements (10 seconds of counting measurements)
  // and 2 measurements (1 second of counting measurements)
  serialOutputFrequency = min(serialOutputFrequency, 20);
  serialOutputFrequency = max(serialOutputFrequency, 2);

  // Checking that 0.5 seconds have passed
  if(currentTime - lastSavedTime > measuringFrequency) {
    // Update last measurement time
    lastSavedTime = currentTime;

    // Getting value from sensors
    int currentTemperature = read_temperature();
    int currentPhotoresistor = read_photoresistor();
    int currentUltrasonic = read_ultrasonic();

    // Add new measurements to the sums to further calculate the average value
    temperatureSum += currentTemperature;
    photoresistorSum += currentPhotoresistor;
    ultrasonicSum += currentUltrasonic;
    // Report a new measurement taken
    currentIndex++;

    // If the number of measurements taken is greater than or equal to the required number of measurements, 
    // start calculating the average values ​​and displaying them
    if (currentIndex >= serialOutputFrequency) {
      // Calculate the average values
      float averageTemperature = temperatureSum / serialOutputFrequency;
      float averagePhotoresistor = photoresistorSum / serialOutputFrequency;
      float averageUltrasonic = ultrasonicSum / serialOutputFrequency;

      // Temperature status updates
      if(averageTemperature > 30) {
        isMore30 = true;
      } else {
        isMore30 = false;
      }

      // Output of read values
      Serial.println("**********************");
      Serial.print("Distance: ");
      Serial.print(averageUltrasonic);
      Serial.println(" cm");
      Serial.print("Temperature: ");
      Serial.print(averageTemperature);
      Serial.println(" C");
      Serial.print("Illumination: ");
      Serial.print(averagePhotoresistor);
      Serial.println(" lux");
      Serial.println("**********************");

      // Clearing variables for new measurements
      currentIndex = 0;
      temperatureSum = 0;
      photoresistorSum = 0;
      ultrasonicSum = 0;
    }

    // Checking the status of the temperature value
    if(isMore30) {
      // Turn off the green LED, turn on the red LED
      digitalWrite(greenLedPin, LOW);
      digitalWrite(redLedPin, HIGH);
    } else {
      // Turn off the red LED, turn on the green LED
      digitalWrite(redLedPin, LOW);
      digitalWrite(greenLedPin, HIGH);
    }
  }
}
