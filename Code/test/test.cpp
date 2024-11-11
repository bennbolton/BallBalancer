#include <Arduino.h>
#include <AccelStepper.h>

// Define Arduino Board connections:
#define ENA_PIN 2      // Enable pin of all stepper motors
#define DIR_A 3        // Direction of stepper motor 3
#define PUL_A 11       // Step of stepper motor 3
#define DIR_B 9        // Direction of stepper motor 1
#define PUL_B 10       // Step of stepper motor 1
#define DIR_C 5        // Direction of stepper motor 2
#define PUL_C 6        // Step of stepper motor 2

// Create instances of the AccelStepper class for each motor:
AccelStepper stepper1(AccelStepper::FULL2WIRE, DIR_A, PUL_A);
AccelStepper stepper2(AccelStepper::FULL2WIRE, DIR_B, PUL_B);
AccelStepper stepper3(AccelStepper::FULL2WIRE, DIR_C, PUL_C);

void setup() {
  
  // Set the maximum speed and acceleration for each stepper:
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);

  // Enable the motors (assuming active LOW for the enable pins):
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); // Disable motors initially by setting HIGH
  stepper1.setSpeed(1000);
  stepper2.setSpeed(1000);
  stepper3.setSpeed(1000);
}

void loop() {
  // put your main code here, to run repeatedly
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
}
