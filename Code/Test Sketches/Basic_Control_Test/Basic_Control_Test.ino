#include <Wire.h>
#include "ICM_20948.h"
#include <AccelStepper.h>
#include <MadgwickAHRS.h> // Library for sensor fusion

#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.

#define AD0_VAL_SENSOR1 0
#define AD0_VAL_SENSOR2 1

ICM_20948_I2C myICM1;
ICM_20948_I2C myICM2;
Madgwick filter; // Fusion filter for both sensors

// Define Arduino Board connections:
#define ENA_PIN 2      // Enable pin of all stepper motors
#define DIR_A 3        // Direction of stepper motor 3
#define STEP_A 11       // Step of stepper motor 3
#define DIR_B 9        // Direction of stepper motor 1
#define STEP_B 10       // Step of stepper motor 1
#define DIR_C 5        // Direction of stepper motor 2
#define STEP_C 6        // Step of stepper motor 2

//Robot Parameters
#define WHEELRADIUS 0.1
#define WHEELDISTANCE 0.075
#define MICROSTEP 8
#define FULLSTEPSREV 200

const double ROOT3 = sqrt(3);
unsigned long timer = 0;
unsigned long IMUtimer = 0;

// Create instances of the AccelStepper class for each motor:
AccelStepper stepper1(AccelStepper::DRIVER, STEP_A, DIR_A);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_C, DIR_C);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_B, DIR_B);


//Struct to hold computed wheel speeds
struct robotWheelSpeeds{
  double one;
  double two;
  double three;
};

//Struct to hold desired motion of robot
struct robotDirection{
  //Foward Velocity
  double u;
  //Lateral Velocity
  double v;
  //Pivotal Velocity
  double r;
};


void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  bool initialized1 = false;
  bool initialized2 = false;

  while (!initialized1 || !initialized2)
  {
    // Initialize sensor 1
    myICM1.begin(WIRE_PORT, AD0_VAL_SENSOR1);
    SERIAL_PORT.print(F("Initialization of sensor 1 returned: "));
    SERIAL_PORT.println(myICM1.statusString());
    myICM1.startupMagnetometer();
    if (myICM1.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized1 = true;
    }

    // Initialize sensor 2
    myICM2.begin(WIRE_PORT, AD0_VAL_SENSOR2);
    SERIAL_PORT.print(F("Initialization of sensor 2 returned: "));
    SERIAL_PORT.println(myICM2.statusString());
    myICM2.startupMagnetometer();
    if (myICM2.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized2 = true;
    }

  // Set the maximum speed and acceleration for each stepper:
  stepper1.setMaxSpeed(1600);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1600);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(1600);
  stepper3.setAcceleration(500);

  // Enable the motors (assuming active LOW for the enable pins):
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); // Disable motors initially by setting HIGH
  }
}

void loop() {
  if (myICM1.dataReady() && myICM2.dataReady() && (micros() - IMUtimer > 1000)) {
    myICM1.getAGMT();
    myICM2.getAGMT();
    updateFilter();
    // printRPY();
    // robotDirection dir = {control(filter.getPitch()), -control(filter.getRoll()), 0};
    robotDirection dir = {0.1,0,0};
    robotWheelSpeeds speeds = getWheelSpeeds(dir);
    setWheelSpeeds(speeds);
    IMUtimer = micros();
  }
  Serial.println(micros() - timer);
  runMotors();
  timer = micros();
}

robotWheelSpeeds getWheelSpeeds(robotDirection dir) {
  robotWheelSpeeds speeds;
  // math is heavily dependent on numbers worked out by hand. Could easily be abstracted later
  speeds.one = ((dir.v - ROOT3*dir.u)/2 + WHEELDISTANCE * dir.r) / WHEELRADIUS;

  speeds.two = (dir.r*WHEELDISTANCE - dir.v) / WHEELRADIUS;

  speeds.three = ((dir.u*ROOT3 + dir.v)/2 + WHEELDISTANCE * dir.r) / WHEELRADIUS;

  return speeds;
}

void setWheelSpeeds(robotWheelSpeeds speeds) {
  stepper1.setSpeed(stepsFromSpeed(speeds.one));
  stepper2.setSpeed(stepsFromSpeed(speeds.two));
  stepper3.setSpeed(stepsFromSpeed(speeds.three));
}

void runMotors(){
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
}

int stepsFromSpeed(double speed) {
  return (int) (speed * 4 * (FULLSTEPSREV * MICROSTEP) / (2*PI));
}

float control(float angVelo) {
  SERIAL_PORT.println(angVelo);
  if (angVelo < 10) {return 0;}
  else {return angVelo/100;}
}

void updateFilter()
{
  // Extract sensor data from sensor 1
  float ax1, ay1, az1;
  float gx1, gy1, gz1;
  float mx1, my1, mz1;

  ax1 = myICM1.accX();
  ay1 = myICM1.accY();
  az1 = myICM1.accZ();

  gx1 = myICM1.gyrX();
  gy1 = myICM1.gyrY();
  gz1 = myICM1.gyrZ();

  mx1 = myICM1.magX();
  my1 = myICM1.magY();
  mz1 = myICM1.magZ();

  // Extract sensor data from sensor 2
  float ax2, ay2, az2;
  float gx2, gy2, gz2;
  float mx2, my2, mz2;

  ax2 = myICM2.accX();
  ay2 = myICM2.accY();
  az2 = myICM2.accZ();

  gx2 = myICM2.gyrX();
  gy2 = myICM2.gyrY();
  gz2 = myICM2.gyrZ();

  mx2 = myICM2.magX();
  my2 = myICM2.magY();
  mz2 = myICM2.magZ();

  // Update fusion filter with data from sensor 1
  filter.update(gx1, gy1, gz1, ax1, ay1, az1, mx1, my1, mz1);

  // Update fusion filter with data from sensor 2
  filter.update(gx2, gy2, gz2, ax2, ay2, az2, mx2, my2, mz2);
}

void printRPY()
{
  SERIAL_PORT.print("Roll: ");
  SERIAL_PORT.print(filter.getRoll());
  SERIAL_PORT.print(", Pitch: ");
  SERIAL_PORT.print(filter.getPitch());
  SERIAL_PORT.print(", Yaw: ");
  SERIAL_PORT.println(filter.getYaw());
}