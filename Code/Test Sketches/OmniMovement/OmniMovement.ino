#include <Arduino.h>
#include <AccelStepper.h>

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


// Create instances of the AccelStepper class for each motor:
AccelStepper stepper1(AccelStepper::DRIVER, STEP_A, DIR_A);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_C, DIR_C);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_B, DIR_B);


//Struct to hold desired motion of robot
struct robotDirection{
  //Foward Velocity
  double u;
  //Lateral Velocity
  double v;
  //Pivotal Velocity
  double r;
};

//Struct to hold computed wheel speeds
struct robotWheelSpeeds{
  double one;
  double two;
  double three;
};

unsigned long dT = 0.02;
unsigned long timer = 0;

void setup() {
  
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

  //Optional Serial for debugging
  Serial.begin(9600);


}

void loop() {
  dT = micros()-timer;
  
  robotDirection direction = vectorFromAngle(230, 0.1);
  //robotDirection direction = {0,0,4*PI}; //spin
  //robotDirection direction = {0.1,0.3,0}; //vector
  robotWheelSpeeds speeds = getWheelSpeeds(direction);
  
  setWheelSpeeds(speeds);
  runMotors();

  // Serial.println(stepsFromSpeed(speeds.one));
  // Serial.println(stepsFromSpeed(speeds.two));
  // Serial.println(stepsFromSpeed(speeds.three));
  // Serial.println();


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

unsigned long prev = 0;
void runMotors(){

  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  // Serial.print("time in micros:  ");
  // Serial.println(micros() - prev);
  // prev = micros();
}

robotDirection vectorFromAngle(int AngleDeg, float speed) {
  robotDirection dir;
  dir.u = speed * cos(AngleDeg * DEG_TO_RAD);
  dir.v = speed * sin(AngleDeg * DEG_TO_RAD);
  dir.r = 0;
  return dir;
}

int stepsFromSpeed(double speed) {
  return (int) (speed * 4 * (FULLSTEPSREV * MICROSTEP) / (2*PI));
}

