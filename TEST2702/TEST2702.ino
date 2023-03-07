// define motor control pins
#include <Servo.h>
#include <AccelStepper.h>

// Define stepper motor connections and steps per revolution
#define DIR_PIN 2
#define STEP_PIN 11
#define STEPS_PER_REVOLUTION 200

// Define motor angle limits
#define ANGLE_MIN -90
#define ANGLE_MAX 90

// Initialize stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define initial position and target position
int currentPosition = 0;
int targetPosition = 0;

const int ENA = 11;
const int IN1 = 8;
const int IN2 = 7;
const int ENB = 3;
const int IN3 = 5;
const int IN4 = 4;

const int servoPin = 10;
const int myservoPin = 9;

// set initial motor speed
int motorSpeed1 = 0;
int motorSpeed2 = 0;
int servoAngle = 0;
int ServoAngle2 = 0;

Servo servo;
Servo myservo;


void setup() {
  // initialize serial communication
  Serial.begin(9600);

  // set motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);

  servo.attach(servoPin); 
  myservo.attach(myservoPin);

  // Set maximum speed and acceleration
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  // Set motor direction
  stepper.setPinsInverted(false, true, false);
  // Set initial position as zero
  stepper.setCurrentPosition(0);
  Serial.begin(9600);
}

void loop() {
  // read input from serial monitor
  if (Serial.available() > 0) {
    char input = Serial.read();
    Serial.println(input);
    switch (input) {
      case 'a': // increase motor speed
        motorSpeed1 += 10;
        motorSpeed2 += 10;
        if (motorSpeed1 > 255 && motorSpeed2 > 255) {
          motorSpeed1 = 255;
          motorSpeed2 = 255;
        }

        break;
        
      case 'b': // decrease motor speed
        motorSpeed1 -= 10;
        motorSpeed2 -= 10;
        if (motorSpeed1 < 0 && motorSpeed2 < 0) {
          motorSpeed1 = 0;
          motorSpeed2 = 0;
        }

        break;
      case 'c':
        servoAngle += 10;
        servo.write(servoAngle);
        break;
      // Decrease the angle of the servo motor
      case 'd':
        servoAngle -= 10;
        servo.write(servoAngle);
        break;
      case 'e':
        // Increase target position by 10 degrees
        targetPosition = currentPosition + 10;
        break;
      case 'f':
        // Decrease target position by 10 degrees
        targetPosition = currentPosition - 10;
        break;
      //shoot
      case 'g':
        ServoAngle2 += 10;
        myservo.write(ServoAngle2);
        break;

      case 'h':
        ServoAngle2 -= 10;
        myservo.write(ServoAngle2);
        break;

      
      default:
        // Invalid command
        Serial.println("Invalid command");
        break;

      
    }

  }
    // Check if the target position is within the angle limits
  if (targetPosition < ANGLE_MIN) {
    targetPosition = ANGLE_MIN;
  } else if (targetPosition > ANGLE_MAX) {
    targetPosition = ANGLE_MAX;
  }

  // Move the motor to the target position
  int targetSteps = map(targetPosition, ANGLE_MIN, ANGLE_MAX, -STEPS_PER_REVOLUTION / 4, STEPS_PER_REVOLUTION / 4);
  int currentSteps = stepper.currentPosition();
  int stepsToMove = targetSteps - currentSteps;
  stepper.move(stepsToMove);

  // Wait for motor to complete movement
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  analogWrite(ENB, motorSpeed1);
  analogWrite(ENA, motorSpeed2);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
          
          
        
  // Update the current position
  currentPosition = map(stepper.currentPosition(), -STEPS_PER_REVOLUTION / 4, STEPS_PER_REVOLUTION / 4, ANGLE_MIN, ANGLE_MAX);

  // Print the current position to the serial monitor
  Serial.println("Current position, speed,Angle:");
  Serial.println(currentPosition);
  Serial.println(motorSpeed1);
  Serial.println(motorSpeed2);
  Serial.println(servoAngle);
}
