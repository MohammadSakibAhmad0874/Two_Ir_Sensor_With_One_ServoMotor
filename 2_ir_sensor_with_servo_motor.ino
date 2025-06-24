#include <Servo.h>

// Pin definitions
const int LEFT_IR_PIN = 2;
const int RIGHT_IR_PIN = 3; 
const int SERVO_PIN = 9;

// Servo positions
const int SERVO_LEFT = 30;
const int SERVO_CENTER = 60;
const int SERVO_RIGHT = 90;

// Line detection states (assuming 0 = line detected, 1 = no line)
const int LINE_DETECTED = 0;
const int NO_LINE = 1;

Servo steeringServo;

void setup() {
  Serial.begin(9600);
  
  // Initialize servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);
  
  // Configure IR sensor pins
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  
  Serial.println("Line Following Robot Initialized");
  Serial.println("Servo Center Position: " + String(SERVO_CENTER));
  
  delay(1000); // Give servo time to reach center position
}

void loop() {
  // Read sensor values
  int leftSensor = digitalRead(LEFT_IR_PIN);
  int rightSensor = digitalRead(RIGHT_IR_PIN);
  
  // Debug output
  printSensorData(leftSensor, rightSensor);
  
  // Line following logic
  followLine(leftSensor, rightSensor);
  
  delay(50); // Small delay for stability
}

void followLine(int leftSensor, int rightSensor) {
  if (leftSensor == LINE_DETECTED && rightSensor == NO_LINE) {
    // Line detected on left side - turn left
    steeringServo.write(SERVO_LEFT);
    Serial.println("Action: Turn LEFT");
    
  } else if (leftSensor == NO_LINE && rightSensor == LINE_DETECTED) {
    // Line detected on right side - turn right
    steeringServo.write(SERVO_RIGHT);
    Serial.println("Action: Turn RIGHT");
    
  } else if (leftSensor == NO_LINE && rightSensor == NO_LINE) {
    // Both sensors detect line or both on line - go straight
    steeringServo.write(SERVO_CENTER);
    Serial.println("Action: Go STRAIGHT");
    
  } else {
    // Both sensors off line - stop or continue straight
    steeringServo.write(SERVO_CENTER);
    Serial.println("Action: STOP/SEARCH");
  }
}

void printSensorData(int leftValue, int rightValue) {
  Serial.print("Left: ");
  Serial.print(leftValue);
  Serial.print(" | Right: ");
  Serial.print(rightValue);
  Serial.print(" | ");
}

// Optional: Function to calibrate sensors
void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  
  for (int i = 0; i < 10; i++) {
    int left = digitalRead(LEFT_IR_PIN);
    int right = digitalRead(RIGHT_IR_PIN);
    
    Serial.print("Sample ");
    Serial.print(i + 1);
    Serial.print(" - Left: ");
    Serial.print(left);
    Serial.print(", Right: ");
    Serial.println(right);
    
    delay(500);
  }
  
  Serial.println("Calibration complete");
}