#include<Servo.h>
#define Left_ir 3
#define Right_ir 4
#define Servo_motor 9

Servo myServo;

void setup() {
  Serial.begin(9600);
  myServo.attach(Servo_motor);
  pinMode(Left_ir, INPUT);
  pinMode(Right_ir, INPUT);
  myServo.write(60);
  // put your setup code here, to run once:

}

void loop() {
  int Left_value = digitalRead(Left_ir); //0 or 1
  int Right_value = digitalRead(Right_ir);
  Serial.print("Left_value: ");
  Serial.print(Left_value);
  Serial.print("  Right_value: ");
  Serial.print(Right_value);
  Serial.print("  ");

  if(Left_value == 0 && Right_value == 1){
    myServo.write(30);
    Serial.println("move: Left_hand_side");

  }
  else if(Left_value == 1 && Right_value == 0){
    Serial.println("move: Right_hand_side");
    myServo.write(90);

  }
  else if(Left_value == 1 && Right_value == 1){
    Serial.println("move: stop");
    myServo.write(60);

  }
  else{
    Serial.println("move: Straight_hand_side");
    myServo.write(60);

  }
  // delay(500);

  // put your main code here, to run repeatedly:
  // Left_value: 1 Right_value: 2
  // Left_value: 1 Right_value: 2
  // Left_value: 1 Right_value: 2
  // Left_value: 1 Right_value: 2
  // Left_value: 1 Right_value: 2
  // Left_value: 1 Right_value: 2

}