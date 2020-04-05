#include <NewPing.h>

// Motor A connections
int enA = 5;
int in1 = 9;
int in2 = 8;
// Motor B connections
int enB = 6;
int in3 = 7;
int in4 = 4;

// Ultrasonic distance measurement
const int TRIG_PIN = 12; // brown
const int ECHO_PIN = 11; // black
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);

void setup() {
  Serial.begin(9600);
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  Serial.println("setup() done, robot ready.");
}

// Management via serial port
void SerialController() {
  Serial.println("Command:");
  while(Serial.available() < 3) {delay(100);}
  byte cmd = Serial.read();
  byte arg = Serial.read();
  byte newline = Serial.read();
  Serial.print("Cmd=");
  Serial.println(cmd);

  Serial.print("Arg=");
  Serial.println(arg);

  switch (cmd) {
    case 'M':
      Serial.print("Move command: ");
      switch (arg) {
        case 'F':
          Serial.println("Forward");
          STOP(); delay(200);
          MoveForward();
          break;
        case 'B':
          Serial.println("Backward");
          STOP(); delay(200);
          MoveBackward();
          break;
        case 'R':
          Serial.println("Right");
          STOP(); delay(200);
          TurnRight();
          break;
        case 'L':
          Serial.println("Left");
          STOP(); delay(200);
          TurnLeft();
          break;
        default:
          Serial.println("UNKNOWN");
          break;
      }
      break;
    case 'S':
      byte speed = arg - 48; // 48 is ASCII for '0'
      Serial.print("Set speed command: speed=");
      Serial.println(speed);
      if (speed == 0) {
        SetSpeed(0);
      } else {
        SetSpeed(100 + speed * 15);
      }
  }
}

void loop() {
  SerialController();
}

void MeasureDistancePinger() {
  int median_cm = sonar.convert_cm(sonar.ping_median(5));
  Serial.print("Distance: ");
  Serial.println(median_cm);
  delay(50);
}

/*
   Fully autonomous movement with obstacle detection using ultrasonic sensor
*/
void SmartMove() {
  const int MIN_DISTANCE = 25;

  SetSpeed(254);
  MoveForward();
  while (true) {
    int obstacleDistance = sonar.convert_cm(sonar.ping_median(5));
    //Serial.print("Distance: ");
    //Serial.println(obstacleDistance);

    // Try to turn a bit to the left
    if (obstacleDistance < MIN_DISTANCE) {
      STOP();
      delay(50);
      MoveBackward();
      delay(500);
      STOP();
      delay(100);
      TurnLeft();
      delay(2000);
      STOP();
      SetSpeed(254);
      MoveForward();
    }
    delay(50);
  }
}

void JustForwardFullSpeed() {
  SetSpeed(254);
  MoveForward();
}
/*
   Utility functions to implement simple movements
*/

void MoveForward() {
  MoveForwardLeft();
  MoveForwardRight();
}

void SetSpeed(int speed) {
  SetSpeedRight(speed);
  SetSpeedLeft(speed);
}

void STOP() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void SetSpeedRight(int speed) {
  analogWrite(enA, speed);
}

void SetSpeedLeft(int speed) {
  analogWrite(enB, speed);
}

void TurnLeft() {
  MoveForwardRight();
  MoveBackwardLeft();
}

void TurnRight() {
  MoveForwardLeft();
  MoveBackwardRight();
}

void MoveBackward() {
  MoveBackwardLeft();
  MoveBackwardRight();
}

void MoveForwardRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void MoveForwardLeft() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void MoveBackwardRight() {
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
}

void MoveBackwardLeft() {
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);
}
