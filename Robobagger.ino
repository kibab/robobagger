#include <NewPing.h>

// Wiring to L298N motor driver
// Motor A connections
int enA = 5;
int in1 = 9;
int in2 = 8;
// Motor B connections
int enB = 6;
int in3 = 7;
int in4 = 4;

// Ultrasonic distance measurement (via HC-SR04)
const int TRIG_PIN = 12; // brown
const int ECHO_PIN = 11; // black
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);
// Min distance to obstacle in the front, in cm
const int MIN_DISTANCE_TO_OBSTACLE = 20;

// Bluetooth HC-05 is connected to RX/TX of Arduino,
// don't forget a voltage divider on Arduino RX line
// because HC-05 has 3.3V signals!

bool obstacleDetected;

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

  obstacleDetected = false;
  Serial.println("setup() done, robot ready.");
}

// Management via serial port
void SerialController() {
  Serial.println("Command:");

  // Check if there is a command on a serial port. Command is always 3 bytes.
  while(Serial.available() < 3) {
    // No command, so just do whatever we were doing but stop if there is an obstacle.
    int distanceToObstacle = sonar.convert_cm(sonar.ping_median(5));
    if (distanceToObstacle < MIN_DISTANCE_TO_OBSTACLE) {
      if (!obstacleDetected) {
        Serial.println("Obstacle ahead, stopping");
        STOP();
        obstacleDetected = true;
      }
    } else obstacleDetected = false;
    delay(100);
  }
  byte cmd = Serial.read();
  byte arg = Serial.read();
  byte newline = Serial.read();
  Serial.print("Cmd=");
  Serial.print(cmd);
  Serial.print(", arg=");
  Serial.println(arg);

  switch (cmd) {
    case 'P': // ping
      Serial.println("pong");
      break;
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
        SetSpeed(115 + speed * 15);
      }
      break;
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
