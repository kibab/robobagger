// Motor A connections
int enA = 5;
int in1 = 9;
int in2 = 8;
// Motor B connections
int enB = 3;
int in3 = 7;
int in4 = 4;

void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  SetSpeed(200);
  MoveForward();
  delay(2000);
  STOP();
  delay(500);
  MoveBackward();
  delay(2000);
  TurnLeft();
  delay(2000);
  STOP();
  delay(500);
  TurnRight();
  delay(2000);
  STOP();
  delay(1000);
}

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
