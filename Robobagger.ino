/*
   Note: NewPing.cpp should be modified to disable ISR() calls in the code,
   otherwise it conflicts with IRremote library.
*/
#include <IRremote.h>
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

// IR Receiver
const int RECV_PIN = 10;
IRrecv irrecv(RECV_PIN);

// IR codes
#define BT_0 0xFF9867
#define BT_1 0xFFA25D
#define BT_2 0xFF629D
#define BT_3 0xFFE21D
#define BT_4 0xFF22DD
#define BT_5 0xFF02FD
#define BT_6 0xFFC23D
#define BT_7 0xFFE01F
#define BT_8 0xFFA857
#define BT_9 0xFF906F
#define BT_STAR 0xFF6897
#define BT_HASH 0xFFB04F
#define BT_UP 0xFF18E7
#define BT_DOWN 0xFF4AB5
#define BT_LEFT 0xFF10EF
#define BT_RIGHT 0xFF5AA5
#define BT_OK 0xFF38C7
// There is a REPEAT code sent in NEC protocol if button remains pressed

int getButton(long code) {
  switch (code) {
    case BT_0:
      return 0;
    case BT_1:
      return 1;
    case BT_2:
      return 2;
    case BT_3:
      return 3;
    case BT_4:
      return 4;
    case BT_5:
      return 5;
    case BT_6:
      return 6;
    case BT_7:
      return 7;
    case BT_8:
      return 8;
    case BT_9:
      return 9;
    default:
      return -1;
  }
}

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
  pinMode(RECV_PIN, INPUT);
  irrecv.enableIRIn();
  irrecv.blink13(true);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  Serial.println("setup() done, robot ready.");
}

void irtest() {
  decode_results results;
  if (irrecv.decode(&results) && results.decode_type == NEC) {
    Serial.println(results.value, HEX);
    irrecv.resume();
  }
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

/*
   Robot that is managed by an IR sensor / IR remote control
*/
void ManagedMove() {
  decode_results cmd;
  int speed = 0;
  bool got_cmd;
  int ctrl = 0;
  while (true) {
    ctrl++;
    got_cmd = false;
    if (irrecv.decode(&cmd)) {
      if (cmd.decode_type == NEC) {
        Serial.print("Speed: ");
        Serial.println(speed);
        Serial.println(cmd.value, HEX);
        got_cmd = true;
      }
      irrecv.resume();
    } else {
      continue;
    }

    if (!got_cmd) {
      continue;
    }

    switch (cmd.value) {
      case BT_UP:
        STOP();
        MoveForward();
        break;
      case BT_DOWN:
        STOP();
        MoveBackward();
        break;
      case BT_LEFT:
        STOP();
        TurnLeft();
        break;
      case BT_RIGHT:
        STOP();
        TurnRight();
        break;
      default:
        int button = getButton(cmd.value);
        if (button >= 0) {
          if (button > 0) {
            speed = 100 + button * 15;
          }
          if (button == 0) {
            speed = 0;
          }
          SetSpeed(speed);
          continue;
        }
        Serial.print("Unhandled code:");
        Serial.println(cmd.value, HEX);
    }
  }
}

void loop() {
  SerialController();
  //  irtest();
  // ManagedMove();
  // MeasureDistancePinger();
  //SmartMove();
  //MoveItMoveIt();
  //JustForwardFullSpeed();
}

void MeasureDistancePinger() {
  int median_cm = sonar.convert_cm(sonar.ping_median(5));
  Serial.print("Distance: ");
  Serial.println(median_cm);
  delay(50);
}

void BackoffL() {
  STOP();
  SetSpeedRight(100);
  SetSpeedLeft(254);
  MoveBackward();
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
      //BackoffL();
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

void MoveItMoveIt() {
  SetSpeed(254);
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
