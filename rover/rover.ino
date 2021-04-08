#include <Servo.h>

#define MOTOR1_ENABLE_PIN 10  // H-bridge enable 1,2 EN
#define MOTOR1_INPUT_1 4    // H-bridge motor 1 input 1A
#define MOTOR1_INPUT_2 3    // H-bridge motor 1 input 2A
#define MOTOR2_ENABLE_PIN 9   // H-bridge enable 3,4 EN
#define MOTOR2_INPUT_1 5    // H-bridge motor 2 input 3A
#define MOTOR2_INPUT_2 6    // H-bridge motor 2 input 4A
#define ECHO_PIN 7        // ultrasonic echo (read)
#define TRIG_PIN 8        // ultrasonic trigger (write)
#define SERVO_PIN 11      // servo (write)

// motor speed range 0-255
const int MOTOR_SPEED = 255;    
// time it takes for the rover to turn 90 deg
const int MOTOR_TURN_TIME = 1000; 
// time a rover will reverse before checking
const int MOTOR_REVERSE_TIME = 1000; 

// ultrasonic detects up to 336cm away - 5cm for error
const long DISTANCE_LIMIT = 336 - 5;
// allowed distance between rover and obstacles (cm)
const long STOPPING_DISTANCE = 100;
// frequency of ultrasonic polling (ms)
const int POLLING_DELAY = 500;

// output from sonar module
long obstacleDistanceCM;
bool isObstacleLeft = false;
bool isObstacleRight = false;

// servo
Servo servo;
// time allowed for servo movements
const int SERVO_TURN_TIME = 2000;

void setup()
{
  // setup motor pins
  pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR1_INPUT_1, OUTPUT);
  pinMode(MOTOR1_INPUT_2, OUTPUT);
  pinMode(MOTOR2_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR2_INPUT_1, OUTPUT);
  pinMode(MOTOR2_INPUT_2, OUTPUT);
  
  // setup ultrasonic pins
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  // start with motors disabled and moving forwards
  motorsStop();
  motorsForward();
  
  servo.attach(SERVO_PIN, 500, 2500);
  servoReset();
  
  Serial.begin(9600);
}

void loop()
{
  obstacleDistanceCM = checkDistance();
  
  // print distance to serial monitor
  if (obstacleDistanceCM < DISTANCE_LIMIT) {    
    Serial.print("Distance from obstacle: ");
    Serial.print(obstacleDistanceCM);
    Serial.println("cm");
  }
  else {
    Serial.println("No obstacle found");
  }
  
  // stop motor if obstacle gets too close
  if (obstacleDistanceCM < STOPPING_DISTANCE) {
    motorsStop();
    Serial.println("Stopping...");
    
    // check left and right
    isObstacleLeft = checkLeft();
    isObstacleRight = checkRight();
    // TODO 45 deg angles
    
    if (!isObstacleLeft) {      // turn left if clear
      Serial.println("Turning left...");
      motorsRotateLeft();
    }
    else if (!isObstacleRight) {  // turn right if clear
      Serial.println("Turning right...");
      motorsRotateRight();
    }
    else {              // begin reversing
      Serial.println("Reversing...");
      motorsBackward();
      motorsStart(MOTOR_SPEED);
      delay(MOTOR_REVERSE_TIME);
      // TODO backtracking mode
    }
  }
  else {
    motorsStart(MOTOR_SPEED);
    motorsForward();
    Serial.println("Moving forward...");    
  }
  
  // delay before next reading
  delay(POLLING_DELAY);
  Serial.println();
}

/*
  ULTRA-SONIC FUNCTIONS
*/

long checkDistance() {
  // trigger a pulse on the sonar module
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2); // allow time to refresh
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); // must be enabled for 10us to trigger
  digitalWrite(TRIG_PIN, LOW);
  
  // measure the duration of the echo
  long durationRaw = pulseIn(ECHO_PIN, HIGH);
  // convert to centimeters given that the speed of sound is 340 m/s
  // and that we need to account for the return trip
  return (durationRaw / 29 / 2);
}

// rotate servo left a specified amount and check for obstacle
// return: true if obstacle present within specified distance (cm)
bool checkLeft(int deg, long maxDist) {
  servoLeft(deg);
  bool rtn = (checkDistance() < maxDist);
  servoReset(); // reset servo to neutral position
  return rtn;
}
bool checkLeft(int deg) {
  return checkLeft(deg, STOPPING_DISTANCE);
}
bool checkLeft() {
  return checkLeft(90);
}

// rotate servo right a specified amount and check for obstacle
// return: true if obstacle present within specified distance (cm)
bool checkRight(int deg, long maxDist) {
  servoRight(deg);
  bool rtn = (checkDistance() < maxDist);
  servoReset(); // reset servo to neutral position
  return rtn;
}
bool checkRight(int deg) {
  return checkRight(deg, STOPPING_DISTANCE);
}
bool checkRight() {
  return checkRight(90);
}

/*
  MOTOR FUNCTIONS
*/

// turn the rover left a specified amount
void motorsRotateLeft(int dur) {
  motorsLeft();
  motorsStart(MOTOR_SPEED);
  delay(dur);
  // stop motors and revert to forward direction
  motorsStop();
  motorsForward();
}
void motorsRotateLeft() {
  motorsRotateLeft(MOTOR_TURN_TIME);
}

// turn the rover right a specified amount
void motorsRotateRight(int dur) {
  motorsRight();
  motorsStart(MOTOR_SPEED);
  delay(dur);
  // stop motors and revert to forward direction
  motorsStop();
  motorsForward();
}
void motorsRotateRight() {
  motorsRotateRight(MOTOR_TURN_TIME);
}

// prepare motors for movement in various directions

void motorsForward() {
  setMotor1Direction(true);
  setMotor2Direction(false);
}

void motorsBackward() {
  setMotor1Direction(false);
  setMotor2Direction(true);
}

void motorsLeft() {
  setMotor1Direction(false);
  setMotor2Direction(false);
}

void motorsRight() {
  setMotor1Direction(true);
  setMotor2Direction(true);
}

// enable motors at a given speed (0-255)
// direction determined by current directions
void motorsStart(int speed) {
  analogWrite(MOTOR1_ENABLE_PIN, speed);
  analogWrite(MOTOR2_ENABLE_PIN, speed);
}
void motorsStart() {
  motorsStart(MOTOR_SPEED);
}
  
// stop motors
void motorsStop() {
  analogWrite(MOTOR1_ENABLE_PIN, 0);
  analogWrite(MOTOR2_ENABLE_PIN, 0);
}

// configure motor directions via input pins (H-Bridge)

void setMotor1Direction(bool isClockwise) {
  if (isClockwise) {
    digitalWrite(MOTOR1_INPUT_1, HIGH);
    digitalWrite(MOTOR1_INPUT_2, LOW);
  }
  else {
    digitalWrite(MOTOR1_INPUT_1, LOW);
    digitalWrite(MOTOR1_INPUT_2, HIGH);
  }
}

void setMotor2Direction(bool isClockwise) {
  if (isClockwise) {
    digitalWrite(MOTOR2_INPUT_1, HIGH);
    digitalWrite(MOTOR2_INPUT_2, LOW);
  }
  else {
    digitalWrite(MOTOR2_INPUT_1, LOW);
    digitalWrite(MOTOR2_INPUT_2, HIGH);
  }
}

// invert motor 1 direction
void invertMotor1() {
  digitalWrite(MOTOR1_INPUT_1, !digitalRead(MOTOR1_INPUT_1));
  digitalWrite(MOTOR1_INPUT_2, !digitalRead(MOTOR1_INPUT_2));
}

// invert motor 2 direction
void invertMotor2() {
  digitalWrite(MOTOR2_INPUT_1, !digitalRead(MOTOR2_INPUT_1));
  digitalWrite(MOTOR2_INPUT_2, !digitalRead(MOTOR2_INPUT_2));
}

/*
  SERVO FUNCTIONS
*/

// reset servo to facing forward
void servoReset() {
  servo.write(90);
  delay(SERVO_TURN_TIME); // wait for servo to reach position
}

// turn servo left a specified amount
void servoLeft(int deg) {
  servo.write(90-deg);
  delay(SERVO_TURN_TIME); // wait for servo to reach position
}
void servoLeft() {
  servoLeft(90);
}

// turn servo right a specified amount
void servoRight(int deg) {
  servo.write(90+deg);
  delay(SERVO_TURN_TIME); // wait for servo to reach position
}
void servoRight() {
  servoRight(90);
}
