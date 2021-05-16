#include <Servo.h>            // note: use of this library disables PWM for pin 9 and 10
// source: Standard Arduino Library
#include <NewPing.h>          // used for ultrasonic functions. Note: not compatable with TinkerCAD
// source: https://bitbucket.org/teckel12/arduino-new-ping/src/master/src/NewPing.cpp

#define MOTOR1_ENABLE_PIN 6   // H-bridge enable 1,2 EN
#define MOTOR1_INPUT_1 4      // H-bridge motor 1 input 1A
#define MOTOR1_INPUT_2 3      // H-bridge motor 1 input 2A
#define MOTOR2_ENABLE_PIN 5   // H-bridge enable 3,4 EN
#define MOTOR2_INPUT_1 12     // H-bridge motor 2 input 3A
#define MOTOR2_INPUT_2 13     // H-bridge motor 2 input 4A
#define ECHO_PIN_1 7          // ultrasonic echo 1 (read) - forward
#define TRIG_PIN_1 8          // ultrasonic trigger 1 (write) - forward
#define ECHO_PIN_2 2          // ultrasonic echo 1 (read) - left
#define TRIG_PIN_2 1          // ultrasonic trigger 1 (write) - left
#define ECHO_PIN_3 10         // ultrasonic echo 1 (read) - right
#define TRIG_PIN_3 9          // ultrasonic trigger 1 (write) - right
#define SERVO_PIN 11          // servo (write)

// define the operation mode of the rover.
// 0 - autonomous navigation for test levels 2-5
// 1 - test level 1.1 (straight line and reverse)
// 2 - test level 1.2 (square path)
const int MODE = 0;

// motor speed range 0-255
const int MOTOR_SPEED = 90;
const int TURN_SPEED = 255;
// calibration to ensure the rover moves in a straight line
const int MOTOR1_CALIBRATION = 0; //left
const int MOTOR2_CALIBRATION = -4; // right
// time it takes for the rover to turn. Different surfaces need a different turn time:
// SURFACE || TURN_TIME
// maze    || 420
// table   || 350
const int MOTOR_TURN_TIME = 420;                     // 90 degrees
const int MOTOR_TURN_TIME_45 = MOTOR_TURN_TIME*0.2;  // 45 degrees (adjusted) 0.35
// time a rover will reverse before checking (ms)
const int MOTOR_REVERSE_TIME = 200;
// track the current status of the rover
bool isReversing = false;

// ultrasonic detects up to 336cm away - 5cm for error
const long DISTANCE_LIMIT = 336 - 5;
// allowed distance between rover and obstacles (cm)
const long STOPPING_DISTANCE = 5;
// allowed distance between rover and obstacles when turning (cm)
const long TURN_CHECK_DISTANCE = 14;
// frequency of ultrasonic polling (ms)
const int POLLING_DELAY = 15;
// frequency of ultrasonic polling for angled walls (ms)
const int ANGLED_POLLING_DELAY = POLLING_DELAY*4;
// max parameters based on expected maze dimensions, used for finding angled surfaces
const int MAX_POSSIBLE_DIST = 30;
// left/right correction distance (cm)
const long SIDE_CORRECTION_DISTANCE = 3.5;
// left/right correction amount (ms)
const int CORRECTION_AMOUNT = 20;
bool hasCorrected = false;

// output from sonar module
long obstacleDistanceCM;
bool isObstacleLeft = false;
bool isObstacleRight = false;

// servo
Servo servo;
// time allowed for servo movements
const int SERVO_TURN_TIME = 350;

// setup NewPing (disable for TinkerCAD)
NewPing sonar_1(TRIG_PIN_1, ECHO_PIN_1, DISTANCE_LIMIT); // forward
NewPing sonar_2(TRIG_PIN_2, ECHO_PIN_2, DISTANCE_LIMIT); // left
NewPing sonar_3(TRIG_PIN_3, ECHO_PIN_3, DISTANCE_LIMIT); // right

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
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_1, OUTPUT);
  
  // start with motors disabled and moving forwards
  motorsStop();
  motorsForward();
  
  servo.attach(SERVO_PIN, 500, 2500);
  servoReset();
  
  Serial.begin(9600);
  delay(4000); // initial delay
}

void loop()
{
  // operate based on mode setting
  switch (MODE) {
    case 1:
      level_1_1();
      break;
    case 2:
      level_1_2();
      break;
    default:
      autonomousNavigation();
      break;
  }
}

// FUNCTION FOR NAVIAGTION
// primary function for test levels 2 - 5
void autonomousNavigation() {
  // ensure the rover is straight
  // check every second iteration to ensure primary sensor is not delayed
  if (hasCorrected) {
    correctLeft();
    correctRight();
    hasCorrected = false;    
  }
  else {
    hasCorrected = true;    
  }
  
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
  
  // stop motor if obstacle gets too close and find next path
  if (obstacleDistanceCM < STOPPING_DISTANCE && !isReversing) {
    autonomousCheckLeftRightPath();
  }
  // if reversing, continue reversing until an alternate path is found
  else if (isReversing) {
    autonomousReversePath();
  }  
  // if measured distance is greater than the max possible distance the rover can be from a wall,
  // should check if the signal has been reflected by an angled wall
  else if (obstacleDistanceCM > MAX_POSSIBLE_DIST) {  
    autonomousAngledPath();
  }
  else {
    motorsForward();
    motorsStartCalibrated(MOTOR_SPEED);
    Serial.println("Moving forward...");    
  }
  
  // delay before next reading
  delay(POLLING_DELAY);
  Serial.println();
}

// stop motor and find next path
void autonomousCheckLeftRightPath() {
  motorsStop();
  Serial.println("Stopping...");
  
  // check left and right at 90 and 45 degree angles
  isObstacleLeft = checkLeftBoth();
  isObstacleRight = checkRightBoth();

  if (checkDistance() > STOPPING_DISTANCE) { // check forward again in case of error  
    Serial.println("Moving forward...");     
  }
  else if (!isObstacleRight) {        // turn right if clear
    Serial.println("Turning right...");
    motorsRotateRight();
  }
  else if (!isObstacleLeft) {    // turn left if clear
    Serial.println("Turning left...");
    motorsRotateLeft();
  }    
  else {                         // begin reversing
    Serial.println("Dead end found, reversing...");
    motorsBackward();
    motorsStartCalibrated(MOTOR_SPEED);
    isReversing = true; // enter reverse mode
    delay(MOTOR_REVERSE_TIME);
  }
}

// continue reversing until an alternate path is found
void autonomousReversePath() {
  motorsStop();
  Serial.println("Checking alternate paths...");
  
  // check left and right
  isObstacleLeft = checkLeft();
  isObstacleRight = checkRight();
  
  if (!isObstacleLeft) {      // turn left if clear
    Serial.println("Alternate path found");
    Serial.println("Turning left...");
    motorsRotateLeft();
    isReversing = false;
  }
  else if (!isObstacleRight) {  // turn right if clear
    Serial.println("Alternate path found");
    Serial.println("Turning right...");
    motorsRotateRight();
    isReversing = false;
  }
  else {
    Serial.println("Reversing...");
    motorsBackward();
    motorsStart(MOTOR_SPEED);
    delay(MOTOR_REVERSE_TIME);
  }
}

// if an angled path is found, turn 45 degrees
void autonomousAngledPath() {
  motorsStop();  
  Serial.println("No obstacle found...");
  Serial.println("Checking if approaching angled wall...");
  
  // check left and right 45 degrees to be parallel with an angled wall
  isObstacleLeft = checkLeft(50, STOPPING_DISTANCE*1.2);
  isObstacleRight = checkRight(50, STOPPING_DISTANCE*1.2);

  // navigate depending on the presence of an angled wall, else wait until close enough
  // TODO improve this detection - sometimes turns early when following a parallel wall
  if (!isObstacleLeft && isObstacleRight) {
    Serial.println("Turning 45 degrees left...");
    motorsRotateLeft(MOTOR_TURN_TIME_45);
  }
  else if (!isObstacleRight && isObstacleLeft) {
    Serial.println("Turning 45 degrees right...");
    motorsRotateRight(MOTOR_TURN_TIME_45);
  }
  // not yet close enough to angled wall
  else {
    motorsForward();
    motorsStartCalibrated(MOTOR_SPEED);
    delay(ANGLED_POLLING_DELAY);
  }
}

/*
 * ULTRA-SONIC FUNCTIONS
*/

// calculates median of 5 measurements
long checkDistance(NewPing sensor) { // uses NewPing (disable for TinkerCAD)
  long rtn = sensor.convert_cm(sensor.ping_median(5));
  if (rtn == 0) { // ensure that there is no error reading of 0
    return DISTANCE_LIMIT;
  }
  return rtn;
}
long checkDistance() {
  return checkDistance(sonar_1);
}
long checkDistanceSingle(int TRIG_PIN, int ECHO_PIN) { // alternative function that doesn't use NewPing
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

// checks left and right with secondary sensors to
// ensure the rover does not stray into a wall
bool correctLeft() {
  long secondaryLeftDist = checkDistance(sonar_2); // uses NewPing (disable for TinkerCAD)
  Serial.print("Dist from Left: ");
  Serial.println(secondaryLeftDist);
  if (secondaryLeftDist - 1 < SIDE_CORRECTION_DISTANCE) {
    motorsRotateRight(CORRECTION_AMOUNT); // straighten rover
    Serial.println("Too close to left wall, correcting...");
    return true;
  }
  return false;  
}
long correctRight() { 
  long secondaryRightDist = checkDistance(sonar_3); // uses NewPing (disable for TinkerCAD)
  Serial.print("Dist from Right: ");
  Serial.println(secondaryRightDist);
  if (secondaryRightDist - 1 < SIDE_CORRECTION_DISTANCE) {
    motorsRotateLeft(CORRECTION_AMOUNT); // straighten rover
    Serial.println("Too close to right wall, correcting...");
    return true;
  }
  return false;  
}

// rotate servo left a specified amount and check for obstacle
// return: true if obstacle present within specified distance (cm)
bool checkLeft(int deg, long maxDist) {
  servoLeft(deg);
  bool rtn = (checkDistance() < maxDist);
  if (!rtn) {
    Serial.println("Left is clear");
  }
  servoReset(); // reset servo to neutral position
  return rtn;
}
bool checkLeft(int deg) {
  return checkLeft(deg, TURN_CHECK_DISTANCE);
}
bool checkLeft() {
  return checkLeft(80); // 80 is used to correct servo
}

// checks left 90 and 45 degrees without reset inbetween
bool checkLeftBoth() {
  servoLeft();
  bool rtn90 = (checkDistance() < TURN_CHECK_DISTANCE);
  servoLeft(45);
  bool rtn45 = (checkDistance() < TURN_CHECK_DISTANCE);
  // disabled 45 check due to false detection of walls,
  // when rover is not perfectly parallel
  bool rtn = rtn90; // (rtn90 && !rtn45) || (!rtn90 && rtn45)
  if (!rtn) {
    Serial.println("Left is clear");
  }
  servoReset(); // reset servo to neutral position
  return rtn;
}

// rotate servo right a specified amount and check for obstacle
// return: true if obstacle present within specified distance (cm)
bool checkRight(int deg, long maxDist) {
  servoRight(deg);
  bool rtn = (checkDistance() < maxDist);
  if (!rtn) {
    Serial.println("Right is clear");
  }
  servoReset(); // reset servo to neutral position
  return rtn;
}
bool checkRight(int deg) {
  return checkRight(deg, TURN_CHECK_DISTANCE);
}
bool checkRight() {
  return checkRight(80); // 80 is used to correct servo
}

// checks right 90 and 45 degrees without reset inbetween
bool checkRightBoth() {
  servoRight();
  bool rtn90 = (checkDistance() < TURN_CHECK_DISTANCE);
  servoRight(45);
  bool rtn45 = (checkDistance() < TURN_CHECK_DISTANCE);
  // disabled 45 check due to false detection of walls,
  // when rover is not perfectly parallel
  bool rtn = rtn90; // (rtn90 && !rtn45) || (!rtn90 && rtn45)
  if (!rtn) {
    Serial.println("Right is clear");
  }
  servoReset(); // reset servo to neutral position
  return rtn;
}

/*
 * MOTOR FUNCTIONS
*/

// turn the rover left a specified amount
void motorsRotateLeft(int dur) {
  motorsStop();
  delay(250);
  motorsLeft();
  motorsStart(TURN_SPEED);
  delay(dur);
  // stop motors and revert to forward direction
  motorsStop();
  delay(250);
  motorsForward();
}
void motorsRotateLeft() {
  motorsRotateLeft(MOTOR_TURN_TIME);
}

// turn the rover right a specified amount
void motorsRotateRight(int dur) {
  motorsStop();
  delay(250);
  motorsRight();
  motorsStart(TURN_SPEED);
  delay(dur);
  // stop motors and revert to forward direction
  motorsStop();
  delay(250);
  motorsForward();
}
void motorsRotateRight() {
  motorsRotateRight(MOTOR_TURN_TIME);
}

// prepare motors for movement in various directions

void motorsForward() {
  setMotor1Direction(false);
  setMotor2Direction(false);
}

void motorsBackward() {
  setMotor1Direction(true);
  setMotor2Direction(true);
}

void motorsLeft() {
  setMotor1Direction(false);
  setMotor2Direction(true);
}

void motorsRight() {
  setMotor1Direction(true);
  setMotor2Direction(false);
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
void motorsStartCalibrated(int speed) {
  // calibration to ensure the rover moves in a straight line
  analogWrite(MOTOR1_ENABLE_PIN, speed+MOTOR1_CALIBRATION);
  analogWrite(MOTOR2_ENABLE_PIN, speed+MOTOR2_CALIBRATION);
}
void motorsStartCalibrated() {
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
 * SERVO FUNCTIONS
*/

// reset servo to facing forward
void servoReset() {
  servo.write(90);
  delay(SERVO_TURN_TIME); // wait for servo to reach position
}

// turn servo left a specified amount
void servoLeft(int deg) {
  servo.write(90+deg);
  delay(SERVO_TURN_TIME); // wait for servo to reach position
}
void servoLeft() {
  servoLeft(90);
}

// turn servo right a specified amount
void servoRight(int deg) {
  servo.write(90-deg);
  delay(SERVO_TURN_TIME); // wait for servo to reach position
}
void servoRight() {
  servoRight(90);
}

/* 
 *  TEST LEVEL DEMO FUNCTIONS
*/

void level_1_1() {
  // move forward
  motorsForward();
  motorsStartCalibrated(MOTOR_SPEED);
  delay(3000);
  // stop 3 seconds
  motorsStop();
  delay(3000);
  // reverse back to starting position
  motorsBackward();
  motorsStartCalibrated(MOTOR_SPEED);
  delay(3000);
  // finish
  motorsStop();
  delay(3000);
}

void level_1_2() {
  // clockwise
  for (int i = 0; i < 4; i++) {
    motorsForward();
    motorsStartCalibrated(MOTOR_SPEED);
    delay(3000);
    motorsStop();
    delay(250);
    motorsRotateRight();  
    delay(250);
  }  
  delay(3000);
  motorsRotateRight();  
  delay(250);
  // anti-clockwise
  for (int i = 0; i < 4; i++) {
    motorsForward();
    motorsStartCalibrated(MOTOR_SPEED);
    delay(3000);
    motorsStop();    
    delay(250);
    motorsRotateLeft();  
    delay(250);  
  }
  delay(3000);
  motorsRotateLeft();    
  delay(250);  
}

// levels 2 and onward can be tested with the autonomousNavigation() function
