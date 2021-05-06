#include <Servo.h>            // note: use of this library disables PWM for pin 9 and 10
// source: Standard Arduino Library
#include <NewPing.h> // used for ultrasonic functions. Note: not compatable with TinkerCAD
// source: https://bitbucket.org/teckel12/arduino-new-ping/src/master/src/NewPing.cpp

// python read serial to csv: https://www.learnrobotics.org/blog/arduino-data-logger-csv/

#define ECHO_PIN 7            // ultrasonic echo (read)
#define TRIG_PIN 8            // ultrasonic trigger (write)
#define SERVO_PIN 11          // servo (write)

// servo
Servo servo;

// ultrasonic detects up to 336cm away - 5cm for error
const long DISTANCE_LIMIT = 336 - 5;

NewPing sonar(TRIG_PIN, ECHO_PIN, DISTANCE_LIMIT);

void setup()
{
    servo.attach(SERVO_PIN, 500, 2500);
    servo.write(90);
    delay(500);
    Serial.begin(9600);
    Serial.println("Ultrasonic");
}

void loop()
{
    Serial.print(sonar.convert_cm(sonar.ping_median(5)));
    Serial.println("cm");
    delay(500);
}
