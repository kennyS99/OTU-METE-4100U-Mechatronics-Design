#include <Servo.h>
#include <AFMotor.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object
}

void loop() {

    // in steps of 1 degree
    myservo.write(0);              // tell servo to go to position in variable 'pos'
    delay(2000);                       // waits 15ms for the servo to reach the position


    myservo.write(180);              // tell servo to go to position in variable 'pos'
    delay(2000);                       // waits 15ms for the servo to reach the position
  }
