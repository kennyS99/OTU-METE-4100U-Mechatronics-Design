#include <AFMotor.h>
#include <util/atomic.h>
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float u;
int pos = 0;
AF_DCMotor motor_Left(1);
AF_DCMotor motor_Right(2);

const byte encoder0pinA = 18; // A pin -> the left motor
int encoder0pinB = 26;
const byte encoder1pinA = 19; // B pin -> The right motor
int encoder1pinB = 27;
// Variables used
byte encoder0PinALast;  // encoderPinA last pulse
int pulseCountLeft = 0; // the number of the pulses
boolean Direction;      // the rotation direction

// Variables used
byte encoder1PinALast;   // encoderPinA last pulse
int pulseCountRight = 0; // the number of the pulses
boolean Direction1;      // the rotation direction

int ppr_left = 2850;   // pulse per rotation
// int ppr_right = 2800;   // pulse per rotation
//ppr 2920, turn 90 degree, 1336
//ppr 2800, turn 90 degree, 1281
// float pivotD = 6; // Pivot Diameter cm
float tireD = 5.9; // tire diameter
float pi = 3.14; // Pi
// float C = pi * pivotD; // Circumference of the base diameter
float revDistance = pi * 10.8;              // distance travelled in 1 revolution
float revTire = revDistance / (pi * tireD); // per tire need to rotate for one robot rotate

boolean a;

void setup() {
  // Begin the serial port
  Serial.begin(57600);

  // Encoders
  pinMode(encoder0pinA, INPUT_PULLUP);
  pinMode(encoder1pinA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder0pinA), motorOnePulse, CHANGE); // attach the interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1pinA), motorTwoPulse, CHANGE); // attach the interrupt
  //checkPulse_moveleft(90);
  a = true;
  Serial.println(a);
}

void loop() {


robotForward(150,150);
delay(2000);
robotStop();
delay(2000);
checkPulse_moveright(90);
robotStop();
delay(2000);

}

void robotForward(int speed_left, int speed_right){
 motor_Left.setSpeed(speed_left);
 motor_Right.setSpeed(speed_right);

 motor_Left.run(BACKWARD);
 motor_Right.run(FORWARD);
}

void robotBackward(int speed_left, int speed_right){
 motor_Left.setSpeed(speed_left);
 motor_Right.setSpeed(speed_right);

 motor_Left.run(FORWARD);
 motor_Right.run(BACKWARD);
}

void robotStop(){
 motor_Left.run(RELEASE);
 motor_Right.run(RELEASE);
}

void robot_turnLeft(){
 motor_Left.setSpeed(255);
 motor_Right.setSpeed(255);

 motor_Left.run(FORWARD);
 motor_Right.run(FORWARD);
}

void robot_turnRight(){
 motor_Left.setSpeed(255);
 motor_Right.setSpeed(255);

 motor_Left.run(BACKWARD);
 motor_Right.run(BACKWARD);
}

void motorOnePulse()
{
  int Lstate = digitalRead(encoder0pinA);
  if ((encoder0PinALast == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if (val == LOW && Direction)
    {
      Direction = false; // Reverse
    }
    else if (val == HIGH && !Direction)
    {
      Direction = true; // Forward
    }
  }
  encoder0PinALast = Lstate;

  if (!Direction)
    pulseCountLeft++;
  else
    pulseCountLeft--;
}

// Function which gets the number of pulses of the second motor
void motorTwoPulse()
{
  int Lstate = digitalRead(encoder1pinA);
  if ((encoder1PinALast == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(encoder1pinB);
    if (val == LOW && Direction1)
    {
      Direction1 = false; // Reverse
    }
    else if (val == HIGH && !Direction1)
    {
      Direction1 = true; // Forward
    }
  }
  encoder1PinALast = Lstate;

  if (!Direction1)
    pulseCountRight++;
  else
    pulseCountRight--;
}

void checkPulse_moveleft(float angle)
{
  pulseCountLeft = 0;
  a = true;
   float turns = (angle / 360) * revTire;
   int pulses = turns * ppr_right;
   
  while (a == true)
    {
      pid_turning(pulses);
    if(u < 0){
    robot_turnLeft();
   }else if (u > 0)
   {
    robot_turnRight();
   }
    //Serial.println(pulseCountRight);
    robotStop();
    a = false;
    }
}

void checkPulse_moveright(float angle)
{
  pulseCountLeft = 0;
  a = true;
   float turns = (angle / 360) * revTire;
   int pulses = turns * ppr_right;
   pid_turning(pulses);
  while (a == true)
  {
    pid_turning(pulses);
    if(u < 0){
    robot_turnRight();
   }else if (u > 0)
   {
    robot_turnLeft();
   } else{
    robotStop();
    a = false;
   }
  }
}
void pid_turning(float target){
  float kp = 4;
  float kd = 0.4;
  float ki = 0.0;
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
//   ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//     pos = abs(pulseCountRight);
//   }
  pos = abs(pulseCountLeft);
  Serial.println(pulseCountLeft);
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  u = kp*e + kd*dedt + ki*eintegral;
  // store previous error
  eprev = e;
}