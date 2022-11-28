#include <AFMotor.h>
#include <PID_v1.h>
#include <Adafruit_VL53L0X.h>
#include <SharpIR.h>
#include<HardwareSerial.h>
#include <elapsedMillis.h>

//-------------------------------------------------------------------//
//Motor and econder define
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
//-------------------------------------------------------------------//

//-------------------------------------------------------------------//
//Turning parameter define
// int ppr_left = 2920;   // pulse per rotation
// int ppr_right = 3000;   // pulse per rotation
int ppr_left = 2920;
int ppr_right = 2900;
//ppr 2920, turn 90 degree, 1336
//ppr 2800, turn 90 degree, 1281
// float pivotD = 6; // Pivot Diameter cm
float tireD = 5.9; // tire diameter
float pi = 3.14; // Pi
// float C = pi * pivotD; // Circumference of the base diameter
float revDistance = pi * 10.8;              // distance travelled in 1 revolution
float revTire = revDistance / (pi * tireD); // per tire need to rotate for one robot rotate
boolean a;

elapsedMillis timeElapsed;
unsigned int interval; //one minute in ms
//-------------------------------------------------------------------//

//-------------------------------------------------------------------//
//ToF sensor define
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 39
#define SHT_LOX2 38

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measureRight; // Right
VL53L0X_RangingMeasurementData_t measureLeft;  // Left
//-------------------------------------------------------------------//


//-------------------------------------------------------------------//
//Sharp IR distance sensor
#define IR A0 // define signal pin
#define model 215 // used 1080 because model GP2Y0A21YK0F is used
SharpIR SharpIR(IR, model);
//-------------------------------------------------------------------//


//-------------------------------------------------------------------//
// Wall following PID parameters
double Setpoint, Input, Output;
double Kp = 6, Ki = 10, Kd = 0.8;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int baseSpeed = 100; // BaseSpeed for the wallfollow algorithm
int speedLeft, speedRight;
int counter = 0; // counter for the positioning of the robot

// Turning PID parameters
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float u;
int pos = 0;
//-------------------------------------------------------------------//
char startWall; // Variable for Which wall to follow
char turning;

void setup() {
  // Begin the serial port
  Serial.begin(57600);

  // Encoders
  pinMode(encoder0pinA, INPUT_PULLUP);
  pinMode(encoder1pinA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder0pinA), motorOnePulse, CHANGE); // attach the interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1pinA), motorTwoPulse, CHANGE); // attach the interrupt
  
  // Shutdown pins for the LOX sensors
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  // Both in reset mode
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  setID(); // Setting the ID's for the TOF sensors to work both at the same time.

  // Setpoint distance from the wall for the PID. unit mm
  Setpoint = 50;

  // turn the PID on
  myPID.SetMode(AUTOMATIC);

  // Delay the robot start
  delay(1000);

  startWall = 'L';
  turning = 'N'; //turning algrithms(N means normal case, C means coner cases)
}
boolean turnTaken = true;
void loop() {
  int frontDistance = SharpIR.distance();
  Serial.print("Front distance:");
  Serial.println(frontDistance);
  if (turning == 'N')
  {
    if (frontDistance <= 2)
    { 
        robotForward(150,150);
        delay(500);
        robotStop();
        delay(500);
        checkWall();
      } else{
        setSpeeds();
    }
  }else if (turning == 'C')
  {
    if (frontDistance <= 2)
    { 
        robotForward(150,150);
        delay(500);
        robotStop();
        delay(500);
        checkWall_Compared();
      } else{
        setSpeeds();
    }
  }else if (turning == 'B')
  { 
    if (frontDistance <= 40)
  {
    robotStop();
    delay(5000);
    turning = 'N';
  }else{
    setSpeeds();
  }
  }
  
    
  if(turnTaken == true){
    switch (counter) {
      case 1:
        startWall = 'R';
        robotForward(150,150);
        delay(500);
        robotStop();
        delay(500);
        turnTaken = false;
        break;
      case 2:
        setSpeeds_delay(3);
        turning = 'B';
        turnTaken = false;
        break;
      case 7:
        startWall = 'L';
        turning = 'N';
        turnTaken = false;
        break;
      case 10:
        turning = 'B';
        turnTaken = false;
        break;
    }
  }
}

void checkWall()
{
  read_dual_sensors(); // Read both the TOF sensors
  
  
  if (startWall == 'R')
  {
      // If not then turn left 90 degrees
      robotBackward(30,30);
      delay(350);
      PID_moveleft(90);
      robotStop();
      delay(500);
      turnTaken = true;
  }
  else if (startWall == 'L')
  {
    // If following the left wall

      // If not then turn right 90 degrees
      robotBackward(30,30);
      delay(350);
      PID_moveright(90);
      robotStop();
      delay(500);
      turnTaken = true;
      
  }
}

void checkWall_Compared(){
  if (turning == 'C')
  {
      if (measureRight.RangeMilliMeter < 100 && measureLeft.RangeMilliMeter > 130)
  {
        // If not then turn left 90 degrees
    robotBackward(30,30);
    delay(350);
    PID_moveleft(90);
    robotStop();
    delay(500);
    turnTaken = true;
  } else if (measureRight.RangeMilliMeter > 130 && measureLeft.RangeMilliMeter < 100)
  {
    // If following the left wall

    // If not then turn right 90 degrees
    robotBackward(30,30);
    delay(350);
    PID_moveright(90);
    robotStop();
    delay(500);
    turnTaken = true;
  }
  } else if (turning == 'B')
  {
    if (measureRight.RangeMilliMeter > 300 && measureLeft.RangeMilliMeter < 100)
  {
    // If following the left wall

    // If not then turn right 90 degrees
    robotBackward(30,30);
    delay(350);
    PID_moveright(90);
    robotStop();
    delay(500);
    counter++; // Increase the counter which keeps track of turns
    turnTaken = true;
  }
  }

}

void setSpeeds()
{
  // Measure the distance
  measureDistance();
  Output = 0;
  // Run the PID controller
  myPID.Compute();
  
  if (startWall == 'R')
  {
    Serial.println("This is right follower");
    // Set the speeds of both motors according to the PID. Experimentally determined
    speedLeft = (baseSpeed + 40) - (int)(Output / 2); // Right speed should be more as we want to follow the left wall.
    speedRight = baseSpeed + (int)(Output / 2);     // Left speed should be higher when it is closer to the wall.

  }
  else if (startWall == 'L')
  {
    // Set the speeds of both motors according to the PID. Experimentally determined
    Serial.println("This is left follower");
    speedRight = (baseSpeed + 30) - (int)(Output / 2); // Right speed should be more as we want to follow the left wall.
    speedLeft = baseSpeed + (int)(Output / 2);     // Left speed should be higher when it is closer to the wall.
  }
  else
  {
    // Stop the robot
    robotStop();
  }

  // Check the range of Output as it is from 0 to 255.
  if (speedLeft > 255)
  {
    speedLeft = 255;
  }
  else if (speedLeft < 0)
  {
    speedLeft = 0;
  }

  // Check the range of Output as it is from 0 to 255.
  if (speedRight > 255)
  {
    speedRight = 255;
  }
  else if (speedRight < 0)
  {
    speedRight = 0;
  }

  // Set the motors to move straight and writing the speed to the motors.
  robotForward(speedLeft,speedRight);
}

void setSpeeds_delay(int delayTime){
  timeElapsed = 0;
  interval = delayTime * 1000;
  while (timeElapsed < interval)
  {
    setSpeeds();
  }
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

void PID_moveleft(float angle)
{
  pulseCountLeft = 0;
  a = true;
  float turns = (angle / 360) * revTire;
  int pulses = turns * ppr_left;
  while (a == true)
  {
    pid_turning(pulses);
    if(u < 0){
    robot_turnLeft();
   }else if (u > 0)
   {
    robot_turnRight();
   } else{
    robotStop();
    a = false;
   }
  }
   counter++; // Increase the counter which keeps track of turns
}
void PID_movebackward(float turns)
{
  pulseCountLeft = 0;
  a = true;
  int pulses = turns * ppr_left;
  while (a == true)
  {
    pid_turning(pulses);
    if(u < 0){
    robotBackward(100,110);
   }else if (u > 0)
   {
    robotForward(100,110);
   } else{
    robotStop();
    a = false;
   }
  }
}
void PID_moveforward(float turns)
{
  pulseCountLeft = 0;
  a = true;
  int pulses = turns * ppr_left;
  while (a == true)
  {
    pid_turning(pulses);
    if(u < 0){
    robotForward(100,100);
    
   }else if (u > 0)
   {
    robotBackward(100,100);
   } else{
    robotStop();
    a = false;
   }
  }
}
void PID_moveright(float angle)
{
  pulseCountLeft = 0;
  a = true;
  float turns = (angle / 360) * revTire;
  int pulses = turns * ppr_right;
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
   counter++; // Increase the counter which keeps track of turns
}
void pid_turning(float target){
  float kp = 10;
  float kd = 2;
  float ki = 0;
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
  // Serial.println(pulseCountLeft);
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

void measureDistance()
{
  double average;
  // Take reading from both the sensors
  read_dual_sensors();
  
  // Set the input to the PID according to which wall we are following
  if (startWall == 'R')
  {
    for (int i = 0; i < 5; i++)
    {
      average = measureRight.RangeMilliMeter + average;
    }
    Input = average/5;
  }
  else if (startWall == 'L')
  {
    for (int i = 0; i < 5; i++)
    {
      average = measureLeft.RangeMilliMeter + average;
    }
    Input = average/5;
  }
}
void setID()
{
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // initing LOX2
  if (!lox2.begin(LOX2_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
}

void read_dual_sensors()
{
  lox1.rangingTest(&measureRight, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measureLeft, false);  // pass in 'true' to get debug data printout!
}