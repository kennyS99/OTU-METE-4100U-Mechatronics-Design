#include "Adafruit_VL53L0X.h"
#include <PID_v1.h>

char startWall = 'L'; // Variable for Which wall to follow
int turns = 3;       // Specify when to switch from left wall to right wall. These are right turns. Used for positioning

// The right motor
// Backwards -> 1 HIGH, 2 LOW, Forwards -> 1 -> LOW, 2 -> HIGH
int IN1 = 24;
int IN2 = 25;
int ENA = 9;

// The left motor
// Backwards -> 1 HIGH, 2 LOW, Forwards -> 1 -> LOW, 2 -> HIGH
int IN3 = 26;
int IN4 = 27;
int ENB = 10;

int irPin = 35; // IR obstacle avoidance sensor

const byte encoder0pinA = 2; // A pin -> the left motor
int encoder0pinB = 4;
const byte encoder1pinA = 3; // B pin -> The right motor
int encoder1pinB = 5;

// Variables used
byte encoder0PinALast;  // encoderPinA last pulse
int pulseCountLeft = 0; // the number of the pulses
boolean Direction;      // the rotation direction

// Variables used
byte encoder1PinALast;   // encoderPinA last pulse
int pulseCountRight = 0; // the number of the pulses
boolean Direction1;      // the rotation direction

int ppr_left = 2850;   // pulse per rotation
int ppr_right = 2920;   // pulse per rotation
int speedT = 150; // Speed
// float pivotD = 6; // Pivot Diameter cm
float tireD = 5.9; // tire diameter
float pi = 3.1415; // Pi
// float C = pi * pivotD; // Circumference of the base diameter
float revDistance = pi * 10.6;              // distance travelled in 1 revolution
float revTire = revDistance / (pi * tireD); // per tire need to rotate for one robot rotate

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

int baseSpeed = 100; // BaseSpeed for the wallfollow algorithm
int speedLeft, speedRight;
int counter = 0; // counter for the positioning of the robot

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
double Kp = 6, Ki = 10, Kd = 0.8;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  // put your setup code here, to run once:

  // Motor right pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Motor left pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Begin the serial port
  Serial.begin(57600);

  // Encoders
  pinMode(encoder0pinA, INPUT_PULLUP);
  pinMode(encoder1pinA, INPUT_PULLUP);
  // Interrupt that is triggered each type output A rises
  // The attach interrupt function has three inputs
  // Firstly,digitalPinToInterrupt(pin):- Pin number of the interrupt,
  // which tells the microprocessor which pin to monitor.
  // Second argument is what function I want to call after  interrupt
  // Third argument is mode :- defines when the interrupt should be triggered.
  // Four constants are predefined as valid values:*
  // LOW to trigger the interrupt whenever the pin is low,*
  // CHANGE to trigger the interrupt whenever the pin changes value*
  // RISING to trigger when the pin goes from low to high,*
  // FALLING for when the pin goes from high to low.

  attachInterrupt(digitalPinToInterrupt(encoder0pinA), motorOnePulse, CHANGE); // attach the interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1pinA), motorTwoPulse, CHANGE); // attach the interrupt

  pinMode(irPin, INPUT);

  // Shutdown pins for the LOX sensors
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  // Both in reset mode
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  setID(); // Setting the ID's for the TOF sensors to work both at the same time.

  if (startWall == 'R')
  {
    // If we are following the right wall then input is the right TOF sensor
    Input = measureRight.RangeMilliMeter;
  }
  else if (startWall == 'L')
  {
    // If we are following the left wall then input is the right TOF sensor
    Input = measureLeft.RangeMilliMeter;
  }

  // Setpoint distance from the wall for the PID.
  Setpoint = 45;

  // turn the PID on
  myPID.SetMode(AUTOMATIC);

  // Delay the robot start
  delay(1000);
}

void loop()
{

  // Read if there is nothing in the front
  int frontVal = digitalRead(irPin);

  // If there is then check if there is something on the sides
  // frontVal = 1 no obstacles
  // frontVal = 0 obstacle
  if (frontVal == 0)
  {
    checkWall(); // Checks the wall according to which wall it is following
  }
  else
  {
    // Set the speeds according to the PID.
    setSpeeds();

    analogWrite(ENB, speedLeft);
    analogWrite(ENA, speedRight);

  }
  
}
// Check for walls on the right or left and make turns accordingly
void checkWall()
{
  read_dual_sensors(); // Read both the TOF sensors

  if (startWall == 'R')
  {

      // If not then turn left 90 degrees
      moveLeft(90);
      motorStop();
      delay(500);
      counter++; // Increase the counter which keeps track of turns
    
  }
  else if (startWall == 'L')
  {
    // If following the left wall

      // If not then turn right 90 degrees
      moveRight(90);
      motorStop();
      delay(500);
      counter++; // Increase the counter which keeps track of turns
      
    //---------Robot Positioning in the Maze ---------------------//
    // If the robot needs to switch to right wall follow then switch
    if (counter == turns)
    {
      motorStop();
      delay(500);
      moveForward(2);
      motorStop();
      delay(4000);
      startWall = 'R';
    //-----------------------------------------------------------//
  }
}
}

// Function that sets the speeds for the 2 motors
void setSpeeds()
{
  // Measure the distance
  measureDistance();

  // Run the PID controller
  myPID.Compute();

  if (startWall == 'R')
  {
    // Set the speeds of both motors according to the PID. Experimentally determined
    speedLeft = baseSpeed + 40 - (int)(Output / 2); // Right speed should be more as we want to follow the left wall.
    speedRight = baseSpeed + (int)(Output / 2);     // Left speed should be higher when it is closer to the wall.

  }
  else if (startWall == 'L')
  {
    // Set the speeds of both motors according to the PID. Experimentally determined

    speedRight = baseSpeed + 20 - (int)(Output / 2); // Right speed should be more as we want to follow the left wall.
    speedLeft = baseSpeed + (int)(Output / 2);     // Left speed should be higher when it is closer to the wall.
  }
  else
  {
    // Stop the robot
    speedLeft = 0;
    speedRight = 0;
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
  motorForward();
}

// Function for measuring the distance of the TOF sensor
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
      average = (measureRight.RangeMilliMeter - 20)  + average;
    }
    
    Input = average/5;
    Serial.print("Distance_right:");
    Serial.println(Input);
  }
  else if (startWall == 'L')
  {
    for (int i = 0; i < 5; i++)
    {
      average = (measureLeft.RangeMilliMeter - 5)  + average;
    }
    Input = average/5;
    Serial.print("Distance_left:");
    Serial.println(Input);
  }
}
//////////////// For testing /////////////
void robotForward(int Right, int Left)
{
  // Motor direction pins
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, Right);
  analogWrite(ENB, Left);
}
/////////////////////////////
void motorForward()
{
  // Motor direction pins
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void motorStop()
{
  // Motor direction pins
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // analogWrite(ENA,255);
  // analogWrite(ENB,255);
  Serial.println("Stoping");
}

void moveForward(float turns)
{
  // Motor direction pins
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  checkPulse_moveright(turns);
}

void moveBackward(int turns)
{
  // Motor direction pins
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  checkPulse_moveright(turns);
}

void moveRight(float angle)
{
  // Motor direction pins
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  pulseCountLeft = 0;

  float turns = (angle / 360) * revTire;

  checkPulse_moveright(turns);
}

void moveLeft(float angle)
{
  // Motor direction pins
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Reset the pulseCountLeft
  pulseCountRight = 0;

  float turns = (angle / 360) * revTire;

  checkPulse_moveleft(turns);
}
void checkPulse_moveleft(float turns)
{
  int pulses = turns * ppr_right;

  while (abs(pulseCountRight) < pulses)
  {
    Serial.println(pulseCountRight);
    analogWrite(ENA, speedT);
    analogWrite(ENB, speedT);
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void checkPulse_moveright(float turns)
{
  int pulses = turns * ppr_left;

  while (abs(pulseCountLeft) < pulses)
  {
    Serial.println(pulseCountLeft);
    analogWrite(ENA, speedT);
    analogWrite(ENB, speedT);
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
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

// Function which gets the number of pulses of the motor
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

// Function for picking up the miner
/* void pickMiner(){
    moveRight(1);
    gripperUp();
    gripperOpen();
    gripperDown();
    gripperClose(180);
    gripperUp();
    gripperOpen();
}

// Min is 150 max is 60
void gripperClose(int angleClose){
  int angle = gripper.read();
  for (pos = angle; pos <= angleClose; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    gripper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

void gripperOpen(){
  int angle = gripper.read();
  for (pos = angle; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    gripper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

void gripperDown(){
  int angle = top.read();
  for (pos = angle; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    top.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

void gripperUp(){
  int angle = top.read();
  for (pos = angle; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    top.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}
*/