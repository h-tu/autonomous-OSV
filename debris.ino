// Hongyu Tu
// Reuse in any form of this code is not allowed

#include "Enes100.h"
#include <Wire.h> //I2C Arduino Library

#define Magnetometer_mX0 0x03
#define Magnetometer_mX1 0x04
#define Magnetometer_mZ0 0x05
#define Magnetometer_mZ1 0x06
#define Magnetometer_mY0 0x07
#define Magnetometer_mY1 0x08

#define pi 3.14

int fu = 3;

//claw
int enA = 11;
int enB = 10;
int in1 = 7;
int in2 = 4;
int in3 = 2;
int in4 = 12;

int count = 0;
int haveChanged = 0;

//
float sig;
#define stretch_Sensor A0

//Left
const int trigPin1 = 3;
#define echoPin1 A0

//middle
const int trigPin2 = 3;
#define echoPin2 A1

//Right
const int trigPin3 = 3;
const int echoPin3 = 13;

const int E1 = 5;    //M1 Speed Control
const int E2 = 6;    //M2 Speed Control
const int M1 = 4;    //M1 Direction Control
const int M2 = 7;    //M2 Direction Control

const int Movin = 200;
const int Wait = 800;

const int rxPin = 8;
const int txPin = 9;

const int markerId = 23;

float x = 0;
float y = 0;

long duration1;
long duration2;
long duration3;
int distance1;
int distance2;
int distance3;

int mX0, mX1, mX_out;
int mY0, mY1, mY_out;
int mZ0, mZ1, mZ_out;
float heading, headingDegrees, headingFiltered, declination;
float Xm, Ym, Zm;
#define Magnetometer 0x1E
float initial = 0;
float mission = 0;

Enes100 enes("OverBuilt & UnderPaid", DEBRIS, markerId, rxPin, txPin);

void setup() {
  Serial.begin(9600);

  Wire.begin();

  Wire.beginTransmission(Magnetometer);
  Wire.write(0x02); // Select mode register
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);

  while (!enes.retrieveDestination()) {
    enes.println("?");
  }
  x = enes.destination.x;
  y = enes.destination.y;
  enes.print("My destination is at (");
  enes.print(enes.destination.x);
  enes.print(", ");
  enes.print(enes.destination.y);
  enes.print(")");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// ****************************************************

void loop() {
  
  enes.updateLocation();

  fix_A(0.5 * pi);
  fix_y(y);
  fix_A(0);
  fix_x(2.6);

  if (haveChanged != 0) {
    fix_A(0.5 * pi);
    fix_y(y);
  }

  fix_A(0);

  initial = getDegree();

  fix_x(x - 0.2); 

  enes.navigated();

  mission = getDegree();

  if (initial - mission >= 1.5) {
    enes.baseObjective(STEEL);
  }
  else {
    enes.baseObjective(COPPER);
  }

  lift();

  enes.bonusObjective(getMass());

  enes.println("Completed!");

  delay(10000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  ****************************************************
float getMass() {
  float a = 0;
  a = analogRead(A2);
  a = (a - 2830.9) / -7.519;
  a -= 260;
  return a;
}

float getDegree() {
  float a = meg();
  delay(1000);
  float b = meg();
  while (b - a > 0.2) {
    a = meg();
    b = meg();
  }
  return (a + b) / 2.0;
}

void lift() {
  letGo();
  delay(1200);
  stopMotors();
  delay(200);

  lowerClaw();
  delay(2000);
  stopMotors();
  delay(200);

  grab();
  delay(1500);
  stopMotors();
  delay(200);

  raiseClaw();
  delay(2000);
  stopMotors();
  delay(5000);

  letGo();
  delay(300);
  stopMotors();
  delay(200);
}

void fix_A(float angle) {
  enes.println("A");
  enes.updateLocation();
  float dif = enes.location.theta - angle;
  if (dif < 0) {
    dif = -1 * dif;
  }
  while ( dif > 0.02 ) {
    if (enes.location.theta > angle) {
      turn_R (235, 235);
    }
    else {
      turn_L (235, 235);
    }
    delay(150);
    stop();
    loc_report();
    delay(100);
    //    enes.println(enes.location.theta);
    dif = enes.location.theta - angle;
    if (dif < 0) {
      dif = -1 * dif;
    }
  }
  loc_report();
  delay(100);
}

void fix_x(float destination) {

  enes.updateLocation();
  float dif = enes.location.x - destination;
  fu ++;
  if (dif < 0) {
    dif = -1 * dif;
  }
  while ( dif > 0.1 ) {
    if (enes.location.x > destination) {
      back (235, 235);
    }
    else {
      advance (235, 235);
    }
    delay(200);
    stop();
    loc_report();

    if (haveChanged == 0) {
      if (enes.location.x > 1.2 && enes.location.x < 2.5) {
        avoid();
      }
    }

    delay(100);

    dif = enes.location.x - destination;
    if (dif < 0) {
      dif = -1 * dif;
    }
  }
  loc_report();
  delay(100);
}

void fix_y(float destination) {
  //  enes.println("Y");
  enes.updateLocation();
  float dif = enes.location.y - destination;
  if (dif < 0) {
    dif = -1 * dif;
  }
  while ( dif > 0.1 ) {
    if ( enes.location.theta > 0) {
      if (enes.location.y > destination ) {
        back (235, 235);
      }
      else {
        advance (235, 235);
      }
    }
    else {
      if (enes.location.y > destination ) {
        advance (235, 235);
      }
      else {
        back (235, 235);
      }
    }

    delay(Movin + 50);
    stop();
    loc_report();
    delay(100);
    dif = enes.location.y - destination;
    if (dif < 0) {
      dif = -1 * dif;
    }
  }
  loc_report();
  delay(100);
}

void go_d() {
  enes.updateLocation();
  fix_A(0.5 * pi);
  fix_y(y);
  fix_A(0);
  fix_x(x - 0.2);
  if (haveChanged != 0) {
    fix_A(0.5 * pi);
    fix_y(y);
  }
}

void avoid() {
  enes.print("Getting in avoid... ");
  int dist = ultra();
  if (dist <= 15 ) {
    haveChanged += 1;
    fix_A(0.5 * pi);
    if (enes.location.y >= 0.8 ) {
      if (enes.location.y >= 1.6) {
        fix_y(1.2);
      }
      else {
        fix_y(1.7);
      }
    }
    else {
      if (enes.location.y <= 0.4) {
        fix_y(0.7);
      }
      else {
        fix_y(0.2);
      }
    }
    fix_A(0);
  }
}

int one(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH);
}

int ultra() {
  duration1 = one(trigPin1, echoPin1);
  distance1 = duration1 * 0.034 / 2;
  if (distance1 < 0 ) {
    distance1 = distance1 * -1;
  }

  duration2 = one(trigPin2, echoPin2);
  distance2 = duration2 * 0.034 / 2;

  if (distance2 < 0 ) {
    distance2 = distance2 * -1;
  }

  duration3 = one(trigPin3, echoPin3);
  distance3 = duration3 * 0.034 / 2;

  if (distance3 < 0 ) {
    distance3 = distance3 * -1;
  }

  delay(100);


  if (distance1 != 0 && distance2 != 0 && distance3 != 0) {
    count = 0;
    enes.print("Left: ");
    enes.println(distance3);
    enes.print("Middle: ");
    enes.println(distance2);
    enes.print("Right: ");
    enes.println(distance1);
    enes.print("Min: ");
    enes.println(min(distance1, distance3));

    delay(100);

    return min(min(distance1, distance2), distance3);
  }
  else {
    count ++;
    enes.println("Reading again... ");
    if (count > 5) {
      fix_x(enes.location.x - 0.05);
      count = 0;
    }
    ultra();
  }
}



void stop(void)                    //Stop
{
  digitalWrite(E1, LOW);
  digitalWrite(E2, LOW);
}

void advance(char a, char b)         //Move forward
{
  analogWrite (E1, a);     //PWM Speed Control
  digitalWrite(M1, HIGH);
  analogWrite (E2, b);
  digitalWrite(M2, HIGH);

}
void back (char a, char b)         //Move backward
{
  analogWrite (E1, a);
  digitalWrite(M1, LOW);
  analogWrite (E2, b);
  digitalWrite(M2, LOW);
}
void turn_L (char a, char b)            //Turn Left
{
  analogWrite (E1, a);
  digitalWrite(M1, LOW);
  analogWrite (E2, b);
  digitalWrite(M2, HIGH);
}
void turn_R (char a, char b)            //Turn Right
{
  analogWrite (E1, a);
  digitalWrite(M1, HIGH);
  analogWrite (E2, b);
  digitalWrite(M2, LOW);
}

void turnL90() {
  turn_L (255, 255);
  delay(1500);
  stop();
  delay(1000);
}

void turnR90() {
  turn_R (255, 255);
  delay(1500);
  stop();
  delay(1000);
}



void lowerClaw() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 200);
}

void raiseClaw() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 200);
}

void grab() {
  digitalWrite(in1, HIGH );
  digitalWrite(in2, LOW);
  analogWrite(enA, 200);
}

void letGo() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH );
  analogWrite(enA, 200);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


float getM() {
  sig = analogRead(stretch_Sensor);
  sig = (sig - 2830.9) / -7.519;
  return sig;
}

void test() {
  enes.println("Start moving...");
  for (int f = 0; f < 8; f++) {
    loc_report();
    if (f < 4) {
      turnL90();
      loc_report();
    }
    else {
      turnR90();
      loc_report();
    }
  }
  advance(255, 255);
  loc_report();
  delay(1000);
  loc_report();
  back(255, 255);
  loc_report();
  delay(1000);
  loc_report();
  stop();
  delay(5000);
  loc_report();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float meg() {
  //---- X-Axis
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mX1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mX0 = Wire.read();
  }
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mX0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mX1 = Wire.read();
  }
  //---- Y-Axis
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mY1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mY0 = Wire.read();
  }
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mY0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mY1 = Wire.read();
  }

  //---- Z-Axis
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mZ1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mZ0 = Wire.read();
  }
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mZ0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mZ1 = Wire.read();
  }

  //---- X-Axis
  mX1 = mX1 << 8;
  mX_out = mX0 + mX1; // Raw data
  // From the datasheet: 0.92 mG/digit
  Xm = mX_out * 0.00092; // Gauss unit
  //* Earth magnetic field ranges from 0.25 to 0.65 Gauss, so these are the values that we need to get approximately.
  //---- Y-Axis
  mY1 = mY1 << 8;
  mY_out = mY0 + mY1;
  Ym = mY_out * 0.00092;
  //---- Z-Axis
  mZ1 = mZ1 << 8;
  mZ_out = mZ0 + mZ1;
  Zm = mZ_out * 0.00092;
  // ==============================
  //Calculating Heading
  heading = atan2(Ym, Xm);

  // Correcting the heading with the declination angle depending on your location
  // You can find your declination angle at: https://www.ngdc.noaa.gov/geomag-web/
  // At my location it's 4.2 degrees => 0.073 rad
  declination = 0.073;
  heading += declination;

  // Correcting when signs are reveresed
  if (heading < 0) heading += 2 * PI;
  // Correcting due to the addition of the declination angle
  if (heading > 2 * PI)heading -= 2 * PI;
  headingDegrees = heading * 180 / PI; // The heading in Degrees unit
  // Smoothing the output angle / Low pass filter
  headingFiltered = headingFiltered * 0.85 + headingDegrees * 0.15;

  return headingFiltered;
  //Sending the heading value through the Serial Port to Processing IDE
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loc_report() {
  if (enes.updateLocation()) {
    enes.println("-");
    enes.print("Current Loc: (");
    enes.print(enes.location.x);
    enes.print(", ");
    enes.print(enes.location.y);
    enes.print(", ");
    enes.print(enes.location.theta);
    enes.println(")");
    enes.print("Destination: (");
    enes.print(x);
    enes.print(", ");
    enes.print(y);
    enes.println(")");
    enes.print("Differences: (");
    enes.print(x - enes.location.x);
    enes.print(", ");
    enes.print(y - enes.location.y);
    enes.println(")");
  }
  else {
    enes.println("Update fail...");
    loc_report();
  }
}
