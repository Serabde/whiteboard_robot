/***********************************************************************
   Rogue 2 Robot Driving Code
   Updated 2018-05-29
   Julia Stoddard
 ***********************************************************************/
// Libraries
#include <RedBot.h>
#include <NewPing.h>

// Constants
#define TRIGGER_PIN A4
#define ECHO_PIN A5
#define MAX_DISTANCE 200 //Maximum valid distance that will register on the sensor, anything above this distance will just show up as this distance (cm)

// Objects
RedBotMotors motors;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

//Initialize Encoder Variables
RedBotEncoder encoder = RedBotEncoder(A2, 10);
int buttonPin = 12;
int countsPerRev = 1192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

float wheelDiam = 3.2;  // diam = 65mm / 25.4 mm/in
float wheelCirc = PI * wheelDiam; // Redbot wheel circumference = pi*D
float wheelDist = 1.75; //distance (inches) from center of chassis to edge of wheel
float distance = PI * 0.5 * wheelDist; //distance to travel for 1/4 of a rotation

//Initializes Ultrasonic Sensor Variables
//const int trigPin = A4;
//const int echoPin = A5;
//int trigPinState = LOW;
unsigned long previousMillis = 0;
long OnTime = 10;
long OffTime = 2;
long duration;
long Distance;
long prevDistance;
long distanceDiff;
int OnOff = 0;

//Initializes counter variables for turning
int i = 0;
int j = 0;

// Keep track of last sensor reading time
unsigned long lastReadingTime = 0;
const int READ_SENSOR_INTERVAL_MS = 10; // Read the sensor every RSIM milliseoncsds
unsigned int distances[50]; // Array to store distances

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP); //Initializes Button Pin
  //pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  //pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600);
}

void loop()
{

  newReadDistance();



  /* if (Distance != 4) //change  distance to whatever distance from sensor to whiteboard will be
     {
       if ((i%2) == 0) //if i is even
         {
         turn (-125,125);    //turn left
         delay(200);         //short delay
         driveStraight(150); //drive straight, change delay number in next line of code to drive longer or shorter
         delay(200);
         turn(-125,125);
         }
      if ((i%2) != 0)
         {
         turn (125,-125);    //turn right
         delay(200);         //short delay
         driveStraight(150); //drive straight, change delay number in next line of code to drive longer or shorter
         delay(200);
         turn(125,-125);
         }
     }*/
}


void newReadDistance(){
  float currentDistance = sonar.ping_cm();
  
}

//Gather Ultrasonic Sensor Data
/*
void ReadDistance()
{
  i = 0;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  Distance = duration * 0.034 / 2; //Key distance variable for ultrasonic sensing
  distanceDiff = (Distance - prevDistance);
  int Dist[10];
  for (i = 0; i < 10; i++) {
    Dist[i] = Distance;
  }
  Serial.print(Dist[1]);
  Serial.print(" || ");
  Serial.print(Dist[9]);
  Serial.println();
  Serial.println(i);

  if (abs(Dist[9] - Dist[0]) > 2 && Distance < 500)
  {
    j++;
    Serial.println("i= ");
    Serial.println(j);
    //motors.drive(100);
    //delay(100);
    //motors.brake();
  }
  else
  {
    Serial.println("Straight Drive");
  }
}
 */



/*

  //Turn 90 degrees. |LmotorPower| = |RmotorPower|. To turn LEFT: L- / R+. To turn RIGHT: L+ / R-.
  void turn (int LmotorPower,int RmotorPower)
  {
  long lCount = 0;
  long rCount = 0;
  long targetCount;
  float numRev;

  long prevlCount, prevrCount;

  long lDiff, rDiff;  // diff between current encoder count and previous count

  int leftPower = LmotorPower;
  int rightPower = RmotorPower;

  int offset = 5;  // offset amount to compensate Right vs. Left drive

  numRev = distance / wheelCirc;  // calculate the target # of rotations
  targetCount = numRev * countsPerRev;    // calculate the target count

  encoder.clearEnc(BOTH);    // clear the encoder count
  delay(100);  // short delay before starting the motors.

  motors.leftMotor(leftPower);  // spin CCW
  motors.rightMotor(rightPower); // spin CCW

  while (lCount < targetCount)
  {
    // while the right encoder is less than the target count -- debug print
    // the encoder values and wait -- this is a holding loop.
    lCount = abs(encoder.getTicks(LEFT));
    rCount = abs(encoder.getTicks(RIGHT));

    motors.leftDrive(leftPower);
    motors.rightDrive(rightPower);

    // calculate the rotation "speed" as a difference in the count from previous cycle.
    lDiff = (lCount - prevlCount);
    rDiff = (rCount - prevrCount);

    // store the current count as the "previous" count for the next cycle.
    prevlCount = lCount;
    prevrCount = rCount;

    // if left is faster than the right, slow down the left / speed up right IF robot is making a RIGHT turn
    if (lDiff > rDiff && LmotorPower > RmotorPower)
    {
      leftPower = leftPower - offset;
      rightPower = rightPower - offset;
    }
    // if right is faster than the left, speed up the left / slow down right IF robot is making a RIGHT turn
    else if (lDiff < rDiff && LmotorPower > RmotorPower)
    {
      leftPower = leftPower + offset;
      rightPower = rightPower + offset;
    }
     // if left is faster than the right, slow down the left / speed up right IF robot is making a LEFT turn
     if (lDiff > rDiff && LmotorPower < RmotorPower)
    {
      leftPower = leftPower + offset;
      rightPower = rightPower + offset;
    }
    // if right is faster than the left, speed up the left / slow down right IF robot is making a LEFT turn
    else if (lDiff < rDiff && LmotorPower < RmotorPower)
    {
      leftPower = leftPower - offset;
      rightPower = rightPower - offset;
    }
    delay(50);  // short delay to give motors a chance to respond.
  }
  // now apply "brakes" to stop the motors.
  motors.brake();
  i++; //increment i counter
  }


  //Drive Straight
  void driveStraight(int motorPower)
  {
  long lCount = 0;
  long rCount = 0;
  long prevlCount, prevrCount;
  long lDiff, rDiff;  // diff between current encoder count and previous count
  int leftPower = motorPower;
  int rightPower = motorPower;
  int offset = 5;  // offset amount to compensate Right vs. Left drive

  encoder.clearEnc(BOTH);    // clear the encoder count
  delay(100);  // short delay before starting the motors.

    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);

    //Serial.print(lCount);
   // Serial.print("\t");
   // Serial.print(rCount);
  //  Serial.print("\t");

    motors.leftDrive(leftPower);
    motors.rightDrive(rightPower);

    lDiff = (lCount - prevlCount);
    rDiff = (rCount - prevrCount);

    prevlCount = lCount;
    prevrCount = rCount;

    // if left is faster than the right, slow down the left / speed up right
    if (lDiff > rDiff && lDiff >0)
    {
      leftPower = leftPower - offset;
      rightPower = rightPower + offset;
    }
    else if (abs(lDiff) > rDiff && lDiff <0)
      {
        leftPower = leftPower + offset;
        rightPower = rightPower + offset;
      }
    // if right is faster than the left, speed up the left / slow down right
    if (lDiff < rDiff && lDiff >0)
    {
      leftPower = leftPower + offset;
      rightPower = rightPower - offset;
    }
    else if (abs(lDiff)<rDiff && lDiff <0)
      {
        leftPower = leftPower - offset;
        rightPower = rightPower - offset;
      }
    delay(50);  // short delay to give motors a chance to respond.
  }

*/
