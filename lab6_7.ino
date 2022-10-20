#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h"

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;
Servo servo;
Sonar sonar(4);

#define minOutput -100
#define maxOutput 100
#define baseSpeed 200
#define kp_line 20
#define kd_line 2
#define kp_obs 40
#define kd_obs 5
#define NEAR 10.0
#define FOLLOW 1
#define AVOID 2
#define MAX(arr) _max(arr, sizeof(arr))
#define MIN(arr) _min(arr, sizeof(arr))

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);
PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);
int turnTime = 1200;
int maxDark = 500;
int maxLight = 500;
int triggerBlack;
int mode; // 1 = follow mode, 2 = avoid mode
//int facing; // 0 = straight, 1 = right, 2 = left
float keepDistance = 8.0;
float blackScale = 1.2;
float blackOffset = 20;

void initAvoid();
void initFollow();

uint16_t _max(uint16_t arr[], int len)
{
  uint16_t maxx = 0;
  for(int i = 0; i < len; i++)
  {
    if(arr[i] > maxx)
      maxx = arr[i];
  }
  return maxx;
}

uint16_t _min(uint16_t arr[], int len)
{
  uint16_t minn = 65536;
  for(int i = 0; i < len; i++)
  {
    if(arr[i] < minn)
      minn = arr[i];
  }
  return minn;
}

void calibrateSensors()
{
  uint16_t calVals[5];
  lineSensors.calibrate();
  lineSensors.readCalibrated(calVals);
  for(int i = 0; i < 5; i++)
  {
    if(calVals[i] > maxLight)
    {
      maxLight = calVals[i];
    }
    if(calVals[i] < maxDark)
    {
      maxDark = calVals[i];
    }
  }
  triggerBlack = maxDark > 0 ? blackScale * maxDark : maxDark + blackOffset;
}

int calculateHeuristic(uint16_t* sensorVals)
{
  int diffMid = 0.75*(sensorVals[2] - maxDark);
  int diffInner = 0.5*(sensorVals[1] - sensorVals[3]);
  int diffOuter = 0.25*(sensorVals[0] - sensorVals[4]);
  return diffMid + diffInner + diffOuter;
}

void calibrateSpeeds(int u)
{
  motors.setSpeeds(baseSpeed + u, baseSpeed - u);
}

bool objectNear()
{
  float dist = sonar.readDist();
  if(dist < NEAR)
  {
    float avg = 0.0;
    for(int i = 0; i < 10; i++)
    {
      avg += sonar.readDist();
    }
    avg /= 10;
    if(avg < NEAR)
      return true;
  }
  return false;
}

bool onBlackLineAgain()
{
  uint16_t sensorVals[5];
  lineSensors.read(sensorVals);
  if(MIN(sensorVals) <= triggerBlack)
  {
    float avg = 0.0;
    for(int i = 0; i < 10; i++)
    {
      lineSensors.read(sensorVals);
      avg += MIN(sensorVals);
    }
    avg /= 10;
    if(avg <= triggerBlack)
      return true;
  }
  return false;
}

void followLine()
{
  uint16_t sensorVals[5];
  lineSensors.read(sensorVals);
  int val = calculateHeuristic(sensorVals);
  int u = pd_line.update(val, 0);
  calibrateSpeeds(u);
  delay(100);
}

void avoidObject()
{
  float dist = sonar.readDist();
  int u = pd_obs.update(dist, keepDistance);
  calibrateSpeeds(u);
  delay(100);
}

void setup()
{
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90); // turn servo forward
  delay(2000);
  calibrateSensors();
  mode = FOLLOW;
  //facing = 0;
}

void loop()
{
  // stage 1
  followLine();
 /* stage 2 
 if(mode == FOLLOW)
 {
    if(objectNear())
    {
      initAvoid();
    }
    else
    {
      followLine();
    }
      
 }
 if(mode == AVOID)
 {
    if(onBlackLineAgain())
    {
      initFollow();
    }
    else
    {
      avoidObject();
    }
 }*/
}


void initAvoid()
{
  // spin body
  motors.setSpeeds(0, 0);
  delay(200);
  motors.setSpeeds(-baseSpeed, baseSpeed); // turn left for turnTime seconds
  delay(turnTime);
  motors.setSpeeds(0, 0);
  // spin servo
  servo.write(0);
  // take note of side facing so we can undo in adjustForFollowMode
  mode = AVOID;
  // go forward a bit
  motors.setSpeeds(baseSpeed, baseSpeed);
  delay(300);
  motors.setSpeeds(0, 0);
}

void initFollow()
{
   // spin body forward
  motors.setSpeeds(0, 0);
  delay(100);
  motors.setSpeeds(-baseSpeed, baseSpeed); // turn left for turnTime seconds
  delay(turnTime);
  motors.setSpeeds(0, 0);
  // turn servo forward
  servo.write(90);
  // take note of side facing so we can undo in adjustForFollowMode
  mode = FOLLOW;
}
