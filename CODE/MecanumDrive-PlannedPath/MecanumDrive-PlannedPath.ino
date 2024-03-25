#include "Arduino.h"
#include "FastTrig.h"

//Wheel order
//1             2
//    Arduino
//
//
//
//3             4

float wheelDiamInch = 2.67;
float wheelRadius = wheelDiamInch/2;
int ticksPerRotation = 40; //20 on and 20 off
float pi = 3.141592;
float wheelBaseWidth = 8.494094;
float wheelBaseLength = 7.015748;
int maxRPM = 200; //Will be higher on battery voltage
double maxRPS = maxRPM/60;

uint8_t wheel1Start;
bool prevEncoder1State = 0;
uint8_t wheelOneSpeed = 0;

uint8_t wheel2Start;
bool prevEncoder2State = 0;
uint8_t wheelTwoSpeed = 0;

uint8_t wheel3Start;
bool prevEncoder3State = 0;
uint8_t wheelThreeSpeed = 0;

uint8_t wheel4Start;
bool prevEncoder4State = 0;
uint8_t wheelFourSpeed = 0;

double segmentDriveTime = 1000000;
double pathStartTime = 0;
int pathPart = 0;

//Wiring Specs
  //Input
    uint8_t encoderFour = 13;
    uint8_t encoderOne = 21;
    uint8_t encoderThree = 22;
    uint8_t encoderTwo = 45;
  //Output
    uint8_t pwmMotor2Forward = 12;//FIX ALL OF THESE
    uint8_t pwmMotor2Reverse = 11;

    uint8_t pwmMotor1Forward = 20;
    uint8_t pwmMotor1Reverse = 19;

    uint8_t pwmMotor3Forward = 23;
    uint8_t pwmMotor3Reverse = 24;
    
    uint8_t pwmMotor4Forward = 44;
    uint8_t pwmMotor4Reverse = 43;

uint8_t WheelOne = 1;
uint8_t WheelTwo = 2;
uint8_t WheelThree = 3;
uint8_t WheelFour = 4;
    

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  pathStartTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  updateWheelSpeed();


}

void plannedPath(){
  switch(pathPart){
    case 0:
      if(millis()-pathStartTime < segmentDriveTime){
        //Go straight towards the coins
        driveToInchCoordinates(10,0);
      }
      else{
        pathPart++;
      }
      break;

    case 1:
      pathStartTime = millis();
      pathPart++;
      segmentDriveTime = 100000;
      break;


    case 2:
      if(millis()-pathStartTime < segmentDriveTime){
        //Pick up the coins
        arm(0,0);
      }
      else{
        pathPart++;
      }
      break;

    case 3:
      pathStartTime = millis();
      pathPart++;
      segmentDriveTime = 100000;
      break;


    case 4:
      if(millis()-pathStartTime < segmentDriveTime){
        //Strafe Left
        driveToInchCoordinates(0,-15);
      }
      else{
        pathPart++;
      }
      break;

    case 5:
      pathStartTime = millis();
      pathPart++;
      segmentDriveTime = 100000;
      break;
      

    case 6:
      if(millis()-pathStartTime < segmentDriveTime){
        //Drive Straight
        driveToInchCoordinates(15,0);
      }
      else{
        pathPart++;
      }
      break;

    case 7:
      pathStartTime = millis();
      pathPart++;
      segmentDriveTime = 100000;
      break;
      

    case 8:
      if(millis()-pathStartTime < segmentDriveTime){
        //Rotate

      }
      else{
        pathPart++;
      }
      break;

    case 9:
      pathStartTime = millis();
      pathPart++;
      segmentDriveTime = 100000;
      break;
      

    case 10:
      if(millis()-pathStartTime < segmentDriveTime){
        //Drive Straight
        driveToInchCoordinates(15,0);
      }
      else{
        pathPart++;
      }
      break;

    case 11:
      pathStartTime = millis();
      pathPart++;
      segmentDriveTime = 100000;
      break;
      

    case 12:
      if(millis()-pathStartTime < segmentDriveTime){
        //Drop coins
        arm(0,0);
      }
      else{
        pathPart++;
      }
      break;

    case 13:
      pathStartTime = millis();
      pathPart++;
      segmentDriveTime = 100000;
      break;
      

    case 14:
      if(millis()-pathStartTime < segmentDriveTime){
        //Drive Straight Backwards
        driveToInchCoordinates(-15,0);
      }
      else{
        pathPart++;
      }
      break;
    
    case 15:
      pathStartTime = millis();
      pathPart++;
      segmentDriveTime = 100000;
      break;
      

    case 16:
      stopAllMotors();
    

  }
}

void arm(float angleOfArm, bool vacuumPowerOn){
  //Arm Angle is 0 at coin pickup height, positive is picked up
  segmentDriveTime = 0;
}

void rotateDegrees(double degreesTarget){
  //Rotates to the specified degrees, rotates clockwise

}

void driveToInchCoordinates(double x, double y){
  //X is straight ahead
  //Y is to the left
  //In respect to robot current heading
  double angle = atan(y/x);
  float power_FR_BL = sin(x-pi/4);
  float power_FL_BR = sin(x-pi/4);
  double robotSpeed = (wheelRadius * maxRPS * 0.8) / (sin(angle) + cos(angle));
  double timeToDrive = (sqrt( sq(x) + sq(y) )) / robotSpeed;
  segmentDriveTime = timeToDrive * 1000;

  wheelSetPercentSpeed(WheelOne, power_FL_BR);
  wheelSetPercentSpeed(WheelTwo, power_FR_BL);
  wheelSetPercentSpeed(WheelThree, power_FL_BR);
  wheelSetPercentSpeed(WheelFour, power_FR_BL);

}

void stopAllMotors(){
  wheelSetPercentSpeed(WheelOne, 0);
  wheelSetPercentSpeed(WheelTwo, 0);
  wheelSetPercentSpeed(WheelThree, 0);
  wheelSetPercentSpeed(WheelFour, 0);
}

//Starts a timer until thencoder experiences a state change
float getWheelSpeed(int wheel){
  float wheelRPS = 0;
  switch(wheel){
    case 1:
      if(getWheelEncoder(wheel) != prevEncoder1State){
        wheelRPS = (wheel1Start - millis()) * ticksPerRotation;
        wheel1Start = millis();
        return wheelRPS;
      }
      else{
        break;
      }

    case 2:
      if(getWheelEncoder(wheel) != prevEncoder2State){
        wheelRPS = (wheel2Start - millis()) * ticksPerRotation;
        wheel2Start = millis();
        return wheelRPS;
      }
      else{
        break;
      }

    case 3:
      if(getWheelEncoder(wheel) != prevEncoder3State){
        wheelRPS = (wheel3Start - millis()) * ticksPerRotation;
        wheel3Start = millis();
        return wheelRPS;
      }
      else{
        break;
      }

    case 4:
      if(getWheelEncoder(wheel) != prevEncoder4State){
        wheelRPS = (wheel4Start - millis()) * ticksPerRotation;
        wheel4Start = millis();
        return wheelRPS;
      }
      else{
        break;
      }
  }
}

void updateWheelSpeed(){
  wheelOneSpeed = getWheelSpeed(WheelOne);
  wheelTwoSpeed = getWheelSpeed(WheelTwo);
  wheelThreeSpeed = getWheelSpeed(WheelThree);
  wheelFourSpeed = getWheelSpeed(WheelFour);
}

double wheelSetPercentSpeed(int wheel, float speed){
  speed += (0.8 * speed) - getCurrentWheelSpeed(wheel);
  switch(wheel){
    case 1:
      if(speed > 0){
        analogWrite( pwmMotor1Forward , speed * 255 );
      }
      else{
        analogWrite( pwmMotor1Reverse , -speed * 255 );
      }
      break;

    case 2:
      if(speed > 0){
        analogWrite( pwmMotor2Forward , speed * 255 );
      }
      else{
        analogWrite( pwmMotor2Reverse , -speed * 255 );
      }
      break;

    case 3:
      if(speed > 0){
        analogWrite( pwmMotor3Forward , speed * 255 );
      }
      else{
        analogWrite( pwmMotor3Reverse , -speed * 255 );
      }
      break;

    case 4:
      if(speed > 0){
        analogWrite( pwmMotor4Forward , speed * 255 );
      }
      else{
        analogWrite( pwmMotor4Reverse , -speed * 255 );
      }
      break;

  }
}

bool getWheelEncoder(int wheel){
  switch(wheel){
    case 1:
      return digitalRead(encoderOne);

    case 2:
      return digitalRead(encoderTwo);

    case 3:
      return digitalRead(encoderThree);

    case 4:
      return digitalRead(encoderFour);
  }
}

float getCurrentWheelSpeed(int wheel){
  switch(wheel){
    case 1:
      return wheelOneSpeed;

    case 2:
      return wheelTwoSpeed;

    case 3:
      return wheelThreeSpeed;

    case 4:
      return wheelFourSpeed;
  }
}
