#include <ZumoShield.h>
#include <NewPing.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
L3G gyro;
#include "TurnSensor.h"


// these might need to be tuned for different motor types
#define QTR_THRESHOLD 400 
#define REVERSE_SPEED 200 // 0 is stopped, 400 is full speed
#define TURN_SPEED 400
#define FORWARD_SPEED 200
#define TURBO_SPEED 400

const int echoPin1 = 6;
const int triggerPin1 = 7;
const int echoPin2 = 3;
const int triggerPin2 = 2;
const int maxDistance = 40;

int initSpeed = 1;

unsigned int sensor_values[6];

NewPing sonar1(triggerPin1, echoPin1, maxDistance);

ZumoMotors motors;
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN); // Tell Zumo that we have no emitter-pin
Pushbutton button(ZUMO_BUTTON);

void setup() {
  
  sensors.init();
  button.waitForButton();  // wait to start calibration

  delay(1000);
  
  for(int i = 0; i < 80; i++) {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70)) {
      forward(-200, 200);
    }
    else {
      forward(200, -200); 
    }
    sensors.calibrate();
    delay(20); //  Since our counter runs to 80, the total delay will be [ 80*20 = 1600 ms ]
  }
  
  forward(0.0, 0.0);

  turnSensorSetup();
  delay(500);
  turnSensorReset();

  button.waitForButton();
}


void turnDegrees(int degrees, boolean reverse, int angle) {

  turnSensorReset();
  
  if (reverse) {
    forward(-FORWARD_SPEED,-FORWARD_SPEED);
    delay(FORWARD_SPEED);
    forward(-TURN_SPEED, TURN_SPEED);
  } else {
    forward(-TURN_SPEED, TURN_SPEED); 
  }
  
  do {
    if (readSonar(sonar1) != 0.0 &! reverse) break;  
    delay(1);
    turnSensorUpdate();
    angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
  }
  while (angle < degrees);
  
  forward(0.0, 0.0);
}


boolean checkLine(int turnRate) {
  sensors.read(sensor_values);
  if (sensor_values[0] < QTR_THRESHOLD || sensor_values[5] < QTR_THRESHOLD) {
    turnDegrees(turnRate, true, 0);
    return true;
  } 
  return false;
}

double readSonar(NewPing sonar) {
  return (double) sonar.convert_cm(sonar.ping());
}

void forward(double speed1, double speed2) {
  motors.setSpeeds(speed1, speed2);
}

void loop() {
  
  checkLine(40);
  if (readSonar(sonar1) == 0.0) {
    turnDegrees(60, false, 0);
  } else if (readSonar(sonar1) <= maxDistance) {
    forward(TURBO_SPEED, TURBO_SPEED);
  }
  
}

