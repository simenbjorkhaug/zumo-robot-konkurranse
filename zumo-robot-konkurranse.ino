#include <ZumoShield.h>
#include <NewPing.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <SoftwareSerial.h>
#include <PLabBTSerial.h>
L3G gyro;
#include "TurnSensor.h"
#include "PLab_PushButton.h"

// these might need to be tuned for different motor types
#define QTR_THRESHOLD 400 
#define REVERSE_SPEED 200 // 0 is stopped, 400 is full speed
#define TURN_SPEED 300
#define FORWARD_SPEED 200
#define TURBO_SPEED 400

const int echoPin1 = 3;
const int triggerPin1 = 2;
const int maxDistance = 30;
const int txPin = A1; // Connected to tx on bt unit
const int rxPin = A4; // Connected to rx on bt unit

const int redButtonPin =  6; // the number of the red LED pin

int initSpeed = 1;

boolean redLedState = false;

unsigned int sensor_values[6];

NewPing sonar1(triggerPin1, echoPin1, maxDistance);

ZumoMotors motors;
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN); // Tell Zumo that we have no emitter-pin
Pushbutton button(ZUMO_BUTTON);
PLabBTSerial btSerial(txPin, rxPin);
PLab_PushButton redButton(redButtonPin); // Create a PushButton object.

void setup() {
  btSerial.begin(9600); // Open serial communication to Bluetooth unit
  pinMode(redButtonPin, OUTPUT);
  Serial.begin(9600);   // Open serial communication to Serial Monitor
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


//...........................................................................
// Always include this method.
// It reads from the BT port and calls BTSerialMessageReceived.
// 
char msg[100];
void updateBTSerial() {
  int availableCount = btSerial.available();
  if (availableCount > 0) {
    btSerial.read(msg, availableCount);
    char *divided = strchr(msg,',');
    int msgValue = 0;
    if (divided != 0) {
       divided[0] = 0; divided++;
       String str(divided);
       msgValue = str.toInt();
    };
    String msgString(msg);
    BTSerialMessageReceived(msgString,msgValue);   
  }
}
//...........................................................................

//...........................................................................
// Always include these two methods .
// They send a message to the BT port, without or with an int value
// 
void BTSerialSendMessage(String msgString) {
  btSerial.println(msgString); 
}

void BTSerialSendMessage(String msgString, int msgValue) {
  btSerial.print(msgString); 
  btSerial.print(",");
  btSerial.println(msgValue);
}
//...........................................................................


void BTSerialMessageReceived(String msgString,int msgValue) {
  Serial.print("Message:"); Serial.print(msgString); // Debug print
  Serial.print(", Value:"); Serial.println(msgValue);  // Debug print
  if (msgString == "#redButtonPressed") {
    return redJavaFxButtonPressed();
  }
}

void redJavaFxButtonPressed() {
   redLedState = !redLedState;   // Toggle LED state.
   digitalWrite(redButtonPin, redLedState);
}


void turnDegrees(int degrees, boolean checkReverse, String direction, int angle) {

  turnSensorReset();
  
  if (checkReverse) {
    if (direction == "LEFT") { 
      reverse(400.00, 350.00, 0.2);  
    } else {
      reverse(350.00, 400.00, 0.2);  
    }
  }
  
  if (direction == "LEFT") {
    forward(-TURN_SPEED, TURN_SPEED);
  } else {
    forward(TURN_SPEED, -TURN_SPEED);
  }
  
  
  do {
    if (readSonar(sonar1) != 0.0) break;  
    delay(1);
    turnSensorUpdate();
    angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
  }
  while (angle < degrees);
  
  // forward(0.0, 0.0);
}

void reverse(double speed1, double speed2, double delayTime) {
  forward(-speed1, -speed2);
  delay(1000 * delayTime);
}


String checkLine() {
  sensors.read(sensor_values);
  if (sensor_values[0] < QTR_THRESHOLD) {
    return "LEFT";
  }
  if (sensor_values[5] < QTR_THRESHOLD) {
    return "RIGHT";
  } 
  return "no hit";
}

double readSonar(NewPing sonar) {
  return (double) sonar.convert_cm(sonar.ping());
}

void forward(double speed1, double speed2) {
  motors.setSpeeds(speed1, speed2);
}

void loop() {

    updateBTSerial();  // Check if we have input on the BT serial port.
    redButton.update();    // Update: Read the red button switch.
    
    if (redButton.pressed()) {  // Was the red button pressed?  
       BTSerialSendMessage("#redButtonPressed");
    }
    
    if (checkLine() != "no hit") {
       turnDegrees(160, true, checkLine(), 0);
    }
    
    if (readSonar(sonar1) == 0.0) {
      
      int i = 0;
      
      while(true) {
       if (readSonar(sonar1) <= maxDistance) break;  
       turnDegrees(1, false, "LEFT", 0); 
       delay(1);
       i++;
      } // spin
      if (i < 200) {
       turnDegrees(20, false, "RIGHT", 0); 
      }
      
    } else if (readSonar(sonar1) <= maxDistance) {
      forward(TURBO_SPEED, TURBO_SPEED);
      int i = 0;
      while(i < 200) {
        if (checkLine() != "no hit") break;
        delay(1);
        i++;
      }
      
   }

   
  
}

