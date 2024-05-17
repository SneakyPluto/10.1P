#include <MeAuriga.h>
#include <Wire.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#define ALLLEDS 0

// Auriga on-board light ring has 12 LEDs
#define LEDNUM 12

//Look at the object feature, once an object is found, object is tracked with left and right movement
#define moveSpeed 100                 // Define an appropriate speed for the robot
#define OBJECT_DISTANCE_THRESHOLD 35  // Define the threshold distance to detect the object
#define PARTIAL_SEEK_TIMEOUT 4000     // 4 seconds timeout for partial seek
#define FOLLOW_DISTANCE_THRESHOLD 10 // Distance threshold to stop moving forward


// on-board LED ring, at PORT0 (onboard)
MeRGBLed led(0, LEDNUM);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower lineFinder(PORT_8);

// Setting up sensors
MeUltrasonicSensor ultraSensorLeft(PORT_9);    /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
MeUltrasonicSensor ultraSensorFront(PORT_10);  // /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */

int16_t moveSpeed = 55;

MeGyro gyro(1, 0x69);

// Define constants for sensor states
#define S1_IN_S2_IN 0b11    // Both sensors on the line
#define S1_IN_S2_OUT 0b10   // Left sensor on the line, right sensor off the line
#define S1_OUT_S2_IN 0b01   // Left sensor off the line, right sensor on the line
#define S1_OUT_S2_OUT 0b00  // Both sensors off the line


float j, f, k;

// Threshold to determine light source direction
const int threshold = 6;  // Adjust as needed based on your environment

void CurveLeft(void) {
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed / 2);
}

void CurveRight() {
  Encoder_1.setMotorPwm(-moveSpeed / 2);
  Encoder_2.setMotorPwm(moveSpeed);
}

void Forward(void) {
  Encoder_1.setMotorPwm(-moveSpeed);  // setMotorPwm writes to the encoder controller
  Encoder_2.setMotorPwm(moveSpeed);   // so setting the speed change instantly
}
void Backward(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void BackwardAndTurnLeft(void) {
  Encoder_1.setMotorPwm(moveSpeed / 4);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void BackwardAndTurnRight(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed / 4);
}


void TurnLeft1(void) {
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void TurnRight1(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

void Stop(void) {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

void ChangeSpeed(int16_t spd) {
  moveSpeed = spd;
}

void turnRightDegree(double degree) {
  gyro.update();
  Serial.println(gyro.getAngleZ());
  double startAngle = gyro.getAngleZ() + 180.00;
  double delta;
  CurveRight();
  while (degree > 0) {
    gyro.update();
    delta = gyro.getAngleZ() + 180.00 - startAngle;
    if (delta < -180.00) {
      delta += 360.00;
    } else if (delta > 180) {
      delta -= 360;
    }
    degree -= delta;
    startAngle = gyro.getAngleZ() + 180.00;
  }
  gyro.update();
  Serial.println(gyro.getAngleZ());
  Stop();
}

void setup() {
  Serial.begin(115200);
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  gyro.begin();

  // Initalise LED
  led.setpin(44);
}

// State definitions
typedef enum {
  SEEK_FULL_CIRCLE,
  SEEK_PARTIAL,
  STOP,
  FOLLOW
} State;


State currentState = SEEK_FULL_CIRCLE;
bool objectDetected = false;
time_t lostTime = 0;  // Time when the object was lost

// Function to check if the object is detected
bool IsObjectDetected(void) {
  int distance = ultraSensorFront.distanceCm();
  return distance < OBJECT_DISTANCE_THRESHOLD;
}

// Function to move towards the object and adjust direction
void MoveTowardsObject(void) {
  int frontDistance = ultraSensorFront.distanceCm();
  // int leftDistance = ultraSensorLeft.distanceCm();

  if (frontDistance > FOLLOW_DISTANCE_THRESHOLD) {
    Forward();
    return;
  } else {
    Stop();
  }
}

// Loop containing move towards object feature
void loop() {
    unsigned long currentTime = millis();  // Get current time

    switch (currentState) {
        case SEEK_FULL_CIRCLE:
            if (IsObjectDetected()) {
                Stop();
                objectDetected = true;
                currentState = FOLLOW;
            } else {
                TurnLeft1();  // Turning in a full circle
            }
            break;

        case SEEK_PARTIAL:
            if (IsObjectDetected()) {
                Stop();
                objectDetected = true;
                currentState = FOLLOW;
            } else {
                // Check if the timeout has been reached
                if (currentTime - lostTime > PARTIAL_SEEK_TIMEOUT) {
                    currentState = SEEK_FULL_CIRCLE;
                } else {
                    // Alternate left and right turns for partial seeking
                    static bool turnLeft = true;
                    if (turnLeft) {
                        TurnLeft1();
                        delay(350);
                    } else {
                        TurnRight1();
                        delay(275);
                    }
                    turnLeft = !turnLeft;
                }
            }
            break;

        case FOLLOW:
            if (IsObjectDetected()) {
                MoveTowardsObject();
            } else {
                objectDetected = false;
                lostTime = currentTime;  // Record the time when the object is lost
                currentState = SEEK_PARTIAL;
            }
            break;

        case STOP:
            if (!IsObjectDetected()) {
                objectDetected = false;
                lostTime = currentTime;  // Record the time when the object is lost
                currentState = SEEK_PARTIAL;
            }
            break;
    }

    // Adding a small delay to avoid busy-waiting
    delay(100);
}
// Seeks for an object then stops once the object is detected
// void SeekObject () {
//   while (1) {
//     double sensorFront = ultraSensorFront.distanceCm();
//     if(sensorFront < 20) {
//       Stop();
//       break;
//     }
//     TurnRight1();
//   }
// }


//Look At Object Feature
// void loop() {
//   // Check timer and update lostTime if needed
//   unsigned long currentTime = millis();
//   if (currentState == STOP && !IsObjectDetected()) {
//     objectDetected = false;
//     lostTime = currentTime;
//     currentState = SEEK_PARTIAL;
//   }

//   switch (currentState) {
//     case SEEK_FULL_CIRCLE:
//       if (IsObjectDetected()) {
//         Stop();
//         objectDetected = true;
//         currentState = STOP;
//       } else {
//         TurnLeft1();  // Turning in a full circle
//       }
//       break;

//     case SEEK_PARTIAL:
//       if (IsObjectDetected()) {
//         Stop();
//         objectDetected = true;
//         currentState = STOP;
//       } else {
//         // Check if the timeout has been reached
//         if (currentTime - lostTime > PARTIAL_SEEK_TIMEOUT) {
//           currentState = SEEK_FULL_CIRCLE;
//           delay(100);
//         } else {
//           // Alternate left and right turns for partial seeking
//           static bool turnLeft = true;
//           if (turnLeft) {
//             TurnLeft1();
//             delay(350);
//           } else {
//             TurnRight1();
//             delay(300);
//           }
//           turnLeft = !turnLeft;
//         }
//       }
//     case STOP:
//       // No additional logic needed here
//       break;
//   }
// }


void seekObject() {
  while (true) {
    double sensorFront = ultraSensorFront.distanceCm();
    if (sensorFront < 25.0) {
      Stop();
      trackObject();
    } else {
      TurnLeft1();
    }
  }
}

void trackObject() {
  double previousReading = ultraSensorFront.distanceCm();
  while (true) {
    double currentReading = ultraSensorFront.distanceCm();
    if (currentReading >= 25.0) {
      seekObject();
      return;
    } else if (currentReading < 25.0 && currentReading > 20.0) {
      Stop();  // Object is in an optimal range
    } else if (currentReading < previousReading) {
      TurnLeft1();  // Object is moving closer, turn left
    } else if (currentReading > previousReading) {
      TurnRight1();  // Object is moving away, turn right
    }
    previousReading = currentReading;
  }
}

void CheckObstacle() {
  double sensorFront = ultraSensorFront.distanceCm();
  if (sensorFront < 10.00) {
    led.setColorAt(2, 0, 35, 35);
    led.show();
    trackLine();
  }
}

void trackLine() {
  int sensorState = lineFinder.readSensors();
  while (sensorState != S1_OUT_S2_IN) {
    TurnRight1();
    sensorState = lineFinder.readSensors();
  }

  while (sensorState != S1_OUT_S2_OUT) {
    TurnRight1();
    sensorState = lineFinder.readSensors();
  }
}

void followLine() {
  int sensorState = lineFinder.readSensors();
  switch (sensorState) {
    case S1_OUT_S2_OUT:
      led.setColorAt(2, 35, 0, 0);
      led.show();
      Serial.println("S1_OUT_S2_OUT");
      Forward();
      break;
    case S1_IN_S2_OUT:
      led.setColorAt(2, 0, 35, 0);
      led.show();
      Serial.println("S1_IN_S2_OUT");
      TurnRight1();
      delay(300);
      Forward();
      break;
    case S1_OUT_S2_IN:
      led.setColorAt(2, 0, 0, 35);
      led.show();
      Serial.println("S1_OUT_S2_IN");
      TurnLeft1();
      delay(300);
      Forward();
      break;
    case S1_IN_S2_IN:
      Serial.println("S1_IN_S2_IN");
      // Decide what to do when both sensors are on the white background.
      // Here, we'll stop the robot.
      Backward();
      break;
    default:
      break;
  }
  // delay(200); // Adjust the delay if necessary
}

void turnLeft(double degree) {
  gyro.update();
  Serial.println(gyro.getAngleZ());
  double startAngle = gyro.getAngleZ() + 180.00;
  double delta;
  TurnLeft1();
  while (degree > 0) {
    gyro.update();
    delta = gyro.getAngleZ() + 180.00 - startAngle;
    if (delta < -180.00) {
      delta += 360.00;
    } else if (delta > 180) {
      delta -= 360;
    }
    degree += delta;
    startAngle = gyro.getAngleZ() + 180.00;
  }
  gyro.update();
  Serial.println(gyro.getAngleZ());
  Stop();
}
