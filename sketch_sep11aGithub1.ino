#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define TRIGGER_PIN 7
#define ECHO_PIN 6
#define MAX_DISTANCE 200
#define LEFT_MOTOR_PIN1 3
#define LEFT_MOTOR_PIN2 5
#define RIGHT_MOTOR_PIN1 4
#define RIGHT_MOTOR_PIN2 6
#define IR_SENSOR_LEFT A0
#define IR_SENSOR_RIGHT A1
#define SERVO_PIN 9

Servo steeringServo;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Adafruit_MPU6050 mpu;

// Thresholds for IR sensor for line tracking
int irThreshold = 500;

// Color sensor configuration (TCS3200 code can be added based on the library)
int colorDetected = 0;  // 1 for green, 2 for red

void setup() {
  Serial.begin(9600);

  // Motor setup
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

  // Servo setup
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);  // Neutral position

  // IMU setup
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 sensor!");
    while (1) {}
  }

  // Initialize IR sensors
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);
}

void loop() {
  // Step 1: Check for obstacle
  if (detectObstacle()) {
    stopVehicle();
    avoidObstacle();
  }

  // Step 2: Check for traffic sign color
  if (detectColor() == 2) {  // Red detected
    turnAround();
  }

  // Step 3: Line tracking with IR sensors
  lineTracking();

  // Step 4: IMU data for balance or adjustments (if necessary)
  adjustStability();
}

bool detectObstacle() {
  int distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 0 && distance < 20) {  // Obstacle detected within 20 cm
    return true;
  }
  return false;
}

void avoidObstacle() {
  // Backup, turn, and move forward again
  moveBackward();
  delay(1000);
  turnRight();
  delay(500);
  moveForward();
}

int detectColor() {
  // Placeholder for color detection logic
  // If using TCS3200, write code here to detect color and return 1 for green, 2 for red
  // Simulating red detection for now:
  return colorDetected;  // Use actual color detection logic
}

void lineTracking() {
  int leftIR = analogRead(IR_SENSOR_LEFT);
  int rightIR = analogRead(IR_SENSOR_RIGHT);

  if (leftIR > irThreshold && rightIR < irThreshold) {
    // Turn slightly right
    steeringServo.write(70);  // Right turn
  } else if (rightIR > irThreshold && leftIR < irThreshold) {
    // Turn slightly left
    steeringServo.write(110);  // Left turn
  } else {
    // Go straight
    steeringServo.write(90);
  }

  moveForward();
}

void adjustStability() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.println(a.acceleration.y);

  // Use acceleration data to adjust stability (if required)
}

void moveForward() {
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

void stopVehicle() {
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void turnRight() {
  steeringServo.write(50);  // Adjust for sharp right turn
  moveForward();
  delay(500);
  steeringServo.write(90);  // Return to straight
}

void turnAround() {
  // Simulate a turn-around when red sign is detected
  stopVehicle();
  delay(500);
  turnRight();
  delay(1000);  // 180-degree turn
  moveForward();
}
