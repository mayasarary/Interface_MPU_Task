#include <LiquidCrystal.h>
#include <Wire.h>
#include <MPU6050.h>
#include <I2Cdev.h>

const int motorPin1 = 2;
const int motorPin2 = 3;
const int encoderPinA = 5;
const int encoderPinB = 6;
const int ledPin = 4;
const int potX = A0;

// LCD pin definitions
const int rs = 7;
const int en = 8;
const int d4 = 9;
const int d5 = 10;
const int d6 = 11;
const int d7 = 12;

volatile long encoderPosition = 0;
int motorSpeed = 0;
int motorDirection = 1;
volatile int lastEncoded = 0;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

MPU6050 mpu;

int targetDegree = 360; // Target degree for one full rotation (yaw)
volatile float currentYaw = 0;

void setup() {
  Wire.begin();
  mpu.initialize();
  Serial.begin(115200);
  if (!mpu.testConnection()) {
    lcd.print("MPU failed");
    while(1); // Halt if MPU connection fails
  }

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  lcd.begin(16, 2);
  lcd.print("Initializing...");
  delay(1000);
}

void loop() {
  updateYaw();
  
  if (fabs(currentYaw) < targetDegree) {
    motorDirection = (currentYaw >= 0) ? 1 : -1;
    motorSpeed = 255; // Set speed to maximum
  } else {
    motorSpeed = 0; // Stop motor
  }

  driveMotor(motorSpeed, motorDirection);

  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(motorSpeed);
  lcd.setCursor(0, 1);
  lcd.print("Yaw: ");
  lcd.print(currentYaw);
  lcd.print(" deg");
  delay(100);
}

void driveMotor(int speed, int direction) {
  digitalWrite(ledPin, speed ? HIGH : LOW);
  if (direction == 1) {
    analogWrite(motorPin1, speed);
    analogWrite(motorPin2, 0);
  } else {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, speed);
  }
}

void updateYaw() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert gyroscope data to yaw
  // This requires proper scaling based on the MPU6050's sensitivity settings and potentially integrating the rate of rotation
  // Here we just simulate a simple example of scaling the gyroscope readings
  currentYaw += gy / 131.0; // Scale by gyroscope sensitivity (typical value for +/- 250 degrees per second)
  
  // Optionally, apply a filter or integrate over time to compute the angle more accurately
}

void updateEncoder() {
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;

  lastEncoded = encoded;
}
