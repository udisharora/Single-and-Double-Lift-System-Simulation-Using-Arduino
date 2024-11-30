#include <LiquidCrystal_I2C.h>

// Initialize the LCD with I2C address 0x27 and 16 columns and 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin definitions for Lift 1
const int motorPin1 = 9;    // IN1 on L298
const int motorPin2 = 8;    // IN2 on L298
const int enablePin = 10;   // Enable pin on L298

// Pin definitions for Lift 2
const int motorPin3 = 12;   // IN3 on L298
const int motorPin4 = 13;   // IN4 on L298
const int enablePin2 = 11;  // Enable pin for second motor

// Button pins for six floors
const int buttonFloor1 = 7; // Button for Floor 1
const int buttonFloor2 = 6; // Button for Floor 2
const int buttonFloor3 = 5; // Button for Floor 3
const int buttonFloor4 = 4; // Button for Floor 4
const int buttonFloor5 = 3; // Button for Floor 5
const int buttonFloor6 = 2; // Button for Floor 6

// Emergency button pin
const int emergencyButtonPin = A0; // Emergency button connected to pin A0

// Current floor variable for two lifts
int currentFloorLift1 = 0; // Floor position for Lift 1
int currentFloorLift2 = 0; // Floor position for Lift 2

// Motor speed control
const int motorSpeed = 200; // PWM speed (0-255)

// Emergency state flag
bool emergencyMode = false;
// Updated hardcoded delay times for Lift 1 (milliseconds)
const int delayLift2[6][6] = {
  {0, 2050, 4350, 6400, 8850, 10250}, // From Floor 1
  {2050, 0, 1850, 4350, 6600, 8850}, // From Floor 2
  {3800, 1950, 0, 2400, 4650, 6350}, // From Floor 3
  {6100, 4350, 2200, 0, 2250, 4250}, // From Floor 4
  {8500, 6600, 4350, 1800, 0, 2050}, // From Floor 5
  {10100, 8850, 6900, 4000, 1950, 0} // From Floor 6
};

// Updated hardcoded delay times for Lift 2 (milliseconds)
const int delayLift1[6][6] = {
  {0, 2550, 4900, 7000, 8750, 10900}, // From Floor 1
  {2500, 0, 2350, 4450, 6000, 8400}, // From Floor 2
  {4550, 2150, 0, 2100, 3950, 5900}, // From Floor 3
  {6200, 4450, 1650, 0, 1850, 4100}, // From Floor 4
  {7700, 6300, 3050, 1550, 0, 1950}, // From Floor 5
  {8700, 8550, 6200, 2800, 1300, 0} // From Floor 6
};
void setup() {
  // Initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  // Initialize button pins
  pinMode(buttonFloor1, INPUT);
  pinMode(buttonFloor2, INPUT);
  pinMode(buttonFloor3, INPUT);
  pinMode(buttonFloor4, INPUT);
  pinMode(buttonFloor5, INPUT);
  pinMode(buttonFloor6, INPUT);

  // Initialize emergency button
  pinMode(emergencyButtonPin, INPUT);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("LIFT SYSTEM READY");
  lcd.setCursor(0, 1);
  lcd.print("FLOORS: 1, 1");
  delay(2000); // Initial message delay
  lcd.clear();

  // Initialize Serial monitor
  Serial.begin(9600);
  Serial.println("Lift System Initialized");
}

void loop() {
  // Check for emergency button press
  if (digitalRead(emergencyButtonPin) == HIGH) {
    emergencyActivate();
  }

  // Check floor request buttons
  if (digitalRead(buttonFloor1) == HIGH) moveElevator(0);
  if (digitalRead(buttonFloor2) == HIGH) moveElevator(1);
  if (digitalRead(buttonFloor3) == HIGH) moveElevator(2);
  if (digitalRead(buttonFloor4) == HIGH) moveElevator(3);
  if (digitalRead(buttonFloor5) == HIGH) moveElevator(4);
  if (digitalRead(buttonFloor6) == HIGH) moveElevator(5);

  // Update current floor display
  lcd.setCursor(0, 0);
  lcd.print("L1: ");
  lcd.print(currentFloorLift1 + 1);
  lcd.print(" L2: ");
  lcd.print(currentFloorLift2 + 1);
}

void emergencyActivate() {
  emergencyMode = true;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EMERGENCY MODE");
  lcd.setCursor(0, 1);
  lcd.print("RETURNING TO 1");
  Serial.println("Emergency: Moving both lifts to Floor 1.");

  // Move both lifts to Floor 1
  if (currentFloorLift1 != 0) {
    moveLift(motorPin1, motorPin2, enablePin, currentFloorLift1, 0, "Lift 1");
    currentFloorLift1 = 0;
  }
  if (currentFloorLift2 != 0) {
    moveLift(motorPin3, motorPin4, enablePin2, currentFloorLift2, 0, "Lift 2");
    currentFloorLift2 = 0;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EMERGENCY DONE");
  lcd.setCursor(0, 1);
  lcd.print("FLOORS RESET");
  delay(2000);
  lcd.clear();
  emergencyMode = false;
}

void moveElevator(int targetFloor) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("FLOOR REQ: ");
  lcd.print(targetFloor + 1);

  int lift1Distance = abs(targetFloor - currentFloorLift1);
  int lift2Distance = abs(targetFloor - currentFloorLift2);

  if (lift1Distance <= lift2Distance) {
    lcd.setCursor(0, 1);
    lcd.print("L1 MOVING...");
    moveLift(motorPin1, motorPin2, enablePin, currentFloorLift1, targetFloor, "Lift 1");
    currentFloorLift1 = targetFloor;
  } else {
    lcd.setCursor(0, 1);
    lcd.print("L2 MOVING...");
    moveLift(motorPin3, motorPin4, enablePin2, currentFloorLift2, targetFloor, "Lift 2");
    currentFloorLift2 = targetFloor;
  }
}

void moveLift(int pin1, int pin2, int enable, int &currentFloor, int targetFloor, String liftName) {
  int delayTime = 0;

  // Determine the delay using hardcoded delay arrays
  if (liftName == "Lift 1") {
    delayTime = delayLift1[currentFloor][targetFloor];
  } else {
    delayTime = delayLift2[currentFloor][targetFloor];
  }

  // Move motor in the required direction
  analogWrite(enable, motorSpeed);
  digitalWrite(pin1, targetFloor > currentFloor);
  digitalWrite(pin2, targetFloor < currentFloor);

  delay(delayTime); // Simulate travel time

  // Stop motor
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  analogWrite(enable, 0);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(liftName + " ARRIVED");
  lcd.setCursor(0, 1);
  lcd.print("AT FLOOR: ");
  lcd.print(targetFloor + 1);
  delay(2000);
  lcd.clear();
}
