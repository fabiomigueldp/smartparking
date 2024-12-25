#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// -----------------------------
// Pin Definitions
// -----------------------------

// IR Sensor Pins (configured as digital inputs with internal pull-up resistors)
const int ENTRY_SENSOR_1_PIN = 40; // First IR sensor at the entry
const int ENTRY_SENSOR_2_PIN = 41; // Second IR sensor at the entry
const int EXIT_SENSOR_1_PIN  = 42; // First IR sensor at the exit
const int EXIT_SENSOR_2_PIN  = 43; // Second IR sensor at the exit

// Servo Motor Pins
const int ENTRY_GATE_PIN = 8;  // Servo motor controlling the entry gate
const int EXIT_GATE_PIN  = 9;  // Servo motor controlling the exit gate

// Ultrasonic Sensor Pins for 8 Parking Slots (HC-SR04)
const int NUM_SLOTS = 8; // Total number of parking slots
const int TRIG_PINS[NUM_SLOTS] = {22, 24, 26, 28, 30, 32, 34, 36}; // Trigger pins for ultrasonic sensors
const int ECHO_PINS[NUM_SLOTS] = {23, 25, 27, 29, 31, 33, 35, 37}; // Echo pins for ultrasonic sensors

// LCD I2C Address
const int LCD_ADDRESS = 0x27; // Change this if your LCD has a different I2C address

// -----------------------------
// Constants
// -----------------------------

const int MAX_SLOTS              = NUM_SLOTS;       // Total number of parking slots
const int OCCUPIED_THRESHOLD_CM  = 100;              // Distance in cm below which a slot is considered occupied

// Servo Angles
const int SERVO_OPEN_ANGLE   = 90;  // Angle to open the gate
const int SERVO_CLOSED_ANGLE = 0;   // Angle to close the gate

// Time Constants (in milliseconds)
const unsigned long SENSOR_ACTIVATION_TIME    = 3000; // Time sensors must be active to trigger gate operation
const unsigned long SENSOR_DEACTIVATION_TIME  = 3000; // Time after sensors are inactive to close the gate
const unsigned long GATE_CLOSE_TIME           = 1000; // Time taken for the servo to close the gate

// -----------------------------
// Global Variables
// -----------------------------

bool slotStatus[NUM_SLOTS] = {false}; // Array to hold the occupancy status of each slot (false = Free, true = Occupied)
int availableSlots = MAX_SLOTS;       // Counter for available parking slots

// Servo Objects
Servo entryGateServo; // Servo object for the entry gate
Servo exitGateServo;  // Servo object for the exit gate

// LCD Object
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2); // Initialize LCD with 16 columns and 2 rows

// Sensor State Variables for Activation (Entry and Exit)
unsigned long entrySensorActivatedTime = 0; // Timestamp when entry sensor was activated
unsigned long exitSensorActivatedTime  = 0; // Timestamp when exit sensor was activated

// Sensor State Variables for Deactivation (Entry and Exit)
unsigned long entrySensorDeactivatedTime = 0; // Timestamp when entry sensor was deactivated
unsigned long exitSensorDeactivatedTime  = 0; // Timestamp when exit sensor was deactivated

// Gate State Machine Definitions
enum GateState { IDLE, OPENING, WAITING, CLOSING }; // Possible states for gate operation

GateState entryGateState = IDLE; // Current state of the entry gate
GateState exitGateState  = IDLE; // Current state of the exit gate

unsigned long entryGateTimer = 0; // Timer for entry gate state transitions
unsigned long exitGateTimer  = 0; // Timer for exit gate state transitions

// -----------------------------
// Function Prototypes
// -----------------------------

void initializeSystem();            // Initializes all system components
void checkInfraredSensors();        // Checks the state of IR sensors for entry and exit
void handleEntryGate();             // Manages the state machine for the entry gate
void handleExitGate();              // Manages the state machine for the exit gate
void checkUltrasonicSensors();      // Checks the occupancy status of all parking slots
void updateLCD();                   // Updates the LCD display with available slots
void printStatusToSerial();         // Prints the current parking status to the Serial Monitor

// -----------------------------
// Setup Function
// -----------------------------

void setup() {
  initializeSystem(); // Call the initialization function
}

// -----------------------------
// Main Loop Function
// -----------------------------

void loop() {
  // Continuously perform the following operations:

  // 1. Check for vehicle detections at entry and exit
  checkInfraredSensors();

  // 2. Handle gate operations based on sensor states
  handleEntryGate();
  handleExitGate();

  // 3. Monitor the occupancy status of parking slots
  checkUltrasonicSensors();

  // 4. Update the LCD display and Serial Monitor with current status
  updateLCD();
  printStatusToSerial();

  // 5. Short delay to prevent excessive CPU usage
  delay(50);
}

// -----------------------------
// Function Definitions
// -----------------------------

/**
 * Initializes all system components, including serial communication,
 * sensor pin configurations, servo motors, ultrasonic sensors, and the LCD display.
 */
void initializeSystem() {
  // Initialize Serial Communication for debugging and monitoring
  Serial.begin(9600);

  // Initialize IR Sensor Pins as INPUT_PULLUP to use internal pull-up resistors
  pinMode(ENTRY_SENSOR_1_PIN, INPUT_PULLUP);
  pinMode(ENTRY_SENSOR_2_PIN, INPUT_PULLUP);
  pinMode(EXIT_SENSOR_1_PIN, INPUT_PULLUP);
  pinMode(EXIT_SENSOR_2_PIN, INPUT_PULLUP);

  // Initialize Servo Motors and set them to the closed position initially
  entryGateServo.attach(ENTRY_GATE_PIN);
  exitGateServo.attach(EXIT_GATE_PIN);
  entryGateServo.write(SERVO_CLOSED_ANGLE); // Ensure entry gate is closed
  exitGateServo.write(SERVO_CLOSED_ANGLE);  // Ensure exit gate is closed

  // Initialize Ultrasonic Sensor Pins
  for (int i = 0; i < MAX_SLOTS; i++) {
    pinMode(TRIG_PINS[i], OUTPUT); // Trigger pin as OUTPUT
    pinMode(ECHO_PINS[i], INPUT);  // Echo pin as INPUT
    digitalWrite(TRIG_PINS[i], LOW); // Ensure trigger pin is LOW
  }

  // Initialize LCD Display
  lcd.init();           // Initialize the LCD
  lcd.backlight();      // Turn on the LCD backlight
  lcd.clear();          // Clear any existing content
  lcd.setCursor(0, 0);  // Set cursor to first line
  lcd.print("Available Slots:"); // Display header
  lcd.setCursor(0, 1);  // Set cursor to second line
  lcd.print(availableSlots);      // Display initial available slots

  // Initial Serial Output for Confirmation
  Serial.println("Parking Lot Status Initialized");
  Serial.print("Available Slots: ");
  Serial.println(availableSlots);
  Serial.println();
}

/**
 * Checks the state of the infrared sensors at both entry and exit.
 * Handles sensor activation and deactivation timings to control gate operations.
 */
void checkInfraredSensors() {
  unsigned long currentTime = millis(); // Get the current timestamp

  // --- ENTRY SENSOR CHECK ---
  bool currentEntrySensorState = 
      (digitalRead(ENTRY_SENSOR_1_PIN) == LOW) || 
      (digitalRead(ENTRY_SENSOR_2_PIN) == LOW); // TRUE if any entry sensor is triggered

  if (currentEntrySensorState) {
    // If entry sensor is currently active (vehicle detected)

    // Reset the deactivation timer since the sensor is active
    entrySensorDeactivatedTime = 0;

    // If this is the first detection, start the activation timer
    if (entrySensorActivatedTime == 0) {
      entrySensorActivatedTime = currentTime;
    } else {
      // If the sensor has been active for at least SENSOR_ACTIVATION_TIME
      if (currentTime - entrySensorActivatedTime >= SENSOR_ACTIVATION_TIME) {
        if (availableSlots > 0 && entryGateState == IDLE) {
          // If there are available slots and the entry gate is idle, open the gate
          entryGateState = OPENING;
          entryGateTimer = currentTime;

          // Decrement available slots as a vehicle is entering
          availableSlots--;
          if (availableSlots < 0) availableSlots = 0; // Prevent negative slots

          Serial.println("Car entered. Decreasing available slots.");
        } else if (availableSlots <= 0) {
          // If parking is full, display a message on the LCD
          Serial.println("Parking Full! Cannot open entry gate.");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Parking Full!");
        }

        // Reset the activation timer to prevent multiple triggers
        entrySensorActivatedTime = 0;
      }
    }
  } 
  else {
    // If entry sensor is not active

    // Reset the activation timer since the sensor is inactive
    entrySensorActivatedTime = 0;

    // If deactivation timer hasn't been started yet, start it now
    if (entrySensorDeactivatedTime == 0) {
      entrySensorDeactivatedTime = currentTime;
    }
  }

  // --- EXIT SENSOR CHECK ---
  bool currentExitSensorState = 
      (digitalRead(EXIT_SENSOR_1_PIN) == LOW) || 
      (digitalRead(EXIT_SENSOR_2_PIN) == LOW); // TRUE if any exit sensor is triggered

  if (currentExitSensorState) {
    // If exit sensor is currently active (vehicle detected)

    // Reset the deactivation timer since the sensor is active
    exitSensorDeactivatedTime = 0;

    // If this is the first detection, start the activation timer
    if (exitSensorActivatedTime == 0) {
      exitSensorActivatedTime = currentTime;
    } else {
      // If the sensor has been active for at least SENSOR_ACTIVATION_TIME
      if (currentTime - exitSensorActivatedTime >= SENSOR_ACTIVATION_TIME) {
        if (exitGateState == IDLE) {
          // If the exit gate is idle, open the gate
          exitGateState = OPENING;
          exitGateTimer = currentTime;

          // Increment available slots as a vehicle is exiting
          availableSlots++;
          if (availableSlots > MAX_SLOTS) availableSlots = MAX_SLOTS; // Prevent exceeding max slots

          Serial.println("Car exited. Increasing available slots.");
        }

        // Reset the activation timer to prevent multiple triggers
        exitSensorActivatedTime = 0;
      }
    }
  } 
  else {
    // If exit sensor is not active

    // Reset the activation timer since the sensor is inactive
    exitSensorActivatedTime = 0;

    // If deactivation timer hasn't been started yet, start it now
    if (exitSensorDeactivatedTime == 0) {
      exitSensorDeactivatedTime = currentTime;
    }
  }
}

/**
 * Manages the state machine for the entry gate.
 * Opens the gate when a vehicle is detected and closes it after SENSOR_DEACTIVATION_TIME.
 */
void handleEntryGate() {
  unsigned long currentTime = millis(); // Get the current timestamp

  switch (entryGateState) {
    case IDLE:
      // Gate is idle; no action required
      break;

    case OPENING:
      // Open the entry gate by setting the servo to SERVO_OPEN_ANGLE
      entryGateServo.write(SERVO_OPEN_ANGLE);
      entryGateTimer = currentTime; // Record the time when the gate was opened
      entryGateState = WAITING;     // Transition to WAITING state
      break;

    case WAITING:
      // Wait for the sensor to be inactive for SENSOR_DEACTIVATION_TIME before closing the gate
      if (entrySensorDeactivatedTime != 0 && 
          (currentTime - entrySensorDeactivatedTime >= SENSOR_DEACTIVATION_TIME)) 
      {
        // Close the gate by setting the servo to SERVO_CLOSED_ANGLE
        entryGateServo.write(SERVO_CLOSED_ANGLE);
        entryGateTimer = currentTime; // Record the time when the gate was closed
        entryGateState = CLOSING;     // Transition to CLOSING state
      }
      break;

    case CLOSING:
      // Wait for the gate to finish closing based on GATE_CLOSE_TIME
      if (currentTime - entryGateTimer >= GATE_CLOSE_TIME) {
        entryGateState = IDLE; // Transition back to IDLE state
      }
      break;
  }
}

/**
 * Manages the state machine for the exit gate.
 * Opens the gate when a vehicle is detected and closes it after SENSOR_DEACTIVATION_TIME.
 */
void handleExitGate() {
  unsigned long currentTime = millis(); // Get the current timestamp

  switch (exitGateState) {
    case IDLE:
      // Gate is idle; no action required
      break;

    case OPENING:
      // Open the exit gate by setting the servo to SERVO_OPEN_ANGLE
      exitGateServo.write(SERVO_OPEN_ANGLE);
      exitGateState = WAITING; // Transition to WAITING state
      break;

    case WAITING:
      // Wait for the sensor to be inactive for SENSOR_DEACTIVATION_TIME before closing the gate
      if (exitSensorDeactivatedTime != 0 && 
          (currentTime - exitSensorDeactivatedTime >= SENSOR_DEACTIVATION_TIME))
      {
        // Close the gate by setting the servo to SERVO_CLOSED_ANGLE
        exitGateServo.write(SERVO_CLOSED_ANGLE);
        exitGateTimer = currentTime; // Record the time when the gate was closed
        exitGateState = CLOSING;     // Transition to CLOSING state
      }
      break;

    case CLOSING:
      // Wait for the gate to finish closing based on GATE_CLOSE_TIME
      if (currentTime - exitGateTimer >= GATE_CLOSE_TIME) {
        exitGateState = IDLE; // Transition back to IDLE state
      }
      break;
  }
}

/**
 * Checks each ultrasonic sensor to determine if a parking slot is occupied.
 * Updates the slotStatus array based on the measured distance.
 */
void checkUltrasonicSensors() {
  for (int i = 0; i < MAX_SLOTS; i++) {
    // Trigger the ultrasonic sensor by sending a HIGH pulse for 10 microseconds
    digitalWrite(TRIG_PINS[i], LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PINS[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PINS[i], LOW);

    // Read the echo pulse duration (with a timeout of 60,000 microseconds)
    long duration = pulseIn(ECHO_PINS[i], HIGH, 60000UL); // Timeout set to 60 ms

    // Calculate the distance in centimeters
    float distance_cm = (duration * 0.0343) / 2.0; // Speed of sound is ~343 m/s

    // Determine the occupancy status based on the distance
    if (distance_cm > 0 && distance_cm < OCCUPIED_THRESHOLD_CM) {
      slotStatus[i] = true; // Slot is Occupied
    } else {
      slotStatus[i] = false; // Slot is Free
    }

    delay(50); // Short delay between sensor checks to prevent interference
  }
}

/**
 * Updates the LCD display with the current number of available parking slots.
 * Refreshes the display every 500 milliseconds.
 */
void updateLCD() {
  static unsigned long lastUpdate = 0; // Stores the last update time
  unsigned long currentTime = millis(); // Get the current timestamp

  if (currentTime - lastUpdate >= 500) { // Update every 500 ms
    lcd.clear(); // Clear the LCD display
    lcd.setCursor(0, 0); // Set cursor to first line
    lcd.print("Available Slots:"); // Display header
    lcd.setCursor(0, 1); // Set cursor to second line
    lcd.print(availableSlots); // Display the number of available slots
    lastUpdate = currentTime; // Update the last update timestamp
  }
}

/**
 * Prints the current parking status to the Serial Monitor.
 * Outputs the number of available slots and the status of each parking slot every 2 seconds.
 */
void printStatusToSerial() {
  static unsigned long lastPrint = 0; // Stores the last print time
  unsigned long currentTime = millis(); // Get the current timestamp

  if (currentTime - lastPrint >= 2000) { // Print every 2000 ms (2 seconds)
    Serial.println("=== Parking Lot Status ===");
    Serial.print("Available Slots: ");
    Serial.println(availableSlots);
    Serial.println("Slot Occupancy Status:");

    // Iterate through each slot and print its status
    for (int i = 0; i < MAX_SLOTS; i++) {
      Serial.print("Slot ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(slotStatus[i] ? "Occupied" : "Free");
    }

    Serial.println(); // Add an empty line for readability
    lastPrint = currentTime; // Update the last print timestamp
  }
}
