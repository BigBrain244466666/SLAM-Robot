#include <Arduino.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <ESP32Encoder.h>
#include <VL53L0X.h>               // Pololu VL53L0X library
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// ====================================================================
// 1. PIN DEFINITIONS
// ====================================================================

// --- I2C Bus (Shared by BNO055 and both VL53L0X) ---
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// --- Motor 1 (Left) Pins ---
#define M1_IN1_PIN 17    // Signal In 1 to DRV8871 (Motor 1)
#define M1_IN2_PIN 16    // Signal In 2 to DRV8871 (Motor 1)
#define M1_ENC_A_PIN 4   // Encoder Signal A
#define M1_ENC_B_PIN 18  // Encoder Signal B

// --- Motor 2 (Right) Pins ---
#define M2_IN1_PIN 25    // Signal In 1 to DRV8871 (Motor 2)
#define M2_IN2_PIN 26    // Signal In 2 to DRV8871 (Motor 2)
#define M2_ENC_A_PIN 27  // Encoder Signal A
#define M2_ENC_B_PIN 32  // Encoder Signal B

// --- Sensor XSHUT Pins (VL53L0X must be initialized sequentially) ---
#define TOF1_XSHUT_PIN 23  // VL53L0X 1 XSHUT pin
#define TOF2_XSHUT_PIN 13  // VL53L0X 2 XSHUT pin

// --- Control & Diagnostics Pins ---
#define SPEAKER_PIN 14       // Speaker/Buzzer output
#define DIAGNOSTICS_SW_PIN 35// Diagnostics switch (Input)
#define WIFI_BT_SW_PIN 19    // Wi-Fi/Bluetooth ON/OFF switch (Input)

// --- PWM Configuration (For DRV8871 Motor Control) ---
#define PWM_FREQ      20000  // 20kHz frequency (good for motors)
#define PWM_RESOLUTION 10    // 10-bit resolution (0-1023)
#define M1_PWM_CHANNEL 0     // LEDC Channel 0 for Motor 1
#define M2_PWM_CHANNEL 1     // LEDC Channel 1 for Motor 2

// ====================================================================
// 2. GLOBAL OBJECTS AND STATE
// ====================================================================

// --- Sensor Objects ---
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // IMU default address 0x28
VL53L0X tof1; // Pololu VL53L0X instance 1
VL53L0X tof2; // Pololu VL53L0X instance 2

// --- Motor and Encoder Objects ---
ESP32Encoder encoder1; // Motor 1 Encoder
ESP32Encoder encoder2; // Motor 2 Encoder

// --- Communication Objects ---
BluetoothSerial SerialBT;

// --- State Variables ---
bool diagRunning = false;
bool lastDiagSwState = HIGH; // Assuming INPUT_PULLUP, HIGH means switch is OFF (not pressed)
bool lastWifiBtSwState = LOW; // Initial state check at boot

// ====================================================================
// 3. CORE FUNCTIONS
// ====================================================================

/**
 * @brief Plays a short, distinct beep sound on the speaker pin.
 */
void beep(int frequency, int duration_ms) {
    ledcAttachPin(SPEAKER_PIN, 0);
    ledcSetup(0, frequency, 10);
    ledcWrite(0, 512); // 50% duty cycle
    delay(duration_ms);
    ledcWrite(0, 0); // Stop tone
    ledcDetachPin(SPEAKER_PIN);
}

/**
 * @brief Sets the speed and direction for one motor using PWM.
 * @param pwmChannel The LEDC channel (M1_PWM_CHANNEL or M2_PWM_CHANNEL).
 * @param in1Pin The IN1 pin of the DRV8871.
 * @param in2Pin The IN2 pin of the DRV8871.
 * @param speed The motor speed (-1023 to 1023). Positive is forward, negative is backward.
 */
void setMotorSpeed(int pwmChannel, int in1Pin, int in2Pin, int speed) {
    int pwmValue = abs(speed);
    pwmValue = constrain(pwmValue, 0, (1 << PWM_RESOLUTION) - 1); // Clamp to 0-1023

    if (speed > 0) {
        // Forward
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    } else if (speed < 0) {
        // Backward (Swap IN1 and IN2 for reverse)
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    } else {
        // Stop (Brake or coast depending on driver setup; DRV8871 with both HIGH is brake)
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
    }
    
    // The DRV8871 expects the PWM signal on the IN pins. Since we are using 
    // digital pins (IN1, IN2) for direction and PWM for overall power,
    // we must update how we apply PWM.
    // For this simple example, we are using the two inputs (IN1, IN2) for 
    // direction control only. The PWM logic would be complex with DRV8871.
    // Given the constraints, we will simplify: if moving, power is set to 100%. 
    // A true PID loop would use PWM on one pin, but DRV8871 works best with 
    // IN1/IN2 for direction and a dedicated PWM pin for power (which we don't have
    // for both motors given the pin count, so we use digital HIGH/LOW).
    
    // We will use the PWM channels to control the overall motor ENABLE (if you have one), 
    // but for now, we'll stick to full speed control based on the direction pins.
    // For a real robot, you'd feed the PWM signal to ONE of the IN pins. 
    
    // Placeholder to use the PWM hardware for future PID:
    ledcWrite(pwmChannel, pwmValue);
}

/**
 * @brief Simple function to command the robot to move forward or stop.
 * @param leftSpeed Target speed for the left motor (M1).
 * @param rightSpeed Target speed for the right motor (M2).
 */
void moveRobot(int leftSpeed, int rightSpeed) {
    // For M1 (Left): Channel 0, Pins 17, 16
    setMotorSpeed(M1_PWM_CHANNEL, M1_IN1_PIN, M1_IN2_PIN, leftSpeed);
    // For M2 (Right): Channel 1, Pins 25, 26
    setMotorSpeed(M2_PWM_CHANNEL, M2_IN1_PIN, M2_IN2_PIN, rightSpeed);
}

/**
 * @brief Configures and initializes both VL53L0X sensors sequentially using the Pololu library.
 */
void initToFSensors() {
    Serial.println("Initializing VL53L0X sensors using Pololu library...");

    // Ensure both XSHUT pins are set to OUTPUT and LOW for hardware reset
    pinMode(TOF1_XSHUT_PIN, OUTPUT);
    pinMode(TOF2_XSHUT_PIN, OUTPUT);
    digitalWrite(TOF1_XSHUT_PIN, LOW); // Reset 1
    digitalWrite(TOF2_XSHUT_PIN, LOW); // Reset 2
    delay(100);

    // --- 1. Initialize ToF 1 (Enable, set address 0x30) ---
    digitalWrite(TOF1_XSHUT_PIN, HIGH);
    delay(10); // Wait for boot

    // Initialize and set new I2C address
    if (tof1.init()) {
        tof1.setAddress(0x30); 
        // Recommended settings for continuous ranging (faster readings)
        tof1.setMeasurementTimingBudget(20000); // 20ms budget
        tof1.startContinuous();
        Serial.println("VL53L0X 1 (0x30) initialized and started.");
    } else {
        Serial.println("VL53L0X 1 FAILED to initialize.");
    }

    // --- 2. Initialize ToF 2 (Enable, set address 0x31) ---
    digitalWrite(TOF2_XSHUT_PIN, HIGH);
    delay(10);

    // Initialize and set new I2C address
    if (tof2.init()) {
        tof2.setAddress(0x31);
        // Recommended settings for continuous ranging
        tof2.setMeasurementTimingBudget(20000); // 20ms budget
        tof2.startContinuous();
        Serial.println("VL53L0X 2 (0x31) initialized and started.");
    } else {
        Serial.println("VL53L0X 2 FAILED to initialize.");
    }
}

/**
 * @brief Executes a full component check and beeps on completion.
 */
void handleDiagnostics() {
    Serial.println("--- Starting Diagnostics ---");
    diagRunning = true;
    
    // Test 1: BNO055 IMU Check
    if (bno.begin()) {
        Serial.println("IMU (BNO055) OK.");
        beep(1000, 50); 
    } else {
        Serial.println("IMU (BNO055) FAILED. Check wiring.");
        beep(200, 500); 
    }
    
    // Test 2: Motor Driver Check (briefly spin motors)
    Serial.println("Testing Motor 1 and 2...");
    moveRobot(500, 500); // Move forward at half power
    delay(200);
    moveRobot(0, 0);     // Stop
    Serial.println("Motor check complete.");
    beep(2000, 100); // Final success beep

    Serial.println("--- Diagnostics Complete ---");
    diagRunning = false;
}

/**
 * @brief Handles Wi-Fi and Bluetooth ON/OFF based on the switch state.
 */
void handleConnectivitySwitch() {
    bool currentSwState = digitalRead(WIFI_BT_SW_PIN); 
    
    // Check for state change
    if (currentSwState != lastWifiBtSwState) {
        lastWifiBtSwState = currentSwState;
        
        if (currentSwState == LOW) { // Assuming LOW (or press) means ON for connectivity
            // Enable connectivity
            Serial.println("Switch ON: Starting Wi-Fi and Bluetooth...");
            // Wi-Fi Setup
            WiFi.begin("YOUR_SLAM_NETWORK", "YOUR_SLAM_PASSWORD");
            // Bluetooth Setup
            SerialBT.begin("SLAM_Robot_Control");
        } else {
            // Disable connectivity
            Serial.println("Switch OFF: Disconnecting Wi-Fi and Bluetooth...");
            WiFi.disconnect(true);
            SerialBT.end();
        }
        beep(1800, 30); // Confirmation beep
    }
}

/**
 * @brief Sends Odometry and Sensor data to the Raspberry Pi over Serial.
 */
void sendSlamDataToRPi() {
    // Collect data
    long count1 = encoder1.getCount();
    long count2 = encoder2.getCount();
    
    sensors_event_t imu_event;
    bno.getEvent(&imu_event); 

    // Pololu VL53L0X returns distance in millimeters, or 65535 if the reading is invalid (timeout).
    uint16_t range1 = tof1.readRangeContinuousMillimeters();
    uint16_t range2 = tof2.readRangeContinuousMillimeters();

    // Use 0 if the reading is invalid (65535) or a timeout occurred.
    int tof1_mm = (range1 != 65535 && !tof1.timeoutOccurred()) ? range1 : 0;
    int tof2_mm = (range2 != 65535 && !tof2.timeoutOccurred()) ? range2 : 0;

    // Data format: ODOM,M1_ENC,M2_ENC,IMU_YAW,TOF1_MM,TOF2_MM
    Serial.printf(
        "ODOM, %ld, %ld, %.2f, %d, %d\n",
        count1,
        count2,
        imu_event.orientation.x, // Yaw/Heading
        tof1_mm,
        tof2_mm
    );
}

/**
 * @brief Receives velocity commands from the Raspberry Pi and executes motor control.
 */
void handleRPiCommands() {
    // Commands come from RPi via USB Serial
    while (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        // Expected format: "CMD,L:500,R:-500" (Left speed, Right speed)
        
        if (command.startsWith("CMD")) {
            // Placeholder for actual parsing logic
            // Example: moveRobot(parsedLeftSpeed, parsedRightSpeed);
        }
    }
}


// ====================================================================
// 4. SETUP AND LOOP
// ====================================================================

void setup() {
    Serial.begin(115200); 
    delay(2000);
    Serial.println("ESP32 SLAM Robot Controller Initializing...");

    // 1. Initialize I2C Bus and Pins
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    pinMode(DIAGNOSTICS_SW_PIN, INPUT_PULLUP); 
    pinMode(WIFI_BT_SW_PIN, INPUT_PULLUP);
    pinMode(SPEAKER_PIN, OUTPUT);
    
    // 2. Setup Motor PWM Channels
    ledcSetup(M1_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(M2_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(M1_IN1_PIN, M1_PWM_CHANNEL); // Attach PWM to IN1 pins
    ledcAttachPin(M2_IN1_PIN, M2_PWM_CHANNEL); 
    // Set IN2 pins as digital output for direction control
    pinMode(M1_IN2_PIN, OUTPUT); 
    pinMode(M2_IN2_PIN, OUTPUT); 

    // 3. Initialize Encoders
    // The ESP32Encoder library used here does not provide 'useAttachedInterrupt'.
    // attachFullQuad() configures the encoder using interrupts internally.
    encoder1.attachFullQuad(M1_ENC_A_PIN, M1_ENC_B_PIN);
    encoder2.attachFullQuad(M2_ENC_A_PIN, M2_ENC_B_PIN);
    encoder1.setCount(0);
    encoder2.setCount(0);

    // 4. Initialize Sensors
    if (!bno.begin()) { Serial.println("ERROR: BNO055 IMU failed."); beep(200, 1000); }
    initToFSensors();

    Serial.println("System Ready.");
    // Initial check for connectivity
    lastWifiBtSwState = digitalRead(WIFI_BT_SW_PIN);
    handleConnectivitySwitch(); 
}

void loop() {
    // 1. Handle Diagnostics Switch (Edge-triggered: HIGH to LOW transition)
    bool currentDiagSwState = digitalRead(DIAGNOSTICS_SW_PIN);
    if (currentDiagSwState == LOW && lastDiagSwState == HIGH && !diagRunning) {
        handleDiagnostics();
    }
    lastDiagSwState = currentDiagSwState;

    // 2. Handle Connectivity Switch (Always check for state change)
    handleConnectivitySwitch();

    // 3. Main Robot Logic (SLAM and Control)
    sendSlamDataToRPi();
    handleRPiCommands();

    // The setMotorSpeed function will be called from handleRPiCommands after parsing.
    
    delay(50); // Loop runs every 50ms (20Hz)
}