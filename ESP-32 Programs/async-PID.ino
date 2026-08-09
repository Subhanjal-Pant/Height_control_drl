#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// --- Pin Definitions ---
const int enA = 25; const int in1 = 16; const int in2 = 17; // Exhaust Valve
const int enB = 26; const int in3 = 19; const int in4 = 18; // Inlet Valve
const int inputPin = 33;
const int springPin = 32;

// --- Hardware & Loop Config ---
VL53L0X sensor;
MPU6050 imu(0x68);

const unsigned long CONTROL_INTERVAL = 40; // 25 Hz Main Control Loop (40 ms)
const unsigned long CYCLE_TIME = 143;      // 7 Hz Solenoid PWM Base Cycle (143 ms)
unsigned long lastControlTime = 0;

// Register address for VL53L0X Interrupt/Data-Ready Status
#define RESULT_INTERRUPT_STATUS 0x13

// --- PID Gains & Limits ---
float Kp = 2.5; 
float Ki = 0.1; 
float Kd = 0.5;
float iTerm = 0.0;
const float I_LIMIT = 50.0; 
const float DEADZONE = 1.5; // Deadband in mm

// --- Target, Actuation & State Variables ---
float targetHeight = 130.0; // Target set-point (mm)
int pInRaw = 0, pSpRaw = 0;
int currentDutyCycle = 0;
int currentStatusFlag = 0;  // 0: Idle, 1: Filling, 2: Draining

// Raw sensor buffers
float lastTofMeasurement = 130.0;
bool newTofAvailable = false;

// ============================================================================
// 2-STATE KINEMATIC KALMAN FILTER (Predict & Correct Separated)
// State Vector X = [ position (mm), velocity (mm/s) ]^T
// ============================================================================
float x_hat[2] = {130.0, 0.0}; 
float P[2][2]  = {{10.0, 0.0}, 
                  {0.0, 10.0}}; 

const float Q_accel = 150.0;   // Process noise variance (mm/s^2)^2
const float R_tof   = 4.0;     // Measurement noise variance (mm^2)
const float GRAVITY = 9806.65; // Gravity in mm/s^2

const float ACCEL_SCALE = 9806.65 / 16384.0; // MPU6050 LSB -> mm/s^2

// --- Step 1: High-Frequency Prediction (IMU) ---
void kalmanPredict(float raw_az, float dt) {
  float a_z = (raw_az * ACCEL_SCALE) - GRAVITY;

  // Kinematic state extrapolation
  x_hat[0] = x_hat[0] + x_hat[1] * dt + 0.5f * a_z * dt * dt;
  x_hat[1] = x_hat[1] + a_z * dt;

  // Covariance extrapolation: P_pred = F * P * F^T + Q
  float P00_new = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + 0.25f * dt * dt * dt * dt * Q_accel;
  float P01_new = P[0][1] + dt * P[1][1] + 0.5f * dt * dt * dt * Q_accel;
  float P10_new = P[1][0] + dt * P[1][1] + 0.5f * dt * dt * dt * Q_accel;
  float P11_new = P[1][1] + dt * dt * Q_accel;

  P[0][0] = P00_new;
  P[0][1] = P01_new;
  P[1][0] = P10_new;
  P[1][1] = P11_new;
}

// --- Step 2: Asynchronous Correction (ToF Laser) ---
void kalmanCorrect(float tof_meas_mm) {
  float y = tof_meas_mm - x_hat[0]; // Innovation
  float S = P[0][0] + R_tof;        // Innovation Covariance

  // Kalman Gain
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  // State Correction
  x_hat[0] = x_hat[0] + K0 * y;
  x_hat[1] = x_hat[1] + K1 * y;

  // Covariance Update
  float P00_old = P[0][0];
  float P01_old = P[0][1];

  P[0][0] = (1.0f - K0) * P00_old;
  P[0][1] = (1.0f - K0) * P01_old;
  P[1][0] = P[1][0] - K1 * P00_old;
  P[1][1] = P[1][1] - K1 * P01_old;
}

// ============================================================================
// NON-BLOCKING TOF SENSOR HELPER
// ============================================================================
bool checkToFReadyAndRead(uint16_t &distance) {
  // Read status register directly to check if a new measurement byte is ready
  uint8_t status = sensor.readRegister(RESULT_INTERRUPT_STATUS);
  if ((status & 0x07) != 0) {
    distance = sensor.readRangeContinuousMillimeters();
    return true;
  }
  return false;
}

// ============================================================================
// SETUP & INITIALIZATION
// ============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000); // Set I2C to Fast Mode (400kHz) for minimal bus overhead

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  analogReadResolution(12);

  if (!sensor.init()) {
    Serial.println("VL53L0X Fail");
    while (1);
  }
  sensor.setTimeout(100); // Reduced timeout duration
  sensor.startContinuous(33); // Continuous mode (~30Hz conversion cycle)
  
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("MPU6050 Fail");
    while (1);
  }
}

// ============================================================================
// MAIN LOOP (Asynchronous Multi-rate Execution)
// ============================================================================
void loop() {
  // 1. Process Telemetry Tuning Commands from Python
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    if (type == 'T') targetHeight = val;
    else if (type == 'P') Kp = val;
    else if (type == 'I') Ki = val;
    else if (type == 'D') Kd = val;
    while(Serial.available() > 0) Serial.read(); 
  }

  // 2. Poll ToF Rangefinder Non-Blockingly
  uint16_t rawDist = 0;
  if (checkToFReadyAndRead(rawDist)) {
    // Basic sanity filtering on hardware readings
    if (rawDist > 20 && rawDist < 400 && !sensor.timeoutOccurred()) {
      lastTofMeasurement = (float)rawDist;
      newTofAvailable = true;
    }
  }

  // 3. Timed Control & State Estimation Loop (25 Hz / 40 ms)
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    float dt = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;

    // Fetch Analog & IMU Readings
    pInRaw = analogRead(inputPin);
    pSpRaw = analogRead(springPin);
    
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Predict state via Accelerometer
    kalmanPredict((float)az, dt);

    // Correct state if ToF dataset was captured
    if (newTofAvailable) {
      kalmanCorrect(lastTofMeasurement);
      newTofAvailable = false; // Reset flag
    }
    
    float currentHeight   = x_hat[0]; 
    float currentVelocity = x_hat[1]; 

    // --- PID Actuation Logic ---
    float error = targetHeight - currentHeight;

    if (abs(error) > DEADZONE) {
      iTerm += error * dt;
      iTerm = constrain(iTerm, -I_LIMIT, I_LIMIT); // Anti-windup
      
      float dTerm = -currentVelocity; // Derivative on Measurement
      float output = (Kp * error) + (Ki * iTerm) + (Kd * dTerm);
      
      currentDutyCycle = constrain(abs((int)output), 20, 100); 
      currentStatusFlag = (output > 0) ? 1 : 2; 
    } else {
      currentDutyCycle = 0;
      currentStatusFlag = 0;
    }

    // Telemetry CSV stream out
    Serial.print(now); Serial.print(",");
    Serial.print(targetHeight, 1); Serial.print(",");
    Serial.print(currentHeight, 2); Serial.print(",");
    Serial.print(currentVelocity, 2); Serial.print(",");
    Serial.print(currentStatusFlag == 2 ? -currentDutyCycle : currentDutyCycle); Serial.print(",");
    Serial.print(pInRaw); Serial.print(",");
    Serial.print(pSpRaw); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }

  // 4. Drive Solenoid PWM continuously across cycle windows
  runBinaryPulsing(currentStatusFlag, currentDutyCycle);
}

// ============================================================================
// PWM SOLENOID ACTUATOR CONTROL (7 Hz Base Frequency Pulsing)
// ============================================================================
void runBinaryPulsing(int status, int dc) {
  unsigned long elapsed = millis() % CYCLE_TIME;
  unsigned long onTime = (dc * CYCLE_TIME) / 100;
  bool valveOpen = (elapsed < onTime);

  if (status == 1 && valveOpen) { // Filling (Inlet Open)
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW); digitalWrite(enB, HIGH);
    digitalWrite(enA, LOW);
  } 
  else if (status == 2 && valveOpen) { // Draining (Exhaust Open)
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(enA, HIGH);
    digitalWrite(enB, LOW);
  } 
  else { // All Valves Closed
    digitalWrite(enA, LOW);  digitalWrite(enB, LOW);
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  digitalWrite(in4, LOW);
  }
}