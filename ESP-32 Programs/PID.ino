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

const unsigned long LOOP_INTERVAL = 40; // 25 Hz Control Loop (40 ms)
const unsigned long CYCLE_TIME = 143;  // 7 Hz Solenoid PWM Base Cycle
unsigned long lastLoopTime = 0;

// --- PID Gains & Limits ---
float Kp = 2.5; 
float Ki = 0.1; 
float Kd = 0.5;
float iTerm = 0.0;
const float I_LIMIT = 50.0; 

// Refined deadband: Reduced from 4.0mm to 1.5mm thanks to Kalman noise suppression
const float DEADZONE = 1.5; 

// --- Target & State Variables ---
float targetHeight = 130.0; // Default nominal set-point (mm)
int pInRaw = 0, pSpRaw = 0;

// ============================================================================
// 2-STATE KINEMATIC KALMAN FILTER (Fusing ToF Laser & IMU Z-Acceleration)
// State Vector X = [ position (mm), velocity (mm/s) ]^T
// ============================================================================
float x_hat[2] = {130.0, 0.0}; // Initial State: [130mm, 0mm/s]
float P[2][2]  = {{10.0, 0.0}, 
                  {0.0, 10.0}}; // Initial Covariance

// Process & Measurement Noise Tuning
const float Q_accel = 150.0;   // Process noise variance from accelerometer (mm/s^2)^2
const float R_tof   = 4.0;     // Measurement noise variance from ToF laser (mm^2)
const float GRAVITY = 9806.65; // Gravity in mm/s^2 (Standard 1G)

// MPU6050 LSB -> mm/s^2 conversion factor (+/-2g range: 1g = 16384 LSB)
const float ACCEL_SCALE = 9806.65 / 16384.0; 

void updateKalmanFilter(float tof_meas_mm, float raw_az, float dt) {
  // 1. Convert raw IMU Z-acceleration to mm/s^2 and subtract gravity
  // Assumes +Z points UP when resting flat.
  float a_z = (raw_az * ACCEL_SCALE) - GRAVITY; 

  // --- PREDICT STEP (Kinematic Model) ---
  float pos_pred = x_hat[0] + x_hat[1] * dt + 0.5f * a_z * dt * dt;
  float vel_pred = x_hat[1] + a_z * dt;

  // Covariance Extrapolation: P_pred = F * P * F^T + Q
  float P00_new = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + 0.25f * dt * dt * dt * dt * Q_accel;
  float P01_new = P[0][1] + dt * P[1][1] + 0.5f * dt * dt * dt * Q_accel;
  float P10_new = P[1][0] + dt * P[1][1] + 0.5f * dt * dt * dt * Q_accel;
  float P11_new = P[1][1] + dt * dt * Q_accel;

  // --- UPDATE / CORRECT STEP (ToF Laser Reading) ---
  float y = tof_meas_mm - pos_pred; // Innovation
  float S = P00_new + R_tof;        // Innovation Covariance

  // Kalman Gain
  float K0 = P00_new / S;
  float K1 = P10_new / S;

  // State Correction
  x_hat[0] = pos_pred + K0 * y;
  x_hat[1] = vel_pred + K1 * y;

  // Covariance Update
  P[0][0] = (1.0f - K0) * P00_new;
  P[0][1] = (1.0f - K0) * P01_new;
  P[1][0] = P10_new - K1 * P00_new;
  P[1][1] = P11_new - K1 * P01_new;
}

// ============================================================================
// SETUP & INITIALIZATION
// ============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  analogReadResolution(12);

  if (!sensor.init()) {
    Serial.println("VL53L0X Fail");
    while (1);
  }
  sensor.setTimeout(500);
  sensor.startContinuous();
  
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("MPU6050 Fail");
    while (1);
  }
}

// ============================================================================
// MAIN CONTROL LOOP (25 Hz Sync)
// ============================================================================
void loop() {
  // 1. Listen for Real-time Tuning Commands from Python over USB
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    if (type == 'T') targetHeight = val;
    else if (type == 'P') Kp = val;
    else if (type == 'I') Ki = val;
    else if (type == 'D') Kd = val;
    while(Serial.available() > 0) Serial.read(); // Clear buffer
  }

  // 2. Timed Loop Execution (40 ms = 25 Hz)
  unsigned long now = millis();
  if (now - lastLoopTime >= LOOP_INTERVAL) {
    float dt = (now - lastLoopTime) / 1000.0f;
    lastLoopTime = now;

    // --- Read Hardware Sensors ---
    uint16_t rawDist = sensor.readRangeContinuousMillimeters();
    pInRaw = analogRead(inputPin);
    pSpRaw = analogRead(springPin);
    
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // --- Sensor Fusion Update ---
    updateKalmanFilter((float)rawDist, (float)az, dt);
    
    float currentHeight   = x_hat[0]; // Filtered Height (mm)
    float currentVelocity = x_hat[1]; // Filtered Velocity (mm/s)

    // --- PID Logic ---
    float error = targetHeight - currentHeight;
    int dutyCycle = 0;
    int statusFlag = 0; // 0: Idle, 1: Filling, 2: Draining

    // Conditional Integration: Only add to iTerm when outside the 1.5mm band
    if (abs(error) > DEADZONE) {
      iTerm += error * dt;
      iTerm = constrain(iTerm, -I_LIMIT, I_LIMIT); // Anti-windup
    }
    // Note: Inside <= 1.5mm band, iTerm holds its accumulated value (frozen)

    // Actuation Control Logic
    if (abs(error) > DEADZONE) {
      // Derivative on Process Velocity (Eliminates Derivative Kick)
      float dTerm = -currentVelocity;

      float output = (Kp * error) + (Ki * iTerm) + (Kd * dTerm);
      
      dutyCycle = constrain(abs((int)output), 20, 100); 
      statusFlag = (output > 0) ? 1 : 2; 
    } else {
      // Inside 1.5mm Deadband: Turn valves off to preserve air and eliminate chatter
      dutyCycle = 0;
      statusFlag = 0;
    }

    // --- Command Valves via PWM ---
    runBinaryPulsing(statusFlag, dutyCycle);

    // --- CSV Telemetry Stream for Python Logging & Paper Calculations ---
    Serial.print(now); Serial.print(",");
    Serial.print(targetHeight, 1); Serial.print(",");
    Serial.print(currentHeight, 2); Serial.print(",");   // Fused Height
    Serial.print(currentVelocity, 2); Serial.print(","); // Fused Velocity
    Serial.print(statusFlag == 2 ? -dutyCycle : dutyCycle); Serial.print(",");
    Serial.print(pInRaw); Serial.print(",");
    Serial.print(pSpRaw); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }
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