#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// ============================================================================
// PIN DEFINITIONS & HARDWARE MAPPING
// ============================================================================
const int enA = 25; const int in1 = 16; const int in2 = 17; // Exhaust Valve Driver
const int enB = 26; const int in3 = 19; const int in4 = 18; // Inlet Valve Driver
const int inputPin = 33;  // Supply Pressure Transducer (ADC Pin)
const int springPin = 32; // Air Spring Pressure Transducer (ADC Pin)

// ============================================================================
// HARDWARE & LOOP CONFIGURATION
// ============================================================================
VL53L0X sensor;
MPU6050 imu(0x68);

const unsigned long LOOP_INTERVAL = 40; // Strict 25 Hz Control Loop (40 ms)
const unsigned long CYCLE_TIME = 143;   // ~7 Hz Solenoid PWM Base Period
unsigned long lastLoopTime = 0;

// ============================================================================
// PID & SYSTEM PARAMETERS
// ============================================================================
const float KP_NOMINAL = 2.5f;
const float KI_NOMINAL = 0.1f;
const float KD_NOMINAL = 0.5f;
const float MASS_NOMINAL = 50.0f;     // Nominal calibrated load mass (kg)
const float P_SUPPLY_NOMINAL = 5.0f; // Nominal supply pressure (bar gauge)
const float P_ATM = 0.0f;            // Atmospheric pressure (bar gauge)

const float EXHAUST_GAIN_SCALE = 0.65f; // Directional scaling for exhaust

float Kp = KP_NOMINAL;
float Ki = KI_NOMINAL;
float Kd = KD_NOMINAL;

float iTerm = 0.0f;
const float OUTPUT_LIMIT = 100.0f; // Max PID output limit for Anti-Windup
const float DEADZONE = 1.5f;        // Target deadband (mm)

// Target & State Variables
float targetHeight = 130.0f; // Nominal height set-point (mm)
int pInRaw = 0, pSpRaw = 0;
float filteredMass = MASS_NOMINAL; 

// ============================================================================
// EMPIRICAL 2D LOOKUP TABLE: Mass Mapping m = f(P_spring, Height)
// ============================================================================
const int NUM_H_POINTS = 5;
const int NUM_P_POINTS = 4;

const float height_grid[NUM_H_POINTS] = {110.0f, 120.0f, 130.0f, 140.0f, 150.0f};
const float pressure_grid[NUM_P_POINTS] = {1.5f, 2.5f, 3.5f, 4.5f};

const float mass_table[NUM_P_POINTS][NUM_H_POINTS] = {
  {  8.0f,   7.5f,   5.0f,   4.2f,   3.5f }, // 1.5 bar
  { 22.0f,  20.0f,  15.0f,  12.5f,  10.0f }, // 2.5 bar
  { 45.0f,  40.0f,  32.0f,  26.0f,  22.0f }, // 3.5 bar 
  { 85.0f,  78.0f,  62.0f,  50.0f,  42.0f }  // 4.5 bar
};

// --- Bilinear Interpolation Function ---
float estimateMassFromTable(float current_h, float current_p) {
  current_h = constrain(current_h, height_grid[0], height_grid[NUM_H_POINTS - 1]);
  current_p = constrain(current_p, pressure_grid[0], pressure_grid[NUM_P_POINTS - 1]);

  int i_h = 0;
  while (i_h < NUM_H_POINTS - 2 && height_grid[i_h + 1] < current_h) i_h++;

  int i_p = 0;
  while (i_p < NUM_P_POINTS - 2 && pressure_grid[i_p + 1] < current_p) i_p++;

  i_h = constrain(i_h, 0, NUM_H_POINTS - 2);
  i_p = constrain(i_p, 0, NUM_P_POINTS - 2);

  float t_h = (current_h - height_grid[i_h]) / (height_grid[i_h + 1] - height_grid[i_h]);
  float t_p = (current_p - pressure_grid[i_p]) / (pressure_grid[i_p + 1] - pressure_grid[i_p]);

  float m00 = mass_table[i_p][i_h];
  float m01 = mass_table[i_p][i_h + 1];
  float m10 = mass_table[i_p + 1][i_h];
  float m11 = mass_table[i_p + 1][i_h + 1];

  float m_interp = (1.0f - t_p) * ((1.0f - t_h) * m00 + t_h * m01) +
                   t_p        * ((1.0f - t_h) * m10 + t_h * m11);

  return m_interp;
}

// ============================================================================
// ASYNCHRONOUS 2-STATE KALMAN FILTER (Predict / Correct Separated)
// ============================================================================
float x_hat[2] = {130.0f, 0.0f}; // State vector: [position (mm), velocity (mm/s)]
float P[2][2]  = {{10.0f, 0.0f}, {0.0f, 10.0f}}; // Covariance Matrix

const float Q_accel = 150.0f;   // Process noise from acceleration uncertainty
const float R_tof   = 4.0f;     // Measurement noise variance (VL53L0X)
const float GRAVITY = 9806.65f; // mm/s^2
const float ACCEL_SCALE = 9806.65f / 16384.0f; // Converts raw LSB to mm/s^2

// --- 1. PREDICT STEP (Runs every 40 ms loop using fast accelerometer) ---
void kalmanPredict(float raw_az, float dt) {
  float a_z = (raw_az * ACCEL_SCALE) - GRAVITY; 

  // State Kinematic Projection
  float pos_pred = x_hat[0] + x_hat[1] * dt + 0.5f * a_z * dt * dt;
  float vel_pred = x_hat[1] + a_z * dt;

  // Symmetric Covariance Propagation (F*P*F^T + Q)
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;

  float P00_new = P[0][0] + dt * (P[1][0] + P[0][1]) + dt2 * P[1][1] + 0.25f * dt4 * Q_accel;
  float P01_new = P[0][1] + dt * P[1][1] + 0.5f * dt3 * Q_accel;
  float P10_new = P[1][0] + dt * P[1][1] + 0.5f * dt3 * Q_accel; // Explicitly symmetric
  float P11_new = P[1][1] + dt2 * Q_accel;

  x_hat[0] = pos_pred;
  x_hat[1] = vel_pred;

  P[0][0] = P00_new; P[0][1] = P01_new;
  P[1][0] = P10_new; P[1][1] = P11_new;
}

// --- 2. CORRECT STEP (Runs ONLY when ToF sensor finishes a measurement) ---
void kalmanCorrect(float tof_meas_mm) {
  float y = tof_meas_mm - x_hat[0]; // Innovation
  float S = P[0][0] + R_tof;        // Innovation Covariance

  float K0 = P[0][0] / S;           // Kalman Gain (Position)
  float K1 = P[1][0] / S;           // Kalman Gain (Velocity)

  // State Update
  x_hat[0] = x_hat[0] + K0 * y;
  x_hat[1] = x_hat[1] + K1 * y;

  // Covariance Update (Joseph form / Standard update)
  float P00_old = P[0][0];
  float P01_old = P[0][1];

  P[0][0] = (1.0f - K0) * P00_old;
  P[0][1] = (1.0f - K0) * P01_old;
  P[1][0] = P[1][0] - K1 * P00_old;
  P[1][1] = P[1][1] - K1 * P01_old;
}

// Non-blocking query to check if VL53L0X has completed a measurement cycle
bool isToFDataReady() {
  // Reads Result Interrupt Status register (0x13) on VL53L0X
  return (sensor.readReg(0x13) & 0x07) != 0; 
}

// Helper: Convert ADC Raw to Pressure (bar gauge)
float adcToBar(int adc_raw) {
  float voltage = (adc_raw / 4095.0f) * 3.3f;
  float bar = (voltage - 0.5f) * (10.0f / 4.0f);
  return constrain(bar, 0.0f, 10.0f);
}

// Forward Declaration
void runBinaryPulsing(int status, int dc);

// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000); // 400 kHz Fast-Mode I2C to minimize bus latency

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  analogReadResolution(12);

  if (!sensor.init()) {
    Serial.println("Error: VL53L0X Initialization Failed!");
    while (1);
  }
  sensor.setTimeout(500);
  sensor.startContinuous(33); // Continuous back-to-back readings (~33ms timing budget)

  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("Error: MPU6050 Initialization Failed!");
    while (1);
  }
}

// ============================================================================
// MAIN CONTROL LOOP (25 Hz Deterministic Execution)
// ============================================================================
void loop() {
  // --- 1. Serial Command Handling ---
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    if (type == 'T' || type == 't') targetHeight = val;
    while (Serial.available() > 0) Serial.read(); // Clear remaining buffer
  }

  // --- 2. Deterministic 40 ms Control Loop ---
  unsigned long now = millis();
  if (now - lastLoopTime >= LOOP_INTERVAL) {
    float dt = (now - lastLoopTime) / 1000.0f;
    lastLoopTime = now;

    // --- SENSOR ACQUISITION ---
    pInRaw = analogRead(inputPin);
    pSpRaw = analogRead(springPin);

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // --- ASYNCHRONOUS EKF STEP 1: Always Predict using IMU ---
    kalmanPredict((float)az, dt);

    // --- ASYNCHRONOUS EKF STEP 2: Correct ONLY if ToF Data is Ready ---
    if (isToFDataReady()) {
      uint16_t rawDist = sensor.readRangeContinuousMillimeters();
      if (!sensor.timeoutOccurred()) {
        kalmanCorrect((float)rawDist);
      }
    }

    float currentHeight   = x_hat[0]; // Clean, non-blocking estimated position
    float currentVelocity = x_hat[1]; // Clean, non-blocking estimated velocity

    // Convert ADC Pressures to Bar Gauge
    float pSupplyBar = adcToBar(pInRaw);
    float pSpringBar = adcToBar(pSpRaw);

    // --- REAL-TIME MASS ESTIMATION ---
    float rawEstimatedMass = estimateMassFromTable(currentHeight, pSpringBar);
    filteredMass = 0.90f * filteredMass + 0.10f * rawEstimatedMass;

    // --- DIRECTIONAL & PRESSURE-GRADIENT GAIN SCHEDULING ---
    float error = targetHeight - currentHeight;
    float massScale = filteredMass / MASS_NOMINAL;
    float pressureScale = 1.0f;

    if (error > 0.0f) {
      // FILL DYNAMIC (Lifting Mass against Gravity)
      float deltaP_fill = max(pSupplyBar - pSpringBar, 0.5f); 
      float nominal_deltaP_fill = max(P_SUPPLY_NOMINAL - pSpringBar, 0.5f);
      
      pressureScale = nominal_deltaP_fill / deltaP_fill;

      Kp = KP_NOMINAL * massScale * pressureScale;
      Kd = KD_NOMINAL * sqrt(massScale) * pressureScale;
      Ki = KI_NOMINAL * pressureScale;

    } else {
      // DRAIN DYNAMIC (Gravity assists collapse)
      float deltaP_drain = max(pSpringBar - P_ATM, 0.2f);
      float nominal_deltaP_drain = 2.5f;

      pressureScale = nominal_deltaP_drain / deltaP_drain;

      Kp = KP_NOMINAL * massScale * pressureScale * EXHAUST_GAIN_SCALE;
      Kd = KD_NOMINAL * sqrt(massScale) * pressureScale * EXHAUST_GAIN_SCALE;
      Ki = KI_NOMINAL * pressureScale * EXHAUST_GAIN_SCALE;
    }

    // --- PID CONTROL CALCULATIONS WITH ANTI-WINDUP ---
    int dutyCycle = 0;
    int statusFlag = 0; // 0: Idle, 1: Filling, 2: Draining

    if (abs(error) > DEADZONE) {
      // Accumulate integral term
      iTerm += error * dt;

      // Derivative on state-estimated velocity (prevents derivative kick)
      float dTerm = -currentVelocity; 
      
      // Calculate Unconstrained PID Output
      float rawOutput = (Kp * error) + (Ki * iTerm) + (Kd * dTerm);

      // Clamped anti-windup back-calculation
      if (rawOutput > OUTPUT_LIMIT) {
        rawOutput = OUTPUT_LIMIT;
        iTerm -= error * dt; // Freeze integration when saturated
      } else if (rawOutput < -OUTPUT_LIMIT) {
        rawOutput = -OUTPUT_LIMIT;
        iTerm -= error * dt;
      }

      dutyCycle = constrain(abs((int)rawOutput), 20, 100); 
      statusFlag = (rawOutput > 0.0f) ? 1 : 2; 

    } else {
      dutyCycle = 0;
      statusFlag = 0;
      iTerm = 0.0f; // Reset integral term inside deadband to prevent overshoot on reference steps
    }

    // Command Valve Outputs
    runBinaryPulsing(statusFlag, dutyCycle);

    // --- CSV TELEMETRY STREAM FOR PAPER DATA COLLECTION ---
    Serial.print(now); Serial.print(",");
    Serial.print(targetHeight, 1); Serial.print(",");
    Serial.print(currentHeight, 2); Serial.print(",");   
    Serial.print(currentVelocity, 2); Serial.print(","); 
    Serial.print(statusFlag == 2 ? -dutyCycle : dutyCycle); Serial.print(",");
    Serial.print(pSupplyBar, 2); Serial.print(",");
    Serial.print(pSpringBar, 2); Serial.print(",");
    Serial.print(filteredMass, 2); Serial.print(",");
    Serial.print(Kp, 2); Serial.print(",");       
    Serial.print(Kd, 2); Serial.print(",");       
    Serial.println(gz);
  }
}

// ============================================================================
// VALVES PWM PULSING FUNCTION
// ============================================================================
void runBinaryPulsing(int status, int dc) {
  if (status == 0 || dc == 0) {
    digitalWrite(enA, LOW);  digitalWrite(enB, LOW);
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  digitalWrite(in4, LOW);
    return;
  }

  unsigned long elapsed = millis() % CYCLE_TIME;
  unsigned long onTime = (dc * CYCLE_TIME) / 100;
  bool valveOpen = (elapsed < onTime);

  if (status == 1 && valveOpen) { // Filling (Inlet Valve)
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW); digitalWrite(enB, HIGH);
    digitalWrite(enA, LOW);
  } 
  else if (status == 2 && valveOpen) { // Draining (Exhaust Valve)
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(enA, HIGH);
    digitalWrite(enB, LOW);
  } 
  else { // Off portion of PWM cycle
    digitalWrite(enA, LOW);  digitalWrite(enB, LOW);
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  digitalWrite(in4, LOW);
  }
}