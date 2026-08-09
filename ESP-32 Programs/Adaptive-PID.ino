#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// --- Pin Definitions ---
const int enA = 25; const int in1 = 16; const int in2 = 17; // Exhaust Valve
const int enB = 26; const int in3 = 19; const int in4 = 18; // Inlet Valve
const int inputPin = 33;  // Supply Pressure Transducer
const int springPin = 32; // Air Spring Pressure Transducer

// --- Hardware & Loop Config ---
VL53L0X sensor;
MPU6050 imu(0x68);

const unsigned long LOOP_INTERVAL = 40; // 25 Hz Control Loop (40 ms)
const unsigned long CYCLE_TIME = 143;   // 7 Hz Solenoid PWM Base Cycle
unsigned long lastLoopTime = 0;

// --- Baseline Nominal PID Gains (at Nominal Mass = 50kg, Supply = 5 bar) ---
// TUNE THESE ONCE USING ZIEGLER-NICHOLS AT 50KG / 5 BAR GOING UPWARD
const float KP_NOMINAL = 2.5;
const float KI_NOMINAL = 0.1;
const float KD_NOMINAL = 0.5;
const float MASS_NOMINAL = 50.0;     // Nominal calibrated load mass (kg)
const float P_SUPPLY_NOMINAL = 5.0; // Nominal supply pressure (bar)
const float P_ATM = 0.0;            // Atmospheric pressure (bar gauge)

// --- Asymmetric Exhaust Scaling Coefficient ---
// TUNE THIS ONCE BY COMPARING DOWNWARD STEP SMOOTHNESS TO UPWARD STEP
const float EXHAUST_GAIN_SCALE = 0.65; 

// Active Dynamic Gains
float Kp = KP_NOMINAL;
float Ki = KI_NOMINAL;
float Kd = KD_NOMINAL;

float iTerm = 0.0;
const float I_LIMIT = 50.0; 
const float DEADZONE = 1.5; // Deadband in mm

// --- Target & State Variables ---
float targetHeight = 130.0; // Nominal height set-point (mm)
int pInRaw = 0, pSpRaw = 0;
float filteredMass = MASS_NOMINAL; 

// ============================================================================
// EMPIRICAL 2D LOOKUP TABLE: Mass Mapping m = f(P_spring, Height)
// ============================================================================
const int NUM_H_POINTS = 5;
const int NUM_P_POINTS = 4;

const float height_grid[NUM_H_POINTS] = {110.0, 120.0, 130.0, 140.0, 150.0};
const float pressure_grid[NUM_P_POINTS] = {1.5, 2.5, 3.5, 4.5};

const float mass_table[NUM_P_POINTS][NUM_H_POINTS] = {
  {  8.0,   7.5,   5.0,   4.2,   3.5 }, // 1.5 bar
  { 22.0,  20.0,  15.0,  12.5,  10.0 }, // 2.5 bar
  { 45.0,  40.0,  32.0,  26.0,  22.0 }, // 3.5 bar 
  { 85.0,  78.0,  62.0,  50.0,  42.0 }  // 4.5 bar
};

// --- Bilinear Interpolation Function ---
float estimateMassFromTable(float current_h, float current_p) {
  current_h = constrain(current_h, height_grid[0], height_grid[NUM_H_POINTS - 1]);
  current_p = constrain(current_p, pressure_grid[0], pressure_grid[NUM_P_POINTS - 1]);

  int i_h = 0;
  while (i_h < NUM_H_POINTS - 2 && height_grid[i_h + 1] < current_h) i_h++;

  int i_p = 0;
  while (i_p < NUM_P_POINTS - 2 && pressure_grid[i_p + 1] < current_p) i_p++;

  // --- BUG FIX 1: Explicit boundary clamping to prevent out-of-bounds memory access ---
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
// 2-STATE KINEMATIC KALMAN FILTER 
// ============================================================================
float x_hat[2] = {130.0, 0.0}; // [position (mm), velocity (mm/s)]
float P[2][2]  = {{10.0, 0.0}, {0.0, 10.0}};

const float Q_accel = 150.0;   
const float R_tof   = 4.0;     
const float GRAVITY = 9806.65; // mm/s^2
const float ACCEL_SCALE = 9806.65 / 16384.0; 

void updateKalmanFilter(float tof_meas_mm, float raw_az, float dt) {
  float a_z = (raw_az * ACCEL_SCALE) - GRAVITY; 

  // Predict
  float pos_pred = x_hat[0] + x_hat[1] * dt + 0.5f * a_z * dt * dt;
  float vel_pred = x_hat[1] + a_z * dt;

  float P00_new = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + 0.25f * dt * dt * dt * dt * Q_accel;
  float P01_new = P[0][1] + dt * P[1][1] + 0.5f * dt * dt * dt * Q_accel;
  float P10_new = P[1][0] + dt * P[1][1] + 0.5f * dt * dt * dt * Q_accel;
  float P11_new = P[1][1] + dt * dt * Q_accel;

  // Correct
  float y = tof_meas_mm - pos_pred; 
  float S = P00_new + R_tof;        

  float K0 = P00_new / S;
  float K1 = P10_new / S;

  x_hat[0] = pos_pred + K0 * y;
  x_hat[1] = vel_pred + K1 * y;

  P[0][0] = (1.0f - K0) * P00_new;
  P[0][1] = (1.0f - K0) * P01_new;
  P[1][0] = P10_new - K1 * P00_new;
  P[1][1] = P11_new - K1 * P01_new;
}

// Helper: Convert ADC Raw to Pressure (bar gauge)
float adcToBar(int adc_raw) {
  float voltage = (adc_raw / 4095.0f) * 3.3f;
  float bar = (voltage - 0.5f) * (10.0f / 4.0f);
  return constrain(bar, 0.0f, 10.0f);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  analogReadResolution(12);

  if (!sensor.init()) { while (1); }
  sensor.setTimeout(500);
  sensor.startContinuous();
  
  imu.initialize();
  if (!imu.testConnection()) { while (1); }
}

void loop() {
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    if (type == 'T') targetHeight = val;
    while(Serial.available() > 0) Serial.read();
  }

  unsigned long now = millis();
  if (now - lastLoopTime >= LOOP_INTERVAL) {
    float dt = (now - lastLoopTime) / 1000.0f;
    lastLoopTime = now;

    // Read Sensors
    uint16_t rawDist = sensor.readRangeContinuousMillimeters();
    pInRaw = analogRead(inputPin);
    pSpRaw = analogRead(springPin);
    
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // EKF Update
    updateKalmanFilter((float)rawDist, (float)az, dt);
    float currentHeight   = x_hat[0]; 
    float currentVelocity = x_hat[1]; 

    // Convert Pressures to bar
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
      // --- FILL DYNAMIC (Lifting Mass against Gravity) ---
      float deltaP_fill = max(pSupplyBar - pSpringBar, 0.5f); // Prevent division by zero
      float nominal_deltaP_fill = max(P_SUPPLY_NOMINAL - pSpringBar, 0.5f);
      
      pressureScale = nominal_deltaP_fill / deltaP_fill;

      // Gains scale directly with mass inertia to lift heavier loads
      Kp = KP_NOMINAL * massScale * pressureScale;
      Kd = KD_NOMINAL * sqrt(massScale) * pressureScale;
      Ki = KI_NOMINAL * pressureScale;

    } else {
      // --- DRAIN DYNAMIC (Gravity assists collapse) ---
      float deltaP_drain = max(pSpringBar - P_ATM, 0.2f);
      float nominal_deltaP_drain = 2.5f; // Nominal benchmark spring pressure differential

      pressureScale = nominal_deltaP_drain / deltaP_drain;

      // --- BUG FIX 2: Corrected Mass Scaling for Draining ---
      // Mass inertia applies going down too; EXHAUST_GAIN_SCALE handles the gravity assist reduction.
      Kp = KP_NOMINAL * massScale * pressureScale * EXHAUST_GAIN_SCALE;
      Kd = KD_NOMINAL * sqrt(massScale) * pressureScale * EXHAUST_GAIN_SCALE;
      Ki = KI_NOMINAL * pressureScale * EXHAUST_GAIN_SCALE;
    }

    // --- PID CONTROL CALCULATIONS ---
    int dutyCycle = 0;
    int statusFlag = 0; // 0: Idle, 1: Filling, 2: Draining

    if (abs(error) > DEADZONE) {
      iTerm += error * dt;
      iTerm = constrain(iTerm, -I_LIMIT, I_LIMIT);

      float dTerm = -currentVelocity; // Derivative action on state-estimated velocity
      float output = (Kp * error) + (Ki * iTerm) + (Kd * dTerm);
      
      dutyCycle = constrain(abs((int)output), 20, 100); 
      statusFlag = (output > 0) ? 1 : 2; 
    } else {
      dutyCycle = 0;
      statusFlag = 0;
    }

    // Command Valves
    runBinaryPulsing(statusFlag, dutyCycle);

    // --- CSV TELEMETRY STREAM ---
    Serial.print(now); Serial.print(",");
    Serial.print(targetHeight, 1); Serial.print(",");
    Serial.print(currentHeight, 2); Serial.print(",");   
    Serial.print(currentVelocity, 2); Serial.print(","); 
    Serial.print(statusFlag == 2 ? -dutyCycle : dutyCycle); Serial.print(",");
    Serial.print(pInRaw); Serial.print(",");
    Serial.print(pSpRaw); Serial.print(",");
    Serial.print(filteredMass, 2); Serial.print(",");
    Serial.print(Kp, 2); Serial.print(",");       
    Serial.print(Kd, 2); Serial.print(",");       
    Serial.println(gz);
  }
}

void runBinaryPulsing(int status, int dc) {
  unsigned long elapsed = millis() % CYCLE_TIME;
  unsigned long onTime = (dc * CYCLE_TIME) / 100;
  bool valveOpen = (elapsed < onTime);

  if (status == 1 && valveOpen) { // Filling
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW); digitalWrite(enB, HIGH);
    digitalWrite(enA, LOW);
  } 
  else if (status == 2 && valveOpen) { // Draining
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(enA, HIGH);
    digitalWrite(enB, LOW);
  } 
  else { // Hold / Off
    digitalWrite(enA, LOW);  digitalWrite(enB, LOW);
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  digitalWrite(in4, LOW);
  }
}