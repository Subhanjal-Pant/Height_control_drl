import numpy as np
import pandas as pd
import matplotlib.pyplot as plt 

def generate_mock_data(filename="trial_one.csv", duration=10.0, freq=50):
    min_height = 145
    max_height = 190
    print(f"Generating data:{filename}")
    num_samples = int(duration*freq)
    time = np.linspace(0, duration, num_samples)
    
    # Since x(t) = A.e^(-alpha*t).sin(wd*t + phi)
    # w_d = w_b*sqrt(1-alpha^2)
    Amplitude = (max_height-min_height)/2.0
    offset = (max_height+min_height)/2.0
    freq_hz = 1.5
    decay_rate = 1.0
    
    tof_height = offset + Amplitude*np.exp(-decay_rate*time)*np.sin(2*np.pi*freq_hz*time)
    
    accel_x = np.random.normal(0, 0.05, num_samples)
    accel_y = np.random.normal(0, 0.05, num_samples)
    accel_z = 9.81 + np.random.normal(0, 0.1, num_samples)
    gyro_x  = np.random.normal(0, 0.01, num_samples)
    gyro_y  = np.random.normal(0, 0.01, num_samples)
    gyro_z  = np.random.normal(0, 0.01, num_samples)
    
    # Pressure in BAR (Base pressure = 3.0 bar)
    inlet_pressure = 3.0 - 0.05 * np.sin(2 * np.pi * 0.2 * time) + np.random.normal(0, 0.005, num_samples)

    tof_height_m = tof_height/1000.0

    air_spring_pressure = 1.5 + 2.0 * (0.1675 - tof_height_m) + np.random.normal(0, 0.01, num_samples)
    
    valve1 = np.where((time%2.0)<1.0, 1, 0)
    valve2 = np.where((time%2.0)>=1.0, 1, 0)
    
    df = pd.DataFrame({
        'time':time, 
        'tof_height': tof_height,
        'accel_x': accel_x,
        'accel_y': accel_y,
        'accel_z': accel_z,
        'inlet_pressure': inlet_pressure,
        'air_spring_pressure': air_spring_pressure,
        'inlet_valve': valve1,
        'exhaust_valve': valve2
    })
    df.to_csv(filename, index=False, float_format="%.4f")
    print(f"Mock data successfully generated: {filename} with {num_samples} rows")
if __name__ == "__main__":
    
    generate_mock_data()
    
    