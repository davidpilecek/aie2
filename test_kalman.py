import numpy as np
import matplotlib.pyplot as plt
import cv2
# Simulated noisy measurements
true_positions = np.linspace(0, 10, 100)  # Ground truth
noisy_positions = true_positions + np.random.normal(0, 0.5, size=100)  # Add noise

# Initialize Kalman filter
kalman = cv2.KalmanFilter(2, 1)  # State: position, velocity; Measurement: position
kalman.transitionMatrix = np.array([[1, 1], [0, 1]], dtype=np.float32)
kalman.measurementMatrix = np.array([[1, 0]], dtype=np.float32)
kalman.processNoiseCov = np.array([[1e-5, 0], [0, 1e-5]], dtype=np.float32)
kalman.measurementNoiseCov = np.array([[1e-1]], dtype=np.float32)
kalman.errorCovPost = np.eye(2, dtype=np.float32)
kalman.statePost = np.array([0, 0], dtype=np.float32)

# Apply Kalman filter
filtered_positions = []
for measurement in noisy_positions:
    kalman.predict()
    corrected = kalman.correct(np.array([[measurement]], dtype=np.float32))
    filtered_positions.append(corrected[0])

# Plot the results

plt.plot(true_positions, label="True Position")
plt.plot(noisy_positions, label="Noisy Measurements", alpha=0.5)
plt.plot(filtered_positions, label="Kalman Filter Output")
plt.legend()
plt.savefig("kalman_filter_plot.png")  # Save the plot as a PNG file