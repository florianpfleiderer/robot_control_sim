# Created on Sat Nov 15 2025 by Florian Pfleiderer

import pandas as pd
import matplotlib.pyplot as plt
from typing import Tuple

def plot_trajectory(true_xs, true_ys, sensor_xs, sensor_ys, kf_xs, kf_ys, target: Tuple[float, float]):
    plt.figure()
    plt.plot(true_xs, true_ys, label="True position")
    # Kalman filter estimate as dashed line
    plt.plot(kf_xs, kf_ys, linestyle='--', color='C2', label="Kalman estimate")
    plt.scatter(sensor_xs, sensor_ys, s=10, alpha=0.4, label="Sensor readings")
    plt.scatter([target[0]], [target[1]], marker="x", s=80, label="Target")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("2D Robot Trajectory")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()

def plot_time_series(true_xs, true_ys, kf_xs=None, kf_ys=None):
    steps = range(len(true_xs))
    plt.figure()
    plt.plot(steps, true_xs, label="x(t)")
    plt.plot(steps, true_ys, label="y(t)")
    # Plot Kalman estimates as dashed lines if provided
    if kf_xs is not None and kf_ys is not None:
        plt.plot(steps, kf_xs, linestyle='--', label="x_kf(t)")
        plt.plot(steps, kf_ys, linestyle='--', label="y_kf(t)")
    plt.xlabel("Step")
    plt.ylabel("Position")
    plt.title("Position over Time")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # Read CSV
    df = pd.read_csv("trajectory_data.csv")
    
    target = (5.0, 5.0)
    # Pass Kalman filter columns `x_kf` and `y_kf` to plotting functions
    plot_trajectory(df["true_x"], df["true_y"], df["sensor_x"], df["sensor_y"], df.get("x_kf"), df.get("y_kf"), target)
    plot_time_series(df["true_x"], df["true_y"], df.get("x_kf"), df.get("y_kf"))