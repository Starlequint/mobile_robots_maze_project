import numpy as np
import matplotlib.pyplot as plt
import time


# ============================================================
# CONFIGURATION
# ============================================================

SAMPLES = 1000
SAMPLE_INTERVAL = 0.05  # 20 Hz

# ============================================================
# SENSOR ACQUISITION
# ============================================================

def read_ir():
    """
    Replace with your Thymio IR sensor acquisition.

    Expected output:
        [x, y, theta]

    Units:
        x, y: mm
        theta: radians
    """

    # Example:
    # prox = thymio["prox.horizontal"]
    # return ir_to_pose(prox)

    raise NotImplementedError("Implement read_ir()")


def read_encoder():
    """
    Replace with your Thymio encoder acquisition.

    Expected output:
        [x, y, theta]

    Units:
        x, y: mm
        theta: radians
    """

    # Example:
    # left = thymio["motor.left.speed"]
    # right = thymio["motor.right.speed"]
    # return encoder_to_pose(left, right)

    raise NotImplementedError("Implement read_encoder()")


# ============================================================
# ANGLE PROCESSING
# ============================================================

def angular_deviation(angles, mean_angle):
    return (angles - mean_angle + np.pi) % (2*np.pi) - np.pi


def calculate_statistics(data):

    data = np.asarray(data)

    mean = np.mean(data, axis=0)

    # Calculate angular mean using circular statistics
    mean[2] = np.arctan2(
        np.mean(np.sin(data[:, 2])),
        np.mean(np.cos(data[:, 2]))
    )

    deviations = data - mean

    deviations[:, 2] = angular_deviation(
        data[:, 2],
        mean[2]
    )

    covariance = np.cov(deviations.T)

    std = np.sqrt(np.diag(covariance))

    return mean, std, covariance


# ============================================================
# ACQUISITION
# ============================================================

def collect_data(read_function, name):

    print(f"\nPreparing {name} calibration...")
    input("Place the Thymio in its calibration position and press ENTER.")

    measurements = []

    print(f"Collecting {SAMPLES} measurements...")

    for i in range(SAMPLES):

        measurement = read_function()

        measurements.append(measurement)

        if i % 100 == 0:
            print(f"{i}/{SAMPLES}")

        time.sleep(SAMPLE_INTERVAL)

    measurements = np.asarray(measurements)

    print(f"{name} acquisition completed.")

    return measurements


# ============================================================
# PLOTTING
# ============================================================

def plot_measurements(data, name):

    labels = ["X [mm]", "Y [mm]", "Theta [rad]"]

    fig, axes = plt.subplots(3, 1, figsize=(12, 8))

    for i, ax in enumerate(axes):

        ax.plot(data[:, i])

        ax.set_ylabel(labels[i])
        ax.set_title(f"{name} - {labels[i]}")
        ax.grid(True)

    axes[-1].set_xlabel("Sample")

    plt.tight_layout()
    plt.show()


# ============================================================
# CALIBRATION
# ============================================================

def calibrate_sensor(read_function, name):

    data = collect_data(read_function, name)

    mean, std, covariance = calculate_statistics(data)

    print("\n====================================")
    print(f"{name} CALIBRATION RESULTS")
    print("====================================")

    print("\nMean:")
    print(mean)

    print("\nStandard deviation:")
    print(std)

    print("\nCovariance matrix R:")
    print(covariance)

    plot_measurements(data, name)

    return covariance


# ============================================================
# MAIN
# ============================================================
from position_from_IR_sensors import position_from_ir_sensors


if __name__ == "__main__":

    R_ir = calibrate_sensor(
        position_from_ir_sensors,
        "INFRARED"
    )

    # R_enc = calibrate_sensor(
    #     read_encoder,
    #     "ENCODER"
    # )

    print("\n\n====================================")
    print("FINAL KALMAN MATRICES")
    print("====================================")

    print("\nR_ir =")
    print(repr(R_ir))

    # print("\nR_enc =")
    # print(repr(R_enc))