import cv2
import numpy as np
import time
import serial
import json
import math
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d, splprep, splev

# ================== SERIAL COMMUNICATION ==================
ser = serial.Serial("COM3", timeout=9600, dsrdtr=False)

# ================== CAMERA PARAMETERS ==================
# Parameters obtained from CAMERA CALIBRATION
fx_calibration = 548.297  # Focal length in pixels (x-axis)
fy_calibration = 550.1771  # Focal length in pixels (y-axis)

# Define the principal points from calibration
cent_x = 366.23  # pixels
cent_y = 238.42  # pixels

# Camera matrix
camera_matrix = np.array([[fx_calibration, 0, cent_x],
                          [0, fy_calibration, cent_y],
                          [0, 0, 1]], dtype=np.float64)

# Radial and tangential distortions from camera calibration
k1, k2, k3 = -0.40630288, 0.21387767, -0.08096464
p1, p2 = -0.00775763, 0.00225154

# Distortion coefficients
dist_coefficients = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

# ================== FUNCTIONS ==================

def calculate_XYZ_centroid(x0, y0, major_diameter, real_length=50):
    """
    Calculate the 3D coordinates (X, Y, Z) of the centroid of the ellipse.

    Parameters:
        x0, y0 (float): Centroid coordinates of the ellipse in pixels.
        major_diameter (float): Major diameter of the ellipse in pixels.
        real_length (float): Real-world length of the tracked object in mm.

    Returns:
        tuple: (X, Y, Z) position of the centroid.
    """
    if major_diameter == 0:
        return 0, 0, 0  # Object out of the field of view

    Z_distance = (fy_calibration * real_length) / major_diameter
    X_distance = ((x0 - cent_x) / fx_calibration) * Z_distance
    Y_distance = ((y0 - cent_y) / fy_calibration) * Z_distance

    return X_distance, Y_distance, Z_distance

def undistort_function(src, camera_matrix, dist_coeffs, R=None, P=None):
    """
    Corrects the distortion of centroid positions using the camera's intrinsic parameters.

    Parameters:
        src (array): Distorted centroid positions in pixels.
        camera_matrix (array): Camera intrinsic matrix.
        dist_coeffs (array): Distortion coefficients.
        R (array, optional): Rotation matrix.
        P (array, optional): Projection matrix.

    Returns:
        array: Corrected centroid positions.
    """
    A = np.zeros((3, 3), dtype=np.float64)
    RR = np.eye(3, dtype=np.float64)

    k = np.zeros(8, dtype=np.float64)
    fx, fy, cx, cy = camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]
    ifx, ify = 1.0 / fx, 1.0 / fy

    # Convert distortion coefficients
    assert dist_coeffs.shape in [(4,), (5,), (8,)], "Invalid distortion coefficients shape"
    k[:dist_coeffs.size] = dist_coeffs

    # Rotation matrix check
    if R is not None:
        assert R.shape == (3, 3), "Invalid rotation matrix shape"
        RR = R

    if P is not None:
        assert P.shape in [(3, 3), (3, 4)], "Invalid projection matrix shape"
        RR = P[:, :3] @ RR

    # Prepare the source points
    src = np.array(src, dtype=np.float64).reshape(-1, 2)
    dst = np.zeros_like(src, dtype=np.float64)

    for i in range(src.shape[0]):
        x, y = src[i, 0], src[i, 1]
        x0, y0 = (x - cx) * ifx, (y - cy) * ify
        x, y = x0, y0

        for _ in range(5):  # Iterative distortion correction
            r2 = x * x + y * y
            icdist = (1 + ((k[7] * r2 + k[6]) * r2 + k[5]) * r2) / (1 + ((k[4] * r2 + k[1]) * r2 + k[0]) * r2)
            deltaX = 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x)
            deltaY = k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y
            x, y = (x0 - deltaX) * icdist, (y0 - deltaY) * icdist

        x = x * fx + cx
        y = y * fy + cy
        dst[i, 0], dst[i, 1] = x, y

    return dst

def euclidean_distance(point1, point2):
    """Compute Euclidean distance between two points."""
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)

def max_diameter_calculation(blob_centroids):
    """
    Calculate the maximum distance between detected blobs.

    Parameters:
        blob_centroids (list): List of detected blob positions.

    Returns:
        tuple: Maximum distance and the two farthest points.
    """
    if len(blob_centroids) < 2:
        return 0, []

    max_distance = 0
    farthest_points = []

    for i in range(len(blob_centroids)):
        for k in range(i + 1, len(blob_centroids)):
            distance = euclidean_distance(blob_centroids[i], blob_centroids[k])
            if distance > max_distance:
                max_distance = distance
                farthest_points = (blob_centroids[i], blob_centroids[k])

    return max_distance, farthest_points

# ================== MAIN LOOP ==================
while True:
    start_time = time.time()
    line = ser.readline().strip()

    if line:
        try:
            data = json.loads(line.decode('utf-8'))
            blob_centroids = np.array(data["blob_centroids_camera"], dtype=np.float32)
        except json.decoder.JSONDecodeError:
            print("Received an invalid JSON string:", line.decode('utf-8'))
            continue

        # Correct the centroid positions
        corrected_centroids = np.array([undistort_function(centroid, camera_matrix, dist_coefficients) for centroid in blob_centroids], dtype=np.float32).reshape(-1, 2)

        # Calculate the maximum diameter and centroid
        major_diameter, centroid = max_diameter_calculation(corrected_centroids)
        centroid_x, centroid_y = centroid[0], centroid[1]

        # Compute the real-world position using the pinhole camera model
        X_distance, Y_distance, Z_distance = calculate_XYZ_centroid(centroid_x, centroid_y, major_diameter)

        end_time = time.time()
        frame_time = end_time - start_time
        fps_computer = 1 / frame_time

        print(f"X_distance: {np.round(X_distance / 10, 2)} cm, Y_distance: {np.round(Y_distance / 10, 2)} cm, Z_distance: {np.round(Z_distance / 10, 2)} cm")

