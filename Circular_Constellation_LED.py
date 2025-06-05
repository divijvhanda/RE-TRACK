1 # Circle tracking code
2 from os import scandir
3 import cv2
4 import numpy as np
5 from matplotlib import pyplot as plt
6 #from scipy.spatial.transform import Rotation as R
7 import time
8 import serial
9 import json import numpy as np
 import math
from scipy.interpolate import interp1d, splprep, splev
import matplotlib.animation as animation
18 # INPUT = DIAMETER OF THE WRIST, which is the approximation of the real
distance (mm) between the two most separated LEDs.
# #################### Receive the data from the camera
################################################
ser = serial.Serial("COM3", timeout=9600, dsrdtr=False)
19 20
21 22 23 24 ########################## Auxiliar parameters
######################################################
# Parameters obtained from CAMERA CALIBRATION
# ##Focal length from CALIBRATION
fx_calibration = 548.297 #pixels (Distance from the camera LENS to the
camera SENSOR)
fy_calibration = 550.1771 #pixels
25 26
27 28 29 30
31 32 33 34 35
36 ##Define the PRINCIPAL POINTS (From the CALIBRATION)
cent_x = 366.23 #pixels
cent_y = 238.42 #pixels
#Camera matrix:
camera_matrix = np.array([[fx_calibration,0,cent_x],
[0,fy_calibration,cent_y],
[0,0,1]],dtype=np.float64)
37 38 39 40 ## ========== RADIAL and TANGENTIAL Distortions obtained from the Camera
CALIBRATION process ===========
##Radial distortions
k1=-0.40630288
k2=0.21387767
k3=-0.08096464

##Tangential distortions
p1=-0.00775763
p2=0.00225154
# Combine them into a single NumPy array
dist_coefficients = np.array([k1, k2, p1, p2, k3], dtype=np.float32)
#INPUT = wristband major diameter.
real_lenght = 50##55 #mm
# ============================== AUXILIAR FUNCTIONS
============================================
# Function to calculate the Euclidean distance between two points
def euclidean_distance(point1, point2):
#each point has 2 coordinates
return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1
[1]) ** 2)
def max_diametre_calculation(blob_centroids):
# INPUTS:
# - blob_centroids: a vector with the (x,y) position in the
IMAGE of the blobs detected
# OUTPUTS:
# - max_distance: max distance between any 2 blobs of the
circular constellation.
#Get the distance between the 2 farthest blobs.
if len(blob_centroids) < 2:
# Not enough points to calculate distance
return 0, []
max_distance=0
farthest_points =[]
for i in range(len(blob_centroids)): #Range over all the blobs
detected --> Get a first blob
for k in range(i + 1, len(blob_centroids)): #Get a second blob,
and calculate the distance between them.
distance = euclidean_distance(blob_centroids[i],
blob_centroids[k])
if distance > max_distance:
max_distance = distance
farthest_points = (blob_centroids[i], blob_centroids[k])
#These are the 2 farthest points
#print(f"The farthest distance is {max_distance} pixels between blobs
at {farthest_points[0]} and {farthest_points[1]}")
return max_distance, farthest_points
# ######################### PINHOLE CAMERA MODEL: Calculation
def calculate_XYZ_centroid(x0,y0, major_diameter):
"""
82 83
84 85 88 Calculate the XYZ of the centroid of the fitted ellipse
INPUTS:
86- x0,y0 = centroid of the ellipsoid
87- major_diameter of the ellipsoid measured in pixels
OUTPUTS:
89- X, Y and Z position of the centroid of the ellipse
"""
90
91 # ============== Calculation of the Z distance from the camera
=========
#Once the blobs have been identified in the particular frame (image),
get the line and the Z distance from the camera.
# - INPUT: blobs_centroids_pixels_ellipse --> position in pixels
of the blobs, from the FITTED ELLIPSE: as COLUMNS
# - OUTPUT: X, Y and Z position of the centroid of the ellipse
92 93 94 95
96 97 98 99 100 101 102 103
104 105 106 107 108
109 110 111 112 113
114 115 116
117 118 119 120 121
122 123 # ========= #### Calculate the position of the CENTRE of the circle
of that distance in X,Y,Z ========= ######
# #### Distance of the line from the camera --> Z axis (Space)
if major_diameter == 0:
# print("Wristband out of the camera field of view")
#Return 0 when the wristband is OUT of the camera field of view.
return 0,0,0
Z_distance = (fy_calibration * real_lenght)/(major_diameter)
# #### X position (of the centre of the line)
X_distance = ((x0-cent_x)/fx_calibration)* Z_distance
# #### Y position (of the centre of the line)
Y_distance = ((y0-cent_y)/fy_calibration)* Z_distance
XYZ_centroid_previous = (X_distance,Y_distance,Z_distance)
#print(Z_distance)
#print("Pos X:" + str(X_distance) + "Pos Y:" + str(Y_distance) +
"Pos Z:" + str(Z_distance))
return X_distance, Y_distance, Z_distance
# ####### Function for the distorsion corrections
##############################
def undistortFunction(src, camera_matrix, dist_coeffs, R=None, P=None):
""" This is the source code for implementing the cv function
undistortPoints
INPUTS: src (distorted centroid positions in the image)
camera_matrix (obtained from the calibration)
dist_coeffs (distorsion coefficients, obtained from the
calibration: RADIAL and TANGENTIAL coefficients)
RETURNS: the corrected centroids (position of each point in the
non-distorted image)
"""
A = np.zeros((3, 3), dtype=np.float64)
RR = np.eye(3, dtype=np.float64)

def calculate_XYZ_centroid(x0,y0, major_diameter):
"""
82 83
84 85 88 Calculate the XYZ of the centroid of the fitted ellipse
INPUTS:
86- x0,y0 = centroid of the ellipsoid
87- major_diameter of the ellipsoid measured in pixels
OUTPUTS:
89- X, Y and Z position of the centroid of the ellipse
"""
90
91 # ============== Calculation of the Z distance from the camera
=========
#Once the blobs have been identified in the particular frame (image),
get the line and the Z distance from the camera.
# - INPUT: blobs_centroids_pixels_ellipse --> position in pixels
of the blobs, from the FITTED ELLIPSE: as COLUMNS
# - OUTPUT: X, Y and Z position of the centroid of the ellipse
92 93 94 95
96 97 98 99 100 101 102 103
104 105 106 107 108
109 110 111 112 113
114 115 116
117 118 119 120 121
122 123 # ========= #### Calculate the position of the CENTRE of the circle
of that distance in X,Y,Z ========= ######
# #### Distance of the line from the camera --> Z axis (Space)
if major_diameter == 0:
# print("Wristband out of the camera field of view")
#Return 0 when the wristband is OUT of the camera field of view.
return 0,0,0
Z_distance = (fy_calibration * real_lenght)/(major_diameter)
# #### X position (of the centre of the line)
X_distance = ((x0-cent_x)/fx_calibration)* Z_distance
# #### Y position (of the centre of the line)
Y_distance = ((y0-cent_y)/fy_calibration)* Z_distance
XYZ_centroid_previous = (X_distance,Y_distance,Z_distance)
#print(Z_distance)
#print("Pos X:" + str(X_distance) + "Pos Y:" + str(Y_distance) +
"Pos Z:" + str(Z_distance))
return X_distance, Y_distance, Z_distance
# ####### Function for the distorsion corrections
##############################
def undistortFunction(src, camera_matrix, dist_coeffs, R=None, P=None):
""" This is the source code for implementing the cv function
undistortPoints
INPUTS: src (distorted centroid positions in the image)
camera_matrix (obtained from the calibration)
dist_coeffs (distorsion coefficients, obtained from the
calibration: RADIAL and TANGENTIAL coefficients)
RETURNS: the corrected centroids (position of each point in the
non-distorted image)
"""
A = np.zeros((3, 3), dtype=np.float64)
RR = np.eye(3, dtype=np.float64)

r2 = x * x + y * y
icdist = (1 + ((k[7] * r2 + k[6]) * r2 + k[5]) * r2) / (1 +
((k[4] * r2 + k[1]) * r2 + k[0]) * r2)
deltaX = 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x)
deltaY = k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y
x = (x0 - deltaX) * icdist
y = (y0 - deltaY) * icdist
xx = RR[0, 0] * x + RR[0, 1] * y + RR[0, 2]
yy = RR[1, 0] * x + RR[1, 1] * y + RR[1, 2]
ww = 1.0 / (RR[2, 0] * x + RR[2, 1] * y + RR[2, 2])
x, y = xx * ww, yy * ww #--> this are the NORMALISED COORDINATES
#Now convert those normalised coordinates into pixel coordinates
x = x * fx + cx
y = y * fy + cy
dst[i, 0], dst[i, 1] = x, y
corrected_centroids = dst
return corrected_centroids
# ======================================================== MAIN
FUNCTION
=================================================================
# Main function to use either fit_ellipse_scipython or
max_diametre_calculation
def calculate_max_diameter_and_centroid(led_positions):
""" Return the MAX DIAMETER and the CENTROID
"""
max_distance, farthest_points = max_diametre_calculation(
led_positions)
if len(farthest_points) < 2:
# Not enough points to determine the farthest points
print("Blobs␣not␣detected␣individually␣or␣wristband␣OUT␣of␣the␣
field␣of␣view")
return 0, (0, 0), 2
# print("farthest points: " + str(farthest_points))
# Approximate the centroid as the midpoint of the farthest points
x0 = (farthest_points[0][0] + farthest_points[1][0]) / 2
y0 = (farthest_points[0][1] + farthest_points[1][1]) / 2
return max_distance, (x0, y0)
# ####################################################### MAIN CODE
#######################
while True:
# # Variables for FPS calculation
# fps = 0
# frame_counter = 0
# start_time = time.time()
# Start the timer just before reading
start_time = time.time()
line = ser.readline().strip()
if line:
# Parse the JSON string into a Python dictionary
# Check if line is a valid JSON string
try:
# Attempt to decode the JSON data
data = json.loads(line.decode(’utf - 8’))
# Extract the data
blob_centroids = data["blob_centroids_camera"]
# Convert the list of points to a NumPy array
blob_centroids = np.array(blob_centroids, dtype=np.float32)
# print(blob_centroids)
except json.decoder.JSONDecodeError:
# Handle the case where JSON data is invalid
print("Received␣an␣invalid␣JSON␣string:", line.decode(’utf - 8’
))
continue
######################################
# ############################# Undistort the points
#########################################################
# First, correct the centroids
undistorted_centroids_coded = []  # store the corrected pixels coded
by
me(
from the cv2

source)
undistorted_centroids_cv2 = []  # It gives the same values as the
function
coded
by
me
non_corrected = []  # To store the non-corrected position of the
centroids
for centroid in blob_centroids:
# non_corrected.append(centroid)
# undistorted_centroids_cv2.append(cv2.undistortPoints(
centroid, camera_matrix, dist_coefficients, R = None, P = None))
# print(undistortFunction(centroid,camera_matrix,
dist_coefficients, R = None, P = None))  # This is the function I
have
defined
undistorted_centroids_coded.append(undistortFunction(centroid
                                                     , camera_matrix, dist_coefficients, R=None, P=None))
non_corrected.append(centroid)
# Convert the list of lists to a NumPy array of shape (N, 2)
corrected_centroids_cv2 = np.array(undistorted_centroids_cv2,
                                   dtype=np.float32)
corrected_centroids_cv2 = corrected_centroids_cv2.reshape(-1, 2)
corrected_centroids_coded = np.array(undistorted_centroids_coded,
dtype=np.float32)
corrected_centroids_coded = corrected_centroids_coded.reshape(-1,
2)
255
256 257 258 259
260 #Non-corrected centroids from the distortions.
non_corrected = np.array(non_corrected, dtype=np.float32)
non_corrected = non_corrected.reshape(-1, 2)
# Calculate the position of the centroid and its X,Y,Z position
in the 3D space with both REGRESSION AND NOT REGRESSION
corrected_centroids = corrected_centroids_coded
non_corrected_centroids = non_corrected
261 262 263
264 265 266 ######################################
######################### CALCULATE THE CENTROID OF THE CIRCLE
DEFINED with both REGRESSION and NO REGRESSION
############################################
# ==================== Also calculate the MAXIMUM diameter
================================================
267
268 269 #Try using the Halif and Flusser method for fitting the ellipse.
If it does not work, use the non-regression algorithm.
major_diameter, centroid= calculate_max_diameter_and_centroid(
corrected_centroids)
270
271 272 273
274 275 centroid_x= centroid[0]
centroid_y=centroid[1]
#Apply the pinhole model, for the max diameter and the centroid
obtained:
X_distance, Y_distance, Z_distance = calculate_XYZ_centroid(
centroid_x,centroid_y,major_diameter)
#stop the time:
end_time = time.time() #this calculation is done in SECONDS
276
277 278 279
280 281 # #Print the result
print(’X_distance:’+str(np.round(X_distance/10,2))+"
+ str(np.round(Y_distance/10,2))+"
␣Y_distance:"
␣Z_distance:"+ str(np.round(
Z_distance/10,2)))
282
283 284 285 286 #Print the time
frame_time= end_time-start_time
fps_computer= 1/frame_time
# print(fps_computer)