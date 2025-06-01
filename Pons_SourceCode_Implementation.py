# OpenMV Image capture and Blob detection
# Sends the distorted blob coordinates to the computer via USB 3.0

from ulab import numpy as np
import math
import sensor, image, time
import json

## =========== SETTING UP USB PORT FOR SENDING INFORMATION

## =========== PARAMETERS FOR ISOLATING THE IR LEDs
EXPOSURE_MICROSECONDS = 5000  # Low exposure to get the frame rate to be very high.
TRACKING_THRESHOLDS = [(160, 255)]  # When you lower the exposure you darken everything.
SEARCHING_RESOLUTION = sensor.VGA  # Set the frame size for the camera module: 640x480
SEARCHING_AREA_THRESHOLD = 16  # Filters blobs smaller than threshold
SEARCHING_PIXEL_THRESHOLD = SEARCHING_AREA_THRESHOLD  # Pixels that must meet tracking thresholds

# Alternate resolution/parameters (optional)
TRACKING_RESOLUTION = sensor.VGA  # 320x240
# sensor.QQVGA  # 160x120
TRACKING_AREA_THRESHOLD = 10  # We ensure only significant blobs are detected
TRACKING_PIXEL_THRESHOLD = TRACKING_AREA_THRESHOLD

MAX_AREA_THRESHOLD = 2500  # Filter blobs too large (e.g., ambient light)

## =========== INITIALIZE THE SENSOR
sensor.reset()  # Reset and initialize the sensor
sensor.set_pixformat(sensor.GRAYSCALE)  # Set pixel format to GRAYSCALE
sensor.set_framesize(SEARCHING_RESOLUTION)
sensor.skip_frames(time=2000)  # Skip images during 2000 ms

# Turn off Auto_gain and auto_exposure
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_MICROSECONDS)
# sensor.skip_frames(time=10000)

# =========== AUXILIAR FUNCTIONS
 # Blob detection and filtering function
def filter_blob(blob):
    """
    Filters out blobs too large or too small to be IR LEDs.
    Returns True if blob is valid.
    """
    if SEARCHING_AREA_THRESHOLD <= blob.area() <= MAX_AREA_THRESHOLD:
        return True
    else:
        return False

# =========== MAIN LOOP
clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()  # Capture image

    # Switch to SEARCHING resolution (initial detection)
    sensor.set_framesize(SEARCHING_RESOLUTION)

    # =========== REAL-TIME TRACKING
    clock = time.clock()  # Track FPS
    # blob_detection_position_OpenMV.py 2
    while(True):
        clock.tick()
        img = sensor.snapshot()

        # ==== Find IR Blobs and MAX DISTANCE between them ====
        blob_centroids = []  # Store blob centroids
        i = 1

        for blob in img.find_blobs(TRACKING_THRESHOLDS, area_threshold=SEARCHING_AREA_THRESHOLD,
                                   pixels_threshold=SEARCHING_PIXEL_THRESHOLD):
            # Filter blob by area
            is_a_blob = filter_blob(blob)

            if is_a_blob == True:
                # Draw rectangle and cross on blob
                img.draw_rectangle(blob.rect())
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_string(blob.cx(), blob.cy(), "Led" + str(i), color=(255, 0, 0))

                # Save centroid
                blob_centroids.append((blob.cx(), blob.cy()))
                i = i + 1

        # =========== DATA TO SEND ===========
        # Send the LED blobs to the python script
        data = {
            "blob_centroids_camera": blob_centroids
        }
        print("sending" + json.dumps(data))


        time.sleep_ms(100)
