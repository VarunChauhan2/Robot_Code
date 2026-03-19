#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import os
from redline_detect import detect_green_tape_box, filter_mask_by_size

# Load photo_39.jpg
img = cv.imread('sample_images/photo_39.jpg')
frame = cv.resize(img, (854, 480))
frame_height, frame_width = frame.shape[:2]

# Prepare green mask
hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
green_lower = np.array([35, 75, 70])
green_upper = np.array([85, 255, 255])
mask_green = cv.inRange(hsv, green_lower, green_upper)

kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
mask_green = cv.morphologyEx(mask_green, cv.MORPH_CLOSE, kernel)
mask_green = cv.morphologyEx(mask_green, cv.MORPH_OPEN, kernel)
mask_green = filter_mask_by_size(mask_green, min_area=30)

print(f"Green mask pixels: {np.count_nonzero(mask_green)}")

# Prepare red mask
red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])
red_lower_2 = np.array([160, 100, 100])
red_upper_2 = np.array([180, 255, 255])

mask_red = cv.inRange(hsv, red_lower, red_upper)
mask_red |= cv.inRange(hsv, red_lower_2, red_upper_2)

kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
mask_red = cv.morphologyEx(mask_red, cv.MORPH_CLOSE, kernel)
mask_red = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)
mask_red = filter_mask_by_size(mask_red, min_area=200)

print(f"Red mask pixels: {np.count_nonzero(mask_red)}")

# Call detect_green_tape_box
print("\n=== Calling detect_green_tape_box ===")
green_detected, green_under_red = detect_green_tape_box(
    mask_green, mask_red, frame_height, frame_width, 
    output_dir='output', 
    base_filename='photo_39'
)
print(f"Result: green_detected={green_detected}, green_under_red={green_under_red}")

# Check if debug files were created
print("\n=== Checking for debug files ===")
for fname in ['02c_green_interior_debug_photo_39.jpg', '02c_interior_mask_photo_39.jpg', '02c_red_interior_region_photo_39.jpg']:
    fpath = os.path.join('output', fname)
    exists = os.path.exists(fpath)
    print(f"{fname}: {exists}")
