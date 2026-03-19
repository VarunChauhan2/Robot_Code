import cv2 as cv
import numpy as np
import os
import csv
from datetime import datetime
from redline_stream import (
    get_line_endpoints,
    calculate_line_angle,
    get_centerline_from_edges,
    get_x_on_line,
    get_horizontal_distance,
    find_tape_edges,
)

def point_to_line_distance(px, py, x1, y1, x2, y2):
    """Calculate perpendicular distance from point to line."""
    # Line equation: ax + by + c = 0
    # Distance = |ax + by + c| / sqrt(a^2 + b^2)
    
    if x1 == x2 and y1 == y2:
        # Degenerate line, return distance to point
        return np.sqrt((px - x1)**2 + (py - y1)**2)
    
    numerator = abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1)
    denominator = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    
    return numerator / denominator

def get_robust_centerline(mask, frame_height, frame_width, num_samples=20):
    """
    Extract centerline by sampling the tape width at multiple y-positions.
    More robust to uneven edges and fragmentation.
    
    Args:
        mask: Binary mask of detected red areas
        frame_height, frame_width: Frame dimensions
        num_samples: Number of y-positions to sample
    
    Returns:
        centerline_points: Array of [x, y] points along centerline, or None if insufficient data
    """
    centerline_points = []
    
    for y in range(0, frame_height, max(1, frame_height // num_samples)):
        row = mask[y, :]
        red_x = np.where(row > 0)[0]
        
        if len(red_x) > 0:
            x_min = red_x[0]
            x_max = red_x[-1]
            tape_width = x_max - x_min
            
            # Reject if tape is unreasonably wide (noise) or too narrow
            if 3 < tape_width < frame_width * 0.8:
                x_center = (x_min + x_max) / 2
                centerline_points.append([x_center, y])
    
    return np.array(centerline_points, dtype=np.float32) if centerline_points else None

def filter_mask_by_size(mask, min_area=200, max_area=None):
    """
    Filter mask to remove small noise regions and keep only significant areas.
    Keeps only contours with area >= min_area.
    
    Args:
        mask: Binary mask to filter
        min_area: Minimum contour area to keep (removes small noise)
        max_area: Maximum contour area to keep (optional, removes very large regions)
    
    Returns:
        filtered_mask: Binary mask with only kept contours
    """
    filtered_mask = np.zeros_like(mask)
    
    try:
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv.contourArea(contour)
            if area >= min_area:
                if max_area is None or area <= max_area:
                    cv.drawContours(filtered_mask, [contour], 0, 255, -1)
    except Exception as e:
        print(f"  Error filtering mask by size: {e}")
        return mask  # Return original if filtering fails
    
    return filtered_mask

def fit_centerline_curve(centerline_points):
    """
    Fit a polynomial curve through centerline points.
    Better for uneven/curved tape edges.
    
    Args:
        centerline_points: Array of [x, y] points along the centerline
    
    Returns:
        tuple: (poly, angle_at_mid) - polynomial fit and angle at middle point
    """
    if centerline_points is None or len(centerline_points) < 3:
        return None, None
    
    try:
        # Fit 2nd order polynomial (parabola) for smooth curves
        coeffs = np.polyfit(centerline_points[:, 1], centerline_points[:, 0], 2)
        poly = np.poly1d(coeffs)
        
        # Get angle at middle point (more representative than endpoints)
        y_mid = centerline_points[len(centerline_points)//2, 1]
        angle_at_mid = np.arctan(poly.deriv()(y_mid))
        
        return poly, np.degrees(angle_at_mid)
    except Exception as e:
        print(f"  Error fitting centerline curve: {e}")
        return None, None

def detect_bullseye(hsv, frame_height, frame_width, min_area=200):
    """
    Detect a bullseye pattern with blue outer ring, red center ring, and white separators/center.
    Uses the red center circle as the bullseye center (more accurate than blue outer ring).
    Accounts for camera angles by using a lenient circularity check.
    
    Args:
        hsv: HSV image to search for bullseye
        frame_height, frame_width: Frame dimensions
        min_area: Minimum area for bullseye detection
    
    Returns:
        tuple: (bullseye_detected, bullseye_centroid, bullseye_radius, blue_area)
            - bullseye_detected: Boolean, True if bullseye is detected
            - bullseye_centroid: (x, y) centroid of RED CENTER, None if not detected
            - bullseye_radius: Approximate radius of red center circle
            - blue_area: Area of blue ring detected
    """
    try:
        # Define HSV ranges for bullseye components
        # Blue outer ring - broader range to catch different shades of blue
        blue_lower = np.array([90, 80, 80])
        blue_upper = np.array([140, 255, 255])
        
        # Red center ring (overlaps with tape red detection)
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        red_lower_2 = np.array([160, 100, 100])
        red_upper_2 = np.array([180, 255, 255])
        
        # White center (low saturation, high value)
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([180, 50, 255])
        
        # Create masks for each component
        mask_blue = cv.inRange(hsv, blue_lower, blue_upper)
        mask_red = cv.inRange(hsv, red_lower, red_upper)
        mask_red |= cv.inRange(hsv, red_lower_2, red_upper_2)
        mask_white = cv.inRange(hsv, white_lower, white_upper)
        
        # Apply morphological operations to clean up masks
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (7, 7))
        mask_blue = cv.morphologyEx(mask_blue, cv.MORPH_CLOSE, kernel)  # Fill gaps in ring
        mask_blue = cv.morphologyEx(mask_blue, cv.MORPH_OPEN, kernel)   # Remove small noise
        
        mask_red = cv.morphologyEx(mask_red, cv.MORPH_CLOSE, kernel)    # Fill gaps in red ring
        mask_red = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)     # Remove small noise
        
        # Find blue contours (outer ring) for validation
        contours_blue, _ = cv.findContours(mask_blue, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if not contours_blue:
            return False, None, None, 0
        
        # Get the largest blue contour (should be the bullseye outer ring)
        largest_blue = max(contours_blue, key=cv.contourArea)
        blue_area = cv.contourArea(largest_blue)
        
        # Require minimum area for bullseye
        if blue_area < min_area * 2:  # Bullseye should be larger than single tape
            return False, None, None, blue_area
        
        # Fit a circle to the blue contour to get the region of interest
        (blue_center_x, blue_center_y), blue_radius = cv.minEnclosingCircle(largest_blue)
        
        # Verify the blue contour is roughly circular (bullseye has circular geometry)
        # Use lenient threshold (0.2-0.9) to account for camera angle perspective distortion
        circle_area = np.pi * blue_radius**2
        circularity = blue_area / circle_area if circle_area > 0 else 0
        
        if circularity < 0.2 or circularity > 0.95:
            # Not circular enough, probably not a bullseye ring
            return False, None, None, blue_area
        
        # Create a circular region of interest around the blue ring
        circle_roi = np.zeros_like(mask_red)
        cv.circle(circle_roi, (int(blue_center_x), int(blue_center_y)), int(blue_radius * 1.1), 255, -1)
        
        # Find red contours inside the blue ring (the red center circle)
        mask_red_in_bullseye = cv.bitwise_and(mask_red, circle_roi)
        contours_red, _ = cv.findContours(mask_red_in_bullseye, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if not contours_red:
            return False, None, None, blue_area
        
        # Get the largest red contour (should be the red center ring)
        largest_red = max(contours_red, key=cv.contourArea)
        red_area = cv.contourArea(largest_red)
        
        # Require minimum red area
        if red_area < min_area * 0.5:
            return False, None, None, blue_area
        
        # Fit a circle to the red contour to get the accurate center
        (red_center_x, red_center_y), red_radius = cv.minEnclosingCircle(largest_red)
        
        # Check if white is present inside the red ring (white center/separators)
        white_in_red = cv.bitwise_and(mask_white, circle_roi)
        white_area_in_circle = cv.countNonZero(white_in_red)
        
        # Verify white is present (should be in center and as separators)
        has_white = white_area_in_circle > (blue_area * 0.01)  # At least 1% white
        
        if not has_white:
            return False, None, None, blue_area
        
        # Detected as bullseye - return the RED CENTER (more accurate than blue outer ring)
        # Use the geometric circle center which is more accurate than centroid
        return True, (red_center_x, red_center_y), red_radius, blue_area
    
    except Exception as e:
        print(f"  Error detecting bullseye: {e}")
        return False, None, None, 0


def detect_green_tape_box(mask_green, mask_red, frame_height, frame_width, output_dir=None, base_filename=None):
    """
    Detect if there is a green tape box with red tape piercing through it (vs passing beside it).
    Checks if red pixels exist in the interior of the green box's bounding region.
    
    Args:
        mask_green: Binary mask of detected green areas
        mask_red: Binary mask of detected red areas
        frame_height, frame_width: Frame dimensions
        output_dir: Optional directory to save debug images
        base_filename: Optional base filename for debug images
    
    Returns:
        tuple: (green_detected, green_under_red)
            - green_detected: Boolean, True if green tape box is detected
            - green_under_red: Boolean, True if red pixels pass through the interior of the green box
    """
    try:
        # Find contours in green mask
        contours_green, _ = cv.findContours(mask_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours_green:
            print(f"  DEBUG: No green contours found in mask")
            return False, False
        
        # Check total green pixels instead of individual contour (handles fragmented masks)
        total_green_pixels = np.count_nonzero(mask_green)
        print(f"  DEBUG: Total green pixels in mask: {total_green_pixels}")
        
        # Require minimum total pixels for green box detection
        # Threshold of 50 pixels allows for fragmented small green boxes
        if total_green_pixels < 50:
            print(f"  DEBUG: Total green pixels too small: {total_green_pixels} < 50")
            return False, False
        
        # Combine all green contours to get the full bounding box (handles cases where red tape pierces through)
        # Concatenate all contours into a single array
        all_contours = np.vstack(contours_green)
        green_area = np.count_nonzero(mask_green)  # Use actual pixel count as area
        
        print(f"  DEBUG: Total green contours: {len(contours_green)}")
        print(f"  DEBUG: Combined green area: {green_area:.0f} pixels")
        print(f"  DEBUG: Green box area threshold met - proceeding with detection")
        
        # Get ROTATED bounding box of ALL green area (handles tilted boxes and fragmentation)
        rotated_rect = cv.minAreaRect(all_contours)
        box_corners = cv.boxPoints(rotated_rect)  # Get the 4 corners
        box_corners = np.asarray(box_corners, dtype=int)  # Convert to integers (np.int0 is deprecated)
        
        # Validate the bounding box fit
        # Calculate the bounding box area
        rotated_box_area = rotated_rect[1][0] * rotated_rect[1][1]
        green_pixels_ratio = green_area / rotated_box_area if rotated_box_area > 0 else 0
        
        # Check how many box corners are outside the frame
        corners_out_of_frame = 0
        for corner in box_corners:
            if corner[0] < 0 or corner[0] >= frame_width or corner[1] < 0 or corner[1] >= frame_height:
                corners_out_of_frame += 1
        
        # Get rotation angle
        angle = abs(rotated_rect[2])  # Get absolute rotation angle
        # Normalize angle to 0-90 degrees (since 90° and -90° are equivalent)
        if angle > 90:
            angle = 180 - angle
        
        print(f"  DEBUG: Rotated box area: {rotated_box_area:.0f}, pixel ratio: {green_pixels_ratio:.3f}, corners out of frame: {corners_out_of_frame}/4, angle: {angle:.1f}°")
        
        # If the ratio is too low (< 0.1), the box is way too large, likely due to out-of-frame edge pixels
        # OR if highly rotated (>30°) AND ratio is low-moderate (0.2-0.6), likely a poor fit
        # BUT: if we have multiple large contours, they likely represent fragmented pieces, so keep combined
        
        # Count significant contours (likely parts of the same box that got fragmented)
        significant_contours = sum(1 for c in contours_green if cv.contourArea(c) > 5000)
        
        print(f"  DEBUG: Significant contours (>5000px): {significant_contours}")
        
        use_largest_only = False
        if green_pixels_ratio < 0.1 and len(contours_green) > 1:
            print(f"  DEBUG: Bounding box fit is poor ({green_pixels_ratio:.1%}), using largest contour only")
            use_largest_only = True
        # Don't fall back to largest-only if we have multiple significant contours (red line piercing case)
        elif corners_out_of_frame >= 2 and green_pixels_ratio < 0.7 and len(contours_green) > 1 and significant_contours < 2:
            print(f"  DEBUG: Box partially out of frame ({corners_out_of_frame}/4 corners), using largest contour only")
            use_largest_only = True
        elif angle > 30 and 0.2 <= green_pixels_ratio < 0.6 and len(contours_green) > 1 and significant_contours < 2:
            print(f"  DEBUG: Highly rotated ({angle:.1f}°) with moderate fill ({green_pixels_ratio:.1%}), likely poor fit, using largest contour only")
            use_largest_only = True
            use_largest_only = True
        
        if use_largest_only:
            largest_green = max(contours_green, key=cv.contourArea)
            rotated_rect = cv.minAreaRect(largest_green)
            box_corners = cv.boxPoints(rotated_rect)
            box_corners = np.asarray(box_corners, dtype=int)
            rotated_box_area = rotated_rect[1][0] * rotated_rect[1][1]
            green_pixels_ratio = green_area / rotated_box_area if rotated_box_area > 0 else 0
        
        # If the pixel ratio is too low even after fallback, the fit is unreliable
        # Be more strict for single contours with poor ratios
        if len(contours_green) == 1 and angle > 45 and green_pixels_ratio < 0.4:
            print(f"  DEBUG: Single rotated contour with poor fit ({green_pixels_ratio:.1%}), rejecting detection")
            return False, False
        
        print(f"  DEBUG: Green box fit validated ({green_pixels_ratio:.1%})")
        contours_red, _ = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours_red:
            print(f"  DEBUG: No red contours found in mask (green detected but no red to overlap)")
            return True, False
        
        # Create a mask for the green box interior (where we'll look for red)
        green_box_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
        cv.drawContours(green_box_mask, [box_corners], 0, 255, -1)  # Fill the rotated box
        
        # Shrink the interior: erode the mask to get interior-only region
        interior_margin = max(3, int(np.sqrt(rotated_box_area) * 0.1))  # ~10% of box diagonal
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (interior_margin * 2 + 1, interior_margin * 2 + 1))
        green_interior_mask = cv.erode(green_box_mask, kernel, iterations=1)
        
        # Check if red pixels exist in the interior
        red_interior = cv.bitwise_and(mask_red, green_interior_mask)
        red_pixels_interior = np.count_nonzero(red_interior)
        
        print(f"  DEBUG: Red pixels found in green box interior: {red_pixels_interior}")
        
        green_under_red = red_pixels_interior >= 30
        
        # Save debug images if output_dir is provided
        if output_dir and base_filename:
            print(f"  DEBUG: Saving green box debug images...")
            try:
                # Create visualization showing green box bounds and interior search region
                debug_viz = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
                
                # Draw outer green box in green
                cv.drawContours(debug_viz, [box_corners], 0, (0, 255, 0), 2)
                
                # Overlay the interior search region in yellow - more efficient
                debug_viz[green_interior_mask > 0] = [0, 255, 255]
                
                # Overlay the red mask in red channel (only where red exists)
                debug_viz[:, :, 2] = cv.bitwise_or(debug_viz[:, :, 2], mask_red)
                
                # Add text annotations
                center_x, center_y = int(rotated_rect[0][0]), int(rotated_rect[0][1])
                angle = rotated_rect[2]
                cv.putText(debug_viz, f"Green Box Center: ({center_x},{center_y})", 
                          (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv.putText(debug_viz, f"Angle: {angle:.1f}deg, Box Area: {rotated_box_area:.0f}", 
                          (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv.putText(debug_viz, f"Green Pixels: {green_area:.0f}, Ratio: {green_pixels_ratio:.1%}", 
                          (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv.putText(debug_viz, f"Interior Margin: {interior_margin}px", 
                          (10, 120), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv.putText(debug_viz, f"Red Pixels in Interior: {red_pixels_interior}", 
                          (10, 150), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                cv.putText(debug_viz, f"Detection: {'UNDER RED' if green_under_red else 'NOT UNDER RED'}", 
                          (10, 180), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if green_under_red else (100, 255, 100), 2)
                
                debug_path = os.path.join(output_dir, f"02c_green_interior_debug_{base_filename}.jpg")
                success1 = cv.imwrite(debug_path, debug_viz)
                print(f"  Saved green interior debug: {debug_path} (success: {success1})")
                
                # Also save the interior region mask
                interior_path = os.path.join(output_dir, f"02c_interior_mask_{base_filename}.jpg")
                success2 = cv.imwrite(interior_path, green_interior_mask)
                print(f"  Saved interior mask: {interior_path} (success: {success2})")
                
                # Save red pixels found in interior
                red_interior_path = os.path.join(output_dir, f"02c_red_interior_region_{base_filename}.jpg")
                success3 = cv.imwrite(red_interior_path, red_interior)
                print(f"  Saved red interior region: {red_interior_path} (success: {success3})")
            except Exception as debug_err:
                print(f"  Error saving debug images: {debug_err}")
                import traceback
                traceback.print_exc()
        else:
            print(f"  DEBUG: output_dir or base_filename is None/empty - skipping debug image save")
            print(f"  DEBUG: output_dir={output_dir}, base_filename={base_filename}")
        
        return True, green_under_red
    
    except Exception as e:

        print(f"  Error detecting green tape box: {e}")
        return False, False



def detect_curved_turn_adaptive(frame, mask, adaptive_threshold=True, base_threshold=0.003):
    """
    Improved turn detection that adapts to tape width variations.
    Detects curved turns by analyzing the rate of change of the centerline's x-position.
    
    Args:
        frame: The full image frame
        mask: Red color mask of the frame
        adaptive_threshold: Whether to adapt threshold based on tape width
        base_threshold: Base curvature threshold (default 0.003)
    
    Returns:
        tuple: (is_turn, turn_type, curvature, centerline_pts, fit_coeffs)
            - is_turn: Boolean indicating if a turn was detected
            - turn_type: String describing turn type ('left_turn', 'right_turn', 'straight')
            - curvature: The rate of change metric
            - centerline_pts: Array of centerline points used for fitting (for visualization)
            - fit_coeffs: Polynomial coefficients or None
    """
    height, width = frame.shape[:2]
    centerline_points = []
    tape_widths = []
    
    # Sample the tape centerline at different heights
    for y in range(0, height, max(1, height // 20)):  # Sample ~20 points across height
        row = mask[y, :]
        red_x = np.where(row > 0)[0]
        
        if len(red_x) > 0:
            x_min, x_max = red_x[0], red_x[-1]
            tape_width = x_max - x_min
            tape_widths.append(tape_width)
            centerline_points.append([(x_min + x_max) / 2, y])
    
    if len(centerline_points) < 3:
        return False, 'straight', 0.0, None, None
    
    centerline_points = np.array(centerline_points, dtype=np.float32)
    tape_widths = np.array(tape_widths)
    
    try:
        # Calculate the rate of change of x position as we go down the centerline
        x_changes = np.diff(centerline_points[:, 0])
        curvature = np.var(x_changes)
        
        # Determine turn direction based on overall x-movement trend
        total_x_change = centerline_points[-1, 0] - centerline_points[0, 0]
        turn_direction = 'left_turn' if total_x_change > 0 else 'right_turn'
        
        # Adaptive threshold based on tape width variations
        threshold = base_threshold
        if adaptive_threshold and len(tape_widths) > 0:
            mean_width = np.mean(tape_widths)
            width_variation = np.std(tape_widths) / (mean_width + 1e-6)
            # Increase threshold if tape width varies significantly
            threshold = max(0.002, base_threshold + width_variation * 0.01)
        
        is_turn = curvature > threshold
        
        if not is_turn:
            return False, 'straight', curvature, centerline_points, None
        
        return True, turn_direction, curvature, centerline_points, None
    
    except Exception as e:
        print(f"  Error analyzing centerline curvature: {e}")
        return False, 'straight', 0.0, None, None


def on_turn_detected(turn_type, curvature):
    """
    Callback function called when a turn is detected.
    Modify this function to perform your desired action (e.g., motor control, logging, etc.)
    
    Args:
        turn_type: String indicating turn type ('left_turn' or 'right_turn')
        curvature: The curvature coefficient of the detected curve
    """
    print(f"  [TURN DETECTED] {turn_type} (curvature: {curvature:.6f})")
    # TODO: Add your turn handling logic here
    # Examples:
    # - Send turn command to motor controller
    # - Log turn event with high priority
    # - Trigger specific robot behavior
    pass

def process_batch(input_dir=None, output_dir=None, curvature_threshold=0.003):
    # If directories not specified, use paths relative to script location
    if input_dir is None:
        input_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sample_images")
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")

    print(f"Looking for input directory: {input_dir}")
    print(f"Output directory: {output_dir}")

    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Get all image files from input directory
    if not os.path.exists(input_dir):
        print(f"Input directory not found: {input_dir}")
        return
    
    image_files = [f for f in os.listdir(input_dir) 
                   if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]
    
    if not image_files:
        print(f"No image files found in {input_dir}")
        return
    
    print(f"Found {len(image_files)} image(s) to process")
    print(f"Curve detection threshold: {curvature_threshold} (lower = more sensitive)")
    
    # Open CSV file for logging
    csv_path = os.path.join(output_dir, "centerline_data.csv")
    csv_file = open(csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Image', 'Centerline_Angle (degrees)', 'Distance_From_Center (pixels)', 'Turn_Detected', 'Turn_Type', 'Curvature', 'Green_Box_Detected', 'Green_Under_Red', 'Bullseye_Detected', 'Bullseye_Center_X', 'Bullseye_Center_Y', 'Bullseye_Distance_X (pixels)', 'Bullseye_Distance_Y (pixels)', 'Timestamp'])
    
    for i, image_file in enumerate(image_files, 1):
        input_path = os.path.join(input_dir, image_file)
        base_filename = os.path.splitext(image_file)[0]
        
        print(f"[{i}/{len(image_files)}] Processing: {image_file}")
        
        # Load image
        img = cv.imread(input_path)
        if img is None:
            print(f"  Failed to load image: {input_path}")
            continue
        
        frame = cv.resize(img, (854, 480))
        frame_height, frame_width = frame.shape[:2]
        
        # ===== TURN DETECTION ON TOP 2/3 OF FRAME =====
        # Crop to top 2/3 of frame for turn detection
        crop_turn_bottom = 2 * frame_height // 3
        frame_turn = frame[:crop_turn_bottom, :]
        
        # Convert to HSV to detect red color
        hsv_full = cv.cvtColor(frame_turn, cv.COLOR_BGR2HSV)
        
        # Define HSV range for red color
        # Stricter ranges to avoid false positives on edges
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        red_lower_2 = np.array([160, 100, 100])
        red_upper_2 = np.array([180, 255, 255])
        
        # Create masks for red ranges on cropped frame
        mask1_full = cv.inRange(hsv_full, red_lower, red_upper)
        mask2_full = cv.inRange(hsv_full, red_lower_2, red_upper_2)
        mask_full = cv.bitwise_or(mask1_full, mask2_full)
        
        # Apply morphological operations to fill small gaps and remove noise
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask_full = cv.morphologyEx(mask_full, cv.MORPH_CLOSE, kernel)  # Fill gaps
        mask_full = cv.morphologyEx(mask_full, cv.MORPH_OPEN, kernel)   # Remove noise
        
        # Filter out small noise regions, keep only significant contours
        mask_full = filter_mask_by_size(mask_full, min_area=200)
        
        # Save turn detection intermediate photos
        turn_cropped_path = os.path.join(output_dir, f"00_turn_cropped_{base_filename}.jpg")
        cv.imwrite(turn_cropped_path, frame_turn)
        print(f"  Saved turn cropped: {turn_cropped_path}")
        
        turn_mask_path = os.path.join(output_dir, f"00_turn_mask_{base_filename}.jpg")
        cv.imwrite(turn_mask_path, mask_full)
        print(f"  Saved turn mask: {turn_mask_path}")
        
        # Detect edges and lines on cropped frame
        red_regions_full = cv.bitwise_and(frame_turn, frame_turn, mask=mask_full)
        gray_full = cv.cvtColor(red_regions_full, cv.COLOR_BGR2GRAY)
        edges_full = cv.Canny(gray_full, 50, 150)
        
        turn_edges_path = os.path.join(output_dir, f"00_turn_edges_{base_filename}.jpg")
        cv.imwrite(turn_edges_path, edges_full)
        print(f"  Saved turn edges: {turn_edges_path}")
        
        lines_full = cv.HoughLinesP(edges_full, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        
        # Detect curved turn using adaptive threshold method
        is_turn, turn_type, curvature, fit_points, fit_coeffs = detect_curved_turn_adaptive(frame_turn, mask_full, adaptive_threshold=True, base_threshold=curvature_threshold)
        
        # Save turn detection visualization
        turn_viz = frame_turn.copy()
        if fit_points is not None and len(fit_points) > 0:
            # Draw the centerline points sampled from the tape
            for pt in fit_points:
                cv.circle(turn_viz, (int(pt[0]), int(pt[1])), 4, (200, 200, 0), -1)  # Cyan points
            
            # Draw lines connecting the sampled centerline points
            for j in range(len(fit_points) - 1):
                pt1 = (int(fit_points[j, 0]), int(fit_points[j, 1]))
                pt2 = (int(fit_points[j + 1, 0]), int(fit_points[j + 1, 1]))
                cv.line(turn_viz, pt1, pt2, (255, 0, 255), 2)  # Magenta line
            
            # Add curvature value to image
            cv.putText(turn_viz, f"Curvature: {curvature:.4f}", 
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 0), 2)
        
        if is_turn:
            cv.putText(turn_viz, f"TURN: {turn_type.upper()}", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv.putText(turn_viz, "STRAIGHT", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        turn_viz_path = os.path.join(output_dir, f"00_turn_detected_{base_filename}.jpg")
        cv.imwrite(turn_viz_path, turn_viz)
        print(f"  Saved turn detected: {turn_viz_path}")
        
        if is_turn:
            on_turn_detected(turn_type, curvature)
        
        # ===== CENTERLINE ANALYSIS ON CROPPED CENTER 1/3 =====
        # Crop to center 1/3 vertically (remove top and bottom 1/3)
        crop_top = frame_height // 3
        crop_bottom = 2 * frame_height // 3
        frame_cropped = frame[crop_top:crop_bottom, :]
        
        # Save cropped image
        cropped_path = os.path.join(output_dir, f"01_cropped_{base_filename}.jpg")
        cv.imwrite(cropped_path, frame_cropped)
        print(f"  Saved cropped: {cropped_path}")
        
        hsv = cv.cvtColor(frame_cropped, cv.COLOR_BGR2HSV)
        
        # create masks for red ranges (same stricter ranges)
        mask1 = cv.inRange(hsv, red_lower, red_upper)
        mask2 = cv.inRange(hsv, red_lower_2, red_upper_2)
        mask = cv.bitwise_or(mask1, mask2)
        
        # Apply morphological operations to fill gaps and improve robustness
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)  # Fill small gaps in tape
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)   # Remove small noise
        
        # Filter out small noise regions, keep only significant contours
        mask = filter_mask_by_size(mask, min_area=200)
        
        # ===== BULLSEYE DETECTION ON FULL FRAME =====
        # Detect bullseye on the full image frame (not cropped)
        hsv_full_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # ===== GREEN TAPE BOX DETECTION ON FULL FRAME =====
        # Detect green tape box using the entire frame
        # Define HSV range for light green color (stricter saturation to avoid background noise)
        green_lower = np.array([35, 75, 70])    # Higher saturation to filter grayish background objects
        green_upper = np.array([85, 255, 255])
        
        # Create mask for green color on full frame
        mask_green_full = cv.inRange(hsv_full_frame, green_lower, green_upper)
        
        # Apply morphological operations to improve green detection
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask_green_full = cv.morphologyEx(mask_green_full, cv.MORPH_CLOSE, kernel)  # Fill gaps
        mask_green_full = cv.morphologyEx(mask_green_full, cv.MORPH_OPEN, kernel)   # Remove noise
        
        # DEBUG: Check green mask before filtering
        green_pixels_before = np.count_nonzero(mask_green_full)
        print(f"  DEBUG: Green mask before filtering - pixels: {green_pixels_before}")
        
        # DEBUG: Check contours in green mask before filtering
        contours_before_filter, _ = cv.findContours(mask_green_full, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        print(f"  DEBUG: Green contours before filtering - count: {len(contours_before_filter)}")
        if contours_before_filter:
            areas = [cv.contourArea(c) for c in contours_before_filter]
            print(f"  DEBUG: Contour areas - min: {min(areas):.0f}, max: {max(areas):.0f}, mean: {np.mean(areas):.0f}")
        
        # Filter out small noise regions from green mask
        # Use very low threshold (30) to avoid filtering out fragmented green boxes
        mask_green_full = filter_mask_by_size(mask_green_full, min_area=30)
        
        # DEBUG: Check green mask statistics
        green_pixels = np.count_nonzero(mask_green_full)
        print(f"  DEBUG: Green mask statistics - total pixels: {green_pixels}, frame size: {frame_height}x{frame_width}")
        if green_pixels > 0:
            print(f"  DEBUG: Green mask coverage: {100.0 * green_pixels / (frame_height * frame_width):.2f}%")
        
        # Create red mask on full frame for green-red overlap detection
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        red_lower_2 = np.array([160, 100, 100])
        red_upper_2 = np.array([180, 255, 255])
        
        mask_red_full = cv.inRange(hsv_full_frame, red_lower, red_upper)
        mask_red_full |= cv.inRange(hsv_full_frame, red_lower_2, red_upper_2)
        
        # Apply morphological operations to red mask
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask_red_full = cv.morphologyEx(mask_red_full, cv.MORPH_CLOSE, kernel)
        mask_red_full = cv.morphologyEx(mask_red_full, cv.MORPH_OPEN, kernel)
        mask_red_full = filter_mask_by_size(mask_red_full, min_area=200)
        
        # Detect green tape box and check for intersection with red tape on full frame
        green_detected_local, green_under_red = detect_green_tape_box(
            mask_green_full, mask_red_full, frame_height, frame_width, output_dir, base_filename
        )
        
        # Save green mask for debugging
        green_mask_path = os.path.join(output_dir, f"02b_green_mask_{base_filename}.jpg")
        cv.imwrite(green_mask_path, mask_green_full)
        print(f"  Saved green mask: {green_mask_path}")
        
        if green_detected_local:
            if green_under_red:
                print(f"  [GREEN BOX] DETECTED UNDER RED TAPE")
            else:
                print(f"  [GREEN BOX] Detected but NOT under red tape (false positive avoided)")
        
        # ===== BULLSEYE DETECTION CONTINUES =====
        bullseye_detected, bullseye_centroid, bullseye_radius, bullseye_blue_area = detect_bullseye(
            hsv_full_frame, frame_height, frame_width, min_area=200
        )
        
        bullseye_distance_x = None
        bullseye_distance_y = None
        if bullseye_detected and bullseye_centroid is not None:
            # Calculate X and Y distances from camera center to bullseye center (using full frame dimensions)
            camera_center_x = frame_width / 2
            camera_center_y = frame_height / 2
            
            bullseye_distance_x = bullseye_centroid[0] - camera_center_x
            bullseye_distance_y = bullseye_centroid[1] - camera_center_y
            
            print(f"  [BULLSEYE] DETECTED at position {bullseye_centroid} (radius: {bullseye_radius:.1f})")
            print(f"  [BULLSEYE] Distance from camera center - X: {bullseye_distance_x:.2f} px, Y: {bullseye_distance_y:.2f} px")
        
        # Save red mask
        mask_path = os.path.join(output_dir, f"02_mask_{base_filename}.jpg")
        cv.imwrite(mask_path, mask)
        print(f"  Saved mask: {mask_path}")
        
        red_regions = cv.bitwise_and(frame_cropped, frame_cropped, mask=mask)
        gray = cv.cvtColor(red_regions, cv.COLOR_BGR2GRAY)
        edges = cv.Canny(gray, 50, 150)
        
        # Save edges
        edges_path = os.path.join(output_dir, f"03_edges_{base_filename}.jpg")
        cv.imwrite(edges_path, edges)
        print(f"  Saved edges: {edges_path}")
        
        lines = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        
        # Use full uncropped frame for visualization output
        out = frame.copy()
        centerline_angle = None
        centerline_distance = None
        
        # Use robust centerline sampling method (more reliable than edge detection)
        centerline_pts = get_robust_centerline(mask, frame_cropped.shape[0], frame_cropped.shape[1], num_samples=20)
        
        if centerline_pts is not None and len(centerline_pts) >= 3:
            print(f"  Red line detected - robust centerline extracted from {len(centerline_pts)} sample points")
            
            # Fit polynomial curve through centerline points
            poly, centerline_angle = fit_centerline_curve(centerline_pts)
            
            if poly is not None and centerline_angle is not None:
                # Draw sampled centerline points
                for pt in centerline_pts:
                    cv.circle(out, (int(pt[0]), int(pt[1] + crop_top)), 4, (200, 200, 0), -1)
                
                # Draw smooth curve through points
                y_vals = np.linspace(centerline_pts[0, 1], centerline_pts[-1, 1], 50)
                x_vals = poly(y_vals)
                curve_points = np.array([[int(x), int(y + crop_top)] for x, y in zip(x_vals, y_vals)])
                cv.polylines(out, [curve_points], False, (0, 255, 0), 3)
                
                # Calculate endpoints for compatibility
                cx1, cy1 = int(centerline_pts[0, 0]), int(centerline_pts[0, 1])
                cx2, cy2 = int(centerline_pts[-1, 0]), int(centerline_pts[-1, 1])
                
                # Calculate horizontal distance from image center to centerline
                img_height, img_width = frame_cropped.shape[:2]
                center_x = img_width / 2
                center_y = img_height / 2
                
                # Use polynomial for distance calculation
                centerline_x_at_center = int(poly(center_y))
                centerline_distance = centerline_x_at_center - center_x
                
                # Draw a circle at image center in the cropped region (offset to full frame)
                cv.circle(out, (int(center_x), int(center_y + crop_top)), 5, (0, 0, 255), -1)
                
                # Draw a line from image center perpendicular to the centerline (offset to full frame)
                cv.line(out, (int(center_x), int(center_y + crop_top)), (centerline_x_at_center, int(center_y + crop_top)), (255, 255, 0), 2)
                
                # Add text on full frame with padding from edge
                distance_text = f"H-Dist: {centerline_distance:.2f}px"
                angle_text = f"Angle: {centerline_angle:.2f}deg"
                cv.putText(out, distance_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv.putText(out, angle_text, (10, 65), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Add turn detection status
                if is_turn:
                    cv.putText(out, f"TURN: {turn_type.upper()}", (10, 100), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    cv.putText(out, "STRAIGHT", (10, 100), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Add green box detection status
                if green_detected_local:
                    if green_under_red:
                        cv.putText(out, "GREEN BOX: UNDER RED", (10, 135), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cv.putText(out, "GREEN BOX: NOT UNDER RED", (10, 135), cv.FONT_HERSHEY_SIMPLEX, 0.7, (100, 255, 100), 2)
                
                # Add bullseye detection status and distance visualization
                if bullseye_detected and bullseye_centroid is not None:
                    # Draw bullseye centroid on full frame
                    bullseye_x_full = int(bullseye_centroid[0])
                    bullseye_y_full = int(bullseye_centroid[1])
                    cv.circle(out, (bullseye_x_full, bullseye_y_full), 12, (255, 0, 255), 3)  # Magenta circle
                    
                    # Draw line from camera center to bullseye center
                    camera_center_x_full = int(frame_width / 2)
                    camera_center_y_full = int(frame_height / 2)
                    cv.line(out, (camera_center_x_full, camera_center_y_full), 
                           (bullseye_x_full, bullseye_y_full), (255, 0, 255), 2)
                    
                    # Draw crosshair at camera center
                    cv.circle(out, (camera_center_x_full, camera_center_y_full), 8, (255, 0, 255), 2)
                    cv.line(out, (camera_center_x_full - 15, camera_center_y_full), 
                           (camera_center_x_full + 15, camera_center_y_full), (255, 0, 255), 1)
                    cv.line(out, (camera_center_x_full, camera_center_y_full - 15), 
                           (camera_center_x_full, camera_center_y_full + 15), (255, 0, 255), 1)
                    
                    # Add bullseye detection text
                    cv.putText(out, "BULLSEYE DETECTED", (10, 170), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                    distance_x_text = f"X Distance: {bullseye_distance_x:.2f}px"
                    distance_y_text = f"Y Distance: {bullseye_distance_y:.2f}px"
                    cv.putText(out, distance_x_text, (10, 205), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                    cv.putText(out, distance_y_text, (10, 240), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                
                print(f"  Centerline angle: {centerline_angle:.2f}deg")
                print(f"  Horizontal distance from center: {centerline_distance:.2f} pixels")
                if is_turn:
                    print(f"  [CURVED TURN] {turn_type} (curvature: {curvature:.6f})")
            else:
                print(f"  Centerline extracted but fitting failed")
        else:
            print(f"  No red line detected - insufficient centerline data")
        
        # ===== VISUALIZE TURN DETECTION FOR DEBUGGING =====
        # (Turn detection intermediate photos already saved above)
        
        # Save final result
        output_path = os.path.join(output_dir, f"04_detected_{base_filename}.jpg")
        cv.imwrite(output_path, out)
        print(f"  Saved detected: {output_path}")
        
        # Log to CSV
        timestamp = datetime.now().isoformat()
        if centerline_angle is not None and centerline_distance is not None:
            csv_writer.writerow([image_file, f"{centerline_angle:.2f}", f"{centerline_distance:.2f}", 
                               is_turn, turn_type, f"{curvature:.6f}", green_detected_local, green_under_red, 
                               bullseye_detected, f"{bullseye_centroid[0]:.2f}" if bullseye_centroid else "N/A", 
                               f"{bullseye_centroid[1]:.2f}" if bullseye_centroid else "N/A",
                               f"{bullseye_distance_x:.2f}" if bullseye_distance_x is not None else "N/A",
                               f"{bullseye_distance_y:.2f}" if bullseye_distance_y is not None else "N/A", timestamp])
        else:
            csv_writer.writerow([image_file, "N/A", "N/A", "N/A", "N/A", "N/A", green_detected_local, green_under_red, 
                               bullseye_detected, f"{bullseye_centroid[0]:.2f}" if bullseye_centroid else "N/A",
                               f"{bullseye_centroid[1]:.2f}" if bullseye_centroid else "N/A",
                               f"{bullseye_distance_x:.2f}" if bullseye_distance_x is not None else "N/A",
                               f"{bullseye_distance_y:.2f}" if bullseye_distance_y is not None else "N/A", timestamp])
    
    csv_file.close()
    print(f"\nBatch processing complete! Results saved to {output_dir}")
    print(f"Centerline data logged to {csv_path}")


if __name__ == "__main__":
    # You can adjust the curvature_threshold parameter:
    # - Lower values (e.g., 0.001): More sensitive, detects slight curves
    # - Higher values (e.g., 0.005): Less sensitive, only sharp turns detected
    # Default is 0.003
    
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_dir = os.path.join(script_dir, "sample_images")
    output_dir = os.path.join(script_dir, "output")
    
    process_batch(input_dir, output_dir, curvature_threshold=100)