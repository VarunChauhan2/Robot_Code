import cv2 as cv
import numpy as np
import time
from datetime import datetime

# I2C configuration
I2C_ADDR = 0x8  # Arduino address
I2C_BUS = 1     # /dev/i2c-1

try:
    from smbus2 import SMBus
    bus = SMBus(I2C_BUS)
except ImportError:
    print(f"Warning: smbus2 module not available (OK if not on Raspberry Pi)")
    bus = None
except Exception as e:
    print(f"Warning: Could not initialize I2C bus: {e}")
    bus = None


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


def detect_green_tape_box(mask_green, mask_red, frame_height, frame_width):
    """
    Detect if there is a green tape box with red tape piercing through it (vs passing beside it).
    Checks if red pixels exist in the interior of the green box's bounding region.
    Optimized for real-time performance.
    
    Args:
        mask_green: Binary mask of detected green areas
        mask_red: Binary mask of detected red areas
        frame_height, frame_width: Frame dimensions
    
    Returns:
        tuple: (green_detected, green_under_red)
            - green_detected: Boolean, True if green tape box is detected
            - green_under_red: Boolean, True if red pixels pass through the interior of the green box
    """
    try:
        # Find contours in green mask
        contours_green, _ = cv.findContours(mask_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours_green:
            return False, False
        
        # Check total green pixels instead of individual contour (handles fragmented masks)
        total_green_pixels = np.count_nonzero(mask_green)
        
        # Require minimum total pixels for green box detection
        # Threshold of 50 pixels allows for fragmented small green boxes
        if total_green_pixels < 50:
            return False, False
        
        # Combine all green contours to get the full bounding box
        all_contours = np.vstack(contours_green)
        green_area = np.count_nonzero(mask_green)
        
        # Get ROTATED bounding box of ALL green area (handles tilted boxes and fragmentation)
        rotated_rect = cv.minAreaRect(all_contours)
        box_corners = cv.boxPoints(rotated_rect)
        box_corners = np.asarray(box_corners, dtype=int)
        
        # Validate the bounding box fit
        rotated_box_area = rotated_rect[1][0] * rotated_rect[1][1]
        green_pixels_ratio = green_area / rotated_box_area if rotated_box_area > 0 else 0
        
        # Check how many box corners are outside the frame
        corners_out_of_frame = 0
        for corner in box_corners:
            if corner[0] < 0 or corner[0] >= frame_width or corner[1] < 0 or corner[1] >= frame_height:
                corners_out_of_frame += 1
        
        # Get rotation angle
        angle = abs(rotated_rect[2])
        if angle > 90:
            angle = 180 - angle
        
        # Use largest contour only if fit is poor
        use_largest_only = False
        if green_pixels_ratio < 0.1 and len(contours_green) > 1:
            use_largest_only = True
        elif corners_out_of_frame >= 2 and green_pixels_ratio < 0.7 and len(contours_green) > 1:
            use_largest_only = True
        # Removed: angle > 30 and ratio 0.2-0.6 condition
        # This was causing false negatives when red tape pierces through fragmented green boxes
        # Better to use combined approach and rely on interior margin to validate detection
        
        if use_largest_only:
            largest_green = max(contours_green, key=cv.contourArea)
            rotated_rect = cv.minAreaRect(largest_green)
            box_corners = cv.boxPoints(rotated_rect)
            box_corners = np.asarray(box_corners, dtype=int)
            rotated_box_area = rotated_rect[1][0] * rotated_rect[1][1]
            green_pixels_ratio = green_area / rotated_box_area if rotated_box_area > 0 else 0
        
        # If the pixel ratio is too low, the fit is unreliable
        if len(contours_green) == 1 and angle > 45 and green_pixels_ratio < 0.4:
            return False, False
        
        # Check if red exists in the interior
        contours_red, _ = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours_red:
            return True, False
        
        # Create a mask for the green box interior
        green_box_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
        cv.drawContours(green_box_mask, [box_corners], 0, 255, -1)
        
        # Shrink the interior: erode the mask to get interior-only region
        # Use a smaller margin - just enough to avoid the edges of the box (5% of smallest box dimension)
        box_width = rotated_rect[1][0]
        box_height = rotated_rect[1][1]
        min_box_dim = min(box_width, box_height)
        interior_margin = max(2, int(min_box_dim * 0.05))
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (interior_margin * 2 + 1, interior_margin * 2 + 1))
        green_interior_mask = cv.erode(green_box_mask, kernel, iterations=1)
        
        # Check if red pixels exist in the interior
        red_interior = cv.bitwise_and(mask_red, green_interior_mask)
        red_pixels_interior = np.count_nonzero(red_interior)
        
        # Lower threshold to 10 pixels - more reliable for detecting red under green
        green_under_red = red_pixels_interior >= 10
        
        return True, green_under_red
    
    except Exception as e:
        print(f"  Error detecting green tape box: {e}")
        return False, False


def detect_bullseye(hsv, frame_height, frame_width, min_area=200):
    """
    Detect a bullseye pattern with blue outer ring, red center ring, and white separators/center.
    Uses the red center circle as the bullseye center.
    Optimized for real-time performance.
    
    Args:
        hsv: HSV image to search for bullseye
        frame_height, frame_width: Frame dimensions
        min_area: Minimum area for bullseye detection
    
    Returns:
        tuple: (bullseye_detected, bullseye_centroid, bullseye_radius)
            - bullseye_detected: Boolean, True if bullseye is detected
            - bullseye_centroid: (x, y) centroid of RED CENTER, None if not detected
            - bullseye_radius: Approximate radius of red center circle
    """
    try:
        # Define HSV ranges for bullseye components
        blue_lower = np.array([90, 80, 80])
        blue_upper = np.array([140, 255, 255])
        
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        red_lower_2 = np.array([160, 100, 100])
        red_upper_2 = np.array([180, 255, 255])
        
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([180, 50, 255])
        
        # Create masks for each component
        mask_blue = cv.inRange(hsv, blue_lower, blue_upper)
        mask_red = cv.inRange(hsv, red_lower, red_upper)
        mask_red |= cv.inRange(hsv, red_lower_2, red_upper_2)
        mask_white = cv.inRange(hsv, white_lower, white_upper)
        
        # Apply morphological operations to clean up masks
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (7, 7))
        mask_blue = cv.morphologyEx(mask_blue, cv.MORPH_CLOSE, kernel)
        mask_blue = cv.morphologyEx(mask_blue, cv.MORPH_OPEN, kernel)
        
        mask_red = cv.morphologyEx(mask_red, cv.MORPH_CLOSE, kernel)
        mask_red = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)
        
        # Find blue contours (outer ring) for validation
        contours_blue, _ = cv.findContours(mask_blue, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if not contours_blue:
            return False, None, None
        
        # Get the largest blue contour
        largest_blue = max(contours_blue, key=cv.contourArea)
        blue_area = cv.contourArea(largest_blue)
        
        # Require minimum area for bullseye
        if blue_area < min_area * 2:
            return False, None, None
        
        # Fit a circle to the blue contour
        (blue_center_x, blue_center_y), blue_radius = cv.minEnclosingCircle(largest_blue)
        
        # Verify the blue contour is roughly circular
        circle_area = np.pi * blue_radius**2
        circularity = blue_area / circle_area if circle_area > 0 else 0
        
        if circularity < 0.2 or circularity > 0.95:
            return False, None, None
        
        # Create a circular region of interest around the blue ring
        circle_roi = np.zeros_like(mask_red)
        cv.circle(circle_roi, (int(blue_center_x), int(blue_center_y)), int(blue_radius * 1.1), 255, -1)
        
        # Find red contours inside the blue ring
        mask_red_in_bullseye = cv.bitwise_and(mask_red, circle_roi)
        contours_red, _ = cv.findContours(mask_red_in_bullseye, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if not contours_red:
            return False, None, None
        
        # Get the largest red contour
        largest_red = max(contours_red, key=cv.contourArea)
        red_area = cv.contourArea(largest_red)
        
        # Require minimum red area
        if red_area < min_area * 0.5:
            return False, None, None
        
        # Fit a circle to the red contour to get the accurate center
        (red_center_x, red_center_y), red_radius = cv.minEnclosingCircle(largest_red)
        
        # Check if white is present inside the red ring
        white_in_red = cv.bitwise_and(mask_white, circle_roi)
        white_area_in_circle = cv.countNonZero(white_in_red)
        
        # Verify white is present
        has_white = white_area_in_circle > (blue_area * 0.01)
        
        if not has_white:
            return False, None, None
        
        # Detected as bullseye
        return True, (red_center_x, red_center_y), red_radius
    
    except Exception as e:
        print(f"  Error detecting bullseye: {e}")
        return False, None, None


def on_turn_detected(turn_type, curvature):
    """
    Callback function called when a turn is detected.
    
    Args:
        turn_type: String indicating turn type ('left_turn' or 'right_turn')
        curvature: The curvature coefficient of the detected curve
    """
    print(f"  ⚠️  TURN DETECTED: {turn_type} (curvature: {curvature:.6f})")


def send_i2c_data(centerline_distance, is_turn=False, turn_type='straight', bullseye_distance_x=None, bullseye_distance_y=None, green_detected=False, green_under_red=False):
    """Send detection data over I2C bus to Arduino.
    
    Args:
        centerline_distance: Horizontal distance to centerline
        is_turn: Boolean indicating if a turn was detected
        turn_type: Type of turn ('straight', 'left_turn', 'right_turn')
        bullseye_distance_x: X offset for bullseye (pixels from center)
        bullseye_distance_y: Y offset for bullseye (pixels from center)
        green_detected: Boolean indicating if green box is detected
        green_under_red: Boolean indicating if red line passes under green box
    """
    if bus is None:
        return
    
    try:
        # Priority: send bullseye data if detected, then green box, then centerline data
        if bullseye_distance_x is not None and bullseye_distance_y is not None:
            # Mode 4: Bullseye detection
            # Convert float distances to integers, clamped to signed byte range (-128 to 127)
            x_offset = int(np.clip(bullseye_distance_x, -128, 127))
            y_offset = int(np.clip(bullseye_distance_y, -128, 127))
            
            data = [4, x_offset, y_offset]
            bus.write_i2c_block_data(I2C_ADDR, 0, data)
        elif green_detected and green_under_red:
            # Mode 5: Green box detection
            # Only send when red passes UNDER the green box
            data = [5]
            bus.write_i2c_block_data(I2C_ADDR, 0, data)
        else:
            # Determine mode based on turn type (centerline data)
            if turn_type == 'left_turn':
                mode = 2
            elif turn_type == 'right_turn':
                mode = 3
            else:  # straight
                mode = 1
            
            # Default values if no distance detected
            if centerline_distance is None:
                offset = 0
                direction = 0  # 0 = right (neutral)
            else:
                # Direction: 1 = left (positive offset), 0 = right (negative offset)
                direction = 1 if centerline_distance > 0 else 0
                # Offset is absolute value of distance, clamped to byte range (0-255)
                offset = int(np.clip(abs(centerline_distance), 0, 255))
            
            if turn_type == 'left_turn':
                data = [mode]
            elif turn_type == 'right_turn':
                data = [mode]
            else:  # straight
                data = [mode, offset, direction]

            bus.write_i2c_block_data(I2C_ADDR, 0, data)
    except Exception as e:
        print(f"  I2C transmission error: {e}")


def process_frame(frame, curvature_threshold=0.003, enable_bullseye=True, enable_green=True):
    """
    Process a single frame and extract all detection data in real-time.
    
    Args:
        frame: Input frame from camera
        curvature_threshold: Threshold for turn detection
        enable_bullseye: Whether to perform bullseye detection
        enable_green: Whether to perform green box detection
    
    Returns:
        dict: Detection results including:
            - centerline_angle: Angle of centerline (degrees)
            - centerline_distance: Horizontal distance from center (pixels)
            - is_turn: Boolean indicating if a turn was detected
            - turn_type: Type of turn ('straight', 'left_turn', 'right_turn')
            - curvature: Curvature metric
            - green_detected: Whether green box was detected
            - green_under_red: Whether red passes under green box
            - bullseye_detected: Whether bullseye was detected
            - bullseye_centroid: Bullseye center position (x, y)
    """
    frame_height, frame_width = frame.shape[:2]
    results = {
        'centerline_angle': None,
        'centerline_distance': None,
        'is_turn': False,
        'turn_type': 'straight',
        'curvature': 0.0,
        'green_detected': False,
        'green_under_red': False,
        'bullseye_detected': False,
        'bullseye_centroid': None,
    }
    
    # Convert to HSV for color detection
    hsv_full = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Define HSV ranges for red color
    red_lower = np.array([0, 100, 100])
    red_upper = np.array([10, 255, 255])
    red_lower_2 = np.array([160, 100, 100])
    red_upper_2 = np.array([180, 255, 255])
    
    # ===== TURN DETECTION ON TOP 2/3 OF FRAME =====
    crop_turn_bottom = 2 * frame_height // 3
    frame_turn = frame[:crop_turn_bottom, :]
    hsv_turn = cv.cvtColor(frame_turn, cv.COLOR_BGR2HSV)
    
    mask1_turn = cv.inRange(hsv_turn, red_lower, red_upper)
    mask2_turn = cv.inRange(hsv_turn, red_lower_2, red_upper_2)
    mask_turn = cv.bitwise_or(mask1_turn, mask2_turn)
    
    # Apply morphological operations
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    mask_turn = cv.morphologyEx(mask_turn, cv.MORPH_CLOSE, kernel)
    mask_turn = cv.morphologyEx(mask_turn, cv.MORPH_OPEN, kernel)
    mask_turn = filter_mask_by_size(mask_turn, min_area=200)
    
    # Detect curved turn
    is_turn, turn_type, curvature, _, _ = detect_curved_turn_adaptive(
        frame_turn, mask_turn, adaptive_threshold=True, base_threshold=curvature_threshold
    )
    
    results['is_turn'] = is_turn
    results['turn_type'] = turn_type
    results['curvature'] = curvature
    
    # ===== EARLY BULLSEYE CHECK (to suppress turn warning if bullseye detected) =====
    bullseye_detected_early = False
    if enable_bullseye:
        bullseye_detected_early, _, _ = detect_bullseye(
            hsv_full, frame_height, frame_width, min_area=200
        )
    
    # Only print turn warning if no bullseye detected
    if is_turn and not bullseye_detected_early:
        on_turn_detected(turn_type, curvature)
    
    # ===== CENTERLINE ANALYSIS ON CROPPED CENTER 1/3 =====
    crop_top = frame_height // 3
    crop_bottom = 2 * frame_height // 3
    frame_cropped = frame[crop_top:crop_bottom, :]
    
    hsv_cropped = cv.cvtColor(frame_cropped, cv.COLOR_BGR2HSV)
    
    mask1 = cv.inRange(hsv_cropped, red_lower, red_upper)
    mask2 = cv.inRange(hsv_cropped, red_lower_2, red_upper_2)
    mask = cv.bitwise_or(mask1, mask2)
    
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = filter_mask_by_size(mask, min_area=200)
    
    # Extract centerline and calculate angle/distance
    centerline_pts = get_robust_centerline(mask, frame_cropped.shape[0], frame_cropped.shape[1], num_samples=20)
    
    if centerline_pts is not None and len(centerline_pts) >= 3:
        poly, centerline_angle = fit_centerline_curve(centerline_pts)
        
        if poly is not None and centerline_angle is not None:
            img_height, img_width = frame_cropped.shape[:2]
            center_x = img_width / 2
            center_y = img_height / 2
            
            centerline_x_at_center = poly(center_y)
            centerline_distance = centerline_x_at_center - center_x
            
            results['centerline_angle'] = centerline_angle
            results['centerline_distance'] = centerline_distance
    
    # ===== GREEN BOX DETECTION ON FULL FRAME =====
    if enable_green:
        green_lower = np.array([35, 75, 70])
        green_upper = np.array([85, 255, 255])
        
        mask_green_full = cv.inRange(hsv_full, green_lower, green_upper)
        mask_green_full = cv.morphologyEx(mask_green_full, cv.MORPH_CLOSE, kernel)
        mask_green_full = cv.morphologyEx(mask_green_full, cv.MORPH_OPEN, kernel)
        mask_green_full = filter_mask_by_size(mask_green_full, min_area=30)
        
        # Create red mask on full frame
        # Use gentler filtering for red detection to catch red under green
        mask_red_full = cv.inRange(hsv_full, red_lower, red_upper)
        mask_red_full |= cv.inRange(hsv_full, red_lower_2, red_upper_2)
        mask_red_full = cv.morphologyEx(mask_red_full, cv.MORPH_CLOSE, kernel)
        mask_red_full = cv.morphologyEx(mask_red_full, cv.MORPH_OPEN, kernel)
        # Use smaller min_area for green box detection since red under green may be fragmented
        mask_red_full = filter_mask_by_size(mask_red_full, min_area=50)
        
        green_detected, green_under_red = detect_green_tape_box(
            mask_green_full, mask_red_full, frame_height, frame_width
        )
        
        results['green_detected'] = green_detected
        results['green_under_red'] = green_under_red
    
    # ===== BULLSEYE DETECTION ON FULL FRAME =====
    if enable_bullseye:
        bullseye_detected, bullseye_centroid, bullseye_radius = detect_bullseye(
            hsv_full, frame_height, frame_width, min_area=200
        )
        
        bullseye_distance_x = None
        bullseye_distance_y = None
        if bullseye_detected and bullseye_centroid is not None:
            # Calculate X and Y distances from bottom center of camera frame to bullseye center
            camera_center_x = frame_width / 2
            camera_bottom_y = frame_height
            
            bullseye_distance_x = bullseye_centroid[0] - camera_center_x
            bullseye_distance_y = camera_bottom_y - bullseye_centroid[1]  # Distance from bottom of frame
        
        results['bullseye_detected'] = bullseye_detected
        results['bullseye_centroid'] = bullseye_centroid
        results['bullseye_distance_x'] = bullseye_distance_x
        results['bullseye_distance_y'] = bullseye_distance_y
    
    return results


def run_stream(camera_index=0, curvature_threshold=0.003, display=False, 
               enable_bullseye=True, enable_green=True):
    """
    Continuously process video stream from camera in real-time.
    
    Args:
        camera_index: Index of the camera (0 for first camera, etc.)
        curvature_threshold: Threshold for curve detection (lower = more sensitive)
        display: Whether to display the video stream (slower on Pi)
        enable_bullseye: Whether to perform bullseye detection
        enable_green: Whether to perform green box detection
    """
    # Initialize webcam
    cap = cv.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_index}. Make sure it's connected and not in use.")
        return
    
    # Set resolution
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 854)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Set frame rate
    cap.set(cv.CAP_PROP_FPS, 15)
    
    print("Starting real-time video stream with full detection...")
    print("Features enabled:")
    print(f"  - Turn Detection: Yes")
    print(f"  - Centerline Analysis: Yes")
    print(f"  - Green Box Detection: {enable_green}")
    print(f"  - Bullseye Detection: {enable_bullseye}")
    print("Press 'q' to quit\n")
    
    # Warm up: discard ~30 frames so auto-exposure can settle
    for _ in range(30):
        cap.read()
    
    time.sleep(0.5)
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break
            
            # Resize frame for consistency
            frame = cv.resize(frame, (854, 480))
            frame_count += 1
            
            # Process frame
            results = process_frame(
                frame, 
                curvature_threshold=curvature_threshold,
                enable_bullseye=enable_bullseye,
                enable_green=enable_green
            )
            
            # Print results to console
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            status_str = ""
            if results['centerline_angle'] is not None:
                status_str += f"Angle: {results['centerline_angle']:7.2f}° | H-Dist: {results['centerline_distance']:7.2f}px"
            else:
                status_str += "No red line detected"
            
            if results['is_turn']:
                status_str += f" | TURN ({results['turn_type']})"
            else:
                status_str += " | STRAIGHT"
            
            if results['green_detected']:
                status_str += f" | GREEN: {'UNDER' if results['green_under_red'] else 'BESIDE'}"
            
            if results['bullseye_detected']:
                status_str += f" | BULLSEYE at {results['bullseye_centroid']}"
                if results.get('bullseye_distance_x') is not None and results.get('bullseye_distance_y') is not None:
                    status_str += f" (X: {results['bullseye_distance_x']:.1f}px, Y: {results['bullseye_distance_y']:.1f}px)"
            
            print(f"[{timestamp}] Frame {frame_count} | {status_str}")
            
            # Send offset over I2C (priority: bullseye > turn > green box (if under red) > centerline)
            if results['bullseye_detected'] and results.get('bullseye_distance_x') is not None:
                send_i2c_data(None, False, 'straight', 
                            bullseye_distance_x=results['bullseye_distance_x'],
                            bullseye_distance_y=results['bullseye_distance_y'])
            elif results['is_turn']:
                # Turn detection takes priority over green box
                send_i2c_data(results['centerline_distance'], results['is_turn'], results['turn_type'])
            elif results['green_detected'] and results.get('green_under_red', False):
                # Only send green box data if red is under it
                send_i2c_data(None, False, 'straight',
                            green_detected=results['green_detected'],
                            green_under_red=results.get('green_under_red', False))
            elif results['centerline_distance'] is not None:
                send_i2c_data(results['centerline_distance'], False, 'straight')
            else:
                send_i2c_data(0, False, 'straight')
            
            # Display frame if enabled (slower on Pi)
            if display:
                cv.imshow('Real-Time Detection Stream', frame)
            
            # Check for 'q' key to quit
            if cv.waitKey(67) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\nStream interrupted by user")
    finally:
        elapsed = time.time() - start_time
        cap.release()
        cv.destroyAllWindows()
        if bus is not None:
            bus.close()
        
        print(f"\nStream stopped.")
        print(f"Processed {frame_count} frames in {elapsed:.2f} seconds")
        if elapsed > 0:
            print(f"Average FPS: {frame_count / elapsed:.2f}")


if __name__ == "__main__":
    # Configuration
    camera_index = 0  # 0 for default camera, 1 if you have multiple cameras
    curvature_threshold = 100  # Lower = more sensitive to curves
    display = False  # Set to True to display video (slower on Pi)
    enable_bullseye = True  # Enable bullseye detection
    enable_green = True  # Enable green box detection
    
    run_stream(
        camera_index=camera_index,
        curvature_threshold=curvature_threshold,
        display=display,
        enable_bullseye=enable_bullseye,
        enable_green=enable_green
    )
