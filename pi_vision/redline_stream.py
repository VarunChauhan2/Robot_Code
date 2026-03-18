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

def calculate_line_angle(x1, y1, x2, y2):
    """Calculate angle of a line in degrees."""
    angle = np.arctan2(y2 - y1, x2 - x1)
    return np.degrees(angle)

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

def on_turn_detected(turn_type, curvature):
    """
    Callback function called when a turn is detected.
    Modify this function to perform your desired action (e.g., motor control, logging, etc.)
    
    Args:
        turn_type: String indicating turn type ('left_turn' or 'right_turn')
        curvature: The curvature coefficient of the detected curve
    """
    print(f"  ⚠️  TURN DETECTED: {turn_type} (curvature: {curvature:.6f})")
    # TODO: Add your turn handling logic here
    # Examples:
    # - Send turn command to motor controller
    # - Log turn event with high priority
    # - Trigger specific robot behavior
    pass


def get_line_endpoints(line):
    """Extract endpoints from a line array."""
    return line[0]

def get_centerline_from_edges(left_line, right_line):
    """Calculate the centerline parallel to and between the left and right edges."""
    x1_left, y1_left, x2_left, y2_left = get_line_endpoints(left_line)
    x1_right, y1_right, x2_right, y2_right = get_line_endpoints(right_line)
    
    # Calculate slopes of edge lines
    slope_left = (y2_left - y1_left) / (x2_left - x1_left) if x2_left != x1_left else float('inf')
    slope_right = (y2_right - y1_right) / (x2_right - x1_right) if x2_right != x1_right else float('inf')
    
    # Use average slope for centerline (parallel to edges)
    avg_slope = (slope_left + slope_right) / 2
    
    # Calculate midpoints along a vertical range to establish centerline
    y_min = min(y1_left, y2_left, y1_right, y2_right)
    y_max = max(y1_left, y2_left, y1_right, y2_right)
    
    # Get x coordinates at y_min and y_max for both edges
    def get_x_on_line(y, x1, y1, x2, y2):
        """Get x coordinate at given y on a line."""
        if y1 == y2:
            return (x1 + x2) / 2
        return x1 + (y - y1) * (x2 - x1) / (y2 - y1)
    
    x_left_at_ymin = get_x_on_line(y_min, x1_left, y1_left, x2_left, y2_left)
    x_right_at_ymin = get_x_on_line(y_min, x1_right, y1_right, x2_right, y2_right)
    center_x_at_ymin = (x_left_at_ymin + x_right_at_ymin) / 2
    
    x_left_at_ymax = get_x_on_line(y_max, x1_left, y1_left, x2_left, y2_left)
    x_right_at_ymax = get_x_on_line(y_max, x1_right, y1_right, x2_right, y2_right)
    center_x_at_ymax = (x_left_at_ymax + x_right_at_ymax) / 2
    
    # Centerline endpoints
    center_x1 = int(center_x_at_ymin)
    center_y1 = int(y_min)
    center_x2 = int(center_x_at_ymax)
    center_y2 = int(y_max)
    
    return (center_x1, center_y1, center_x2, center_y2)

def get_x_on_line(y, x1, y1, x2, y2):
    """Get x coordinate at given y on a line."""
    if y1 == y2:
        return (x1 + x2) / 2
    return x1 + (y - y1) * (x2 - x1) / (y2 - y1)

def get_horizontal_distance(center_x, center_y, cx1, cy1, cx2, cy2):
    """Calculate horizontal distance from image center to centerline."""
    # Find x coordinate of centerline at the image center's y position
    centerline_x = get_x_on_line(center_y, cx1, cy1, cx2, cy2)
    # Horizontal distance is the difference in x coordinates
    # Positive means centerline is to the right, negative means to the left
    return centerline_x - center_x

def send_i2c_data(centerline_distance, is_turn=False, turn_type='straight'):
    """Send horizontal offset over I2C bus to Arduino in format (mode, offset, direction).
    
    Args:
        centerline_distance: Horizontal distance to centerline
        is_turn: Boolean indicating if a turn was detected
        turn_type: Type of turn ('straight', 'left_turn', 'right_turn')
    """
    if bus is None:
        return
    
    try:
        # Determine mode based on turn type
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

def find_tape_edges(lines):
    """Find left and right edges of tape from detected lines."""
    if lines is None or len(lines) < 2:
        return None, None
    
    # Flatten lines and sort by x-coordinate of first point
    lines_list = [line[0] for line in lines]
    lines_sorted = sorted(lines_list, key=lambda l: min(l[0], l[2]))  # Sort by leftmost x
    
    # Left edge is the leftmost line, right edge is the rightmost line
    left_line = np.array([lines_sorted[0]])
    right_line = np.array([lines_sorted[-1]])
    
    return left_line, right_line


def process_frame(frame, curvature_threshold=0.003):
    """
    Process a single frame and extract centerline data and turn detection.
    
    Returns:
        tuple: (centerline_angle, centerline_distance, is_turn, turn_type, curvature)
    """
    frame_height, frame_width = frame.shape[:2]
    
    # ===== TURN DETECTION ON TOP 2/3 OF FRAME =====
    crop_turn_bottom = 2 * frame_height // 3
    frame_turn = frame[:crop_turn_bottom, :]
    
    # Convert to HSV to detect red color
    hsv_turn = cv.cvtColor(frame_turn, cv.COLOR_BGR2HSV)
    
    # Define HSV range for red color (stricter ranges to avoid false positives on edges)
    red_lower = np.array([0, 100, 100])
    red_upper = np.array([10, 255, 255])
    red_lower_2 = np.array([160, 100, 100])
    red_upper_2 = np.array([180, 255, 255])
    
    # Create masks for red ranges on cropped frame
    mask1_turn = cv.inRange(hsv_turn, red_lower, red_upper)
    mask2_turn = cv.inRange(hsv_turn, red_lower_2, red_upper_2)
    mask_turn = cv.bitwise_or(mask1_turn, mask2_turn)
    
    # Apply morphological operations to fill gaps and remove noise
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    mask_turn = cv.morphologyEx(mask_turn, cv.MORPH_CLOSE, kernel)  # Fill gaps
    mask_turn = cv.morphologyEx(mask_turn, cv.MORPH_OPEN, kernel)   # Remove noise
    
    # Filter out small noise regions, keep only significant contours
    mask_turn = filter_mask_by_size(mask_turn, min_area=200)
    
    # Detect curved turn using adaptive threshold method
    is_turn, turn_type, curvature, _, _ = detect_curved_turn_adaptive(frame_turn, mask_turn, adaptive_threshold=True, base_threshold=curvature_threshold)
    if is_turn:
        on_turn_detected(turn_type, curvature)
    
    # ===== CENTERLINE ANALYSIS ON CROPPED CENTER 1/3 =====
    crop_top = frame_height // 3
    crop_bottom = 2 * frame_height // 3
    frame_cropped = frame[crop_top:crop_bottom, :]
    
    hsv = cv.cvtColor(frame_cropped, cv.COLOR_BGR2HSV)
    
    # Create masks for red ranges (same stricter ranges)
    mask1 = cv.inRange(hsv, red_lower, red_upper)
    mask2 = cv.inRange(hsv, red_lower_2, red_upper_2)
    mask = cv.bitwise_or(mask1, mask2)
    
    # Apply morphological operations to fill gaps and improve robustness
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)  # Fill small gaps in tape
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)   # Remove small noise
    
    # Filter out small noise regions, keep only significant contours
    mask = filter_mask_by_size(mask, min_area=200)
    
    centerline_angle = None
    centerline_distance = None
    
    # Use robust centerline sampling method (more reliable than edge detection)
    centerline_pts = get_robust_centerline(mask, frame_cropped.shape[0], frame_cropped.shape[1], num_samples=20)
    
    if centerline_pts is not None and len(centerline_pts) >= 3:
        # Fit polynomial curve through centerline points
        poly, centerline_angle = fit_centerline_curve(centerline_pts)
        
        if poly is not None and centerline_angle is not None:
            # Calculate horizontal distance from image center to centerline
            img_height, img_width = frame_cropped.shape[:2]
            center_x = img_width / 2
            center_y = img_height / 2
            
            # Use polynomial for distance calculation
            centerline_x_at_center = poly(center_y)
            centerline_distance = centerline_x_at_center - center_x
    
    return centerline_angle, centerline_distance, is_turn, turn_type, curvature


def run_stream(camera_index=0, curvature_threshold=0.003):
    """
    Continuously process video stream from webcam.
    
    Args:
        camera_index: Index of the camera (0 for first camera, etc.)
        curvature_threshold: Threshold for curve detection (default 0.003, lower = more sensitive)
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
    
    print("Starting video stream from camera...")
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
            centerline_angle, centerline_distance, is_turn, turn_type, curvature = process_frame(frame, curvature_threshold)
            
            # Print results to console
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            if centerline_angle is not None and centerline_distance is not None:
                print(f"[{timestamp}] Frame {frame_count} | "
                      f"Angle: {centerline_angle:7.2f}° | "
                      f"H-Dist: {centerline_distance:7.2f}px | "
                      f"Status: {'TURN (' + turn_type + ')' if is_turn else 'STRAIGHT':<20} | "
                      f"Curve: {curvature:.4f}")
                # Send offset over I2C
                send_i2c_data(centerline_distance, is_turn, turn_type)
            else:
                print(f"[{timestamp}] Frame {frame_count} | No red line detected")
                # Send zero offset when no line detected
                send_i2c_data(0, False, 'straight')
            
            # Display frame (optional, can be slow on Pi)
            # cv.imshow('Red Line Detection Stream', frame)
            
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
    # You can adjust parameters:
    # camera_index: 0 for default camera, 1 if you have multiple cameras
    # curvature_threshold: Lower for more sensitive turn detection, higher for less sensitive
    # Default is 100 (lower = more sensitive to curves)
    
    run_stream(camera_index=0, curvature_threshold=100)

