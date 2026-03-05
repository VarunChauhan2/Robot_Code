import cv2 as cv
import numpy as np
import time
from datetime import datetime

def calculate_line_angle(x1, y1, x2, y2):
    """Calculate angle of a line in degrees."""
    angle = np.arctan2(y2 - y1, x2 - x1)
    return np.degrees(angle)

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

def detect_curved_turn(frame, mask, curvature_threshold=0.003):
    """
    Detect curved turns by analyzing the rate of change of the centerline's x-position.
    
    Args:
        frame: The full image frame
        mask: Red color mask of the frame
        curvature_threshold: Threshold for rate of change (default 0.003)
    
    Returns:
        tuple: (is_turn, turn_type, curvature)
            - is_turn: Boolean indicating if a turn was detected
            - turn_type: String describing turn type ('left_turn', 'right_turn', 'straight')
            - curvature: The rate of change metric
    """
    height, width = frame.shape[:2]
    centerline_points = []
    
    # Sample the tape centerline at different heights
    for y in range(0, height, max(1, height // 20)):  # Sample ~20 points across height
        row = mask[y, :]
        
        # Find all x-coordinates where red color is detected in this row
        red_x = np.where(row > 0)[0]
        
        if len(red_x) > 0:
            # Calculate the centerline as the middle of the red region
            x_min = red_x[0]
            x_max = red_x[-1]
            x_center = (x_min + x_max) / 2
            centerline_points.append([x_center, y])
    
    if len(centerline_points) < 3:
        return False, 'straight', 0.0
    
    centerline_points = np.array(centerline_points, dtype=np.float32)
    
    try:
        # Calculate the rate of change of x position as we go down the centerline
        x_changes = np.diff(centerline_points[:, 0])  # Change in x between consecutive points
        
        # The curvature metric is the variance of x-changes
        # High variance = the x position is changing inconsistently = curve
        curvature = np.var(x_changes)
        
        # Determine turn direction based on the overall x-movement trend
        # Since top of image is front of robot, moving forward down the image:
        # If x increases (moves right) = left turn
        # If x decreases (moves left) = right turn
        total_x_change = centerline_points[-1, 0] - centerline_points[0, 0]
        turn_direction = 'left_turn' if total_x_change > 0 else 'right_turn'
        
        # Check if variance exceeds threshold
        is_turn = curvature > curvature_threshold
        
        if not is_turn:
            return False, 'straight', curvature
        
        return True, turn_direction, curvature
    
    except Exception as e:
        print(f"  Error analyzing centerline curvature: {e}")
        return False, 'straight', 0.0

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

def process_frame(frame, curvature_threshold=30):
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
    
    # Define HSV range for red color
    red_lower = np.array([0, 100, 100])
    red_upper = np.array([10, 255, 255])
    red_lower_2 = np.array([160, 100, 100])
    red_upper_2 = np.array([180, 255, 255])
    
    # Create masks for red ranges on cropped frame
    mask1_turn = cv.inRange(hsv_turn, red_lower, red_upper)
    mask2_turn = cv.inRange(hsv_turn, red_lower_2, red_upper_2)
    mask_turn = cv.bitwise_or(mask1_turn, mask2_turn)
    
    # Detect curved turn using cropped frame centerline sampling
    is_turn, turn_type, curvature = detect_curved_turn(frame_turn, mask_turn, curvature_threshold)
    
    # ===== CENTERLINE ANALYSIS ON CROPPED CENTER 1/3 =====
    crop_top = frame_height // 3
    crop_bottom = 2 * frame_height // 3
    frame_cropped = frame[crop_top:crop_bottom, :]
    
    hsv = cv.cvtColor(frame_cropped, cv.COLOR_BGR2HSV)
    
    # Create masks for red ranges
    mask1 = cv.inRange(hsv, red_lower, red_upper)
    mask2 = cv.inRange(hsv, red_lower_2, red_upper_2)
    mask = cv.bitwise_or(mask1, mask2)
    
    red_regions = cv.bitwise_and(frame_cropped, frame_cropped, mask=mask)
    gray = cv.cvtColor(red_regions, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray, 50, 150)
    
    lines = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
    
    centerline_angle = None
    centerline_distance = None
    
    if lines is not None:
        # Find left and right edges of tape
        left_line, right_line = find_tape_edges(lines)
        
        if left_line is not None and right_line is not None:
            # Calculate centerline
            cx1, cy1, cx2, cy2 = get_centerline_from_edges(left_line, right_line)
            
            # Calculate angle of centerline
            centerline_angle = calculate_line_angle(cx1, cy1, cx2, cy2)
            
            # Calculate horizontal distance from image center to centerline
            img_height, img_width = frame_cropped.shape[:2]
            center_x = img_width / 2
            center_y = img_height / 2
            
            centerline_distance = get_horizontal_distance(center_x, center_y, cx1, cy1, cx2, cy2)
    
    return centerline_angle, centerline_distance, is_turn, turn_type, curvature

def run_stream(camera_index=0, curvature_threshold=30):
    """
    Continuously process video stream from webcam.
    
    Args:
        camera_index: Index of the camera (0 for first camera, etc.)
        curvature_threshold: Threshold for curve detection (default 30)
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
            else:
                print(f"[{timestamp}] Frame {frame_count} | No red line detected")
            
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
        
        print(f"\nStream stopped.")
        print(f"Processed {frame_count} frames in {elapsed:.2f} seconds")
        if elapsed > 0:
            print(f"Average FPS: {frame_count / elapsed:.2f}")

if __name__ == "__main__":
    # You can adjust parameters:
    # camera_index: 0 for default camera, 1 if you have multiple cameras
    # curvature_threshold: Lower for more sensitive turn detection, higher for less sensitive
    
    run_stream(camera_index=0, curvature_threshold=30)
