import cv2 as cv
import numpy as np
import os
import csv
from datetime import datetime

def get_line_endpoints(line):
    """Extract endpoints from a line array."""
    return line[0]

def calculate_line_angle(x1, y1, x2, y2):
    """Calculate angle of a line in degrees."""
    angle = np.arctan2(y2 - y1, x2 - x1)
    return np.degrees(angle)

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

def detect_curved_turn(frame, mask, curvature_threshold=0.003):
    """
    Detect curved turns by analyzing the rate of change of the centerline's x-position.
    
    Args:
        frame: The full image frame
        mask: Red color mask of the frame
        curvature_threshold: Threshold for rate of change (default 0.003)
    
    Returns:
        tuple: (is_turn, turn_type, curvature, centerline_pts, fit_coeffs)
            - is_turn: Boolean indicating if a turn was detected
            - turn_type: String describing turn type ('left_turn', 'right_turn', 'straight')
            - curvature: The rate of change metric
            - centerline_pts: Array of centerline points used for fitting (for visualization)
            - fit_coeffs: None (not used with this method)
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
        return False, 'straight', 0.0, None, None
    
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
            return False, 'straight', curvature, centerline_points, None
        
        return True, turn_direction, curvature, centerline_points, None
    
    except Exception as e:
        print(f"  Error analyzing centerline curvature: {e}")
        return False, 'straight', 0.0, None, None

def get_x_on_line(y, x1, y1, x2, y2):
    """Get x coordinate at given y on a line."""
    if y1 == y2:
        return (x1 + x2) / 2
    return x1 + (y - y1) * (x2 - x1) / (y2 - y1)

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

def get_horizontal_distance(center_x, center_y, cx1, cy1, cx2, cy2):
    """Calculate horizontal distance from image center to centerline."""
    # Find x coordinate of centerline at the image center's y position
    centerline_x = get_x_on_line(center_y, cx1, cy1, cx2, cy2)
    # Horizontal distance is the difference in x coordinates
    # Positive means centerline is to the right, negative means to the left
    return centerline_x - center_x

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

def process_batch(input_dir="sample_images", output_dir="output", curvature_threshold=0.003):

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
    csv_writer.writerow(['Image', 'Centerline_Angle (degrees)', 'Distance_From_Center (pixels)', 'Turn_Detected', 'Turn_Type', 'Curvature', 'Timestamp'])
    
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
        
        # ===== TURN DETECTION ON FULL FRAME =====
        # Convert to HSV to detect red color
        hsv_full = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Define HSV range for red color
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        red_lower_2 = np.array([160, 100, 100])
        red_upper_2 = np.array([180, 255, 255])
        
        # Create masks for red ranges on full frame
        mask1_full = cv.inRange(hsv_full, red_lower, red_upper)
        mask2_full = cv.inRange(hsv_full, red_lower_2, red_upper_2)
        mask_full = cv.bitwise_or(mask1_full, mask2_full)
        
        # Detect edges and lines on full frame
        red_regions_full = cv.bitwise_and(frame, frame, mask=mask_full)
        gray_full = cv.cvtColor(red_regions_full, cv.COLOR_BGR2GRAY)
        edges_full = cv.Canny(gray_full, 50, 150)
        
        lines_full = cv.HoughLinesP(edges_full, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        
        # Detect curved turn using full frame centerline sampling
        is_turn, turn_type, curvature, fit_points, fit_coeffs = detect_curved_turn(frame, mask_full, curvature_threshold)
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
        
        # create masks for red ranges
        mask1 = cv.inRange(hsv, red_lower, red_upper)
        mask2 = cv.inRange(hsv, red_lower_2, red_upper_2)
        mask = cv.bitwise_or(mask1, mask2)
        
        # Save mask
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
        
        if lines is not None:
            print(f"  Red line detected - {len(lines)} line segment(s) found")
            
            # Find left and right edges of tape
            left_line, right_line = find_tape_edges(lines)
            
            if left_line is not None and right_line is not None:
                # Draw detected edge lines in blue (offset y-coordinates to full frame)
                for line in [left_line, right_line]:
                    x1, y1, x2, y2 = line[0]
                    cv.line(out, (x1, y1 + crop_top), (x2, y2 + crop_top), (255, 0, 0), 2)
                
                # Calculate centerline
                cx1, cy1, cx2, cy2 = get_centerline_from_edges(left_line, right_line)
                
                # Draw centerline in green on full frame (offset y-coordinates)
                cv.line(out, (cx1, cy1 + crop_top), (cx2, cy2 + crop_top), (0, 255, 0), 3)
                
                # Calculate angle of centerline
                centerline_angle = calculate_line_angle(cx1, cy1, cx2, cy2)
                
                # Calculate horizontal distance from image center to centerline
                img_height, img_width = frame_cropped.shape[:2]
                center_x = img_width / 2
                center_y = img_height / 2
                
                centerline_distance = get_horizontal_distance(center_x, center_y, cx1, cy1, cx2, cy2)
                
                # Draw a circle at image center in the cropped region (offset to full frame)
                cv.circle(out, (int(center_x), int(center_y + crop_top)), 5, (0, 0, 255), -1)
                
                # Draw a line from image center perpendicular to the centerline (offset to full frame)
                centerline_x_at_center = get_x_on_line(center_y, cx1, cy1, cx2, cy2)
                cv.line(out, (int(center_x), int(center_y + crop_top)), (int(centerline_x_at_center), int(center_y + crop_top)), (255, 255, 0), 2)
                
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
                
                print(f"  Centerline angle: {centerline_angle:.2f}deg")
                print(f"  Horizontal distance from center: {centerline_distance:.2f} pixels")
                if is_turn:
                    print(f"  🔄 Curved turn detected: {turn_type} (curvature: {curvature:.6f})")
            else:
                print(f"  Could not identify tape edges")
                # Draw all detected lines for debugging (offset y-coordinates to full frame)
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv.line(out, (x1, y1 + crop_top), (x2, y2 + crop_top), (0, 255, 0), 2)
        else:
            print(f"  No red line detected")
        
        # ===== VISUALIZE CURVE FITTING FOR DEBUGGING =====
        if fit_points is not None:
            # Draw the centerline points sampled from the tape
            for pt in fit_points:
                cv.circle(out, (int(pt[0]), int(pt[1])), 4, (200, 200, 0), -1)  # Cyan points
            
            # Draw lines connecting the sampled centerline points to show the actual path
            for i in range(len(fit_points) - 1):
                pt1 = (int(fit_points[i, 0]), int(fit_points[i, 1]))
                pt2 = (int(fit_points[i + 1, 0]), int(fit_points[i + 1, 1]))
                cv.line(out, pt1, pt2, (255, 0, 255), 2)  # Magenta line
            
            # Add curvature value to image (now variance of x-changes)
            cv.putText(out, f"Curvature (variance): {curvature:.4f} (threshold: {curvature_threshold})", 
                       (10, 135), cv.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)
        
        # Save final result
        output_path = os.path.join(output_dir, f"04_detected_{base_filename}.jpg")
        cv.imwrite(output_path, out)
        print(f"  Saved detected: {output_path}")
        
        # Log to CSV
        timestamp = datetime.now().isoformat()
        if centerline_angle is not None and centerline_distance is not None:
            csv_writer.writerow([image_file, f"{centerline_angle:.2f}", f"{centerline_distance:.2f}", 
                               is_turn, turn_type, f"{curvature:.6f}", timestamp])
        else:
            csv_writer.writerow([image_file, "N/A", "N/A", "N/A", "N/A", "N/A", timestamp])
    
    csv_file.close()
    print(f"\nBatch processing complete! Results saved to {output_dir}")
    print(f"Centerline data logged to {csv_path}")


if __name__ == "__main__":
    # You can adjust the curvature_threshold parameter:
    # - Lower values (e.g., 0.001): More sensitive, detects slight curves
    # - Higher values (e.g., 0.005): Less sensitive, only sharp turns detected
    # Default is 0.003
    process_batch("sample_images", "output", curvature_threshold=0.7)