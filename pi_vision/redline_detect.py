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
        
        # ===== TURN DETECTION ON TOP 2/3 OF FRAME =====
        # Crop to top 2/3 of frame for turn detection
        crop_turn_bottom = 2 * frame_height // 3
        frame_turn = frame[:crop_turn_bottom, :]
        
        # Convert to HSV to detect red color
        hsv_full = cv.cvtColor(frame_turn, cv.COLOR_BGR2HSV)
        
        # Define HSV range for red color
        # More permissive ranges to handle uneven lighting in turns
        red_lower = np.array([0, 80, 80])
        red_upper = np.array([10, 255, 255])
        red_lower_2 = np.array([160, 80, 80])
        red_upper_2 = np.array([180, 255, 255])
        
        # Create masks for red ranges on cropped frame
        mask1_full = cv.inRange(hsv_full, red_lower, red_upper)
        mask2_full = cv.inRange(hsv_full, red_lower_2, red_upper_2)
        mask_full = cv.bitwise_or(mask1_full, mask2_full)
        
        # Apply morphological operations to fill small gaps and remove noise
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask_full = cv.morphologyEx(mask_full, cv.MORPH_CLOSE, kernel)  # Fill gaps
        mask_full = cv.morphologyEx(mask_full, cv.MORPH_OPEN, kernel)   # Remove noise
        
        # Detect edges and lines on cropped frame
        red_regions_full = cv.bitwise_and(frame_turn, frame_turn, mask=mask_full)
        gray_full = cv.cvtColor(red_regions_full, cv.COLOR_BGR2GRAY)
        edges_full = cv.Canny(gray_full, 50, 150)
        
        lines_full = cv.HoughLinesP(edges_full, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        
        # Detect curved turn using adaptive threshold method
        is_turn, turn_type, curvature, fit_points, fit_coeffs = detect_curved_turn_adaptive(frame_turn, mask_full, adaptive_threshold=True, base_threshold=curvature_threshold)
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
        
        # Apply morphological operations to fill gaps and improve robustness
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)  # Fill small gaps in tape
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)   # Remove small noise
        
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
                
                print(f"  Centerline angle: {centerline_angle:.2f}deg")
                print(f"  Horizontal distance from center: {centerline_distance:.2f} pixels")
                if is_turn:
                    print(f"  🔄 Curved turn detected: {turn_type} (curvature: {curvature:.6f})")
            else:
                print(f"  Centerline extracted but fitting failed")
        else:
            print(f"  No red line detected - insufficient centerline data")
        
        # ===== VISUALIZE TURN DETECTION FOR DEBUGGING =====
        if fit_points is not None:
            # Draw the centerline points sampled from the tape (top 2/3 of frame, no offset needed)
            for pt in fit_points:
                cv.circle(out, (int(pt[0]), int(pt[1])), 4, (200, 200, 0), -1)  # Cyan points
            
            # Draw lines connecting the sampled centerline points to show the actual path
            for i in range(len(fit_points) - 1):
                pt1 = (int(fit_points[i, 0]), int(fit_points[i, 1]))
                pt2 = (int(fit_points[i + 1, 0]), int(fit_points[i + 1, 1]))
                cv.line(out, pt1, pt2, (255, 0, 255), 2)  # Magenta line
            
            # Add curvature value to image
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
    
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_dir = os.path.join(script_dir, "sample_images")
    output_dir = os.path.join(script_dir, "output")
    
    process_batch(input_dir, output_dir, curvature_threshold=100)