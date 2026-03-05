import cv2 as cv
import numpy as np
import time
import os

def take_photos_on_keypress(output_dir=None):
    # If no output directory specified, use sample_images in the script's directory
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(__file__), "sample_images")
    
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Initialize webcam with DirectShow backend (Windows-specific)
    cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    
    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera 1. Make sure it's connected and not in use.")
        return

    # [*1] Set resolution
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 854)  # Set width to 640 pixels
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)  # Set height to 480 pixels

    # [*2] Set frame rate
    cap.set(cv.CAP_PROP_FPS, 30)  # Set to 30 frames per second
    
    # Add a small delay to allow camera to warm up
    time.sleep(1)

    # warm up: discard ~20-30 frames so auto-exposure can settle
    for _ in range(30):
        cap.read()
    
    # Find the next available photo number
    photo_count = 0
    if os.path.exists(output_dir):
        existing_files = os.listdir(output_dir)
        for file in existing_files:
            if file.startswith("photo_") and file.endswith(".jpg"):
                try:
                    num = int(file[6:-4])  # Extract number from "photo_X.jpg"
                    photo_count = max(photo_count, num)
                except ValueError:
                    pass
    
    print("Press any key to capture a photo, press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        # Resize frame for consistency
        frame = cv.resize(frame, (854, 480))
        
        # Display the frame
        cv.imshow('Photo Capture - Press key to capture, q to quit', frame)
        
        # Wait for key press
        key = cv.waitKey(1) & 0xFF
        
        # If 'q' is pressed, exit
        if key == ord('q'):
            print("Exiting...")
            break
        
        # If any other key is pressed (except 255 which means no key), take a photo
        if key != 255:
            photo_count += 1
            output_path = os.path.join(output_dir, f"photo_{photo_count}.jpg")
            cv.imwrite(output_path, frame)
            print(f"Photo saved to {output_path}")

    cap.release()
    cv.destroyAllWindows()

# Run the function
if __name__ == "__main__":
    take_photos_on_keypress()
