import cv2 as cv
import numpy as np

def take_single_photo(output_path="captured_image.jpg"):
    # Initialize webcam with DirectShow backend (Windows-specific)
    cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    
    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera 1. Make sure it's connected and not in use.")
        return

    # [*1] Set resolution
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)  # Set width to 640 pixels
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)  # Set height to 480 pixels

    # [*2] Set frame rate
    cap.set(cv.CAP_PROP_FPS, 30)  # Set to 30 frames per second
    
    # Add a small delay to allow camera to warm up
    import time
    time.sleep(1)

    # warm up: discard ~20-30 frames so auto-exposure can settle
    for _ in range(30):
        cap.read()
    # now take the real frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")

    # Resize frame for consistency
    frame = cv.resize(frame, (480, 480))
    # Save the captured image
    cv.imwrite(output_path, frame)
    print(f"Photo saved to {output_path}")
    cv.imshow('Red Line Detection', frame)

    # Wait for 'q' key press to close
    while True:
        if cv.waitKey(0) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

# Run the function
if __name__ == "__main__":
    take_single_photo()
