import cv2 as cv
import numpy as np
import os

def red_line_detect_from_file(image_path="captured_image.jpg", output_path="red_line_detected.jpg"):
    
    # attempt to load provided path, falling back between .jpg/.jpeg
    img = cv.imread(image_path)
    if img is None:
        alt = None
        if image_path.lower().endswith('.jpg'):
            alt = image_path[:-4] + '.jpeg'
        elif image_path.lower().endswith('.jpeg'):
            alt = image_path[:-5] + '.jpg'
        if alt:
            img = cv.imread(alt)
            if img is not None:
                image_path = alt

    if img is None:
        print(f"Failed to load image: {image_path}")
        return

    frame = cv.resize(img, (480, 480))

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # [*3] Define HSV range for red color
    red_lower = np.array([0, 100, 100])
    red_upper = np.array([10, 255, 255])
    red_lower_2 = np.array([160, 100, 100])
    red_upper_2 = np.array([180, 255, 255])

    # create masks for red ranges
    mask1 = cv.inRange(hsv, red_lower, red_upper)
    mask2 = cv.inRange(hsv, red_lower_2, red_upper_2)
    mask = cv.bitwise_or(mask1, mask2)

    red_regions = cv.bitwise_and(frame, frame, mask=mask)
    gray = cv.cvtColor(red_regions, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray, 50, 150)

    lines = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

    out = frame.copy()
    if lines is not None:
        print("Red line detected")
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv.line(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
    else:
        print("No red line detected")

    cv.imshow('Red Line Detection', out)
    cv.imwrite(output_path, out)

    # wait until 'q' is pressed
    while True:
        if cv.waitKey(0) & 0xFF == ord('q'):
            break

    cv.destroyAllWindows()


if __name__ == "__main__":
    red_line_detect_from_file("captured_image.jpg")