import cv2
import numpy as np

def detect_circles(image_path):
    # Load image
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Use Hough Circle Transform to detect circular shapes
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
                               param1=50, param2=30, minRadius=10, maxRadius=40)
    
    centers = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            centers.append(center)
            
            # Draw detected circles
            cv2.circle(image, center, circle[2], (0, 255, 0), 2)
            cv2.circle(image, center, 3, (0, 0, 255), -1)
    
    # Show the result
    cv2.imshow('Detected Circles', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return centers

# Example usage
image_path = 'checkers.jpg'  # Replace with your image path
detected_centers = detect_circles(image_path)
print("Detected Centers:", detected_centers)
