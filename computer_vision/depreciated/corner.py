import cv2
import numpy as np

def detect_blue_corners(image_path, show_result=True):
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Image not found: {image_path}")
    h, w = img.shape[:2]

    # Convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # HSV range for blue
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Morphological clean-up
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Define image corners
    image_corners = {
        "TL": (0, 0),
        "TR": (w, 0),
        "BL": (0, h),
        "BR": (w, h)
    }
    max_distance = min(w, h) * 0.15  # Threshold: 15% of size

    found_corners = {}

    for cnt in contours:
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        point = (cx, cy)

        # Check if this point is close to any corner
        for label, corner in image_corners.items():
            dist = np.linalg.norm(np.array(point) - np.array(corner))
            if dist < max_distance and label not in found_corners:
                found_corners[label] = point
                break

    # Ensure all 4 corners were found
    ordered_labels = ["TL", "TR", "BL", "BR"]
    ordered_points = [found_corners.get(lbl, None) for lbl in ordered_labels]



    if show_result:
        for point in ordered_points:
            cv2.circle(img, point, 10, (0, 255, 0), 2)
        print("Ordered Corners (TL, TR, BL, BR):", ordered_points)
        cv2.imshow("Detected Corners", img)
        cv2.imshow("Blue Mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    
    return ordered_points

# Example usage
if __name__ == "__main__":
    detect_blue_corners("piecereal/colourboard3.png")
