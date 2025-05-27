import cv2
import numpy as np

def nothing(x):
    pass

# Load test image
image = cv2.imread("realsenseboard4.png")  # Replace with your cropped piece image
if image is None:
    raise ValueError("Image not found. Make sure the path is correct.")

# Convert to HSV
# Convert to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


# Create trackbars with your provided starting values
cv2.namedWindow("HSV Calibration - Blue")
cv2.createTrackbar("H Lower", "HSV Calibration - Blue", 88, 179, nothing)
cv2.createTrackbar("H Upper", "HSV Calibration - Blue", 133, 179, nothing)
cv2.createTrackbar("S Lower", "HSV Calibration - Blue", 58, 255, nothing)
cv2.createTrackbar("S Upper", "HSV Calibration - Blue", 255, 255, nothing)
cv2.createTrackbar("V Lower", "HSV Calibration - Blue", 65, 255, nothing)
cv2.createTrackbar("V Upper", "HSV Calibration - Blue", 255, 255, nothing)

while True:
    # Read current HSV bounds
    h_lower = cv2.getTrackbarPos("H Lower", "HSV Calibration - Blue")
    h_upper = cv2.getTrackbarPos("H Upper", "HSV Calibration - Blue")
    s_lower = cv2.getTrackbarPos("S Lower", "HSV Calibration - Blue")
    s_upper = cv2.getTrackbarPos("S Upper", "HSV Calibration - Blue")
    v_lower = cv2.getTrackbarPos("V Lower", "HSV Calibration - Blue")
    v_upper = cv2.getTrackbarPos("V Upper", "HSV Calibration - Blue")

    lower_bound = np.array([h_lower, s_lower, v_lower])
    upper_bound = np.array([h_upper, s_upper, v_upper])

    # Apply mask
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    result = cv2.bitwise_and(image, image, mask=mask)

    # Show result
    stacked = np.hstack([image, result])
    cv2.imshow("Calibration", stacked)
    cv2.imshow("Mask Only", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print(f"Final HSV Range for Blue:\nLower: {lower_bound}\nUpper: {upper_bound}")
        break

cv2.destroyAllWindows()
