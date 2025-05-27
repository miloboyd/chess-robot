import cv2
import numpy as np

def nothing(x):
    pass

# Load test image
image = cv2.imread("realsenseboard6.png")  # Replace with your actual test image
if image is None:
    raise ValueError("Image not found. Make sure the path is correct.")

# Convert to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create calibration window
cv2.namedWindow("HSV Calibration - Yellow")

# Default yellow HSV range from you
cv2.createTrackbar("H Lower", "HSV Calibration - Yellow", 5, 179, nothing)
cv2.createTrackbar("H Upper", "HSV Calibration - Yellow", 45, 179, nothing)
cv2.createTrackbar("S Lower", "HSV Calibration - Yellow", 43, 255, nothing)
cv2.createTrackbar("S Upper", "HSV Calibration - Yellow", 255, 255, nothing)
cv2.createTrackbar("V Lower", "HSV Calibration - Yellow", 127, 255, nothing)
cv2.createTrackbar("V Upper", "HSV Calibration - Yellow", 255, 255, nothing)

while True:
    # Read values from trackbars
    h_lower = cv2.getTrackbarPos("H Lower", "HSV Calibration - Yellow")
    h_upper = cv2.getTrackbarPos("H Upper", "HSV Calibration - Yellow")
    s_lower = cv2.getTrackbarPos("S Lower", "HSV Calibration - Yellow")
    s_upper = cv2.getTrackbarPos("S Upper", "HSV Calibration - Yellow")
    v_lower = cv2.getTrackbarPos("V Lower", "HSV Calibration - Yellow")
    v_upper = cv2.getTrackbarPos("V Upper", "HSV Calibration - Yellow")

    lower_bound = np.array([h_lower, s_lower, v_lower])
    upper_bound = np.array([h_upper, s_upper, v_upper])

    # Generate mask and result
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    result = cv2.bitwise_and(image, image, mask=mask)

    # Show output
    stacked = np.hstack([image, result])
    cv2.imshow("Calibration", stacked)
    cv2.imshow("Mask Only", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print(f"\nFinal HSV Range for Yellow:")
        print(f"Lower: {lower_bound}")
        print(f"Upper: {upper_bound}")
        break

cv2.destroyAllWindows()
