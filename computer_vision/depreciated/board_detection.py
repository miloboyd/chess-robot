import cv2
import numpy as np


def find_chess_grid_centers(image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    # This looks for internal corners where black and white squares meet
    # For a standard 8x8 chessboard, there are 7x7 internal corners
    ret, corners = cv2.findChessboardCorners(gray, (7, 7), None)

    # If corners are found, we can calculate grid centers
    if ret:
        # Make a copy of the image to draw on
        result = image.copy()

        # Draw the corners
        cv2.drawChessboardCorners(result, (7, 7), corners, ret)

        # Create arrays to store grid centers
        grid_centers = []

        # Find the grid spacing by averaging distances between corners
        spacing_x = np.mean([corners[i + 1][0][0] - corners[i][0][0] for i in range(0, len(corners) - 1, 7)])
        spacing_y = np.mean([corners[i + 7][0][1] - corners[i][0][1] for i in range(0, len(corners) - 7)])

        # We need to extrapolate to get the outer grid centers
        # First corner of the board is half a grid back from the first internal corner
        top_left_x = corners[0][0][0] - spacing_x / 2
        top_left_y = corners[0][0][1] - spacing_y / 2

        # For an 8x8 board
        for row in range(8):
            for col in range(8):
                # Calculate center of each grid
                center_x = int(top_left_x + col * spacing_x)
                center_y = int(top_left_y + row * spacing_y)

                # Store grid center
                grid_centers.append((center_x, center_y))

                # Draw a circle at the grid center
                cv2.circle(result, (center_x, center_y), 5, (0, 255, 0), -1)

                # Optionally, label the grid
                cv2.putText(result, f"{row},{col}", (center_x + 10, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        return result, grid_centers

    else:
        print("Could not find chessboard corners!")
        return image, []


def alternative_grid_detection(image):
    """
    Alternative method if findChessboardCorners fails
    Uses contour detection to find the grid squares
    """
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply adaptive thresholding
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, 11, 2)

    # Invert if needed
    # thresh = cv2.bitwise_not(thresh)

    # Find contours
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours by area and shape to find squares
    squares = []
    centers = []
    result = image.copy()

    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Filter small contours
        if area < 100:
            continue

        # Approximate contour
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

        # If approximated contour has 4 points, it might be a square
        if len(approx) == 4:
            # Check if it's approximately square
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h

            if 0.8 <= aspect_ratio <= 1.2:
                squares.append(approx)

                # Calculate center
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    centers.append((cx, cy))

                    # Draw center
                    cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)

    # Draw all found squares
    cv2.drawContours(result, squares, -1, (0, 255, 0), 2)

    return result, centers


# Main function to try both methods
def detect_chess_grid(image_path):
    # Load image
    img = cv2.imread(image_path)

    if img is None:
        print(f"Could not read image from {image_path}")
        return

    # Try the chessboard corner detection method first
    result, centers = find_chess_grid_centers(img)

    # If first method fails (no centers found), try alternative method
    if len(centers) == 0:
        print("Trying alternative method...")
        result, centers = alternative_grid_detection(img)

    # Show result
    cv2.imshow('Chess Grid Detection', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return centers

_name_ = "_main_"
# Example usage
if _name_ == "_main_":
    # Replace with your chessboard image path
    centers = detect_chess_grid("chessboard_blank.png")
    print(f"Found {len(centers)} grid centers")
