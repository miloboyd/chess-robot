import cv2
import numpy as np


# Load an image

def process_image(image):
    img = cv2.imread(image)

    #rgb2gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 127, 255, 0)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cv2.imshow('gray', gray)

    #blur
    #gaussian_blur = cv2.GaussianBlur(gray,(1,1),0)

    #cv2.imshow('guass', gaussian_blur)

    #binary
    #ret,otsu_binary = cv2.threshold(gaussian_blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    #cv2.imshow('binary', otsu_binary)

    #canny edge detect
    canny = cv2.Canny(gray,20,255)

    cv2.imshow('canny', canny)
    return canny

def find_chess_grid_centers(image):
    # Convert to grayscale
    gray = image
    #Try different parameters for more robust corner detection
    for pattern_size in [(7, 7), (6, 6), (5, 5)]:
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, 
                                                cv2.CALIB_CB_ADAPTIVE_THRESH + 
                                                cv2.CALIB_CB_NORMALIZE_IMAGE + 
                                                cv2.CALIB_CB_FILTER_QUADS)
        if ret:
            # Refine corners for better accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            break
    
    # If corners are found, we can calculate grid centers
    if ret:
        # Make a copy of the image to draw on
        result = image.copy()

        # Draw the corners
        cv2.drawChessboardCorners(result, pattern_size, corners, ret)

        # Create arrays to store grid centers
        grid_centers = []

        # Calculate the spacing based on pattern size
        rows, cols = pattern_size
        
        # Find the grid spacing by averaging distances between corners
        spacing_x = np.mean([corners[i + 1][0][0] - corners[i][0][0] for i in range(0, len(corners) - 1, cols)])
        spacing_y = np.mean([corners[i + cols][0][1] - corners[i][0][1] for i in range(0, len(corners) - cols)])

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
                grid_centers.append((center_x, center_y, row, col))

                # Draw a circle at the grid center
                cv2.circle(result, (center_x, center_y), 5, (0, 255, 0), -1)

                # Optionally, label the grid
                cv2.putText(result, f"{row},{col}", (center_x + 10, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        return result, grid_centers, spacing_x, spacing_y

    else:
        print("Could not find chessboard corners using standard method!")
        return image, [], 0, 0


def alternative_grid_detection(image):
    """
    Alternative method if findChessboardCorners fails
    Uses contour detection to find the grid squares
    """
    # Convert to grayscale
    gray = image

    # Apply adaptive thresholding
    #thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    # Find contours
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
    
    # Try to organize centers into a grid
    if len(centers) >= 64:  # If we found at least 64 centers
        # Sort by y coordinate first (rows)
        centers.sort(key=lambda p: p[1])
        
        # Estimate the number of centers per row
        row_count = int(np.sqrt(len(centers)))
        
        # Group into rows
        grid_centers = []
        for i in range(0, len(centers), row_count):
            row = centers[i:i+row_count]
            # Sort each row by x coordinate
            row.sort(key=lambda p: p[0])
            grid_centers.extend([(x, y, i//row_count, j) for j, (x, y) in enumerate(row)])
        
        # Calculate approximate spacing
        if len(grid_centers) >= 2:
            spacing_x = abs(grid_centers[1][0] - grid_centers[0][0])
            spacing_y = abs(grid_centers[row_count][1] - grid_centers[0][1])
            return result, grid_centers[:64], spacing_x, spacing_y  # Return only the first 64 centers
    
    # If we couldn't organize into a grid
    return result, centers, 0, 0


def detect_pieces(image, grid_centers, square_size_x, square_size_y):
    """
    Detect chess pieces on the board based on grid centers
    """
    result = image.copy()
    piece_positions = []
    
    # Check each grid cell for pieces
    for center_x, center_y, row, col in grid_centers:
        # Define region of interest (ROI) for the current square
        half_x = int(square_size_x * 0.4)  # Use slightly smaller than half to avoid edges
        half_y = int(square_size_y * 0.4)
        
        x1, y1 = max(0, center_x - half_x), max(0, center_y - half_y)
        x2, y2 = min(image.shape[1]-1, center_x + half_x), min(image.shape[0]-1, center_y + half_y)
        
        # Extract ROI
        roi = image[y1:y2, x1:x2]
        
        if roi.size == 0:  # Skip if ROI is empty
            continue
            
        # Convert to grayscale
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Calculate standard deviation of pixel intensities
        # Higher std dev indicates more variation (likely a piece)
        std_dev = np.std(roi_gray)
        
        # Convert to HSV for color analysis
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Calculate average color in the ROI
        avg_color = np.mean(roi_hsv, axis=(0, 1))
        
        # Determine if the square has a piece based on standard deviation
        # This threshold might need adjustment based on your images
        has_piece = std_dev > 25
        
        # Determine piece color (if piece exists)
        piece_color = None
        if has_piece:
            # Check if the piece is dark or light based on value channel
            avg_brightness = np.mean(roi_gray)
            
            # Adjust these thresholds based on your lighting conditions
            if avg_brightness < 120:
                piece_color = "black"
            else:
                piece_color = "white"
                
            # Store piece information
            piece_positions.append({
                "position": (row, col),
                "center": (center_x, center_y),
                "color": piece_color
            })
            
            # Draw piece detection on the result image
            piece_color_bgr = (0, 0, 255) if piece_color == "black" else (255, 255, 255)
            cv2.circle(result, (center_x, center_y), 10, piece_color_bgr, 2)
            
            # Label
            cv2.putText(result, f"{piece_color[0]}", (center_x - 5, center_y + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return result, piece_positions


# Main function to detect chessboard and pieces
def detect_chess_board_with_pieces(img):
    # Load image

    if img is None:
        print(f"Could not read image from {img}")
        return

    # Try the chessboard corner detection method first
    result, centers, spacing_x, spacing_y = find_chess_grid_centers(img)

    # If first method fails (no centers found), try alternative method
    if len(centers) == 0:
        print("Trying alternative method...")
        result, centers, spacing_x, spacing_y = alternative_grid_detection(img)
        
    # Detect pieces if we have valid grid centers
    if len(centers) > 0 and spacing_x > 0 and spacing_y > 0:
        print(f"Found {len(centers)} grid centers")
        result, pieces = detect_pieces(img, centers, spacing_x, spacing_y)
        print(f"Detected {len(pieces)} pieces on the board")
        
        # Print piece positions
        for piece in pieces:
            row, col = piece["position"]
            color = piece["color"]
            print(f"{color} piece at position {row},{col}")
    else:
        print("Could not detect chess grid reliably")

    # Show result
    cv2.imshow('Chess Grid and Piece Detection', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return centers, pieces if 'pieces' in locals() else []


# Function to get board representation as a 2D array
def get_board_representation(pieces, size=8):
    """
    Convert detected pieces to a 2D board representation
    Empty squares are represented as None
    Pieces are represented by their color
    """
    # Initialize empty board
    board = [[None for _ in range(size)] for _ in range(size)]
    
    # Fill in the pieces
    for piece in pieces:
        row, col = piece["position"]
        if 0 <= row < size and 0 <= col < size:
            board[row][col] = piece["color"]
    
    return board


# Print board representation
def print_board(board):
    """
    Print a visual representation of the chess board
    """
    print("  a b c d e f g h")
    print(" +-----------------+")
    for i, row in enumerate(board):
        print(f"{8-i}|", end=" ")
        for square in row:
            if square is None:
                print(".", end=" ")
            elif square == "white":
                print("W", end=" ")
            else:  # black
                print("B", end=" ")
        print(f"|{8-i}")
    print(" +-----------------+")
    print("  a b c d e f g h")



if __name__ == "__main__":
    image = process_image("pawn.png")
    centers, pieces = detect_chess_board_with_pieces(image)
    
    if pieces:
        # Get and print board representation
        board = get_board_representation(pieces)
        print("\nDetected board configuration:")
        print_board(board)
