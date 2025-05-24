import cv2
import numpy as np
import matplotlib.pyplot as plt
import computer_vision.depreciated.python_chess2 as chs
#from computer_vision import python_chess3 as chs
from computer_vision import square_processing as sp

def analyze_chessboard(image_path, auto_calib=True, corners=[], DEBUG=False):
    """
    Analyze a chessboard image and return a 2D array representing the board state.
    
    Args:
        image_path (str): Path to the chessboard image
        
    Returns:
        np.ndarray: 8x8 array where 0=empty, 1=white piece, -1=black piece
    """
    # Read the image
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Could not read image at {image_path}")
    
  
    #cv2.imshow("Original Chessboard Image", img)  # Display the image in a window
# Initialize approx
    approx = None
    
    
    if auto_calib == True:
        approx, success = detect_blue_corners(image_path)
        if not success:
            approx = select_points(image_path)
    elif corners is not None and len(corners) > 0:
        approx = corners
    else:
        approx = select_points(image_path)


    # Order the points [top-left, top-right, bottom-right, bottom-left]
    pts = np.array(approx, dtype=np.float32).reshape(-1, 2)
    s = pts.sum(axis=1)
    ordered_pts = np.zeros((4, 2), dtype=np.float32)
    ordered_pts[0] = pts[np.argmin(s)]  # Top-left
    ordered_pts[2] = pts[np.argmax(s)]  # Bottom-right
    diff = np.diff(pts, axis=1)
    ordered_pts[1] = pts[np.argmin(diff)]  # Top-right
    ordered_pts[3] = pts[np.argmax(diff)]  # Bottom-left


    if None in ordered_pts or any(np.isnan(pt).any() for pt in ordered_pts):
        raise ValueError("Corner detection failed — one or more points are missing or invalid.")

    # Create a copy of the original image to draw points on
    img_with_points = img.copy()

    # Draw the corner points on the original image
    colors = [(0, 0, 255),   # Red for top-left
            (0, 255, 0),   # Green for top-right
            (255, 0, 0),   # Blue for bottom-right
            (255, 255, 0)] # Cyan for bottom-left

    point_labels = ["TL", "TR", "BR", "BL"]

    for i, point in enumerate(ordered_pts):
        x, y = int(point[0]), int(point[1])
        cv2.circle(img_with_points, (x, y), 10, colors[i], -1)
        cv2.putText(img_with_points, point_labels[i], (x+10, y+10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors[i], 2)

    # Display the original image with corner points
    cv2.imshow("Original with Corners", img_with_points)

    # Get width and height of the chessboard
    width = int(max(
        np.linalg.norm(ordered_pts[0] - ordered_pts[1]),
        np.linalg.norm(ordered_pts[2] - ordered_pts[3])
    ))
    height = int(max(
        np.linalg.norm(ordered_pts[0] - ordered_pts[3]),
        np.linalg.norm(ordered_pts[1] - ordered_pts[2])
    ))

    # Define the destination points for perspective transform
    dst = np.array([
        [0, 0],
        [width - 1, 0],
        [width - 1, height - 1],
        [0, height - 1]
    ], dtype=np.float32)

    # Calculate the perspective transform matrix and apply it
    matrix = cv2.getPerspectiveTransform(ordered_pts, dst)
    warped = cv2.warpPerspective(img, matrix, (width, height))

    # Create a copy of the warped image to draw points on
    warped_with_points = warped.copy()

    # Draw the corner points on the warped image
    # Since we know exactly where these points should be after transformation
    warped_corners = [
        (0, 0),                  # Top-left
        (width - 1, 0),          # Top-right
        (width - 1, height - 1), # Bottom-right
        (0, height - 1)          # Bottom-left
    ]

    for i, point in enumerate(warped_corners):
        x, y = point
        cv2.circle(warped_with_points, (x, y), 10, colors[i], -1)
        cv2.putText(warped_with_points, point_labels[i], (x+10, y+10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors[i], 2)

    # Display the warped chessboard with corner points
    #cv2.imshow("Corrected Chessboard with Corners", warped_with_points)
    # Step 2: Divide the chessboard into 64 squares
    square_width = width // 8
    square_height = height // 8
    
    # Create a figure to display all squares
    
    fig, axes = plt.subplots(8, 8, figsize=(15, 15))
    if not DEBUG:
        plt.close(fig)
    
    # Initialize the board representation
    board = np.zeros((8, 8), dtype=np.int8)
    
    # Process each square
    square_images = []
    for row in range(8):
        for col in range(8):
            # Extract square
            """
            y1 = row * square_height
            y2 = (row + 1) * square_height
            x1 = col * square_width
            x2 = (col + 1) * square_width
            
            square = warped[y1:y2, x1:x2].copy()
            """
           # Define base square boundaries
            base_x1 = col * square_width
            base_y1 = row * square_height
            base_x2 = (col + 1) * square_width
            base_y2 = (row + 1) * square_height

            # Padding to zoom out (e.g. 25% of square size)
            padding_x = square_width // 20
            padding_y = square_height // 20

            # Expand the boundaries
            x1 = max(base_x1 - padding_x, 0)
            y1 = max(base_y1 - padding_y, 0)
            x2 = min(base_x2 + padding_x, warped.shape[1])
            y2 = min(base_y2 + padding_y, warped.shape[0])

            # Extract expanded square region
            square = warped[y1:y2, x1:x2].copy()


            square_images.append(square)
            
            # Detect if there's a piece and its color
            #is_piece, piece_color = detect_piece_and_color(square)
            is_piece, piece_color, z = sp.detect_chess_piece_colour(square)
            
            
            # Update the board array
            if is_piece:
                if piece_color == "white":
                    board[row, col] = 1
                else:  # piece_color == "black"
                    board[row, col] = -1
            
            # Display the square with its classification
            square_rgb = cv2.cvtColor(square, cv2.COLOR_BGR2RGB)
            axes[row, col].imshow(square_rgb)
            axes[row, col].set_title(f"{'Empty' if not is_piece else piece_color}", fontsize=8)
            axes[row, col].axis('off')
    
    #plt.tight_layout()
    #plt.show()
    
    # Display the final board state
    fig, ax = plt.subplots(figsize=(10, 10))
    cmap = plt.cm.colors.ListedColormap(['darkgrey','white', 'lightgrey', ])
    bounds = [-1.5, -0.5, 0.5, 1.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
    ax.imshow(board, cmap=cmap, norm=norm)
    
    # Add piece labels
    for row in range(8):
        for col in range(8):
            if board[row, col] == 0:
                text = ""
            elif board[row, col] == 1:
                text = "W"
            else:
                text = "B"
            ax.text(col, row, text, ha='center', va='center', fontsize=16)
    
    # Add row and column labels
    columns = 'ABCDEFGH'
    rows = '87654321'
    for i in range(8):
        ax.text(i, -0.5, columns[i], ha='center', fontsize=12)
        ax.text(-0.5, i, rows[i], va='center', fontsize=12)
    
    ax.set_xticks(np.arange(8) - 0.5, minor=True)
    ax.set_yticks(np.arange(8) - 0.5, minor=True)
    ax.grid(which='minor', color='black', linestyle='-', linewidth=2)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("Chess Board State (0=Empty, 1=White, 2=Black)")
    plt.tight_layout()
    plt.show()
    
    return board, approx

def detect_piece_and_color(square_image):
    """
    Detect if there's a chess piece in the square and determine its color.
    
    Args:
        square_image (np.ndarray): Image of a single chess square
        
    Returns:
        tuple: (is_piece, color)
            - is_piece (bool): True if a piece is detected
            - color (str): "white" or "black" (or None if no piece)
    """
    # Convert to grayscale
    gray = cv2.cvtColor(square_image, cv2.COLOR_BGR2GRAY)
    
    # Get square dimensions
    height, width = gray.shape
    
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    #blurred = cv2.GaussianBlur(gray, (45, 45), 0)
    """
    # Use Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)
    
    # Dilate edges to connect broken lines
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(edges, kernel, iterations=1)
   """
  # Apply thresholding to separate piece from background
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                  cv2.THRESH_BINARY_INV, 11, 2)
  
    
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter contours by size
    min_area = (width * height) * 0.15  # At least 5% of square
    max_area = (width * height) * 0.95   # At most 90% of square
    
    # Variable to store the largest valid contour
    largest_contour = None
    largest_area = 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area < area < max_area and area > largest_area:
            largest_contour = contour
            largest_area = area
    
    if largest_contour is None:
        return False, None
    
    # Create a mask for the piece
    piece_mask = np.zeros_like(gray)
    cv2.drawContours(piece_mask, [largest_contour], -1, 255, -1)
    
    # Extract piece pixels
    piece_pixels = gray[piece_mask > 0]
    
    if len(piece_pixels) == 0:
        return False, None
    
    # Calculate average brightness of the piece
    avg_brightness = np.mean(piece_pixels)
    
    # Determine color based on brightness
    # We need to consider the square color
    # Check if square is light or dark
    corner_brightness = np.mean([
        gray[5, 5], gray[5, width-5], 
        gray[height-5, 5], gray[height-5, width-5]
    ])
    
    # Adjust brightness threshold based on square color
    if corner_brightness > 128:  # Light square
        color = "white" if avg_brightness > 110 else "black"
    else:  # Dark square
        color = "white" if avg_brightness > 90 else "black"
    
    return True, color

def detect_blue_corners(image_path, show_result=False):
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
    ordered_points = []

    for lbl in ordered_labels:
        pt = found_corners.get(lbl)
        if pt is None:
            ordered_points.append((np.nan, np.nan))
        else:
            ordered_points.append(pt)

    success = all(
        isinstance(pt, (list, tuple)) and
        len(pt) == 2 and
        not np.isnan(pt[0]) and
        not np.isnan(pt[1])
        for pt in ordered_points
    )



    if show_result:
        for point in ordered_points:
            cv2.circle(img, point, 10, (0, 255, 0), 2)
        print("Ordered Corners (TL, TR, BL, BR):", ordered_points)
        cv2.imshow("Detected Corners", img)
        cv2.imshow("Blue Mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    
    return ordered_points, success

def select_points(image_path, num_points=4, max_height=900, max_width=1600):
    """
    Opens an image and allows the user to select points by clicking.
    Resizes the image to fit on screen if necessary.
    
    Parameters:
    image_path (str): Path to the image file
    num_points (int): Number of points to select (default is 4)
    max_height (int): Maximum height for display (default 900)
    max_width (int): Maximum width for display (default 1600)
    
    Returns:
    numpy.ndarray: Array of (x, y) coordinates of selected points in original image scale
    """
    # Read the image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not read image at {image_path}")
        return None
    
    # Get original dimensions
    original_height, original_width = img.shape[:2]
    
    # Calculate scaling factor to fit on screen
    scale_width = min(1.0, max_width / original_width)
    scale_height = min(1.0, max_height / original_height)
    scale = min(scale_width, scale_height)
    
    # Resize image if needed
    if scale < 1.0:
        new_width = int(original_width * scale)
        new_height = int(original_height * scale)
        img = cv2.resize(img, (new_width, new_height))
        print(f"Image resized from {original_width}x{original_height} to {new_width}x{new_height}")
    
    # Make a copy of the image for drawing
    img_copy = img.copy()
    
    # Flag to track if scaling was applied
    scaled = scale < 1.0
    
    # List to store the selected points
    points = []
    
    # Mouse callback function
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Add the point to our list
            points.append((x, y))
            
            # Draw a circle at the clicked position
            cv2.circle(img_copy, (x, y), 5, (0, 255, 0), -1)
            
            # Display the point number
            cv2.putText(img_copy, str(len(points)), (x+10, y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Update the displayed image
            cv2.imshow("Image", img_copy)
            
            # If we've collected all points, print them
            if len(points) == num_points:
                print(f"All {num_points} points selected: {points}")
    
    # Create a window and set the mouse callback
    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", mouse_callback)
    
    # Display the image
    cv2.imshow("Image", img)
    
    # Wait until all points are selected or user presses ESC
    while len(points) < num_points:
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC key
            break
    
    # Close all windows
    cv2.destroyAllWindows()
    
    # If we scaled the image, convert points back to original scale
    if scaled:
        scale_factor = 1.0 / scale
        original_points = [(int(x * scale_factor), int(y * scale_factor)) for x, y in points]
        return np.array(original_points)
    
    return np.array(points)

def main(img1, img2):
    analyzer = chs.game()
    current_board = []
    corners = []
    i = 1
    results = ""
    while i ==1:
        if current_board is None or len(current_board) == 0:
            current_board, corners  = analyze_chessboard(img1, auto_calib=False)
            print(current_board)
        else:
            board_array, _ = analyze_chessboard(img2, auto_calib=False, corners=corners)
            #print("Board representation (0=empty, 1=white, 2=black):")

            results = chs.analyze_binary_board_state(analyzer, board_array)

            # Access the analysis results
            if results["detected_move"]:
                print(f"Move detected: {results['detected_move']}")
            else:
                print("No valid move detected")

            # The PGN so far
            print(results["pgn"])
            i = 2
    return results['detected_move']
def main2():
    current_board, corners  = analyze_chessboard("piecereal/colourboard3.png", auto_calib=True,DEBUG=True)
    



if __name__ == "__main__":
    img1 = "chessboards/screen1.png"
    img2 = "chessboards/screen2.png"
    #move = main(img1, img2)
    main2()
    #print(move)
    
    """
    print(board_array)
    change_squares = np.bitwise_xor(current_board, board_array)
    print("changed squares \n", change_squares)
    start_square = np.bitwise_and(current_board, change_squares)
    print("start square \n", start_square)

    new_occupied = np.bitwise_or(board_array, change_squares)
    new_occupied = np.bitwise_xor(new_occupied, current_board)
    print("end square \n", new_occupied)
    """