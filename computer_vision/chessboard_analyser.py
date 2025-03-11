import cv2
import numpy as np
import matplotlib.pyplot as plt

def analyze_chessboard(image_path):
    """
    Analyze a chessboard image and return a 2D array representing the board state.
    
    Args:
        image_path (str): Path to the chessboard image
        
    Returns:
        np.ndarray: 8x8 array where 0=empty, 1=white piece, 2=black piece
    """
    # Read the image
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Could not read image at {image_path}")
    
  
    cv2.imshow("Original Chessboard Image", img)  # Display the image in a window

    # Step 1: Find the chessboard corners
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Preprocessing to enhance the contrast of the chessboard
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Use adaptive thresholding to handle different lighting conditions
    thresh = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find the largest contour (assuming it's the chessboard)
    board_contour = max(contours, key=cv2.contourArea)
    
    
    # Get the corner points of the board
    peri = cv2.arcLength(board_contour, True)
    approx = cv2.approxPolyDP(board_contour, 0.02 * peri, True)
    
    # If we don't get exactly 4 corners, try to use the minimum area rectangle
    if len(approx) != 4:
        rect = cv2.minAreaRect(board_contour)
        box = cv2.boxPoints(rect)
        approx = np.int0(box)
    
    # Order the points [top-left, top-right, bottom-right, bottom-left]
    pts = np.array([p[0] for p in approx], dtype=np.float32)
    s = pts.sum(axis=1)
    ordered_pts = np.zeros((4, 2), dtype=np.float32)
    ordered_pts[0] = pts[np.argmin(s)]  # Top-left
    ordered_pts[2] = pts[np.argmax(s)]  # Bottom-right
    diff = np.diff(pts, axis=1)
    ordered_pts[1] = pts[np.argmin(diff)]  # Top-right
    ordered_pts[3] = pts[np.argmax(diff)]  # Bottom-left
    
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
    
    # Display the warped chessboard
    
    cv2.imshow("Corrected Chessboard Image", warped)  # Display the image in a window

    # Step 2: Divide the chessboard into 64 squares
    square_width = width // 8
    square_height = height // 8
    
    # Create a figure to display all squares
    fig, axes = plt.subplots(8, 8, figsize=(15, 15))
    
    # Initialize the board representation
    board = np.zeros((8, 8), dtype=np.int8)
    
    # Process each square
    square_images = []
    for row in range(8):
        for col in range(8):
            # Extract square
            y1 = row * square_height
            y2 = (row + 1) * square_height
            x1 = col * square_width
            x2 = (col + 1) * square_width
            
            square = warped[y1:y2, x1:x2].copy()
            square_images.append(square)
            
            # Detect if there's a piece and its color
            is_piece, piece_color = detect_piece_and_color(square)
            
            # Update the board array
            if is_piece:
                if piece_color == "white":
                    board[row, col] = 1
                else:  # piece_color == "black"
                    board[row, col] = 2
            
            # Display the square with its classification
            square_rgb = cv2.cvtColor(square, cv2.COLOR_BGR2RGB)
            axes[row, col].imshow(square_rgb)
            axes[row, col].set_title(f"{'Empty' if not is_piece else piece_color}", fontsize=8)
            axes[row, col].axis('off')
    
    #plt.tight_layout()
    #plt.show()
    
    # Display the final board state
    fig, ax = plt.subplots(figsize=(10, 10))
    cmap = plt.cm.colors.ListedColormap(['white', 'lightgrey', 'darkgrey'])
    bounds = [-0.5, 0.5, 1.5, 2.5]
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
    
    return board

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
    min_area = (width * height) * 0.05  # At least 5% of square
    max_area = (width * height) * 0.9   # At most 90% of square
    
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

# Example usage
if __name__ == "__main__":
    # Replace with your image path
    board_array = analyze_chessboard("chessboards/chesscom2.png")
    print("Board representation (0=empty, 1=white, 2=black):")
    print(board_array)