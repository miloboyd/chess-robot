import os
import cv2
import numpy as np


def process_image(image_path):
    img = cv2.imread(image_path)

    if img is None:
        raise FileNotFoundError(f"Error: Could not read image '{image_path}'. Check the path.")

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #_, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    # Apply Canny edge detection
    canny = cv2.Canny(gray, 20, 255)
    
    return canny


def detect_contours(image_path):
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Unable to load image.")
        return
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply thresholding to binarize the image
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    
    cv2.imshow("binary",thresh)
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter out rectangular contours
    filtered_contours = []
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        if len(approx) != 4:  # Ignore rectangles (4-sided contours)
            filtered_contours.append(contour)
    
    # Draw filtered contours on the original image
    cv2.drawContours(image, filtered_contours, -1, (0, 255, 0), 2)
    
     # Print the number of contours found
    print(f"Number of contours found: {len(contours)}")
    # Display the result
    cv2.imshow("Contours Detected", image)
    
def detect_contours2(image_path):
 
    # Read the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not read image at {image_path}")
        return None
    
    # Create a copy for drawing
    contour_image = image.copy()
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply bilateral filter to reduce noise while preserving edges
    filtered = cv2.bilateralFilter(gray, 11, 17, 17)

    # Use adaptive thresholding and Canny edge detection
    thresh = cv2.adaptiveThreshold(filtered, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                  cv2.THRESH_BINARY_INV, 11, 2)
    edges = cv2.Canny(filtered, 10, 100)
    combined = cv2.bitwise_or(thresh, edges)
 
    # Dilate to connect broken edge segments
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(combined, kernel, iterations=1)
    
    # Find contours
    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter contours: remove too small and rectangular shapes
    min_contour_area = 100  # Adjust based on your image
    non_rectangle_contours = []
    rectangular_contours = []  # For display/debugging
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_contour_area:
            continue
            
        # Check if shape is rectangular
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
        
        # If contour has 4 vertices, it's likely a rectangle/square
        if len(approx) == 4:
            # Additional check for squareness/rectangularity
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            # Check if it's square-like (aspect ratio close to 1)
            # or rectangle-like (sides are relatively straight)
            rect_area = w * h
            contour_area = cv2.contourArea(cnt)
            area_ratio = float(contour_area) / rect_area
            
            # If shape is rectangular (area ratio close to 1 means shape fills rectangle well)
            if area_ratio > 0.85:
                rectangular_contours.append(cnt)
                continue
                
        # If we get here, contour is not a rectangle/square
        non_rectangle_contours.append(cnt)
    
    # Draw non-rectangular contours
    cv2.drawContours(contour_image, non_rectangle_contours, -1, (0, 255, 0), 2)
    
    for cnt in contours:

        M = cv2.moments(cnt)

        cx = int(M['m10'] / M['m00'])

        cy = int(M['m01'] / M['m00'])

        

        # Draw a circle at the center of mass

        cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1) 

    # Display results if requested
  
    # Create rectangle contour image for debugging
    rect_image = image.copy()
    cv2.drawContours(rect_image, rectangular_contours, -1, (0, 0, 255), 2)
    
    print(f"Found {len(non_rectangle_contours)} non-rectangular contours")
    print(f"Excluded {len(rectangular_contours)} rectangular contours")
    
    cv2.imshow('Original', image)
    cv2.imshow('Combined Edges', combined)
    cv2.imshow('Excluded Rectangles', rect_image)
    cv2.imshow('Non-Rectangular Contours', contour_image)
    return contours


def find_center_of_mass(image):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 127, 255, 0)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)



    for cnt in contours:

        M = cv2.moments(cnt)

        cx = int(M['m10'] / M['m00'])

        cy = int(M['m01'] / M['m00'])

        

        # Draw a circle at the center of mass

        cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1) 



    return image


def detect_chess_piece(image_path):
    """
    Detect if there is a chess piece in the given image and determine its color.
    
    Args:
        image_path (str): Path to the image file
        
    Returns:
        tuple: (piece_detected, color, output_image)
            - piece_detected (bool): True if a chess piece is detected
            - color (str): "white", "black", or None if no piece detected
            - output_image (np.ndarray): Processed image with contours
    """
    # Read the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not read image at {image_path}")
        return False, None, None
    
    # Create a copy for visualization
    output = image.copy()
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply slight blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply thresholding to separate piece from background
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                  cv2.THRESH_BINARY_INV, 11, 2)
    
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter contours by size
    height, width = image.shape[:2]
    min_area = (width * height) * 0.05  # At least 5% of image
    max_area = (width * height) * 0.95  # At most 95% of image
    
    piece_detected = False
    piece_color = None
    piece_mask = None
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            # Create a mask for the detected piece
            piece_mask = np.zeros_like(gray)
            cv2.drawContours(piece_mask, [contour], -1, 255, -1)
            
            # Draw contour on output image
            cv2.drawContours(output, [contour], -1, (0, 255, 0), 2)
            piece_detected = True
            
            # Only analyze the largest contour if multiple are found
            break
    
    if piece_detected and piece_mask is not None:
        # Create a mask to extract only the piece pixels
        masked_piece = cv2.bitwise_and(gray, gray, mask=piece_mask)
        
        # Calculate average brightness of the piece
        # (ignore zero values which are from the background)
        non_zero_pixels = masked_piece[piece_mask > 0]
        if len(non_zero_pixels) > 0:
            avg_brightness = np.mean(non_zero_pixels)
            
            # Determine if piece is black or white based on brightness
            # The threshold may need adjustment based on lighting conditions
            if avg_brightness > 128:  # Assuming 8-bit grayscale (0-255)
                piece_color = "white"
                cv2.putText(output, "WHITE", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                piece_color = "black"
                cv2.putText(output, "BLACK", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            print(f"Chess piece detected! Color: {piece_color} (Avg brightness: {avg_brightness:.1f})")
        else:
            print("Chess piece detected, but couldn't determine color.")
    else:
        print("No chess piece detected.")
    
    # Display images (optional)
    cv2.imshow('Original', image)
    cv2.imshow('Thresholded', thresh)
    if piece_mask is not None:
        cv2.imshow('Piece Mask', piece_mask)
    cv2.imshow('Detection Result', output)
   
    return piece_detected, piece_color, output




if __name__ == "__main__":
    img_path = "pieces/empty2.png"
    img = cv2.imread(img_path)
    #cv2.imshow("original", img)
    #det = detect_contours2(img_path)
    res = detect_chess_piece(img_path)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
   
