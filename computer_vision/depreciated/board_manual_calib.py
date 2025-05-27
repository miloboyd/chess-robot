import cv2
import numpy as np

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

# Example usage
if __name__ == "__main__":
    # Replace with your image path
    image_path = "chessboards/chesscom.png"
    
      # Select 4 points
    coordinates = select_points(image_path)
    
    # Print the coordinates
    if coordinates is not None and coordinates.shape[0] == 4:
        print(f"Coordinates array: \n{coordinates}")
   