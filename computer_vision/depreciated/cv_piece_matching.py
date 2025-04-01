import os
import cv2
import numpy as np

def process_image(image_path):
    img = cv2.imread(image_path)

    if img is None:
        raise FileNotFoundError(f"Error: Could not read image '{image_path}'. Check the path.")

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection
    canny = cv2.Canny(gray, 20, 255)
    return canny

def match_piece(img1_path, img2_path):
    # Load and process images
    img1 = process_image(img1_path)
    img2 = process_image(img2_path)

    # Initialize ORB detector
    orb = cv2.ORB_create()

    # Detect keypoints and compute descriptors
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    # Ensure descriptors exist before matching
    if des1 is None or des2 is None:
        print(f"Skipping {img1_path} and {img2_path}: No descriptors found.")
        return 0, None  # No similarity

    # Create BFMatcher object with Hamming distance
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors
    matches = bf.match(des1, des2)

    # Sort matches based on distance (lower is better)
    matches = sorted(matches, key=lambda x: x.distance)

    # Compute similarity score
    num_keypoints = min(len(kp1), len(kp2))
    num_matches = len(matches)
    similarity = (num_matches / num_keypoints) * 100 if num_keypoints > 0 else 0  # Percentage similarity

    # Draw matches
    img_matches = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # Display the result
    cv2.imshow('Feature Matching', img_matches)
    print(f"Similarity Score: {similarity:.2f}%")
    return similarity, img2  # Return similarity and the second image for display

def resize_images_to_same_height(img1, img2):
    # Get the height of the images
    height1, width1 = img1.shape[:2]
    height2, width2 = img2.shape[:2]

    # Calculate the scale factor to resize the second image to the first image's height
    scale_factor = height1 / height2

    # Resize the second image to the same height as the first image
    img2_resized = cv2.resize(img2, (int(width2 * scale_factor), height1))
    
    return img1, img2_resized

def main():
    # Define the reference (query) image
    reference_image = "pieces/queen.png"

    # Get the directory where the script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Construct full path to the "pawns" directory
    pieces_dir = os.path.join(script_dir, "pieces")

    # Check if the directory exists
    if not os.path.exists(pieces_dir):
        raise FileNotFoundError(f"Error: Folder '{pieces_dir}' not found.")

    highest_similarity = 0  # Initialize highest similarity score
    best_match_image = None  # To store the best match image

    # Loop through all images in the "pawns" folder
    for filename in os.listdir(pieces_dir):
        image_path = os.path.join(pieces_dir, filename)

        # Skip the reference image
        if filename == os.path.basename(reference_image):
            continue

        # Ensure it's an image file
        if not filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            continue

        print(f"Comparing {reference_image} to {filename}...")

        # Run the match function
        similarity, img2 = match_piece(reference_image, image_path)

        # Track the highest similarity and corresponding image
        if similarity > highest_similarity:
            highest_similarity = similarity
            best_match_image = img2
            best_match_filename = filename

    # Display the reference image and the best matching image
    if best_match_image is not None:
        print(f"\nHighest similarity found with {best_match_filename} ({highest_similarity:.2f}%)")
        pawn_img = cv2.imread(reference_image)

        # Convert best match image to 3 channels if it's grayscale
        if len(best_match_image.shape) == 2:  # Grayscale image has 2 dimensions
            best_match_image = cv2.cvtColor(best_match_image, cv2.COLOR_GRAY2BGR)

        # Resize images to have the same height
        pawn_img_resized, best_match_resized = resize_images_to_same_height(pawn_img, best_match_image)

        # Display the reference and the best match image side by side
        combined_image = np.hstack((pawn_img_resized, best_match_resized))
        cv2.imshow("Best Match", combined_image)

    # Wait for a key press to close windows
    print("Press any key to exit...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
