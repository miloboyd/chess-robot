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

    print(f"Similarity Score: {similarity:.2f}%")
    return similarity, img_matches  # Return similarity and the feature matching image

def resize_images_to_same_height(img_list):
    """ Resizes all images to the height of the first image while maintaining aspect ratio """
    if not img_list:
        return []

    base_height = img_list[0].shape[0]
    resized_images = []

    for img in img_list:
        height, width = img.shape[:2]
        scale_factor = base_height / height
        new_width = int(width * scale_factor)
        resized_img = cv2.resize(img, (new_width, base_height))
        resized_images.append(resized_img)

    return resized_images

def main():
    # Define the reference (query) image
    reference_image = "pawns/pawn2.png"

    # Get the directory where the script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Construct full path to the "pawns" directory
    pawns_dir = os.path.join(script_dir, "pieces")

    # Check if the directory exists
    if not os.path.exists(pawns_dir):
        raise FileNotFoundError(f"Error: Folder '{pawns_dir}' not found.")

    comparison_images = []  # To store all matched images
    similarity_scores = []  # To track similarity scores for sorting

    # Loop through all images in the "pawns" folder
    for filename in os.listdir(pawns_dir):
        image_path = os.path.join(pawns_dir, filename)

        # Skip the reference image
        if filename == os.path.basename(reference_image):
            continue

        # Ensure it's an image file
        if not filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            continue

        print(f"Comparing {reference_image} to {filename}...")

        # Run the match function
        similarity, img_match = match_piece(reference_image, image_path)

        if img_match is not None:
            similarity_scores.append((similarity, img_match))
    
    # Sort results by similarity score (highest first)
    similarity_scores.sort(reverse=True, key=lambda x: x[0])

    # Collect sorted images
    comparison_images = [img for _, img in similarity_scores]

    # Resize all images to have the same height for stacking
    resized_images = resize_images_to_same_height(comparison_images)

    # Stack all images horizontally if there are any
    if resized_images:
        combined_image = np.hstack(resized_images)
        cv2.imshow("All Comparisons", combined_image)

    # Wait for a key press to close windows
    print("Press any key to exit...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
