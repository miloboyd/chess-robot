
from inference_sdk import InferenceHTTPClient

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="q8JpLsQv6Y0C2oBu7Pyd"
)

result = CLIENT.infer("pawns/pawn.png", model_id="chesspiece-detection-twhpv/5")
#print(result["predictions"][0]["class"])
print(result)

"""
import cv2
import numpy as np
from inference_sdk import InferenceHTTPClient

# Initialize Inference Client
CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="q8JpLsQv6Y0C2oBu7Pyd"
)

# Load Image
image_path = "pawns/pawn.png"
image = cv2.imread(image_path)

# ✅ Step 1: Preprocess Image for Better Detection
def preprocess_image(image):
    # Convert to LAB color space for contrast enhancement
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    l = clahe.apply(l)

    # Merge LAB channels back
    lab = cv2.merge((l, a, b))
    enhanced_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


# Preprocess Image
#image = preprocess_image(image)

# ✅ Step 2: Run Inference with Tuned Parameters
result = CLIENT.infer(
    image_path,
    model_id="chesspiece-detection-twhpv/5?confidence=0.5&iou=0.5"  # ✅ Correct format
)

# ✅ Step 3: Draw Bounding Boxes on Image
for prediction in result["predictions"]:
    x, y, w, h = int(prediction["x"]), int(prediction["y"]), int(prediction["width"]), int(prediction["height"])
    label = prediction["class"]
    confidence = prediction["confidence"]

    # Define bounding box coordinates
    x1, y1 = x - w // 2, y - h // 2
    x2, y2 = x + w // 2, y + h // 2

    # Draw rectangle
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Put label text
    label_text = f"{label} ({confidence:.2f})"
    cv2.putText(image, label_text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# ✅ Step 4: Display the Image
cv2.imshow("Chess Piece Detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
"""