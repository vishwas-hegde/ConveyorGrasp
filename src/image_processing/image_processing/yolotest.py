import torch
from ultralytics import YOLO
import matplotlib.pyplot as plt
import cv2
from PIL import Image
import numpy as np

# Load the YOLO model (change 'yolov8n.pt' to your specific YOLO model if needed)
model = YOLO('yolov8n.pt')  # You can replace with 'yolov8s.pt', 'yolov8m.pt', etc.

# Function to plot the image with bounding boxes
def plot_with_bboxes(image, results):
    plt.figure(figsize=(10, 10))
    plt.imshow(image)
    
    # Draw bounding boxes
    ax = plt.gca()
    for result in results:
        x1, y1, x2, y2 = result['box']
        label = result['label']
        confidence = result['confidence']
        
        # Create rectangle for the bounding box
        rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1, linewidth=2, edgecolor='red', facecolor='none')
        ax.add_patch(rect)
        
        # Add label and confidence
        ax.text(
            x1, y1 - 10,
            f'{label} {confidence:.2f}',
            color='red',
            fontsize=12,
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='red')
        )
    
    plt.axis('off')
    plt.show()

# Function to run inference and format results
def run_inference(image_path):
    # Load the image
    image = Image.open(image_path)
    original_image = np.array(image)
    
    # Run YOLO inference
    results = model(image_path)[0]  # Take the first prediction
    
    # Parse the bounding boxes, labels, and confidences
    parsed_results = []
    for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
        parsed_results.append({
            'box': box.tolist(),
            'label': model.names[int(cls)],  # Convert class index to label
            'confidence': conf.item()
        })
    
    # Plot the results
    plot_with_bboxes(original_image, parsed_results)

# Path to your input image
image_path = 'path/to/your/image.jpg'  # Replace with the path to your image

# Run the inference
run_inference(image_path)
