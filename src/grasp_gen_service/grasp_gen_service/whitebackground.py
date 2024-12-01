import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
image = cv2.imread('/home/vishwas/RBE595-vbm/src/grasp_gen_service/grasp_gen_service/Figure_1.png')

# Convert to RGB (since OpenCV loads in BGR format)
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

print("Image shape:", image_rgb.shape)

# Create a mask using the GrabCut algorithm for object segmentation
mask = np.zeros(image_rgb.shape[:2], np.uint8)

# Define a rectangle that roughly includes the object (Coke can)
# Values (x, y, width, height) - Adjust based on the image content
rect = (120, 70, 260, 300)

# Create the background and foreground models (required by GrabCut)
bgd_model = np.zeros((1, 65), np.float64)
fgd_model = np.zeros((1, 65), np.float64)

# Apply the GrabCut algorithm
cv2.grabCut(image_rgb, mask, rect, bgd_model, fgd_model, 5, cv2.GC_INIT_WITH_RECT)

# Modify the mask: set sure foreground (1) and probable foreground (3) as 1, else 0
mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')

# Extract the object from the image
object_extracted = image_rgb * mask2[:, :, np.newaxis]

# Create a white background
white_background = np.ones_like(image_rgb, dtype=np.uint8) * 255

# Combine the extracted object with the white background
final_image = white_background * (1 - mask2[:, :, np.newaxis]) + object_extracted

print("Image shape:", final_image.shape)

# Plot the result
plt.figure(figsize=(8,8))
plt.imshow(final_image)
plt.title("Coke Can on White Background")
plt.axis('off')
plt.show()

