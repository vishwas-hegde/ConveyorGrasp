"""
Script to load the GRConvNet model and process the RGB-D data
"""
import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import numpy as np
import torch
from grasp import detect_grasps
import post_process
import cv2
from imageio import imread
import matplotlib.pyplot as plt

class GRConvNet_Grasp():

      def __init__(self):
            current_dir = os.getcwd() + '/src/grasp_gen_service/grasp_gen_service'
            model_path = current_dir + '/trained_models/GRConvNet/epoch_19_iou_0.98'
            self.network = torch.load(model_path)
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.network.to(self.device)

      def center_crop(self, img, crop_width, crop_height):
            """
            Center crop the image
            """
            height, width = img.shape[:2]
            center_x, center_y = width // 2, height // 2

            x1 = center_x - crop_width // 2
            y1 = center_y - crop_height // 2
            x2 = center_x + crop_width // 2
            y2 = center_y + crop_height // 2

            return img[y1:y2, x1:x2]
      
      def process_data(self, rgb_img, depth_img, crop, detections):
            """
            Process input data using GRConvNet with proper coordinate transformation.
            Handles coordinate transformations and ensures grasp plotting is accurate.
            """
            xmin, ymin, xmax, ymax = detections['detections'][0]["bbox"]
            
            center_x = (xmin + xmax) / 2
            center_y = (ymin + ymax) / 2
            # print(" in plpot data yolo centers-> center_x, center_y =", center_x, center_y)
            x_padding = 70
            y_padding = 70
            
            padded_xmin = max(0, xmin - x_padding)
            padded_ymin = max(0, ymin - y_padding)
            padded_xmax = min(640, xmax + 10)
            padded_ymax = min(480, ymax + 70)

            # print("Original crop:", [xmin, ymin, xmax, ymax])
            # print("Padded crop:", [padded_xmin, padded_ymin, padded_xmax, padded_ymax])

            rgb_img_cropped = rgb_img[padded_ymin:padded_ymax, padded_xmin:padded_xmax]
            depth_img_cropped = depth_img[padded_ymin:padded_ymax, padded_xmin:padded_xmax]

            crop_height, crop_width = rgb_img_cropped.shape[:2]
            # print(f"Padded crop dimensions: {crop_width} and {crop_height}")

            depth_img_cropped = self.process_depth_image(depth_img_cropped)
            # rgb_img_resized = cv2.resize(rgb_img_cropped, (224, 224))
            # depth_img_resized = cv2.resize(depth_img_cropped, (224, 224))

            rgb_input = rgb_img_cropped.astype(np.float32) / 255.0
            rgb_input -= rgb_input.mean()
            rgb_input = rgb_input.transpose((2, 0, 1))
            depth_input = np.clip((depth_img_cropped - depth_img_cropped.mean()), -1, 1)

            x = np.concatenate((np.expand_dims(depth_input, 0), rgb_input), 0)
            x = torch.from_numpy(x.astype(np.float32)).to(self.device)
            # print("Input Image size", x.shape)

            with torch.no_grad():
                  x = x.unsqueeze(0)
                  output = self.network.predict(x)
            # print("Output Image size", output['pos'].shape)
            q_img, ang_img, width_img = post_process.post_process_output(
                  output['pos'], output['cos'], output['sin'], output['width']
            )
            # print("q_img.shape, ang_img.shape, width_img.shape = ", q_img.shape, ang_img.shape, width_img.shape)

            gs = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=5)
            g_centers_ = []
            for g in gs:
                  # print(f"Grasp in 224x224 - Center: {g.center}, Angle: {g.angle}, "
                  #       f"Length: {g.length}, Width: {g.width}")
                  g_centers_.append(g.center)

            new_centers = []
            for center in g_centers_:                  
                  x_original = center[1] + padded_xmin
                  y_original = center[0] + padded_ymin
                  
                  new_centers.append((round(x_original, 1), round(y_original, 1)))
                  # print(f"Transformed center: {new_centers[-1]}")

            """width_scaling = crop_width / 224.0
            height_scaling = crop_height / 224.0"""
            
            grasp_info = []
            # fig, ax = plt.subplots(1)
            # ax.imshow(rgb_img)
            for k, center in enumerate(new_centers):
                  angle_ = gs[k].angle
                  width_original = gs[k].width 
                  height_original = gs[k].length 
                  x_cen, y_cen = new_centers[k]
                  rect = plt.Rectangle(
                        (x_cen - width_original /2, y_cen - height_original / 2),
                        height_original, width_original,
                        angle=np.degrees(angle_),
                        edgecolor='r', facecolor='none'
                  )
                  # ax.add_patch(rect)
                  grasp_info.append({
                        "center_x": x_cen,
                        "center_y": y_cen,
                        "width": width_original,
                        "height": height_original,
                        "angle": gs[k].angle
                  })

            # ax.set_title('Grasp in Original Image Space')
            # plt.show()

            return grasp_info, depth_img_cropped, gs



      """####################### Initial Code #####################"""
      # def process_data(self, rgb_img, depth_img, crop, detections):
      #       """
      #       Process the input data using GRConvNet
      #       Input:
      #             rgb_img: RGB image 480x640
      #             depth_img: Depth image 480x640
      #       Output:
      #             gs: Grasp rectangle [center_x, center_y, width, height, angle]
      #             processed depth image 224x224
      #       """
      #       # center crop the images
      #       # rgb_img = self.center_crop(rgb_img, 224, 224)
      #       # depth_img = self.center_crop(depth_img, 224, 224)

      #       # resize rgb image to 224x224
      #       # rgb_img = cv2.resize(rgb_img, (224, 224))
      #       # depth_img = cv2.resize(depth_img, (224, 224))

      #       # crop from 150 to 330 in y-axis and 100 to 250 in x-axis
      #       # rgb_img = rgb_img[150:330, 100:250]
      #       # depth_img = depth_img[150:330, 100:250]

      #       # crop the image
      #       print("Crop=", crop)
      #       xmin, ymin, xmax, ymax = crop[0], crop[1], crop[2], crop[3]
      #       x_padding = 70
      #       y_padding = 70
      #       #pad 40 pixels on all sides
      #       xmin = max(0, xmin - x_padding)
      #       ymin = max(0, ymin - y_padding)
      #       xmax = min(640, xmax + 10)
      #       ymax = min(480, ymax + 70)

      #       print("rgb_image shape", rgb_img.shape)
      #       rgb_img = rgb_img[ymin:ymax, xmin:xmax]
      #       depth_img = depth_img[ymin:ymax, xmin:xmax]

      #       # Process the depth image to remove NaN values
      #       depth_img = self.process_depth_image(depth_img)

      #       rgb_img = cv2.resize(rgb_img, (224, 224))
      #       depth_img = cv2.resize(depth_img, (224, 224))
      #       originalrbg = rgb_img.copy()              # Save the original rgb image
      #       originaldepth = depth_img.copy()          # Save the original depth image
      #       # print(rgb_img.shape)
      #       # print(depth_img.shape)
      #       # normalize rgb image
      #       rgb_img = rgb_img.astype(np.float32) / 255.0
      #       rgb_img -= rgb_img.mean()
      #       rgb_img = rgb_img.transpose((2, 0, 1))

      #       # normalize depth image
      #       depth_img = np.clip((depth_img - depth_img.mean()), -1, 1)

      #       # Create the input tensor
      #       x = np.concatenate(
      #                   (np.expand_dims(depth_img, 0),
      #                   rgb_img),
      #                   0
      #             )
      #       # x = np.expand_dims(depth_img, 0)

      #       x = torch.from_numpy(x.astype(np.float32)).to(self.device)
      #       print("X shape", x.shape)
      #       with torch.no_grad():
      #             x = x.unsqueeze(0)
      #             output = self.network.predict(x)

      #       # extract the output
      #       q_img = output['pos']
      #       cos_img = output['cos']
      #       sin_img = output['sin']
      #       width_img = output['width']
      #       print("q, cos, sin, width img shape", q_img.shape, cos_img.shape, sin_img.shape, width_img.shape)

      #       # import matplotlib.pyplot as plt
      #       # plt.imshow(q_img.permute(0,2,3,1).squeeze().cpu().numpy(), cmap='gray')
      #       # plt.show()
      #       # plt.imshow(cos_img.permute(0,2,3,1).squeeze().cpu().numpy(), cmap='gray')
      #       # plt.show()
      #       # plt.imshow(width_img.permute(0,2,3,1).squeeze().cpu().numpy(), cmap='gray')
      #       # plt.show()

      #       # post process the output to get the grasp rectangle
      #       q_img, ang_img, width_img = post_process.post_process_output(q_img, cos_img, sin_img, width_img)
      #       print("q, ang_img, width img shape", q_img.shape, ang_img.shape, width_img.shape)
            
      #       # print("Grasp Process = ", detections)
      #       # for detection in detections.get("detections", []):
      #       #       bbox = detection["bbox"]
      #       #       center = detection["info"]["center"]
      #       #       bbox_color = "red"
      #       #       center_color = "blue"

      #       #       # Draw bounding box
      #       #       x_min, y_min, x_max, y_max = bbox
      #       #       rect = plt.Rectangle(
      #       #       (x_min, y_min),
      #       #       x_max - x_min,
      #       #       y_max - y_min,
      #       #       linewidth=2,
      #       #       edgecolor=bbox_color,
      #       #       facecolor="none",
      #       #       linestyle="--",
      #       #       )
      #       #       ax.add_patch(rect)
      #       #       print("rect", rect)

      #       #       # Plot center point
      #       #       center_x = (x_min + x_max) / 2
      #       #       center_y = (y_min + y_max) / 2
      #       #       ax.plot(center_x, center_y, marker="o", color=center_color, markersize=5)
      #       #       print("In process data -> center_x, center_y =", center_x, center_y)
      #       #       # Display detection info
      #       #       ax.text(
      #       #       x_min,
      #       #       y_min - 10,
      #       #       f"ID: {detection['id']} Class: {detection['class']}",
      #       #       color=bbox_color,
      #       #       fontsize=8,
      #       #       bbox=dict(facecolor="white", alpha=0.5),
      #       #       )
            
      #       gs = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=5)
      #       # print("Grasps=", len(gs), gs)
      #       # Round off the values
      #       center_x = round(gs[0].center[0], 5)
      #       center_y = round(gs[0].center[1], 5)
      #       width = round(gs[0].width, 5)
      #       height = round(gs[0].length, 5)
      #       angle = round(gs[0].angle, 5)
      #       # print("SDS")
      #       ax = plt.subplot(111)
      #       ax.imshow(originalrbg)
      #       g_center = []
      #       for g in gs:
      #             print("Center = ", g.center)
      #             g_center.append(g.center)
      #             g.plot(ax)

      #       # Calculate scaling factors
      #       crop_width = xmax - xmin
      #       crop_height = ymax - ymin
      #       scale_x = crop_width / 224.0
      #       scale_y = crop_height / 224.0

      #       new_centers = []
      #       for center in g_center:
      #       # First scale to crop size
      #             x_scaled = center[0] * scale_x
      #             y_scaled = center[1] * scale_y
                  
      #             # Then add offset to get original image coordinates
      #             x_original = int(x_scaled + xmin)
      #             y_original = int(y_scaled + ymin)
      #             new_centers.append((x_original, y_original))

      #       # Similarly scale width and height
      #       adjusted_grasp_pose = {
      #       "center_x": center_x * scale_x + xmin,
      #       "center_y": center_y * scale_y + ymin,
      #       "width": width * scale_x,
      #       "height": height * scale_y,
      #       "angle": angle  # angle remains the same
      #       }

            
      #       ax.set_title('Grasp')

      #       return [new_centers[0][1], new_centers[0][0],
      #             adjusted_grasp_pose["width"], adjusted_grasp_pose["height"],
      #             adjusted_grasp_pose["angle"]], originaldepth, gs
            # grasp_pose = {
            #       "center_x": center_x,
            #       "center_y": center_y,
            #       "width": width,
            #       "height": height,
            #       "angle": angle
            # }

            # return [center_x, center_y, width, height, angle], originaldepth, gs
    
    
      def process_depth_image(self, depth, out_size=300, return_mask=False, crop_y_offset=0):
            """
            Process depth image to be fed into the network.
            """
            depth_crop = depth.copy()
            
            depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
            depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

            kernel = np.ones((3, 3),np.uint8)
            depth_nan_mask = cv2.dilate(depth_nan_mask, kernel, iterations=1)

            depth_crop[depth_nan_mask==1] = 0

            # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
            depth_scale = np.abs(depth_crop).max()
            depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.

            depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)

            # Back to original size and value range.
            depth_crop = depth_crop[1:-1, 1:-1]
            depth_crop = depth_crop * depth_scale

            return depth_crop

      def test_load(self):
            """
            Test the model loading and prediction
            """
            rgb_img = imread('/home/vishwas/RBE595-vbm/src/grasp_gen_service/grasp_gen_service/cornellds/pcd0100r.png')
            depth_img = imread('/home/vishwas/RBE595-vbm/src/grasp_gen_service/grasp_gen_service/cornellds/pcd0100d.tiff')

            # rgb_img = rgb_img[190:414, 173:173+224]
            # depth_img = depth_img[190:414, 173:173+224]
            rgb_img = self.center_crop(rgb_img, 224, 224)
            # rgb_img = self.process_rgb_image(rgb_img)
            depth_img = self.center_crop(depth_img, 224, 224)

            depth_img = self.process_depth_image(depth_img)
            originalrbg = rgb_img.copy()

            rgb_img = cv2.resize(rgb_img, (224, 224))

            # normalize rgb image
            rgb_img = rgb_img.astype(np.float32) / 255.0
            rgb_img -= rgb_img.mean()
            rgb_img = rgb_img.transpose((2, 0, 1))

            # normalize depth image
            depth_img = np.clip((depth_img - depth_img.mean()), -1, 1)

            x = np.concatenate(
                        (np.expand_dims(depth_img, 0),
                        rgb_img),
                        0
                  )

            x = torch.from_numpy(x.astype(np.float32)).to(self.device)

            # Forward pass
            with torch.no_grad():
                  x = x.unsqueeze(0)

                  output = self.network.predict(x)

            q_img = output['pos']
            cos_img = output['cos']
            sin_img = output['sin']
            width_img = output['width']

            q_img, ang_img, width_img = post_process.post_process_output(q_img, cos_img, sin_img, width_img)
            gs = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=1)

            # print(gs)

            # fig = plt.figure(figsize=(10, 10))
            # plt.ion()
            # plt.clf()
            ax = plt.subplot(111)
            ax.imshow(originalrbg)
            g_center = []
            for g in gs:
                  print("Center = ",g.center)
                  g_center.append(g.center)
                  print("Angle = ",g.angle)
                  print("Length =", g.length)
                  print("Width = ", g.width)

            print("g_center",g_center)
            ax.set_title('Grasp')
            ax.axis('off')
            plt.show()
            # fig.savefig('grasp.png')


# if __name__ == '__main__':
#     main()