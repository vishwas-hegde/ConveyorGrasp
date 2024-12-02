"""
Script to load the GRConvNet model and process the RGB-D data
"""
import os
import sys
current_dir = os.getcwd() + '/src/grasp_gen_service/grasp_gen_service'
sys.path.append(current_dir)

import numpy as np
import torch.utils.data
import post_process
from imageio import imread
import cv2
import torch
from grasp import detect_grasps
import matplotlib.pyplot as plt

class GRConvNet_Grasp():

    def __init__(self):
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

    def process_data(self, rgb_img, depth_img, crop):
        """
        Process the input data using GRConvNet
        Input:
            rgb_img: RGB image 480x640
            depth_img: Depth image 480x640
        Output:
            gs: Grasp rectangle [center_x, center_y, width, height, angle]
            processed depth image 224x224
        """
        # center crop the images
        # rgb_img = self.center_crop(rgb_img, 224, 224)
        # depth_img = self.center_crop(depth_img, 224, 224)

        # resize rgb image to 224x224
        # rgb_img = cv2.resize(rgb_img, (224, 224))
        # depth_img = cv2.resize(depth_img, (224, 224))

        # crop from 150 to 330 in y-axis and 100 to 250 in x-axis
        # rgb_img = rgb_img[150:330, 100:250]
        # depth_img = depth_img[150:330, 100:250]

        # crop the image
        xmin, ymin, xmax, ymax = crop[0], crop[1], crop[2], crop[3]

        #pad 40 pixels on all sides
        xmin = max(0, xmin - 40)
        ymin = max(0, ymin - 40)
        xmax = min(640, xmax + 10)
        ymax = min(480, ymax + 40)

        rgb_img = rgb_img[ymin:ymax, xmin:xmax]
        depth_img = depth_img[ymin:ymax, xmin:xmax]

        # Process the depth image to remove NaN values
        depth_img = self.process_depth_image(depth_img)

        # rgb_img = cv2.resize(rgb_img, (224, 224))
        originalrbg = rgb_img.copy()              # Save the original rgb image
        originaldepth = depth_img.copy()          # Save the original depth image
        print(rgb_img.shape)
        print(depth_img.shape)
        # normalize rgb image
        rgb_img = rgb_img.astype(np.float32) / 255.0
        rgb_img -= rgb_img.mean()
        rgb_img = rgb_img.transpose((2, 0, 1))

        # normalize depth image
        depth_img = np.clip((depth_img - depth_img.mean()), -1, 1)

        # Create the input tensor
        x = np.concatenate(
                    (np.expand_dims(depth_img, 0),
                     rgb_img),
                    0
                )
        # x = np.expand_dims(depth_img, 0)

        x = torch.from_numpy(x.astype(np.float32)).to(self.device)

        with torch.no_grad():
            x = x.unsqueeze(0)
            output = self.network.predict(x)

        # extract the output
        q_img = output['pos']
        cos_img = output['cos']
        sin_img = output['sin']
        width_img = output['width']

        # post process the output to get the grasp rectangle
        q_img, ang_img, width_img = post_process.post_process_output(q_img, cos_img, sin_img, width_img)
        gs = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=1)

        # Round off the values
        center_x = round(gs[0].center[0], 5)
        center_y = round(gs[0].center[1], 5)
        width = round(gs[0].width, 5)
        height = round(gs[0].length, 5)
        angle = round(gs[0].angle, 5)

        ax = plt.subplot(111)
        ax.imshow(originalrbg)
        for g in gs:
            print(g.center)
            print(g.angle)
            print(g.length)
            print(g.width)

            g.plot(ax)
        ax.set_title('Grasp')
        ax.axis('off')
        plt.show()

        return [center_x, center_y, width, height, angle], originaldepth, gs
    
    
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
        for g in gs:
            print(g.center)
            print(g.angle)
            print(g.length)
            print(g.width)

            g.plot(ax)
        ax.set_title('Grasp')
        ax.axis('off')
        plt.show()
        # fig.savefig('grasp.png')



if __name__ == '__main__':
    model = GRConvNet_Grasp()
    model.test_load()