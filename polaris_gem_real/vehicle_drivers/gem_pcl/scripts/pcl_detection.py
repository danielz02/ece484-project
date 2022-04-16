#!/usr/bin/env python3

from __future__ import print_function

import cv2
import numpy as np


import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import copy


class LidarDetection:

    def __init__(self, resolution=0.1, side_range=(-15., 15.), fwd_range=(-20., 20.), height_range=(-1.3, 0)):

        self.resolution   = resolution
        self.side_range   = side_range
        self.fwd_range    = fwd_range
        self.height_range = height_range
        
        self.cvBridge     = CvBridge()
        self.birdsEyeViewPub = rospy.Publisher("/BirdsEye", Image, queue_size=1)

        self.pointCloudSub   = rospy.Subscriber("/lidar1/velodyne_points", PointCloud2, self.pointCloudHandler, queue_size=10)
        
        x_img = np.floor(-0 / self.resolution).astype(np.int32)
        y_img = np.floor(-0 / self.resolution).astype(np.int32)

        self.vehicle_x = x_img - int(np.floor(self.side_range[0] / self.resolution))
        self.vehicle_y = y_img + int(np.ceil(self.fwd_range[1] / self.resolution))
        
        self.x_front = float('nan')
        self.y_front = float('nan')


    def pointCloudHandler(self, data):

        # generator
        gen = point_cloud2.readgen = point_cloud2.read_points(cloud=data, field_names=('x', 'y', 'z', 'ring'))

        lidarPtBV = []
        for p in gen:
            lidarPtBV.append((p[0],p[1],p[2])) # append x, y, z

        # birdview
        self.construct_birdview(lidarPtBV)


    def construct_birdview(self, data):

        """
            Call back function that get the distance between vehicle and nearest wall in given direction

            The calculated values are stored in the class member variables

            Input: data - lidar point cloud

        """

        # create image from_array
        x_max = 1 + int((self.side_range[1] - self.side_range[0]) / self.resolution)
        y_max = 1 + int((self.fwd_range[1] - self.fwd_range[0]) / self.resolution)
        im    = np.zeros([y_max, x_max], dtype=np.uint8)

        if len(data) == 0:
            return im

        #---------------------------------------------------------------------------------------------------

        # Reference: http://ronny.rest/tutorials/module/pointclouds_01/point_cloud_birdseye/

        data = np.array(data)

        # Extract points for each axis
        x_points = data[:, 0]
        y_points = data[:, 1]
        z_points = data[:, 2]

        # Only keep points in the range specified above
        x_filter = np.logical_and((x_points >= self.fwd_range[0]), (x_points <= self.fwd_range[1]))
        y_filter = np.logical_and((y_points >= self.side_range[0]), (y_points <= self.side_range[1]))
        z_filter = np.logical_and((z_points >= self.height_range[0]), (z_points <= self.height_range[1]))

        filter   = np.logical_and(x_filter, y_filter)
        filter   = np.logical_and(filter, z_filter)
        indices  = np.argwhere(filter).flatten()

        # xy_filter   = np.logical_and(x_filter, y_filter)
        # xyz_filter  = np.logical_and(xy_filter, z_filter)
        # indices  = np.argwhere(xyz_filter).flatten()

        # Keepers
        x_points = x_points[indices]
        y_points = y_points[indices]
        z_points = z_points[indices]

        # print(z_points)


        def scale_to_255(a, min_val, max_val, dtype=np.uint8):
            a = (((a-min_val) / float(max_val - min_val) ) * 255).astype(dtype)
            tmp = copy.deepcopy(a)
            a[:] = 0
            a[tmp>0] = 255
            return a


        # clip based on height for pixel values
        pixel_vals = np.clip(a=z_points, a_min=self.height_range[0], a_max=self.height_range[1])
        pixel_vals = scale_to_255(pixel_vals, min_val=self.height_range[0], max_val=self.height_range[1])


        # getting sensor reading for front       
        filter_front = np.logical_and((y_points>-3), (y_points<3))
        filter_front = np.logical_and(filter_front, x_points > 1)
        filter_front = np.logical_and(filter_front, pixel_vals > 128)
        indices      = np.argwhere(filter_front).flatten()



        self.x_front = np.mean(x_points[indices])
        self.y_front = np.mean(y_points[indices])

        print(self.x_front, self.y_front)


        # convert points to image coords with resolution
        x_img = np.floor(-y_points / self.resolution).astype(np.int32)
        y_img = np.floor(-x_points / self.resolution).astype(np.int32)


        # shift coords to new original
        x_img -= int(np.floor(self.side_range[0] / self.resolution))
        y_img += int(np.ceil(self.fwd_range[1] / self.resolution))
            
        # Generate a visualization for the perception result
        im[y_img, x_img] = pixel_vals

        img = im.astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        center = (self.vehicle_x, self.vehicle_y)
        cv2.circle(img, center, 5, (0,0,255), -1, 8, 0)
        
        center = self.convert_to_image(self.x_front, self.y_front)

        cv2.circle(img, center, 5, (0,255,0), -1, 8, 0)

        if not np.isnan(self.x_front) and not np.isnan(self.y_front):
            cv2.arrowedLine(img, (self.vehicle_x,self.vehicle_y), center, (255,0,0))
        
        x1, y1 = self.convert_to_image(15,5)
        x2, y2 = self.convert_to_image(1,-5)
        cv2.rectangle(img, (x1, y1), (x2, y2), (255,0,0))

        birds_eye_im = self.cvBridge.cv2_to_imgmsg(img, 'bgr8')

        self.birdsEyeViewPub.publish(birds_eye_im)









    def convert_to_image(self, x, y):

        """
            Convert point in vehicle frame to position in image frame
            Inputs: 
                x: float, the x position of point in vehicle frame
                y: float, the y position of point in vehicle frame
            Outputs: Float, the x y position of point in image frame 
        """

        x_img = np.floor(-y / self.resolution).astype(np.int32)
        y_img = np.floor(-x / self.resolution).astype(np.int32)

        x_img -= int(np.floor(self.side_range[0] / self.resolution))
        y_img += int(np.ceil(self.fwd_range[1] / self.resolution))
        return (x_img, y_img)




    def processLidar(self):

        """
            Compute the distance between vehicle and object in the front
            Inputs: None
            Outputs: Float, distance between vehicle and object in the front 
        """
        
        if not np.isnan(self.x_front) and not np.isnan(self.y_front):
            front = np.sqrt(self.x_front**2+self.y_front**2)
        else:
            front = -1
        
        return front


def run_gem_pcl():

    rospy.init_node('gem_pcl', anonymous=True)

    lp = LidarDetection()

    rate = rospy.Rate(15)

    while not rospy.is_shutdown():

        # Get the current position and orientation of the vehicle
        lidar_reading = lp.processLidar()

        if lidar_reading != -1:
            print("Pedestrian detected " + str(lidar_reading) + " meters away!")
        else:
            print("Pedestrian not detected.")


if __name__ == "__main__":
    run_gem_pcl()
    rospy.spin()