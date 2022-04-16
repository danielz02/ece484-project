#!/usr/bin/env python3

#================================================================
# File name: gem_rviz_text.py                                                                  
# Description: show sensor info in Rviz                                                              
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 07/10/2021                                                                
# Date last modified: 08/02/2021                                                          
# Version: 0.1                                                                    
# Usage: rosrun gem_gnss gem_rviz_text.py                                                                     
# Python version: 3.8                                                             
#================================================================

import rospy
import math
import random
import numpy as np

from geometry_msgs.msg import Twist
from novatel_gps_msgs.msg import Inspva
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Float32, Float64

class GEMOverlay(object):

    def __init__(self):
          
        self.text_pub    = rospy.Publisher("/gem_rviz_text", OverlayText, queue_size=5)
        self.gps_sub     = rospy.Subscriber("/novatel/inspva", Inspva, self.gps_callback)
        self.speed_sub   = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.speed_callback)
        self.overlaytext = self.update_overlaytext()
        self.gps_update  = False
        self.lat         = 0.0
        self.lon         = 0.0
        self.yaw         = 0.0
        self.speed       = 0.0 # m/s
        self.rate        = rospy.Rate(30)

    def speed_callback(self, msg):
        self.speed = round(msg.data, 2)

    def gps_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)
        self.yaw = round(msg.azimuth, 6)

    def update_overlaytext(self, lat=0.0, lon=0.0, yaw=0.0, speed=0.0):
        text            = OverlayText()
        text.width      = 200
        text.height     = 90
        text.left       = 10
        text.top        = 10
        text.text_size  = 12
        text.line_width = 2
        text.font       = "DejaVu Sans Mono"
        text.text       = """Lat   = %s
                             Lon   = %s
                             Yaw   = %s
                             Speed = %s
                          """ % (str(lat), str(lon), str(yaw), str(speed))
        text.fg_color   = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color   = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        return text
    
    def update_overlay_textonly(self, new_text):
        self.overlaytext.text = new_text
    
    def start_demo(self):
        
        while not rospy.is_shutdown():

            if(self.gps_update):
                gps_text = """Lat   = %s
                              Lon   = %s
                              Yaw   = %s
                              Speed = %s
                           """ % (str(self.lat), str(self.lon), str(self.yaw), str(self.speed))
                self.update_overlay_textonly(gps_text)
            else:
                self.overlaytext = self.update_overlaytext()
                self.gps_update  = True

            self.text_pub.publish(self.overlaytext)
            self.rate.sleep()
  
def gem_overlay():

    rospy.init_node('gem_rviz_markers', anonymous=True)

    gem_overlay_object = GEMOverlay()

    try:
        gem_overlay_object.start_demo()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    gem_overlay()