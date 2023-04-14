#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from img_capture.msg import controlData
import os

# Global Variables
path = "/home/jhsrobo/ROVMIND/ros_workspace/src/img_capture/img"
count = 1
coralCount = 1
coralMode = False

def img_callback(screenshot):
    global path, count, coralCount, coralMode
        if coralMode:
            cv2.imwrite("{}/coral/{}.png".format(path, coralCount), screenshot)
            coralCount += 1
        else:
            cv2.imwrite("{}/{}.png".format(path, count), screenshot)
            count += 1

def control_callback(controlData):
    global coralMode
    coralMode = controlData.coral

# Subscribers
camera_sub = rospy.Subscriber('screenshots', Image, img_callback)
control_sub = rospy.Subscriber('control', controlData, control_callback)

def main():
    rospy.init_node('camera_feed')
    try: os.system('rm {}/*.png'.format(path))
    except: pass
    try: os.system('rm {}/coral/*.png'.format(path))
    except: pass

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
