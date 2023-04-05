import cv2
import rospy
from img_capture.msg import controlData, camData
import os

# Global Variables
path = "/home/JHSrobo/ROVMIND/ros_workspace/src/img_capture/img"
count = 1
coralCount = 1
ip = None
coralMode = False
videoFeed = None

# Subscribers
camera_sub = rospy.Subscriber('cameras', camData, cam_callback)
control_sub = rospy.Subscriber('control', control_data, control_callback)

def cam_callback(camData):
    global path, count, coralCount, ip, coralMode, videoFeed
    if camData.screenshot:
        if ip != camData.ip:
            videoFeed.release()
            videoFeed = cv2.VideoCapture('http://{}:5000'.format(ip))
        image = videoFeed.read()
        if coralMode:
            cv2.imwrite("{}/coral/{}.png".format(path, coralCount), image)
            coralCount++
        else:
            cv2.imwrite("{}/{}.png".format(path, count), image)
            count++

def control_callback(controlData):
    global coralMode
    coralMode = controlData.coral

def main():
    rospy.init_node('camera_feed')
    os.system("rm /home/jhsrobo/ROVMIND/ros_workspace/src/img_capture/imgs/*.png")
    os.system("rm /home/jhsrobo/ROVMIND/ros_workspace/src/img_capture/imgs/coral/*.png")

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
