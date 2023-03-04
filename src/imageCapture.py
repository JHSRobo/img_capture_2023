import cv2
import numpy as np
import time
import threading
import flask
import rospy
from std_msgs.msg import UInt8, Float32, Int32
from copilot_interface.msg import controlData
from sensor_msgs.msg import Image
import os

# Global Variables
frame = 0
path = "/home/JHSrobo/ROVMIND/ros_workspace/src/img_capture/img/"

# Class for holding all the camera logic. Switches and reads the camera, adding an overlay to it.
class CameraSwitcher:

    def __init__(self):
        # self.verified is a dictionary that has keys of camera numbers and values of ip addresses -
        # self.verified = { 1: "192.168.1.101", 2: "192.168.1.102" }
        self.verified = {}
        # self.num is an integer that represents the current camera number and is the key for self.verified
        self.num = 0
        self.change = False
        self.cap = None

        # Create Subscribers
        self.camera_sub = rospy.Subscriber('/control', controlData, self.control_callback)

        self.config = {}

        # Initialize multithreading
        self.camera_thread = threading.Thread(target=self.find_cameras)
        self.camera_thread.setDaemon(True)
        self.camera_thread.start()

        # Counter variables for naming photos
        self.count = 0
        self.coralcount = 0

    @property
    def ip(self):
        #"""Ensures that the IP of the camera is always the correct number
        #without sacraficing redability. Otherwise, returns False
        #"""
        try:
            return self.verified[self.num]
        except KeyError:
            rospy.logerr("camera_viewer: passed a camera number that doesn't exist")
            if len(self.verified.keys()) == 0:
                return ""
            return self.verified[list(self.verified.keys())[0]]
    
    # Code for displaying most recent frame + overlay
    def read(self):
        if self.change:
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
            self.change = False

        # Read the most recent frame from the video stream into ret and frame
        ret, frame = self.cap.read()
        if frame is None:
            self.change = True
            return False
        if ret is None:
            rospy.logwarn('camera_viewer: ret is None, can\'t display new frame')
            return False
        else: 
            return frame

    # Delay until camera IP is added to verified
    def wait(self):
        rospy.loginfo('camera_viewer: waiting for cameras - None connected')
        while not self.verified:
            if rospy.is_shutdown():
                return
            rospy.logdebug('camera_viewer: still no cameras connected')
            time.sleep(1)

        # Initializes first camera
        self.num = 1
        rospy.loginfo("camera_viewer: loading capture from camera {}".format(self.num))
        if self.ip:
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
        else:
            rospy.logerr("camera_viwer: there is no camera at spot 1 after waiting.")

    # Changes cameras, and potentially takes photo (callback)
    def control_callback(self, control_data):
        # Checks to make sure it hasn't already selected that camera
        if self.num != control_data.camera:
            if self.ip:
                self.change = True
                self.num = control_data.camera
                rospy.loginfo("camera_viewer: changing to camera {}".format(self.num))
        if control_data.screenshot:
            self.save_frame(coral=control_data.coral)

    # Creates a web server on port 12345 and waits until it gets pinged
    # Then it adds the camera IP to self.verified
    def find_cameras(self):
        # Create the app
        app = flask.Flask(__name__)

        # Function that runs when the app gets a connection

        @app.route('/', methods=["POST", "GET"])
        def page():
            # If the IP that pinged flask is not already connected, and the length of request form is not 0
            if flask.request.remote_addr not in self.verified.values() and len(flask.request.form) > 0:
                rospy.loginfo(flask.request.remote_addr)
                # Add the IP to self.verified
                self.verified[self.give_num(flask.request.remote_addr)] = flask.request.remote_addr
                rospy.loginfo('camera_viewer: cameras currently connected: {}'.format(self.verified))
            return ""

        # Run the app
        rospy.loginfo('camera_viewer: camera web server online')
        app.run(host='0.0.0.0', port=12345)

    # Assign number (lower possible) to new cameras in self.verified
    def give_num(self, ip):
        if ip in self.config:
            return self.config[ip]
        else:
            try:
                available = [num for num in range(1, 8) if num not in self.verified and num not in self.config.values()][0]
            except IndexError:
                rospy.logerr('camera_viewer: camera detected, but there are no available numbers')
            return available

    # Closes the program nicely
    def cleanup(self):
        flask.request.environ.get('werkzeug.server.shutdown')()
        self.camera_thread.terminate()
        self.camera_thread.join()

    def save_frame(self, coral=False):
        global frame
        if frame is not False:
            if not coral:
                cv2.imwrite("{}/{}.png".format(path, self.count), frame)
            else: 
                cv2.imwrite("{}/coral/{}.png".format(path, self.coralcount), frame)

    # Renders the window
def main():
    global frame
    rospy.init_node('camera_feed')
    switcher = CameraSwitcher()
    switcher.wait()
    os.system("rm /home/jhsrobo/ROVMIND/ros_workspace/src/img_capture/imgs/*.png")
    os.system("rm /home/jhsrobo/ROVMIND/ros_workspace/src/img_capture/imgs/coral_pg/*.png")

    while not rospy.is_shutdown():
        frame = switcher.read()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()