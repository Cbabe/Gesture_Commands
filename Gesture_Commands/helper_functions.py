""" Some convenience functions for translating between various representations
    of a robot pose. """

from geometry_msgs.msg import Pose, Point, Quaternion
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.duration import Duration
import math
import numpy as np
import cv2

def stamped_transform_to_pose(t):
    t = t.transform
    return Pose(position=Point(x=t.translation.x, y=t.translation.y, z=t.translation.z),
                orientation=Quaternion(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w))


class FilledShape:
    def __init__(self, img):
        self.img = img
    
    def detect(self, contour, debug):
        shape = "undefined" 
        print(contour)
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.rectangle(self.img, (x, y-10), (x + w, y + 10), (0, 0, 255), -1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        number = ""
        if debug:
            cv2.drawContours(self.img, [contour], 0, (0, 255, 0), 2)

            for pt in approx:
                cv2.circle(self.img, (pt[0][0], pt[0][1]), 5, (255, 0, 0), -1)
            number = str(len(approx)) + " "
        if len(approx) == 3:
            shape = "triangle"
        elif len(approx) == 4:
            # print(w, h, w / h)
            if 0.95 < w / h < 1.05:
                shape = "Square"
            else:
                shape = "Rectangle"
        elif len(approx) == 5:
            shape = "Pentagon"
        else:
            shape = "Circle"
        cv2.putText(self.img, number + shape, (x, y), font, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

    def preprocessing_image(self):
        img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(img_gray, 127, 255, 0)
        kernel = np.ones((5, 5), np.uint8)
        cv2.dilate(threshold, kernel, iterations=1)
        threshold = cv2.GaussianBlur(threshold, (15, 15), 0)
        img_contour, contours= cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return threshold, contours


def capture(frame, debug=False):
    img_object = FilledShape(frame)
    threshold, contours = img_object.preprocessing_image()
    if contours is not None and contours[0][0][0]!=-1:
        print("found")
        for contour in contours:
            img_object.detect(contour, debug)
    cv2.imshow('Threshold', threshold)
    cv2.imshow('Original', frame)
    
def find_shapes(img):
    # img gets passed as a cv2.imread()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    cv2.imshow("img", img)
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)
        cv2.drawContours(img, [approx], 0, (0, 0, 0), 5)
        x = approx.ravel()[0]
        y = approx.ravel()[1] - 5

        if len(approx) == 3:    # Triangle
            cv2.putText(img, "Triangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
        
        elif len(approx) == 4:
            x1 ,y1, w, h = cv2.boundingRect(approx)
            aspectRatio = float(w)/h
            print(aspectRatio)

            if aspectRatio >= 0.95 and aspectRatio <= 1.05:     # Square
                cv2.putText(img, "square", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
            else:       # Rectangle
                cv2.putText(img, "rectangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
        
        elif len(approx) == 5:      # Pentagon
            cv2.putText(img, "Pentagon", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
        
        elif len(approx) == 10:     # Star
            cv2.putText(img, "Star", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
        
        else:       # Circle
            cv2.putText(img, "Circle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
        return img
class TFHelper(object):
    """ TFHelper Provides functionality to convert poses between various
        forms, compare angles in a suitable way, and publish needed
        transforms to ROS """
    def __init__(self, node):
        self.logger = node.get_logger()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.tf_broadcaster = TransformBroadcaster(node)
        self.node = node        # hold onto this for logging
        self.transform_tolerance = Duration(seconds=0.08)    # tolerance for mismatch between scan and odom timestamp

    def euler_from_quaternion(self, x, y, z, w):
        """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = list(self.euler_from_quaternion(*orientation_tuple))
        return (pose.position.x, pose.position.y, angles[2])
