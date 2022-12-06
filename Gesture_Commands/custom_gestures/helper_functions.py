""" Some convenience functions for translating between various representations
    of a robot pose. """

from urllib.robotparser import RobotFileParser
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.time import Time
from rclpy.duration import Duration
import math
import numpy as np
import cv2
from copy import deepcopy
from numpy.random import random_sample
import PyKDL

def stamped_transform_to_pose(t):
    t = t.transform
    return Pose(position=Point(x=t.translation.x, y=t.translation.y, z=t.translation.z),
                orientation=Quaternion(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w))
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
def draw_random_sample(choices, probabilities, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
            choices: the values to sample from represented as a list
            probabilities: the probability of selecting each element in choices represented as a list
            n: the number of samples
    """
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples

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

    def convert_translation_rotation_to_pose(self, translation, rotation):
        """ Convert from representation of a pose as translation and rotation
            (Quaternion) tuples to a geometry_msgs/Pose message """
        return Pose(position=Point(x=translation[0],
                                   y=translation[1],
                                   z=translation[2]),
                    orientation=Quaternion(x=rotation[0],
                                           y=rotation[1],
                                           z=rotation[2],
                                           w=rotation[3]))

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = list(euler_from_quaternion(*orientation_tuple))
        # convert to degrees
        angles[2] = angles[2] * 180 / (2* math.pi)
        return (pose.position.x, pose.position.y, angles[2])

    def angle_normalize(self, z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(self, a, b):
        """ Calculates the difference between angle a and angle b (both should
            be in radians) the difference is always based on the closest
            rotation from angle a to angle b.
            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = self.angle_normalize(a)
        b = self.angle_normalize(b)
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

    def fix_map_to_odom_transform(self, robot_pose, odom_pose):
        """ This method constantly updates the offset of the map and
            odometry coordinate systems based on the latest results from
            the localizer.

            robot_pose: should be of type geometry_msgs/msg/Pose and represent 
                the robot's position within the map
            odom_pose: should be of type geometry_msgs/msg/Pose and represent
                the robot's position within the odometry coordinate system
            timestamp: the timestamp to associate with this transform
            """
        odom_pose_frame = PyKDL.Frame(V=PyKDL.Vector(x=odom_pose.position.x,
                                                     y=odom_pose.position.y,
                                                     z=odom_pose.position.z),
                                      R=PyKDL.Rotation.Quaternion(x=odom_pose.orientation.x,
                                                                  y=odom_pose.orientation.y,
                                                                  z=odom_pose.orientation.z,
                                                                  w=odom_pose.orientation.w))
        robot_pose_frame = PyKDL.Frame(V=PyKDL.Vector(x=robot_pose.position.x,
                                                      y=robot_pose.position.y,
                                                      z=robot_pose.position.z),
                                      R=PyKDL.Rotation.Quaternion(x=robot_pose.orientation.x,
                                                                  y=robot_pose.orientation.y,
                                                                  z=robot_pose.orientation.z,
                                                                  w=robot_pose.orientation.w))
        odom_to_map = robot_pose_frame * PyKDL.Frame.Inverse(odom_pose_frame)
        self.translation = odom_to_map.p
        self.rotation = odom_to_map.M.GetQuaternion()

    def send_last_map_to_odom_transform(self, map_frame, odom_frame, timestamp):
        if (not hasattr(self, 'translation') or
            not hasattr(self, 'rotation')):
            return
        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = map_frame
        transform.child_frame_id = odom_frame
        transform.transform.translation.x = self.translation[0]
        transform.transform.translation.y = self.translation[1]
        transform.transform.translation.z = self.translation[2]
        transform.transform.rotation.x = self.rotation[0]
        transform.transform.rotation.y = self.rotation[1]
        transform.transform.rotation.z = self.rotation[2]
        transform.transform.rotation.w = self.rotation[3]
        self.tf_broadcaster.sendTransform(transform)

    def get_matching_odom_pose(self, odom_frame, base_frame, timestamp):
        """ Find the odometry position for a given timestamp.  We want to avoid blocking, so if the transform is
            not ready, we return None.

            returns: a tuple where the first element is the stamped transform and the second element is the
                     delta in time between the requested time and the most recent transform available """
        if self.tf_buffer.can_transform(odom_frame, base_frame, timestamp):
            # we can get the pose at the exact right time
            return (stamped_transform_to_pose(self.tf_buffer.lookup_transform(odom_frame, base_frame, timestamp)), Duration(seconds=0.0))
        elif self.tf_buffer.can_transform(odom_frame,
                                          base_frame,
                                          Time()):
            most_recent = self.tf_buffer.lookup_transform(odom_frame,
                                                          base_frame,
                                                          Time())
            delta_t = Time.from_msg(timestamp) - Time.from_msg(most_recent.header.stamp)
            return (None, delta_t)
        else:
            return (None, None)

    def convert_scan_to_polar_in_robot_frame(self, msg, base_frame):
        """ Convert the scan data to a polar representation in the robot frame.
            The reason that we have to do this differently than in the warmup project
            is that the Turtlebot4's lidar frame is not oriented the same as the Neato.
            If you use the results in (r, theta) you will have the correct angles and distances
            relative to the robot.

            Note: theta is in radians
        """
        laser_pose = stamped_transform_to_pose(
            self.tf_buffer.lookup_transform(base_frame,
                                            msg.header.frame_id,
                                            Time()))
        rot = PyKDL.Rotation.Quaternion(x=laser_pose.orientation.x,
                                        y=laser_pose.orientation.y,
                                        z=laser_pose.orientation.z,
                                        w=laser_pose.orientation.w)
        laser_yaw = rot.GetRPY()[2]
        return (msg.ranges, np.linspace(msg.angle_min+laser_yaw, msg.angle_max+laser_yaw, len(msg.ranges)))

