"""Command neato via keyboard inputs"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import time
import math
from threading import Thread
from nav_msgs.msg import Odometry
from custom_gestures.model import KeyPointClassifier
from Gesture_Commands.helper_functions import TFHelper
import custom_gestures.landmark_utils as u
import numpy as np
from Gesture_Commands.helper_functions import *

class gesture_command(Node):
    def __init__(self):
        super().__init__('Command_Gesture')
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.helper = TFHelper(self)
        self.kpclf = KeyPointClassifier()

        self.gestures = {
            0: "Fists: Stop",
            1: "Pointer: Draw",
            2: "Two: Left",
            3: "Three:Right",
            4: "Four: Forwards",
            5: "Five: Backwards",
            6: "No Known Gesture Detected",
            7: "Triangle",
            8: "Square",
            9: "Circle"
        }

        self.gesture_index = None
        self.pose = [0.0, 0.0, 0.0]

        # For webcam input:
        self.cap = cv2.VideoCapture(0)
        print("found webcam")
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        print("created velocity publisher")
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.process_pose, 10)
        print("created odometry subscriber")
        

        camera_thread = Thread(target=self.process_image)
        camera_thread.start()
        print("started camera thread...")

        control_thread = Thread(target=self.control)
        control_thread.start()
        print("started control thread...")
   
    
    def process_image(self):
        with self.mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5,min_tracking_confidence=0.5) as hands:
            while self.cap.isOpened():
                success, image = self.cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    continue

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = hands.process(image)

                # Draw the hand annotations on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                no_gesture_index = 6
                self.gesture_index = no_gesture_index

                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        landmark_list = u.calc_landmark_list(image, hand_landmarks)
                        keypoints = u.pre_process_landmark(landmark_list)
                        self.gesture_index = self.kpclf(keypoints)

                        self.mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())
                # Flip the image horizontally for a selfie-view display.
                final = cv2.flip(image, 1)
                cv2.putText(final, self.gestures[self.gesture_index],
                            (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, 255)
                cv2.imshow('MediaPipe Hands', final)
                if cv2.waitKey(5) & 0xFF == 27:
                    self.cap.release()
                
    def control(self):
        while True:
            speed_msg = Twist()
            #print(self.gesture_index)
            if self.gesture_index==0:
                # Stop
                speed_msg.linear.x = 0.0
                speed_msg.angular.z = 0.0
                #print("no gesture")
            elif self.gesture_index==1:
                # Point/Draw
                speed_msg.linear.x = 0.0
                speed_msg.angular.z = 0.0
                #print("drawing")
            elif self.gesture_index==2:
                #Turn Left
                speed_msg.linear.x = 0.0
                speed_msg.angular.z = 1.0
                #print("left")
            elif self.gesture_index==3:
                #Turn Right
                speed_msg.linear.x = 0.0
                speed_msg.angular.z = -1.0
                #print("right") 
            elif self.gesture_index==4:
                #Forwards
                speed_msg.linear.x = 1.0
                speed_msg.angular.z = 0.0
                #print("forwards")
            elif self.gesture_index==5:
                #Backwards
                speed_msg.linear.x = -1.0
                speed_msg.angular.z = 0.0
                #print("backward")
            elif self.gesture_index==6:
                #No gesture
                speed_msg.linear.x = 0.0
                speed_msg.angular.z = 0.0
                #print("no gesture detected")
            elif self.gesture_index == 7:
                #print("triangle")
                self.drive_triangle()
            elif self.gesture_index == 8:
                #print("square")
                self.drive_square()

            elif self.gesture_index == 9:
                #print("circle")
                self.drive_circle()
        
            #Publish
            self.vel_pub.publish(speed_msg)
            #print("publishing speed message...")

    
    def drive_square(self):
        msg = Twist()

        # drive first side
        # turn 90 degrees
        # drive second side
        # turn 90 degrees
        # drive third side
        # turn 90 degrees
        # drive fourth side

        # drive one side of square
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)
        time.sleep(2.0)

        # turn 90 degrees
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.vel_pub.publish(msg)
        time.sleep(0.5)
        
        # drive second square side
        msg.angular.z = 0.0
        msg.linear.x = 1.0
        self.vel_pub.publish(msg)
        time.sleep(2.0)

        # turn 90 degrees
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.vel_pub.publish(msg)
        time.sleep(0.5)
        
        # drive third square side
        msg.angular.z = 0.0
        msg.linear.x = 1.0
        self.vel_pub.publish(msg)
        time.sleep(2.0)

        # turn 90 degrees
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.vel_pub.publish(msg)
        time.sleep(0.5)
        
        # drive fourth square side
        msg.angular.z = 0.0
        msg.linear.x = 1.0
        self.vel_pub.publish(msg)
        time.sleep(2.0)

        print("finished square")
    
    # DOES NOT WORK RN
    def drive_triangle(self):
        msg = Twist()
        # drive first triangle side
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)
        time.sleep(2.0)

        # turn 60 degrees
        print("triangle turn...")
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.vel_pub.publish(msg)
        time.sleep(3.0)

        # drive second triangle side
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)
        time.sleep(2.0)

        # turn 60 degrees
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.vel_pub.publish(msg)
        time.sleep(3.0)

        # drive third triangle side
        msg.angular.z = 0.0
        msg.linear.x = 1.0
        self.vel_pub.publish(msg)
        time.sleep(2.0)
        
        print("finished triangle")



    def drive_circle(self):
        msg = Twist()

        msg.linear.x = 1.0
        msg.angular.z = 1.0
        self.vel_pub.publish(msg)
        time.sleep(3.0)

        print("finished circle")

    def process_pose(self, msg):
        # print(msg)
        
        temp_pose = self.helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        pose_list = list(temp_pose)
        pose_list[2] = pose_list[2] * 180 / (2* math.pi)
        self.pose = tuple(pose_list)
        print(self.pose)
    


            
def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = gesture_command()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()

