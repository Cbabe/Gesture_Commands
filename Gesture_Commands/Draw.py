"""Command neato via gestures"""
 
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
       self.mp_drawing = mp.solutions.drawing_utils # Media Pipe
       self.mp_drawing_styles = mp.solutions.drawing_styles # Media Pipe
       self.mp_hands = mp.solutions.hands # Media Pipe
       self.helper = TFHelper(self) # help with transformations and odometry
       self.kpclf = KeyPointClassifier() # ML Classification
 
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
       } # Gestures we can detect
 
       self.gesture_index = None
       self.previous_point=None
       self.x_average=[] # Average x position of pointer finger
       self.y_average=[] # Average y position of pointer finger
       self.pose = [0.0, 0.0, 0.0] # Initial robot position
       self.binary_image=None  # Initialize binary image
       # For webcam input:
       self.cap = cv2.VideoCapture(0) # Use Laptop video
       print("found webcam")
       self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10) # Publish angular and linear velocity to the robot
       print("created velocity publisher")
       self.odom_sub = self.create_subscription(Odometry, 'odom', self.process_pose, 10) # receive odometry from the robot
       print("created odometry subscriber")
      
       # Uses two thread one for the camera so the video is always showing and the other to control the robot
       camera_thread = Thread(target=self.process_image) # Creates Camera Thread
       camera_thread.start() # Starts Camera Thread
       print("started camera thread...")
 
       control_thread = Thread(target=self.control) # Creates Control Thread
       control_thread.start() # Starts Control Thread
       print("started control thread...")
 
 
   def process_image(self):
       with self.mp_hands.Hands(model_complexity=0, min_detection_confidence=0.7,min_tracking_confidence=0.7) as hands: # Initialize hands object
           while self.cap.isOpened(): # While camera is open
               success, image = self.cap.read() # Get image from camera
               image_height, image_width, _ = image.shape # Get image dimensions
               if self.binary_image is None: # If binary image does not exist yet create it
                   self.binary_image=np.zeros((image_width,image_height,3),np.uint8) # Blank binary image
               if not success:
                   print("Ignoring empty camera frame.")
                   # If loading a video, use 'break' instead of 'continue'.
                   continue
               # To improve performance, optionally mark the image as not writeable to
               # pass by reference.
               image.flags.writeable = False
               image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # Convert from RGR to RGB
               results = hands.process(image) # Finds hands in the image
  
               # Draw the hand annotations on the image.
               image.flags.writeable = True
               image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) # Convert back from RGB to BGR
               no_gesture_index = 6 # Index for no know gesture to be displayed
               self.gesture_index = no_gesture_index # Pre assign the index to no known gesture
 
               if results.multi_hand_landmarks: # If a hand is in the image
                   for hand_landmarks in results.multi_hand_landmarks: # look at each hand
                       landmark_list = u.calc_landmark_list(image, hand_landmarks) # Find the points on the hand
                       keypoints = u.pre_process_landmark(landmark_list) # Find keypoints
                       self.gesture_index = self.kpclf(keypoints) # Classify the position of the points on the hand to a gesture
 
                       self.mp_drawing.draw_landmarks(
                           image,
                           hand_landmarks,
                           self.mp_hands.HAND_CONNECTIONS,
                           self.mp_drawing_styles.get_default_hand_landmarks_style(),
                           self.mp_drawing_styles.get_default_hand_connections_style()) # Add points to image
                       x=round(hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width) # Get x position of pointer finger
                       y=round(hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_height) # Get y position of pointer finger
                       # print(f"x:{i}, y:{y}")
                       # If pointer finger is outside the bounds correct it to the edge of the bound
                       if x>=image_width:
                           x=image_width-1
                       if x<0:
                           x=0
                       if y>=image_height:
                           y=image_height-1
                       if y<0:
                           y=0
                       #Determines if the pointer finger is in relative the same place for a long period of time
 
                       if len(self.x_average)==0: # Start count of the amount of detections the pointer finger has been in the same place
                           print("start over")
                           self.x_average.append(x) # Add position to the x list
                           self.y_average.append(y) # Add position to the y list
                       else:
                           print(f"x_avg:({self.x_average}) x: ({x})")
                           print(f"y_avg:({self.y_average}) y: ({y})")
                           # Is the new pointer finger position within 20 pixels of the old position if so add the position to the list
                           if sum(self.x_average)/len(self.x_average)-20<x and sum(self.x_average)/len(self.x_average)+20>x and sum(self.y_average)/len(self.y_average)-20<y and sum(self.y_average)/len(self.y_average)+20>y:
                               print("Same place")
                               self.x_average.append(x) # Add position to the x list
                               self.y_average.append(y) # Add position to the y list
                               if len(self.x_average)>10: # If the last 10 positions have been close to each other then set waypoint
                                   self.x_average=[] # Start x list again
                                   self.y_average=[] # Start y list again
                                   if self.previous_point is None:
                                       self.previous_point=(x,y) # if it is the first point set it to the previous point
                                   else:
                                       # Create line from the previous point to the new point
                                       cv2.line(self.binary_image, self.previous_point, (x,y), [255,255,255], 3)
                                       newImage = self.binary_image.copy()
                                       print("Capture")
                                       capture(newImage) # Perform image recognition
                                       self.previous_point=(x,y)
 
                           else:
                               # If the finger position is to far away from the last position
                               print("Miss")
                               self.x_average=[]
                               self.y_average=[]
 
                          
               # Flip the image horizontally for a selfie-view display.
               cv2.putText(image, self.gestures[self.gesture_index],
                           (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, 255)
               cv2.imshow('Binary Image', self.binary_image)
               cv2.imshow('MediaPipe Hands', image)
               if cv2.waitKey(5) & 0xFF == 27:
                   self.cap.release()
   # Command the Neato
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
               # Draw
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
       # Drive in square based on odometry
       msg = Twist() # Create blank velocity message
       side = 0.5 # side length of square
 
       for i in range(4): # Repeat for each side
           start_x = self.pose[0] # Start x position using odometry
           start_y = self.pose[1] # Start y position using odometry
           start_theta = self.pose[2] # Start angle using odometry
           # drive one side
           msg.linear.x = 0.75 # Go forwards
           msg.angular.z = 0.0
           self.vel_pub.publish(msg)
           while math.sqrt((self.pose[0]-start_x)**2 + (self.pose[1] -start_y)**2) < side:
               if i == 0:
                   print("driving first side of square...")
               if i == 1:
                   print("driving second side of square...")
               if i == 2:
                   print("driving third side of square...")
               if i == 3:
                   print("driving fourth side of square...")
               time.sleep(0.01)
 
           # turn 90 degrees
           msg.linear.x = 0.0
           msg.angular.z = 0.75
           self.vel_pub.publish(msg)
           while abs(self.pose[2] - start_theta) < 90:
               if i == 0:
                   print("driving first turn..")
               if i == 1:
                   print("driving second turn...")
               if i == 2:
                   print("driving third turn..")
               if i == 3:
                   print("driving fourth turn..")
               time.sleep(0.01)
      
       # stop the Neato
       msg.linear.x = 0.0
       msg.angular.z = 0.0
       self.vel_pub.publish(msg)
       time.sleep(2.0)
 
       print("finished square")
  
 
   def drive_triangle(self):
       # Drive in triangle based on odometry
       msg = Twist()
       side = 0.5 # side length of triangle
 
       for i in range(3):
           start_x = self.pose[0]
           start_y = self.pose[1]
           start_theta = self.pose[2]
           # drive one side
           msg.linear.x = 0.75
           msg.angular.z = 0.0
           self.vel_pub.publish(msg)
           while math.sqrt((self.pose[0]-start_x)**2 + (self.pose[1] -start_y)**2) < side:
               if i == 0:
                   print("driving first side. of triangle..")
               if i == 1:
                   print("driving second side of triangle...")
               if i == 2:
                   print("driving third side of triangle...")
               time.sleep(0.01)
 
           # turn 120 degrees
           msg.linear.x = 0.0
           msg.angular.z = 0.75
           self.vel_pub.publish(msg)
           while abs(self.pose[2] - start_theta) < 120:
               if i == 0:
                   print("driving first turn..")
               if i == 1:
                   print("driving second turn...")
               if i == 2:
                   print("driving third turn..")
               time.sleep(0.01)
      
       # stop the Neato
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
 
      
       print("finished triangle")
 
 
 
   def drive_circle(self):
       # Drive in circle based on timing
       msg = Twist()
       msg.linear.x = 1.0 # Constant linear velocity
       msg.angular.z = 1.0 # Constant angular velocity
       self.vel_pub.publish(msg)
       start = time.time()
       while time.time() - start < 8.0:
           print("driving in a circle...")
           time.sleep(0.01)
 
       print("finished circle")
 
   def process_pose(self, msg):
 
       temp_pose = self.helper.convert_pose_to_xy_and_theta(msg.pose.pose) # tuple
       pose_list = list(temp_pose) # list
       pose_list[2] = (pose_list[2] * 180 / (math.pi)) % 360
       if pose_list[2] < 0:
           pose_list[2] += 360
 
       self.pose = pose_list
  
 
 
          
def main(args=None):
   rclpy.init(args=args)         # Initialize communication with ROS
   node = gesture_command()   # Create our Node
   rclpy.spin(node)              # Run the Node until ready to shutdown
   rclpy.shutdown()              # cleanup
 
if __name__ == '__main__':
   main()
 
