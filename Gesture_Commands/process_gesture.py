"""Command neato via keyboard inputs"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import numpy as np
import matplotlib.pyplot as plt
import queue



class process_gesture(Node):
    def __init__(self):
        super().__init__('teleop')
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.waypoints=[]
        self.previous_captures= queue.LifoQueue(5)

        # For webcam input:
        self.cap = cv2.VideoCapture(0)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.control()
    
    def control(self):
        with self.mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5,min_tracking_confidence=0.5) as hands:
            plt.axis([0, 700, 0, 700])
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
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:

                        self.mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())
                        image_hight, image_width, _ = image.shape
                        i=hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width
                        y=hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_hight
                        #print(f'Index finger tip coordinate: (',
                        #    f'{i}, 'f'{y})')
                        self.previous_captures.put([i,y])
                        print(self.previous_captures)
                # Flip the image horizontally for a selfie-view display.
                final = cv2.flip(image, 1)
                cv2.imshow('MediaPipe Hands', final) 
                if cv2.waitKey(5) & 0xFF == 27:
                    self.cap.release()
                    cv2.destroyAllWindows()
                    break
                       
def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = process_gesture()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()

