"""Command neato via keyboard inputs"""

import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
<<<<<<< Updated upstream:Gesture_Commands/Command_Gesture.py
from custom_gestures.model import KeyPointClassifier
import custom_gestures.landmark_utils as u
=======
from model import KeyPointClassifier
import landmark_utils as u

>>>>>>> Stashed changes:Gesture_Commands/custom-gestures/Command_Gesture.py
class gesture_command(Node):
    def __init__(self):
        super().__init__('teleop')
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands

        self.kpclf = KeyPointClassifier()

        self.gestures = {
            0: "Open Hand Forward",
            1: "Thumb up Backward",
            2: "OK Right",
            3: "Peace Left",
            4: "Fists",
            5: "No Hand Detected",
            6: "Alien"
        }

        # For webcam input:
        self.cap = cv2.VideoCapture(0)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.control()
    
    def control(self):
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
                no_gesture_index = 5
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        landmark_list = u.calc_landmark_list(image, hand_landmarks)
                        keypoints = u.pre_process_landmark(landmark_list)
                        no_gesture_index = self.kpclf(keypoints)

                        self.mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())
                # Flip the image horizontally for a selfie-view display.
                final = cv2.flip(image, 1)
                cv2.putText(final, self.gestures[no_gesture_index],
                            (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, 255)
                cv2.imshow('MediaPipe Hands', final)
                
                speed_msg = Twist()
                no_gesture_index
                if no_gesture_index==0:
                    #Forward
                    speed_msg.linear.x = 1.0
                elif no_gesture_index==1:
                    #Backward
                    speed_msg.linear.x = -1.0
                elif no_gesture_index==2:
                    #Turn Right
                    speed_msg.angular.z = 1.0
                elif no_gesture_index==3:
                    #Turn Left
                    speed_msg.angular.z = -1.0
                #Publish
                self.publisher.publish(speed_msg)
                if cv2.waitKey(5) & 0xFF == 27:
                    self.cap.release()
                    break
    
    def drive_square(self):
        while True:
            msg = Twist()

            msg.linear.x = 1.0
            msg.angular.z = 0.0

            self.vel_pub.publish(msg)
            time.sleep(3)

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.vel_pub.publish(msg)
            time.sleep(0.5)

            msg.angular.z = 1.0
            msg.linear.x = 0.0

            self.vel_pub.publish(msg)
            time.sleep(2.06)

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.vel_pub.publish(msg)
            time.sleep(0.5)

    def drive_circle(self):
        while True:
            msg = Twist()

            msg.linear.x = 1.0
            msg.angular.z = 1.0

            self.vel_pub.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = gesture_command()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()

