# Gesture_Commands

# Installation:
```pip install cvzone```</br>
```pip install opencv-python```</br>
```pip install mediapipe```

# How to Run:

### To run Gesture commands:
Run the file: ```Command_Gesture.py```
### To run Draw:
Run the file: ```Draw.py```
### To Retrain the Cass Model:
Run the file: ```custom_gestures/classify_gestures.ipynb```
# Report 
## Phase 1
### Our Idea
We decided to implement a customizable hand gesture recognition control system. Hand gesture recognition is the process of determining hand shapes and movements in a video feed. We started by looking at other applications of hand gesture recognition, particularly in American Sign Language ASL recognition which has been heavily researched. But, we decided that we wanted to train our own gestures so we couldn’t really entirely rely on a pre-trained ASL recognition model. We looked into using OpenCV and while OpenCV had some libraries to support gesture recognition, we would have to implement much of the functionality we needed to support creating our own gestures.

### Researching Tools
After adequate research on machine learning libraries, we decided to use MediaPipe. MediaPipe is a machine learning framework developed by Google that comes with pre-built functionalities for the identification and tracking of faces, eyes, hands, people, etc… Of these functions, we focused on the hand detection one and used the key points identified in the image (fingertips, 1st knuckles, 2nd knuckles, etc…) to define our gestures. An example of this can be the speed control, which uses MediaPipe to identify the tip of the thumb and index fingers, and then calculates the 3D distance between the two, making speed proportional to this last value. This allows us to regulate speed easily and intuitively, slowing down the neato when someone brings these two fingers together and picking up speed as they get further apart.

### Training for Custom Gestures
Once we got MediaPipe running, we needed to look into how to recognize specific gestures and not just hands in an image. We searched for machine learning algorithms trained to recognize gestures and found many, however, they were either already trained to pre-set gestures (peace sign, open palm, etc.) and we wanted to be able to create our own gestures that our program could recognize using the MediaPipe technology. Finally, we found a source that would allow us to train our own algorithm that could learn as many gestures as we wanted. We did have issues with getting this up and running though as we would run the training code provided but the detection program would recognize the new gestures. We soon found out that it was not capturing the new images and the image dataset was not updating. It turned out to be a file problem where the dataset file was incorrectly referenced in the training code and when we fixed this the new images were recorded in real-time and could then be used to retrain the machine. Once we were able to smoothly create custom gestures our program could detect (like the alien sign or the fist), we moved on to behaviors the robot would perform in response to seeing these gestures.

### Performing the Behaviors
The first thing we did was tie simple behaviors like driving straight and turning to gestures like the thumbs up, peace sign, OK-sign, etc (as discussed in phase 2, we eventually changed these random gestures to a more intuitive number gestured system to control the Neato’s movements).<br/>
For driving in shapes (like circle, square, and triangle), we first used the time module. For example, to have the Neato drive in a triangle, we would have it drive straight for some amount of time (using the time.wait() command), turn for some amount of time that equates to turning 120 degrees, then driving straight again, repeating until a triangle is made. This method is not only very inconsistent, as depending on conditions like the floor the Neato may move different amounts in the same time, but it is also very time consuming as getting the right timing for angle turns and side lengths would take a decent amount of trial-and-error.
And so, we quickly pivoted to trying to use the Neato’s odom topic in order to have it more easily and consistently drive in shapes. With the utilization of the odom topic, we could simply tell the robot to turn until it reached the intended number of degrees and drive until it moved the intended distance. While it was a struggle at first to access the odom topic and to have the program dynamically change depending on the state of the robot’s pose, we did figure it out and were able to successfully have the robot drive in shapes.

## Phase 2
### Reconfiguring to Numbers
In order to lay the foundation for adding more features to our project, we decided to reconfigure the gestures the Neato understands to ones that are more intuitive and less random. We decided it would be most efficient to use numbers and associate them with the following behaviors:
#### Table of Gestures and Commands
| Gesture (Right Hand):     | Command |
| :---        |    :----:   |
| Fist      | Stop       |
| Pointer/Index   | Right        |
| Two  | Left        |
| Four  | Backward        |
| Five  | Forward        |
| Triangle  | Triangle        |
| Circle  | Circle        |
[Link to table](https://docs.google.com/document/d/1tlvMmWw1aRIrQCUnLkVYdPTNQ6Ecm9czYmTtKJKjy5w/edit?usp=sharing)

### Gesture Control
#### Simple Gestures
<img width="600" src="https://github.com/Cbabe/Gesture_Commands/blob/main/gestures_good(1).gif">

#### Drive in a Triangle
<img width="600" src="https://github.com/Cbabe/Gesture_Commands/blob/main/triangle_good(1).gif">

#### Drive in a Square
<img width="600" src="https://github.com/Cbabe/Gesture_Commands/blob/main/square_good(1).gif">


### Path Drawing
Once we reached a stage in the project where the Minimum Viable Product had been developed, we started working on extensions. We decided to focus on the challenge of drawing custom paths for the Neato to follow. These paths would be “drawn” in the air by the hand giving the commands, read through the webcam and finally processed by the Neato. We believe this process can be divided into three smaller tasks: tracking the hand that draws the shape, recognising what shape this is, and converting this to an actionable path. <br/>
Starting with hand tracking, we identify the tip of the finger using MediaPipe and record all the positions of this element with time. Plotting all the locations on a single image will then allow us to essentially connect the dots and “trace” the path the finger has drawn and obtain the required shape.
Then we implement shape recognition, which is carried out using OpenCV contours function, and we take the path identified in the previous section and fit it to the closest matching shape. Currently, the shapes we can detect are circles, triangles, rectangles, and hexagons. This will ideally work similarly to Quickdraw! by Google or the shape recognition function on many note-taking apps, using AI to identify the shape. We are still working out how to effectively do this, whether we are tracking the pointer finger continuously and drawing lines between positions, or whether we use the pointer finger to set waypoints and then connect the waypoints. <br/>
Lastly, the conversion to an actionable path is done by pre-defining what path and actions correspond to what shape. Knowing that the user just drew a square for example, the program will execute its pre-written square-driving function using the odometry information discussed previously in Performing the Behaviors. This architecture of drawing predefined shapes graphically and recognizing them was our next-level goal, which we were not able to achieve successfully with the time we had. However with more time we have a clear idea of what our next steps would be.

### Next Steps
Instead of trying to track the exact movement of the fingertip used to draw and then running that shape through a recognition algorithm, we would use the waypoint method. We would still track the finger tip, but not continuously. When drawing, we would drop way points whenever the fingertip, and by extension the hand, paused fro some short amount of time. For example, when drawing a triangle. A waypoint would be dropped as soon as the program recognizes the drawing gesture, then as the user draws the first side of the triangle and stops to change direction and draw the second side, the program drops another waypoint at that tip of the triangle because the hand stopped moving for a second or two to indicate a waypoint would be dropped. This process would be the same for any shape, for example when drawing a square we’d simply pause the needed time at every corner, and could also easily be extended to general paths like a line or a zig-zag pattern. The weakness of this method is that curves in shapes and paths wouldn’t be well recorded. So a circle, depending on how many waypoints one makes, could make the Neato actually move in a hexagon or octagon for example.

## Conclusion
As a team and individuals we are very satisfied with our work. Our MVP was achieved and we all learned more about computer vision and algorithms as intended. Teaming goals were also met as communication was very good during this meeting and we were attentive about splitting work well and knowing when we needed to come together to work on something. While we would have liked to get the custom path drawing working, we were also able to figure out how to actually achieve that which is very satisfying, the issue was simply a lack of time.

