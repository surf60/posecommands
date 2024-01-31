import cv2
import mediapipe as mp
import math
import rclpy
from geometry_msgs.msg import Twist
## initialize pose estimator
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_holistic = mp.solutions.holistic
cap = cv2.VideoCapture(0) 

rclpy.init()
msg = Twist()
node = rclpy.create_node('Turtle_controll')
publisher = node.create_publisher(Twist,'/turtle1/cmd_vel',10)

lower_range_l = -5
lower_range_r = -5
upper_range_l = -6.5
upper_range_r = -6.0

def num_to_range(num, inMin, inMax, outMin, outMax):
    return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))
    
while cap.isOpened():
    # read frame
    _, frame = cap.read()

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # process the frame for pose detection
    pose_results = pose.process(frame_rgb)

    left_elbow = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_ELBOW]
    right_elbow = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_ELBOW]        
    
    left_angle = left_elbow.y
    right_angle = right_elbow.y

    if left_angle > 1:
        msg.linear.x = 1
    elif left_angle < -1:
        msg.linear.x = -1
    else:
        msg.linear.x = 0

    if right_angle > 1:
        msg.angular.z = 1
    elif right_angle < -1:
        msg.angular.z = -1
    else:
        msg.angular.z = 0
    # draw skeleton on the frame
    publisher.publish(msg)
    mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
    # display the frame
    cv2.imshow('Output', frame)
    
    if cv2.waitKey(1) == ord('q'):
        break
          
cap.release()
cv2.destroyAllWindows()

while rclpy.ok():
    rclpy.spin_once(node)
    
node.destroy_node()
rclpy.shutdown()