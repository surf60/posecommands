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

def drive_map(left_angle,right_angle):
    left_angle, right_angle = num_to_range(left_angle,lower_range_l,upper_range_r,-1,1), num_to_range(right_angle,lower_range_l,upper_range_r,-1,1)
    return (abs(left_angle),abs(right_angle))
    
while cap.isOpened():
    # read frame
    _, frame = cap.read()
    # convert to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # process the frame for pose detection
    pose_results = pose.process(frame_rgb)
    left_shoulder = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_SHOULDER]
    left_elbow = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_ELBOW]
    right_elbow = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_ELBOW]        
    
    left_angle = math.atan(left_shoulder.x-left_elbow.x/left_elbow.y-left_shoulder.y)
    right_angle = math.atan(right_shoulder.x-right_elbow.x/right_elbow.y-right_shoulder.y)

    left_angle, right_angle = num_to_range(left_angle,0.6,1.2,-1,1), num_to_range(right_angle,0.6,1.2,-1,1)

    print(left_angle,right_angle)
    left_angle, right_angle = drive_map(left_angle,right_angle)
    # draw skeleton on the frame
    msg.linear.x, msg.angular.z = left_angle, right_angle
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