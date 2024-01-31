import joblib
import cv2
import mediapipe as mp
import pandas as pd
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
## initialize pose estimator
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_holistic = mp.solutions.holistic
model = joblib.load('20240122_174537_model_97.8342354297638.pkl')
class_labels = ['ad','au','lup','rup']
command_label = [[-1.0,0.0],[1.0,0.0],[1.0,1.0],[1.0,-1.0]]

rclpy.init()
msg = Twist()
node = rclpy.create_node('Turtle_controll')
publisher = node.create_publisher(Twist,'/turtle1/cmd_vel',10)

cap = cv2.VideoCapture(0) 

while cap.isOpened():
    # read frame
    _, frame = cap.read()
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # process the frame for pose detection
    pose_results = pose.process(frame_rgb)
    left_shoulder = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_SHOULDER]
    left_elbow = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_ELBOW]
    right_elbow = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_ELBOW]        
    left_wrist = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_WRIST]
    right_wrist = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_WRIST]     

    data = [left_shoulder.x/(1/left_shoulder.z),left_shoulder.y/(1/left_shoulder.z),right_shoulder.x/(1/right_shoulder.z),right_shoulder.y/(1/right_shoulder.z),left_elbow.x/(1/left_elbow.z),left_elbow.y/(1/left_elbow.z),right_elbow.x/(1/right_elbow.z),right_elbow.y/(1/right_elbow.z),left_wrist.x/(1/left_wrist.z),left_wrist.y/(1/left_wrist.z),right_wrist.x/(1/right_wrist.z),right_wrist.y/(1/right_wrist.z)]
    data = np.array(data).reshape(1,-1)
    output = model.predict(data)
    output = np.argmax(output)
    print(class_labels[output])
    msg.linear.x, msg.angular.z = command_label[output][0], command_label[output][1]
    publisher.publish(msg)
    # draw skeleton on the frame
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