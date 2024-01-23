import cv2
import mediapipe as mp
#import math
import time
from datetime import datetime
## initialize pose estimator
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f'data{current_time}.txt'
out = open(filename,'x')
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_holistic = mp.solutions.holistic
start = time.time()
state = ['lup','rup','ad','au','']
i=0
cap = cv2.VideoCapture(0) 
while cap.isOpened():
    # read frame
    _, frame = cap.read()
    try:
        # resize the frame for portrait video
        #frame = cv2.resize(frame, (350, 600))
        # convert to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # process the frame for pose detection
        pose_results = pose.process(frame_rgb)
        #print(pose_results.pose_landmarks)
        left_shoulder = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_SHOULDER]
        left_elbow = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_ELBOW]
        right_elbow = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_ELBOW]        
        left_wrist = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_WRIST]
        right_wrist = pose_results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_WRIST]
        #left_angle = abs(math.atan(left_shoulder.x-left_elbow.x/left_elbow.y-left_shoulder.y))
        #right_angle = abs(math.atan(right_shoulder.x-right_elbow.x/right_elbow.y-right_shoulder.y))
        
        #left_angle, right_angle = num_to_range(left_angle,0.2,1.2,-1,1), num_to_range(left_angle,0.2,1.2,-1,1)
        #print(left_angle, right_angle)
        # draw skeleton on the frame
        #out.write(f'left_shoulder {left_shoulder}right_shoulder {right_shoulder}left_elbow {left_elbow}right_elbow {right_elbow}left_wrist {left_wrist}right_wrist {right_wrist}')
        if time.time() - start > 160:
            time.sleep(5)
            i += 1
            print(state[i])
            start = time.time()
        out.write(f'{state[i]},{left_shoulder.x},{left_shoulder.y},{left_shoulder.z},{left_shoulder.visibility},{right_shoulder.x},{right_shoulder.y},{right_shoulder.z},{right_shoulder.visibility},{left_elbow.x},{left_elbow.y},{left_elbow.z},{left_elbow.visibility},{right_elbow.x},{right_elbow.y},{right_elbow.z},{right_elbow.visibility},{left_wrist.x},{left_wrist.y},{left_wrist.z},{left_wrist.visibility},{right_wrist.x},{right_wrist.y},{right_wrist.z},{right_wrist.visibility}\n')
        mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        # display the frame
        cv2.imshow('Output', frame)
    except:
        #break
        pass
    
    if cv2.waitKey(1) == ord('q'):
        break
          
cap.release()
cv2.destroyAllWindows()