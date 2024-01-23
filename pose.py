import cv2
import mediapipe as mp
import math
## initialize pose estimator
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_holistic = mp.solutions.holistic


def num_to_range(num, inMin, inMax, outMin, outMax):
    return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))

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
        
        left_angle = abs(math.atan(left_shoulder.x-left_elbow.x/left_elbow.y-left_shoulder.y))
        right_angle = abs(math.atan(right_shoulder.x-right_elbow.x/right_elbow.y-right_shoulder.y))
        
        left_angle, right_angle = num_to_range(left_angle,0.2,1.2,-1,1), num_to_range(left_angle,0.2,1.2,-1,1)
        print(left_angle, right_angle)
        # draw skeleton on the frame
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