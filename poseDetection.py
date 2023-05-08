import cv2, numpy as np, mediapipe as mp, math


class PoseDetector:
    def __init__ (self):
        
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(min_detection_confidence=0.75)
        self.mpDraw = mp.solutions.drawing_utils
        self.landmarks = []
        self.pushupIsDown = False 
        self.jumped = False
        self.pushup_reps = 0
        self.jumping_jack_reps = 0

        

    def set_image(self, image):
        self.image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.height, self.width, _ = self.image.shape

    def set_pose(self):
        self.results = self.pose.process(self.image)
        if self.results.pose_landmarks != None:
            self.landmarks = self.results.pose_landmarks.landmark
            self.mpDraw.draw_landmarks(self.image, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
            return True, cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)
        else:
            self.landmarks = []
            return False, cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)
        
    def get_landmark(self, bodyPart):
        try: 
            self.landmarks[bodyPart]
        except (IndexError): 
            return []
         
        x = self.landmarks[bodyPart].x * self.width
        y = self.landmarks[bodyPart].y  * self.height
        z = self.landmarks[bodyPart].z 

        return[x,y,z]
    
    def get_angle(self, bp1, bp2, bp3): # bp -> body part, calc angle between 3 body parts
        poseBodyPart = self.mpPose.PoseLandmark.RIGHT_SHOULDER

        if type(bp1) == type(poseBodyPart): # if the coords of body parts are not given
            p1 = self.get_landmark(bp1)
            p2 = self.get_landmark(bp2)
            p3 = self.get_landmark(bp3)
        
        elif type(bp1) == type([]): # if coords of body parts are given
            p1 = bp1
            p2 = bp2
            p3 = bp3

        else: # if we have random input
            return 0

        
        if p1 != [] and p2 != [] and p3 != []: # requires all 3 values to calculate angle

            locations = [p1, p2, p3]
            distances = self.get_distances(locations)
            a, b, c = np.asarray(distances)/1000
            
            angleInRadians = math.acos((a ** 2 + b ** 2 - c ** 2)/ (2 * a * b))
            angleInDegrees = angleInRadians * (180.0 / math.pi)

            return angleInDegrees
        else:
            return 0

    
    def get_distances(self, points): # all set of distances between any number of points (used for a,b,c in a triangle)
        distances = []
        numOfPoints = len(points)
        for i in range(numOfPoints):
            point1 = points[i]
            if i == numOfPoints - 1:
              point2 = points[0]
            else:
              point2 = points[i+1]

            x_dis = abs(point1[0] - point2[0])
            y_dis = abs(point1[1] - point2[1])
            
            
            distance = math.sqrt((abs(y_dis ** 2 + x_dis ** 2)))

            distances.append(distance)
        return distances


    def get_slope(self, bodyPart1, bodyPart2): # slope between 2 body parts (used for upper body straightness detection)
        if self.get_landmark(bodyPart1) != [] and self.get_landmark(bodyPart2) != []:
            
            x1, y1, z1 = self.get_landmark(bodyPart1)
            x2, y2, z2 = self.get_landmark(bodyPart2)

            
            slope = (y1-y2)/(x1-x2) 
            slope = int(slope) + ((z1-z2) * int(slope)) # multiply by slope so depth difference is proportional to slope
            return slope
            
        else:
            return 4000

        
    def pushupDown(self, left_slope, right_slope, angle_right_lower, angle_left_lower, angle_right_upper, angle_left_upper):
        if (-0.5 < left_slope < 0.5) and (-0.5 < right_slope < 0.5): # body is flat
                if (angle_right_lower > 150) and (angle_left_lower > 150): # legs are straight
                    if (0 < angle_right_upper < 70) or (0 < angle_left_upper < 70): #arms are bent
                        return True
                    
                         
        return False
    
    def pushupUp(self, left_slope, right_slope, angle_right_lower, angle_left_lower, angle_right_upper, angle_left_upper):
        if (-0.5 < left_slope < 0.5) and (-0.5 < right_slope < 0.5): # body is flat
                if (angle_right_lower > 150) and (angle_left_lower > 150): # legs are straight
                    if (angle_right_upper > 130) and (angle_left_upper > 130): #arms are straight
                        return True
        return False
    
    
    
    
    
    def runPushupDetection(self, image, displayAngles):
        right_shoulder = self.mpPose.PoseLandmark.RIGHT_SHOULDER
        right_elbow = self.mpPose.PoseLandmark.RIGHT_ELBOW
        right_wrist = self.mpPose.PoseLandmark.RIGHT_WRIST

        left_shoulder = self.mpPose.PoseLandmark.LEFT_SHOULDER
        left_elbow = self.mpPose.PoseLandmark.LEFT_ELBOW
        left_wrist = self.mpPose.PoseLandmark.LEFT_WRIST

        left_hip = self.mpPose.PoseLandmark.LEFT_HIP
        right_hip = self.mpPose.PoseLandmark.RIGHT_HIP

        right_knee = self.mpPose.PoseLandmark.RIGHT_KNEE
        left_knee = self.mpPose.PoseLandmark.LEFT_KNEE

        right_ankle = self.mpPose.PoseLandmark.RIGHT_ANKLE
        left_ankle = self.mpPose.PoseLandmark.LEFT_ANKLE


        angle_right_upper = self.get_angle(right_shoulder, right_elbow, right_wrist)
        angle_left_upper = self.get_angle(left_shoulder, left_elbow, left_wrist)

        left_slope = self.get_slope(left_shoulder, left_hip) 
        right_slope = self.get_slope(right_shoulder, right_hip)

        angle_right_lower = self.get_angle(right_hip, right_knee, right_ankle)
        angle_left_lower = self.get_angle(left_hip, left_knee, left_ankle)


        down = self.pushupDown(left_slope, right_slope, angle_right_lower, angle_left_lower, angle_right_upper, angle_left_upper)
        up = self.pushupUp(left_slope, right_slope, angle_right_lower, angle_left_lower, angle_right_upper, angle_left_upper)


        if down:
            self.pushupIsDown = True
        

            
        if self.pushupIsDown and up:
            self.pushup_reps += 1
            self.pushupIsDown = False

            

        if displayAngles:
            cv2.putText(image, f"{(int(angle_right_upper))} right upper body", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
            cv2.putText(image, f"{(int(angle_left_upper))} left upper body", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)

            cv2.putText(image, f"{(int(left_slope))} left slope", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
            cv2.putText(image, f"{(int(right_slope))} right slope", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255), 3)

            cv2.putText(image, f"{(int(angle_right_lower))} right lower body", (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
            cv2.putText(image, f"{(int(angle_left_lower))}  left lower body", (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255), 3)
        
            return image
        
    def jumpingJackUp(self, lower_body_angle, angle_right_upper, angle_left_upper):
        if lower_body_angle > 35: # legs are far apart

            if angle_right_upper > 150: # arms are wide
                if angle_left_upper > 150: # arms are wide
                    return True
        return False
    
    def jumpingJackDown(self, lower_body_angle, angle_right_upper, angle_left_upper):
        if 25 > lower_body_angle > 0 : # legs are close together
            if 20 > angle_right_upper > 0: # arms are close to body
                if 20 > angle_left_upper > 0: # arms are close to body
                    return True

        return False    
    def runJumpingJackDetection(self, image, displayAngles):
        right_shoulder = self.get_landmark(self.mpPose.PoseLandmark.RIGHT_SHOULDER)
        right_elbow = self.mpPose.PoseLandmark.RIGHT_ELBOW
        right_wrist = self.get_landmark(self.mpPose.PoseLandmark.RIGHT_WRIST)

        left_shoulder = self.get_landmark(self.mpPose.PoseLandmark.LEFT_SHOULDER)
        left_elbow = self.mpPose.PoseLandmark.LEFT_ELBOW
        left_wrist = self.get_landmark(self.mpPose.PoseLandmark.LEFT_WRIST)
        
        left_hip = self.get_landmark(self.mpPose.PoseLandmark.LEFT_HIP)
        right_hip = self.get_landmark(self.mpPose.PoseLandmark.RIGHT_HIP)

        right_ankle = self.get_landmark(self.mpPose.PoseLandmark.RIGHT_ANKLE)
        left_ankle = self.get_landmark(self.mpPose.PoseLandmark.LEFT_ANKLE)

        x = (left_hip[0] + right_hip[0])/2
        y = (left_hip[1] + right_hip[1])/2

        center = [x,y] # mid point of hips

        lower_body_angle = self.get_angle(right_ankle, center, left_ankle)    

        angle_right_upper = self.get_angle(right_hip, right_shoulder, right_wrist)
        angle_left_upper = self.get_angle(left_hip, left_shoulder, left_wrist)

        up = self.jumpingJackUp(lower_body_angle, angle_right_upper, angle_left_upper)
        down = self.jumpingJackDown(lower_body_angle, angle_right_upper, angle_left_upper)

        if up:
            self.jumped = True
        
        if self.jumped and down:
            self.jumping_jack_reps += 1
            self.jumped = False

        if displayAngles:
            cv2.putText(image, f"{(int(angle_right_upper))} right upper body", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
            cv2.putText(image, f"{(int(angle_left_upper))} left upper body", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)

            cv2.putText(image, f"{(int(lower_body_angle))} lower body", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
        
            return image



    def get_pushup_reps(self):
        return self.pushup_reps
    
    def get_jumping_jack_reps(self):
        return self.jumping_jack_reps

def main():

    detector = PoseDetector()

    while True:
        
        check, frame = video.read()

        detector.set_image(frame)
        foundPose, poseDrawn = detector.set_pose()

        if foundPose:
            poseDrawn = detector.runPushupDetection(poseDrawn, True) # in addition to running the detection, it outputs an image with angle values written on as text


        cv2.putText(poseDrawn, f"{(int(detector.get_pushup_reps()))} reps", (600, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)

            
        cv2.imshow("Camera", poseDrawn)
        key = cv2.waitKey(1)


        if key == ord("x"):
            break


if __name__ == "__main__":
    video = cv2.VideoCapture(0)
    main()
