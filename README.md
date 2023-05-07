# 👑 **REP COUNTER** 👑

## Introduction 

  This project utilizes Mediapipes cutting edge Pose esitmation model and OpenCv's image processing to count repetitions of exercises in real-time. The program can calculate angles and distances between specific body parts, and use this information to count the number of repetitions performed during an exercise. The full explanation and details are listed in the **[Explanation](https://github.com/rohandesai1/RepCounter/blob/main/README.md#explanation)** section. To get started, please read **[Usage](https://github.com/rohandesai1/RepCounter/blob/main/README.md#usage)**.

## Usage
  
  To run this, you must do the following:
  
  1. Enter this command to clone the repository
  <br></br>
  `git clone https://github.com/rohandesai1/RepCounter` 
  <br></br>
  2. Insall the required dependencies
  <br></br>
  `pip install -r requirements.txt`
  <br></br>
  3. Run the Program.
  <br></br>
  `python pushups.py` or `python jumpingJacks.py`
   
## Explanation
   * ## Step 1: Video Processing 
     -
       ```Python

        #PART 1 OF THE MAIN FUNCTION

        def main():

          detector = PoseDetector() 

          while True: 

              check, frame = video.read() 

              detector.set_image(frame)

         # SET IMAGE

         def set_image(self, image):
           self.image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
           self.height, self.width, _ = self.image.shape

        ```
     - The main funciton feeds the video to be analyzed into the PoseDetector class, which will now use Mediapipe to search for "landmarks", or the location of each body part. 
       
   
       ```Python
       # PART 1 OF THE __init__ METHOD

       def __init__ (self):
            self.mpPose = mp.solutions.pose
            self.pose = self.mpPose.Pose(min_detection_confidence=0.75)
            self.mpDraw = mp.solutions.drawing_utils
            self.landmarks = []


       def set_pose(self):
            self.results = self.pose.process(self.image)
            if self.results.pose_landmarks != None:
                self.landmarks = self.results.pose_landmarks.landmark
                self.mpDraw.draw_landmarks(self.image, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
                return True, cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)
            else:
                self.landmarks = []
                return False, cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)

       ```

     - If MediaPipe locates any landmarks, they are added to `self.landmarks` and drawn onto the returned image
       ```Python
       # PART 2 OF THE MAIN FUNCTION
       foundPose, poseDrawn = detector.set_pose()

        if foundPose:
            poseDrawn = detector.runPushupDetection(poseDrawn, True)
        ```
     - Once the person is located, the detection is initiated. 
     
   * ## Step 2: Get Angles/Slopes Between Body Parts
   
     ```Python
     # PART 1 OF RUN PUSHUP DETECTION
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
      ```

   - Each body part "index" needed for pushup detection is stored as a pose object 
      ```Python
      # PART 2 OF RUN PUSHUP DETECTION
      angle_right_upper = self.get_angle(right_shoulder, right_elbow, right_wrist)
      angle_left_upper = self.get_angle(left_shoulder, left_elbow, left_wrist)

      left_slope = self.get_slope(left_shoulder, left_hip) 
      right_slope = self.get_slope(right_shoulder, right_hip)

      angle_right_lower = self.get_angle(right_hip, right_knee, right_ankle)
      angle_left_lower = self.get_angle(left_hip, left_knee, left_ankle)
      ```
  
  - These objects are then inputted into methods which calculate the angles and slopes needed 
     ```Python
     # DISTANCE METHOD 
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
        
     # ANGLE METHOD 
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
         a, b, c = np.asarray(distances)/1000 # normalize

         angleInRadians = math.acos((a ** 2 + b ** 2 - c ** 2)/ (2 * a * b))
         angleInDegrees = angleInRadians * (180.0 / math.pi)

         return angleInDegrees
        else:
          return 0
   - The angle between 3 points is calculated by first finding the distances between the 3 points, and then using the arc cosine trigonometric function. Slope is calculated through the regular slope forumla, but also takes the depth into account so pushup detection works at any camera angle
      ```Python
      def get_slope(self, bodyPart1, bodyPart2): # slope between 2 body parts (used for upper body straightness detection)
        if self.get_landmark(bodyPart1) != [] and self.get_landmark(bodyPart2) != []:
            
            x1, y1, z1 = self.get_landmark(bodyPart1)
            x2, y2, z2 = self.get_landmark(bodyPart2)

            
            slope = (y1-y2)/(x1-x2) 
            slope = int(slope) + ((z1-z2) * int(slope)) # multiply by slope so depth difference is proportional to slope
            return slope
            
        else:
            return 4000
      ```
   - Both angle and slope functions utilize the `self.get_landmark` function listed below
     ```Python
     def get_landmark(self, bodyPart):
        try: 
            self.landmarks[bodyPart]
        except (IndexError): # if the landmark/body part cannot be located 
            return []
         
        x = self.landmarks[bodyPart].x * self.width
        y = self.landmarks[bodyPart].y  * self.height
        z = self.landmarks[bodyPart].z 

        return[x,y,z]
     ```
