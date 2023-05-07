# 👑 **REP COUNTER** 👑

## Introduction 

  This project utilizes Mediapipes cutting edge Pose esitmation model and OpenCv's image processing to count repetitions of exercises in real-time. The program can calculate angles and distances between specific body parts, and use this information to count the number of repetitions performed during an exercise. The full explanation and details are listed in the **Explanation** section. To get started, please read **Usage**.

## Explanation
   * ## Step 1: Video Processing 
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

   * The main funciton feeds the video to be analyzed into the PoseDetector class, which will now use Mediapipe to search for "landmarks", or the location of each body part. 
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
  * If MediaPipe locates any landmarks, they are added to `self.landmarks` and drawn onto the returned image
