# 👑 **REP COUNTER** 👑

## Introduction 

  This project utilizes Mediapipes cutting edge Pose esitmation model and OpenCv's image processing to count repetitions of exercises in real-time. The program can calculate angles and distances between specific body parts, and use this information to count the number of repetitions performed during an exercise. The full explanation and details are listed in the **Explanation** section. To get started, please read **Usage**.

## Explanation
   * ## Step 1: Video Processing 
     - ```Python
     
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

   - The main funciton feeds the video to be analyzed into the PoseDetector class, where all the core functionalities are housed.  
   
