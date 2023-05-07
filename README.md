# 👑 **REP COUNTER** 👑

## Introduction 

  This project utilizes Mediapipes cutting edge Pose esitmation model and OpenCv's image processing to count repetitions of exercises in real-time. The program can calculate angles and distances between specific body parts, and use this information to count the number of repetitions performed during an exercise. The full explanation and details are listed in the **Explanation** section. To get started, please read **Usage**.

## Explanation
  
   * ## Step 1: Video Processing 
     ```Python
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
   ```

