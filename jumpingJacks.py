import poseDetection
import cv2

video = cv2.VideoCapture(0)

def main():

    detector = poseDetection.PoseDetector()

    while True:
        
        check, frame = video.read()

        detector.set_image(frame)
        foundPose, poseDrawn = detector.set_pose()

        if foundPose:
            poseDrawn = detector.runJumpingJackDetection(poseDrawn, True) # in addition to running the detection, it outputs an image with angle values written on as text


        cv2.putText(poseDrawn, f"{(int(detector.get_jumping_jack_reps()))} reps", (600, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

            
        cv2.imshow("Camera", poseDrawn)
        key = cv2.waitKey(1)


        if key == ord("x"):
            break
main()