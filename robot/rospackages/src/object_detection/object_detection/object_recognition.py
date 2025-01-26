import cv2
import argparse
from ultralytics import YOLO
import supervision as sv

# Augmenting camera resolution. 
def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 live")
    parser.add_argument(
        "--webcam-resolution", 
        default=[1280, 720],
        nargs=2, 
        type = int
    )
    args=parser.parse_args()
    return args


def main():
    args=parse_arguments()
    frame_width, frame_height = args.webcam_resolution

    #Configuring the resolution of webcam 
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    #Loading pre-trained YOLOv8 model  
    model = YOLO("yolov8l.pt")


    #Creating bouding boxes and labels around the object using roboflow supervision
    label_annotator = sv.LabelAnnotator(
        text_thickness=2,
        text_scale=1
    )

    box_annotator = sv.BoxAnnotator(
        thickness=2
    )

    while True:
        #creating video capture
        ret, frame = cap.read()

        result = model(frame)[0]
        detections = sv.Detections.from_ultralytics(result)
        frame = label_annotator.annotate(scene=frame, detections=detections)
        frame = box_annotator.annotate(scene=frame, detections=detections)
        
        
        cv2.imshow("yolov8", frame)

        # To break from the while loop (aka, the camera footage), you need to press esc key.  
        if(cv2.waitKey(30) == 27):
            break

if __name__ == "__main__":
    main()