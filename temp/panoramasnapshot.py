import cv2
import numpy as np
from imutils import paths
import imutils
import datetime
import timeit
import os

#sets up camera object and defines picture resolution
cam = cv2.VideoCapture(1)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cv2.namedWindow("test")

#gets current directory
#sets input folder for the panorama images
#sets output folder for the resulted panorama image
path = (os.getcwd())
inputfolder = path + "/temp/panoramainput/"
outputfolder = path + "/temp/panoramashots/"

while True:
    ret, frame = cam.read()
    cv2.imshow("test", frame)
    if not ret:
        break
    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = (str(datetime.datetime.now()) + str(".png"))
        cv2.imwrite(inputfolder + img_name, frame)
        print("{} written!".format(img_name))

#releases webcam object
cam.release()
cv2.destroyAllWindows()

## specifies file path where images to stich are located
imagePaths = sorted(list(paths.list_images(inputfolder)))
images = []

## creates a list of all images to be used for stitching
for imagePath in imagePaths:
    image = cv2.imread(imagePath)
    images.append(image)

print("Number of images: " + str(len(images)))
## initializes stitcher object and stitches the images together
stitcher = cv2.Stitcher_create()
(status, stitched) = stitcher.stitch(images)

start_time = timeit.timeit()
if status == 1:
    print("Error Status: 1 - Not enough key points detected in images.")
    print("You either need additional photos or clearer images. \n Make sure the camera is stable when taking pictures.")
elif status == 2:
    print("Error Status: 2 - Images do not have enough distinguishable points. ")
    print("Try adjusting the lighting or taking additional images.")
elif status == 3:
    print("Error Status: 3 - Unknown Error.")
else:
    ## Saves pre-cropped imaged
    cv2.imwrite(outputfolder + str(datetime.datetime.now()) + "pre-cropped-output.png", stitched)
    ## creates a 10 pixel border around the resulting stitched image
    stitched = cv2.copyMakeBorder(stitched, 10, 10, 10, 10, cv2.BORDER_CONSTANT, (0, 0, 0))
    ## converts the image to black and white in order to
    ## crop out the black border created by stitching
    gray = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]

    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	   cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key=cv2.contourArea)

	# allocate memory for the mask which will contain the
	# rectangular bounding box of the stitched image region
    mask = np.zeros(thresh.shape, dtype="uint8")
    (x, y, w, h) = cv2.boundingRect(c)
    cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)

    minRect = mask.copy()
    sub = mask.copy()

	# keep looping until there are no non-zero pixels left in the
	# subtracted image
    while cv2.countNonZero(sub) > 0:
        # erode the minimum rectangular mask and then subtract
        # the thresholded image from the minimum rectangular mask
        # so we can count if there are any non-zero pixels left
        minRect = cv2.erode(minRect, None)
        sub = cv2.subtract(minRect, thresh)

    cnts = cv2.findContours(minRect.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key=cv2.contourArea)
    (x, y, w, h) = cv2.boundingRect(c)

	# use the bounding box coordinates to extract the our final
	# stitched image
    stitched = stitched[y:y + h, x:x + w]

    cv2.imwrite(outputfolder + str(datetime.datetime.now()) + "-cropped-output.png", stitched)

    end_time = timeit.timeit()
    print(end_time - start_time)

	# display the output stitched image to our screen
    cv2.imshow("Stitched", stitched)
    cv2.waitKey(0)
