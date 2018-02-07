
import cv2
import sys
import numpy as np

lookingforsign = True   # first look for sign, then look for yellow line 
threshold = 20          # number of consecutive frames that detect a sign
count = 0		# used to count the frames for the threshold
pixelsignsize = 180     # will only detect signs larger than this value
percentofscreen = 5     # threshold to determine if we stop or not
notStopped = True       # boolean to calculate percentofscreen

cascPath = sys.argv[1]
cascPath2 = sys.argv[2]
myCascade = cv2.CascadeClassifier(cascPath)
video_capture = cv2.VideoCapture(int(cascPath2))

# function used to define a region to look for the yellow line
def region_of_interest(image):
    rows, cols = image.shape[:2]
    bottom_left  = [cols*0.1, rows*0.95]
    top_left     = [cols*0.4, rows*0.6]
    bottom_right = [cols*0.9, rows*0.95]
    top_right    = [cols*0.6, rows*0.6]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]],dtype=np.int32)
    mask=np.zeros_like(image)
    if len (image.shape)>2:
	channel_count=image.shape[2]
	ignore_mask_color = (255,)*channel_count
    else:
	ignore_mask_color=255
    cv2.fillPoly(mask,vertices,ignore_mask_color)
   
    return cv2.bitwise_and(image,mask)

while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()
    #cv2.imshow('Video', frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if(lookingforsign):	
	    detObjs = myCascade.detectMultiScale(
		gray,
		scaleFactor=1.1,
		minNeighbors=5,
		minSize=(30, 30),
	    )

	    # Draw a rectangle around the detected objects
	    for (x, y, w, h) in detObjs:
		if(w>pixelsignsize and h>pixelsignsize):
		    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
		    count = count+1
	    if(len(detObjs) == 0):
		    count = 0

	    if(count > threshold):
		print("Found a stop sign.  STOP!!! Check for pedestrians... GO!!!")
		lookingforsign = False
		count = 0
    else:
	    #convert BGR frame to HSV to easily detect yello
	    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	    
	    #set yellow thresholds
	    lower_yellow=np.array([20,100,100],dtype="uint8")
	    upper_yellow=np.array([30,255,255],dtype="uint8")
	    
	    #create yellow mask
	    yellow=cv2.inRange(hsv,lower_yellow,upper_yellow)
	    mask_yellow=cv2.bitwise_and(gray,yellow)
	    frame = region_of_interest(mask_yellow)
	 
	    #determin if percenofscreen has been reached   
	    if (notStopped):	
	        for pixel in frame:
	            if(cv2.countNonZero(frame)>percentofscreen*.01*frame.size):
		        print("we should stop the car...")
			#notStopped = False
                        lookingforsign = True
			break

    # Display the resulting frame
    #cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()

