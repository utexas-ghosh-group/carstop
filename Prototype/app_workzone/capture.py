import cv2
import numpy as np

import prototype.app_workzone.darknet as darknet
from options import focalLength, knownHeight, mapfile

# prepare camera
cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)
cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

# Prepare yolo
yolo_net, yolo_meta, yolo_im = darknet.load(720, 1280)

def call_meth(img):
    dist_out =[]

    #define the centroids of each axis to calculate relative position of each person    
    h,w,c = img.shape
    Xcent = w/2
    Ycent = h/2
    
    #count of the number of people found
    numDet = 0

    #string containing all the relative positions of people in the frame
    locs = ""
    
    result = darknet.detect(yolo_net, yolo_meta, yolo_im, img[:,:,::-1])
    
    for i in range (0, len(result)):
        tl = (result[i]['topleft']['x'], result[i]['topleft']['y'])
        br = (result[i]['bottomright']['x'], result[i]['bottomright']['y'])
        label = result[i]['label']
    
        if (label == "person") or (label =="car"):
            person_h = result[i]['bottomright']['y'] - result[i]['topleft']['y']
            distance = (knownHeight * focalLength) / person_h
            textLabel = "Distance: " + str(distance ) + " m"
            img = cv2.rectangle(img, tl, br, (0, 255, 0), 3)
            img = cv2.putText(img, textLabel, tl, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
            xmid = (result[i]['topleft']['x'] + result[i]['bottomright']['x'])/2
            ymid = (result[i]['topleft']['y'] + result[i]['bottomright']['y'])/2
            angle = ((img.shape[1]/2 - xmid)/ np.abs(img.shape[1]/2))*np.pi/4     
            distance = distance / np.cos(np.abs(angle))
            
            x = (xmid - Xcent)/Xcent
            y = (ymid - Ycent)/Ycent
        
            locs+= str(x) + " " + str(y) + "\n"
        
            numDet+=1
            dist_out.append((label, distance, np.degrees(angle)))         
    output = str(numDet) + "\n" + locs
    return (result,img,output,dist_out)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Display the resulting frame
    cv2.imshow('frame',frame)
    result, img, output, dist_out = call_meth(frame)
    print(dist_out)
    with open(mapfile, 'w') as out:
        for line in dist_out:
            out.write(','.join(str(word) for word in line))
            out.write('\r\n')
    cv2.imshow('frame', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
    cv2.waitKey(50)
cap.release()
cv2.destroyAllWindows() 
