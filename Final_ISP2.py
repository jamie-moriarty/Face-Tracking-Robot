# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from Adafruit_PWM_Servo_Driver import PWM

# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

mult = 4                  #max 3   
servoMin = 150*mult  # Min pulse length out of 4096
servoMax = 600*mult  # Max pulse length out of 4096
pwm.setPWMFreq(60*mult)      # Set frequency to 60 Hz
servorange = servoMax-servoMin


face_cascade = cv2.CascadeClassifier('/home/pi/opencv-3.2.0/data/haarcascades/haarcascade_frontalface_default.xml')
profile_cascade = cv2.CascadeClassifier('/home/pi/opencv-3.2.0/data/haarcascades/haarcascade_profileface.xml')

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(320, 240))
(cCols,cRows) = camera.resolution

# allow the camera to warmup
time.sleep(0.1)

midScreenx = cCols/2
midScreeny = cRows/2

midPos = 375*mult
midPosition = (midPos,midPos,midPos)
position = (midPos,midPos,midPos)
image_scale = 1

differenceX = 0
differenceY = 0
differenceZ = 0

def move_servos(x,y,z):
    pwm.setPWM(0, 0, x)
    pwm.setPWM(1, 0, y)
    pwm.setPWM(2, 0, z)

def hide_1( position ):
    shy_position = (servoMin + servorange/3,servoMax - servorange/3,servoMin + servorange/5)
    while (position != shy_position):
        position = (
            position[0] + max(min(shy_position[0]-position[0],1),-1),
            position[1] + max(min(shy_position[1]-position[1],1),-1),
            position[2] + max(min(shy_position[2]-position[2],1),-1) )
        move_servos(position[0],position[1],midPos)
        time.sleep(.001)

    time.sleep(4.)
    
    while (position != midPosition):
        position = (
            position[0] + max(min(midPosition[0]-position[0],1),-1),
            position[1] + max(min(midPosition[1]-position[1],1),-1),
            position[2] + max(min(midPosition[2]-position[2],1),-1) )
        move_servos(position[0],position[1],midPos)
        time.sleep(.001)

def hide_2( position ):
    shy_position = (servoMax - servorange/3,servoMax - servorange/3,servoMax - servorange/5)
    while (position != shy_position):
        position = (
            position[0] + max(min(shy_position[0]-position[0],1),-1),
            position[1] + max(min(shy_position[1]-position[1],1),-1),
            position[2] + max(min(shy_position[2]-position[2],1),-1) )
        move_servos(position[0],position[1],midPos)
        time.sleep(.001)

    time.sleep(4.)
    
    while (position != midPosition):
        position = (
            position[0] + max(min(midPosition[0]-position[0],1),-1),
            position[1] + max(min(midPosition[1]-position[1],1),-1),
            position[2] + max(min(midPosition[2]-position[2],1),-1) )
        move_servos(position[0],position[1],midPos)
        time.sleep(.001)

def gotocentre( position ):
    while (position != midPosition):
        position = (
            position[0] + max(min(midPosition[0]-position[0],1),-1),
            position[1] + max(min(midPosition[1]-position[1],1),-1),
            position[2] + max(min(midPosition[2]-position[2],1),-1) )
        move_servos(position[0],position[1],position[2])
        time.sleep(.001)

# facial detection
  
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	img = frame.array
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	profile = profile_cascade.detectMultiScale(gray, 1.3, 1) 
        for (x, y, w, h) in profile:
                pt1 = (int(x * image_scale), int(y * image_scale))
                pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                cv2.rectangle(img, (x,y), (x+w, y+h), (0, 255, 0), 2)
                # get the xy corner co-ords, calc the centre location
                x1 = pt1[0]
                x2 = pt2[0]
                y1 = pt1[1]
                y2 = pt2[1]
                centrex = x1+((x2-x1)/2)
                centrey = y1+((y2-y1)/2)
                centre = (centrex, centrey)

                if centre is not None:
                    cx = centre[0]
                    cy = centre[1]
                    if abs(cx-midScreenx)>5 and abs(cy-midScreeny)>5:
                      differenceX = 1
                      differenceY = 1
                      print ' move 1 '

                    if   abs(cx-midScreenx)>70:
                      differenceX = 7
                    elif abs(cx-midScreenx)>60:
                      differenceX = 6
                    elif abs(cx-midScreenx)>50:
                      differenceX = 5
                    elif abs(cx-midScreenx)>40:
                      differenceX = 4
                    elif abs(cx-midScreenx)>30:
                      differenceX = 3
                    elif abs(cx-midScreenx)>20:
                      differenceX = 2
                      
                    if   abs(cy-midScreeny)>40:
                      differenceY = 7
                    elif abs(cy-midScreeny)>35:
                      differenceY = 6
                    elif abs(cy-midScreeny)>30:
                      differenceY = 5
                    elif abs(cy-midScreeny)>25:
                      differenceY = 4
                    elif abs(cy-midScreeny)>15:
                      differenceY = 3
                    elif abs(cy-midScreeny)>12:
                      differenceY = 2

                    # Change difference for Up/Down L/R

                    if cx-midScreenx>0:
                      differenceX = -differenceX
                    if cy-midScreeny<0:
                      differenceY = -differenceY

                    # Z axis Tilt - facial recognition does not work if camera is at an angle..

##                    if position[0]+differenceX>servoMax-servorange/4 and position[2]<servoMax-servorange/5:
##                      differenceZ = 1
##                      print 'x axis at top range '
##                    elif position[0]+differenceX<servoMin+servorange/4 and position[2]>servoMin+servorange/5:
##                      differenceZ = -1
##                      print 'x axis at lower range '
##
##                    elif position[0]>servoMin+servorange/4 and position[0]<servoMax-servorange/4:
##                      " if x asis is not in upper or lower range go back to center one at a time "
##                      if position[2] > servoMin+servorange/5:
##                          differenceZ = -1
##                      elif position[2] < servoMin+servorange/5:
##                          differenceZ = 1
##
                          
                    position = (max(min(position[0]+differenceX,servoMax),servoMin),
                              max(min(position[1]+differenceY,servoMax),servoMin),
                              max(min(position[2]+differenceZ,servoMax),servoMin))

                    if(differenceX>0):
                      print 'Pan Right ', " ", position, " ", differenceX, " ", centre

                    elif(differenceX<0):
                      print 'Pan Left ', position, " ", differenceX, " ", centre, centre[0], " ", midScreenx
                    if(differenceY>0):
                      print 'Tilt Down', position, " ", differenceY, " ", centre, " ", centre[0], " ", midScreenx

                    elif(differenceY<0):
                      print 'Tilt Up', position, " ", differenceY, " ", centre, " ", centre[0], " ", midScreeny

                    move_servos(position[0],position[1],position[2])
                    differenceX = 0
                    differenceY = 0
                    differenceZ = 0


	faces = face_cascade.detectMultiScale(gray, 1.3, 7)
	for (fx, fy, fw, fh) in faces:
                pt1 = (int(fx * image_scale), int(fy * image_scale))
                pt2 = (int((fx + fw) * image_scale), int((fy + fh) * image_scale))
                cv2.rectangle(img, (fx,fy), (fx+fw, fy+fh), (255, 0, 0), 2)
                # get the xy corner co-ords, calc the centre location
                fx1 = pt1[0]
                fx2 = pt2[0]
                fy1 = pt1[1]
                fy2 = pt2[1]
                centrefx = fx1+((fx2-fx1)/2)
                centrefy = fy1+((fy2-fy1)/2)
                centref = (centrefx, centrefy)
                print 'face center:', " " , centref , " " , midPosition

	#faces = face_cascade.detectMultiScale(gray, 1.3, 1)+profile_cascade.detectMultiScale(gray, 1.3, 1)

	if len(faces) == 1:
            if centrefx<midScreenx:
                hide_1( position )
                print "hide_1" , " " , centrefx
            if centrefx>midScreenx:
                hide_2( position )
                print "hide_2" , " " , centrefx

                      
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	# show the frame
	cv2.imshow("Frame", img)
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
