import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

#Constants GPIO
GPIOMotor1 = 33 # motor da esquerda
GPIOMotor2 = 12 # motor da direita
FREQUENCY = 50
MIN_ROT = 15

COLOR_RED = [0,0,255]
V0 = 17
KP = V0/120
ROBOT_PLAY = False

def line_detection(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 170, apertureSize = 3)
    lines = cv2.HoughLines(edges,1,np.pi/180, 50)
    #print(len(lines))
    #for x in range(0, len(lines), 1):
    aw = 1
    bw = 0
    if lines is not None:
        for r,theta in lines[0]:
            # Stores the value of cos(theta) in a
            a = np.cos(theta)

            # Stores the value of sin(theta) in b
            b = np.sin(theta)

            # x0 stores the value rcos(theta)
            x0 = a*r

            # y0 stores the value rsin(theta)
            y0 = b*r

            # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
            x1 = int(x0 + 1000*(-b))

            # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
            y1 = int(y0 + 1000*(a))

            # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
            x2 = int(x0 - 1000*(-b))

            # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
            y2 = int(y0 - 1000*(a))

            # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
            # (0,0,255) denotes the colour of the line to be
            #drawn. In this case, it is red.
            cv2.line(img,(x1,y1), (x2,y2), COLOR_RED, 2)
            aw = a
            bw = b
    return img, aw, bw

def get_angle(coef_ang):
    angle = np.arccos(coef_ang)
    angle *= (180/np.pi)
    if angle > 90:
        angle -= 180
    return angle

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) 
GPIO.setup(GPIOMotor1, GPIO.OUT)
GPIO.setup(GPIOMotor2, GPIO.OUT)
pwm1 = GPIO.PWM(GPIOMotor1, FREQUENCY) 
pwm2 = GPIO.PWM(GPIOMotor2, FREQUENCY)
pwm1.start(0)
pwm2.start(0)

cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)
v_esq = 0
v_dir = 0
while(True): #Capture frame-by-frame
    ret, frame = cap.read()
    if ret:
        frame_line, a, b = line_detection(frame)
        angle = get_angle(a)
        cv2.imshow('Video Original',frame)
    if ROBOT_PLAY:
        v_esq = V0+(KP*angle)
        v_dir = V0-(KP*angle)
        print(v_esq, v_dir)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        if ROBOT_PLAY:
            ROBOT_PLAY = False
            v_esq = 0
            v_dir = 0
        else:
            ROBOT_PLAY = True
    pwm1.ChangeDutyCycle(v_esq)
    pwm2.ChangeDutyCycle(v_dir)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()

'''
frame = cv2.imread('image3.jpeg')
frame = cv2.resize(frame,(320, 240))
cv2.imshow('Frame Original', frame)
ini = time.perf_counter()
frame_line, a, b = line_detection(frame)
print(time.perf_counter() - ini)
cv2.imshow('Frame Detected 1', frame_line)
#print(a,b)
cv2.waitKey(0)
'''

GPIO.cleanup()
cv2.destroyAllWindows()
