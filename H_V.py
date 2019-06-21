# imports
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# constantes
COLOR_RED   = [0,0,255]
COLOR_GREEN = [0,255,0]
COLOR_BLUE  = [255,0,0]
GPIOMotor1 = 33 #motor da esquerda
GPIOMotor2 = 12 #motor da direita
FREQUENCY = 1000
MIN_ROT = 20
V0 = MIN_ROT*(4/3)
KP = 1.6*V0/90
RES_X = 320
RES_Y = 240
ROBOT_PLAY = False

def line_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 50, apertureSize = 3)
    lines = cv2.HoughLines(edges,1,np.pi/180, 50)

    # se nao encontrar linha retorna os valores default
    if lines is not None:
        for r,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*r
            y0 = b*r
            x1 = int(x0+1000*(-b))
            y1 = int(y0+1000*(a))
            x2 = int(x0-1000*(-b))
            y2 = int(y0-1000*(a))
            cv2.line(frame,(x1,y1), (x2,y2), COLOR_RED, 2)
            # distancia do meio ao ponto na reta na base do frame 
            if x1 != x2: c_ang = (y1-y2)/(x1-x2)
            else: c_ang = 1000
            px_meio = (RES_X-1)//2
            px_linha = ((RES_Y-1)-(y1-(c_ang*x1)))//c_ang
            dist_meio = px_linha - px_meio
            return theta, dist_meio

    return 0, 0

def get_speed(angle, dist_meio):
    fator = KP * (angle+(dist_meio/5))
    v_esq = V0 + fator
    v_dir = V0 - fator
    
    if v_esq == v_dir: v_esq = v_dir = v_dir+5 # vel bonus de alinhamento
    
    # correcao nos valores para o GPIO.PWM()
    if v_esq < 0:
        v_esq = 0
        v_dir = MIN_ROT
    if v_dir < 0:
        v_dir = 0
        v_esq = MIN_ROT
    if v_esq > 100: v_esq = 100
    if v_dir > 100: v_dir = 100
    return v_esq, v_dir
    
def get_angle(coef_ang):
    angle = coef_ang # angulo em rads
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
cap.set(3, RES_X)
cap.set(4, RES_Y)
v_esq = 0
v_dir = 0
a = 0

while(True): #Capture frame-by-frame
    ret, frame = cap.read()
    if ret:
        a, dm = line_detection(frame)
        angle = get_angle(a)
        cv2.imshow('Line Writer', frame)
    if ROBOT_PLAY:
        v_esq, v_dir = get_speed(angle, dm)
    
    if cv2.waitKey(1) & 0xFF == ord('s'):
        if ROBOT_PLAY:
            ROBOT_PLAY = False
            v_esq = v_dir = 0
        else:
            v_esq = v_dir = 100
            ROBOT_PLAY = True
    pwm1.ChangeDutyCycle(v_esq)
    pwm2.ChangeDutyCycle(v_dir)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()
