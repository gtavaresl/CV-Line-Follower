# imports
import cv2
import numpy as np
import time
import pygame
import RPi.GPIO as GPIO

# constantes
COLOR_RED   = [0,0,255]
COLOR_GREEN = [0,255,0]
COLOR_BLUE  = [255,0,0]
GPIOMotor1 = 33 #motor da esquerda
GPIOMotor2 = 12 #motor da direita
FREQUENCY = 1000
V0 = 45
V_ROT = 30
KP = 1.6*V0/90
RES_X = 320
RES_Y = 240
ROBOT_PLAY = False

# Define some colors
darkgrey = (40, 40, 40)
lightgrey = (150, 150, 150)
 
# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)
 
    def print(self, screen, textString):
        textBitmap = self.font.render(textString, True, lightgrey)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
       
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15
       
    def indent(self):
        self.x += 10
       
    def unindent(self):
        self.x -= 10
 
pygame.init()
 
# Set the width and height of the screen [width,height]
size = [250, 200]
screen = pygame.display.set_mode(size)
 
pygame.display.set_caption("KH-712")

clock = pygame.time.Clock()

# Get ready to print
textPrint = TextPrint()

def line_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 75, 100, apertureSize = 3)
    lines = cv2.HoughLines(edges,1,np.pi/180, 15)

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
    fator = KP * (angle+(dist_meio/3))
    v_esq = V0 + fator
    v_dir = V0 - fator
    
    #if v_esq == v_dir: v_esq = v_dir = v_dir+5 # vel bonus de alinhamento
    
    # correcao nos valores para o GPIO.PWM()
    if v_esq < 0:
        v_esq = 0
        v_dir = V_ROT
    if v_dir < 0:
        v_dir = 0
        v_esq = V_ROT
    if v_esq > 100: v_esq = 100
    if v_dir > 100: v_dir = 100
    return v_esq, v_dir, fator
    
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
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
rec = cv2.VideoWriter("output.avi", fourcc,17 , (RES_X, RES_Y))
v_esq = 0
v_dir = 0
fator = 0
a = 0
loop = True

while(loop): #Capture frame-by-frame
    ret, frame = cap.read()
    if ret:
        a, dm = line_detection(frame)
        angle = get_angle(a)
        cv2.imshow('Line Writer', frame)
        rec.write(frame)
    if ROBOT_PLAY:
        v_esq, v_dir, fator = get_speed(angle, dm)
    
    if cv2.waitKey(1) & 0xFF == ord('s'):
        if ROBOT_PLAY:
            ROBOT_PLAY = False
            v_esq = v_dir = 0
        else:
            v_esq = v_dir = 100
            ROBOT_PLAY = True
    pwm1.ChangeDutyCycle(v_esq)
    pwm2.ChangeDutyCycle(v_dir)

    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(darkgrey)
    textPrint.reset()
 
    # Get count of joysticks
    textPrint.print(screen, "ROBOT_PLAY: {}".format(ROBOT_PLAY))
    textPrint.print(screen, "Angle: {}".format(angle))
    textPrint.print(screen, "Distance: {}".format(dm))
    textPrint.print(screen, "Velocidades:")
    textPrint.indent()
    textPrint.print(screen, "Esquerda: {}".format(v_esq))
    textPrint.print(screen, "Direita: {}".format(v_dir))
    textPrint.print(screen, "Fator: {}".format(fator))
    textPrint.unindent()
    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
   
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        loop = False
        break

GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()
pygame.quit()