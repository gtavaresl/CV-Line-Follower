import cv2
import math
import time

SATURATION_BLACK = 100
SATURATION_WHITE = 125
FAIXA_RANGE = 50
COLOR_WHITE = [255, 255, 255]
COLOR_BLUE  = [255, 000, 000]
COLOR_GREEN = [000, 255, 000]
COLOR_RED   = [000, 000, 255]

def MinimosQuadrados(data):
    a = 0 # coef angular
    b = 0 # coef linear
    x_med = 0
    y_med = 0
    sum_quad_difx = 0
    sum_difx_y = 0

    for i in range (0, len(data), 1):
        x_med += data[i][0]/len(data)
        y_med += data[i][1]/len(data)

    for i in range (0, len(data), 1):
        sum_quad_difx += pow((data[i][0] - x_med), 2)
        sum_difx_y += (data[i][0] - x_med) * data[i][1]
    if sum_quad_difx != 0:
        a = sum_difx_y/sum_quad_difx
    else:
        a = 10000
    b = y_med - (x_med*a)
    return a, b

def LineWrite(img, p_aa):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    p_collection = []
    p_med = p_aa
    p_med_aux = 0
    for y in range(hsv.shape[0]-1, hsv.shape[0]//2, -1): #percorre linhas (down-up)
        px_ini = 0
        px_fim = 0
        cont_white = 0

        if p_med != 0 and p_med - FAIXA_RANGE > 0:
            range_col_ini = p_med - FAIXA_RANGE
        else:
            range_col_ini = 0

        if p_med != 0 and p_med + FAIXA_RANGE < hsv.shape[1]:
            range_col_fim = p_med + FAIXA_RANGE
        else:
            range_col_fim = hsv.shape[1]

        for x in range(range_col_ini, range_col_fim, 1): #percorre colunas (left-right) acha so um pt no meio da faixa preta
            pixel = hsv[y,x][2]
            if pixel <= SATURATION_BLACK:
                #estou num ponto preto
                if(px_ini == 0): px_ini = x
                if(px_fim < x): px_fim = x
            if pixel >= SATURATION_WHITE: #estou num ponto branco
                cont_white += 1
                if px_fim - px_ini > 10:
                    if cont_white > 10:
                        break
                    else:
                       img[y,x] = COLOR_WHITE
                else:
                   px_ini = 0
                   px_fim = 0
        if px_fim - px_ini > 10:
            p_med_aux = px_ini + (px_fim - px_ini)//2
            if abs(p_med_aux - p_med) < 10 or p_med == 0:
                img[y, p_med_aux] = COLOR_RED
                p_collection.append([p_med_aux, y])
                p_med = p_med_aux
                if len(p_collection) == 1:
                    p_aa = p_med

    a, b = MinimosQuadrados(p_collection)
    cv2.line(img, (int((hsv.shape[0] - b)/a), hsv.shape[0]), (int((hsv.shape[0]/2 - b)/ a), hsv.shape[0]//2), COLOR_BLUE, 3)
    cv2.line(img, (hsv.shape[1]//2, 0), (hsv.shape[1]//2, hsv.shape[0]), COLOR_GREEN, 1)
    return img, a



for i in range(12):
	frame = cv2.imread('testes/i'+str(i)+'.jpg')
	frame = cv2.resize(frame, (320,240))
	cv2.imshow('IMG Original', frame)
	inicio = time.time()
	img, angle = LineWrite(frame, 0)
    print(180*math.atan(angle)/math.pi)
	fim = time.time()
	print(fim - inicio)
	cv2.imshow('IMG',img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

'''
cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)

for i in range(20): # primeiras leituras
    ret, frame = cap.read()

while(True):
    #Capture frame-by-frame
    ret, frame = cap.read()
    p_med = 0
    if ret == True:
        #cv2.imshow('Video Original', frame)
        img, p_med = LineWrite(frame, p_med)
        cv2.imshow('IMG Writer',img)
        #cv2.waitKey(0)
    if cv2.waitKey(60) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
'''