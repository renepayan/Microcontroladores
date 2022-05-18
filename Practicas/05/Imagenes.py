'''
    Este programa sirve para convertir cualquier imagen a blanco y negro, y prepararla para el
    micro controlador
'''
TK_SILENCE_DEPRECATION=1 
from multiprocessing.dummy import Array
from tkinter import Tk     # from tkinter import Tk for Python 3.x
from tkinter.filedialog import askopenfilename
import cv2
import math

cantidadDeLeds:int = int(input("Cuantos leds tiene el POV: "))
Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file
originalImage = cv2.imread(filename)
grayImage = cv2.cvtColor(originalImage, cv2.COLOR_BGR2GRAY)
(thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
imagenReescalada = cv2.resize(blackAndWhiteImage, (2*cantidadDeLeds,2*cantidadDeLeds))
rows,cols = blackAndWhiteImage.shape
hexFinal:str = ".WORD "
cv2.imshow('Black white image', blackAndWhiteImage)
#cv2.imshow('Original image',originalImage)
#cv2.imshow('Gray image', grayImage)
#cv2.imshow('Reescalated image', imagenReescalada)
for i in range(0,360):
    valorDecimal:int = 0
    x = math.sin(math.radians(i)) #Obtenemos seno y coseno del angulo
    y = math.cos(math.radians(i))
    for j in range(0,cantidadDeLeds):
        nx:int = cantidadDeLeds + x*j #Ahora calculamos las coordenadas, acorde al numero de led
        ny:int = cantidadDeLeds + y*j
        #print("X: "+str(int(nx))+",Y: "+str(int(ny))+", Color: "+str(imagenReescalada[int(nx),int(ny)]))
        if(imagenReescalada[int(nx),int(ny)] == 0): #Es un pixel con color            
            valorDecimal += 1 << j
    hexFinal+=hex(valorDecimal)+", "
hexFinal = hexFinal.replace("0x","0x00")
hexFinal+=hex(65535)
hexFinal = hexFinal.replace("a","A")
hexFinal = hexFinal.replace("b","B")
hexFinal = hexFinal.replace("c","C")
hexFinal = hexFinal.replace("d","D")
hexFinal = hexFinal.replace("e","E")
hexFinal = hexFinal.replace("f","F")
print(hexFinal)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
