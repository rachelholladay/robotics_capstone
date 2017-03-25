
import cv2
import cv2.cv as cv
from time import time
import numpy

boxes = []

def on_mouse(event, x, y, flags, params):
    # global img

    if event == cv.CV_EVENT_LBUTTONDOWN:
        print 'Start Mouse Position: '+str(x)+', '+str(y)
        sbox = [x, y]
        boxes.append(sbox)

    elif event == cv.CV_EVENT_LBUTTONUP:
        print 'End Mouse Position: '+str(x)+', '+str(y)
        ebox = [x, y]
        boxes.append(ebox)
        cv2.line(img, (boxes[-2][0], boxes[-2][1]), (boxes[-1][0], boxes[-1][1]), (255, 0, 0), 5)

img = numpy.zeros((500, 500), numpy.uint8)
cv2.namedWindow('DrawingCanvas')
cv.SetMouseCallback('DrawingCanvas', on_mouse)

while(1):
    cv2.imshow('DrawingCanvas',img)
    if cv2.waitKey(10) == ord('q'):
        break
cv2.destroyAllWindows()

#TODO if we can quit then after this write all data to a file
