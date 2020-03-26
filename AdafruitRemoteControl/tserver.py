from imutils import build_montages
from datetime import datetime
import numpy as np
import imagezmq 
import imutils
import cv2

imageHub = imagezmq.ImageHub()

while True:
    (rpiName, frame) = imageHub.recv_image()
    imageHub.send_reply(b'OK')
    frame = imutils.rotate(imutils.resize(frame,width=1024),180)
    cv2.imshow("Frame",frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()
