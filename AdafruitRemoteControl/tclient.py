from imutils.video import VideoStream
import imagezmq
import socket
import time
import cv2
import argparse

parser = argparse.ArgumentParser(description='Client for camera streaming')
parser.add_argument('--ip',required=True,type=str,default=None,help='IP address of server')
args = parser.parse_args()
assert args.ip is not None

sender = imagezmq.ImageSender(connect_to="tcp://"+args.ip+":5555")
rpiName = socket.gethostname()
vs = VideoStream(usePiCamera=True,resolution=(320,240),framerate=20).start()
time.sleep(2.0)

while True:
    frame = vs.read()
    sender.send_image(rpiName,frame)

    #cv2.imshow("Frame",frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()
vs.release()
