from utils import *

tgt = 'rtsp://192.168.100.100:8557/h264'
# Blake video loader 
streamload  = StreamLoader(tgt)

while streamload.get_frame() is None:
    continue

while True:
    ret,frame = streamload.get_frame()
    if ret:
        cv2.imshow('frame',cv2.resize(frame,(960,540)))
        k = cv2.waitKey(1) & 0xFF

        if k == ord('q'):
            break

streamload.free()
