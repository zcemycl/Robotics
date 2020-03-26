from utils import *

def draw_circle(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouseX,mouseY = x,y

columns = os.get_terminal_size().columns

# setup camera view
print('Initialize Camera'.center(columns,'-'))
streamload = StreamLoader()

while streamload.get_frame() is None:
    continue

print('-'*columns)

#############################

# move the arm away from the view
print('Initialize Arm'.center(columns,'-'))
armcal = ArmCalibrate(streamload)
armcal.Initialize()
print('-'*columns)


#############################

print('Camera Calibration Stage'.center(columns,'-'))

recalibrateBol = input('Need to recalibrate the camera? (Y/N)')
if recalibrateBol.lower() == 'y':
    calcam = CalibrateCamera(streamload)
    print(' - Need 10 test patterns or more for camera calibration')
    print(' - Please press s to store the frame')
    print(' - Move the chessboard after one shot is taken')
    imgsArr = calcam.record()
    print('Processing'.center(columns,' '))
    mtx,dist = calcam.calibrate()
    calcam.save()
    print('-'*columns)
    print('Apply Undistortion'.center(columns,'-'))
    calcam.undistort(2)
    print('-'*columns)
elif recalibrateBol.lower() == 'n':
    calcam = CalibrateCamera(streamload,False)
    mtx,dist = calcam.load()

else: 
    streamload.free()


############################

print('Arm Calibration Stage'.center(columns,'-'))
armcal.DepthCalibrate()
print('-'*columns)

###########################

print('Record Arm Calibration Points'.center(columns,'-'))
ArmCameraSyncBol = input('Need to re-sync Arm and Camera? (Y/N)')
if ArmCameraSyncBol.lower() == 'y':
    H = armcal.ArmCamHomography(mtx,dist)
    armcal.save()
elif ArmCameraSyncBol.lower() == 'n':
    _,_,H = armcal.load()
print('-'*columns)

############################

print('Testing'.center(columns,'-'))
armcal.Testing(mtx,dist,H)
print('-'*columns)

############################

print('Power off'.center(columns,'-'))
armcal.device.close()
streamload.free()
print('-'*columns)


