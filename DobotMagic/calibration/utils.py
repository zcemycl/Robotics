import cv2
import os
import time
import tqdm
import numpy as np
from pydobot import Dobot
from serial.tools import list_ports
import matplotlib.pyplot as plt
from threading import Lock, Thread
from config  import Config

class StreamLoader:
    def __init__(self, tgt=None):
        if tgt is None:
            tgt = 'rtsp://192.168.100.100:8557/h264'
        print("Opening", tgt)
        self.video = cv2.VideoCapture(tgt)
        self.frame = None
        self.frame_lock = Lock()
        self.last_frame = None
        self.thread = Thread(target=self.update, args=())
        self.thread.start()

    def update(self):
        while True:
            frame_buffer = self.video.read()
            ret, frame = frame_buffer
            self.frame_lock.acquire()
            self.last_frame = ret, frame
            self.frame_lock.release()

    def get_frame(self):
        self.frame_lock.acquire()
        frame = self.last_frame
        self.frame_lock.release()
        return frame

    def free(self):
        self.video.release()
        cv2.destroyAllWindows()

class CalibrateCamera:
    def __init__(self,streamload,recalibrate=True):
        self.mtx = None
        self.dist = None
        self.streamload = streamload
        self.imgsArr = []
        self.objpoints,self.imgpoints = [],[]
        self.columns = os.get_terminal_size().columns

    def record(self):
        count = 0 
        while True:
            ret,frame = self.streamload.get_frame()
            if ret:
                cv2.imshow("Calibration",cv2.resize(frame,(960,540)))
                k = cv2.waitKey(1) & 0xFF

                if k == ord('q'):
                    break
                elif k == ord('s'):
                    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                    self.imgsArr.append(gray)
                    count+=1
                    print('   ScreenShot {} taken'.format(count))

        cv2.destroyAllWindows()
        return self.imgsArr
    def calibrate(self,chessboardspec=(9,6),sqwide=25):
        criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,.001)
        objp = np.zeros((np.prod(chessboardspec),3),np.float32)
        objp[:,:2]=np.mgrid[:chessboardspec[0],:chessboardspec[1]].T.reshape(-1,2)*sqwide
        for img in self.imgsArr:
            ret,corners = cv2.findChessboardCorners(img,chessboardspec,None)
            if ret == True:
                self.objpoints.append(objp)
                corners2 = cv2.cornerSubPix(img,corners,(11,11),(-1,-1),criteria)
                self.imgpoints.append(corners2)
                
                img2 = cv2.drawChessboardCorners(img.copy(),chessboardspec,corners2,ret)
                cv2.imshow('img',cv2.resize(img2,(960,540)))
                cv2.waitKey(500)
        ret,mtx,dist,rvecs,tvecs = cv2.calibrateCamera(self.objpoints,
                self.imgpoints,img.shape[::-1],None,None)
        self.mtx = mtx
        self.dist = dist
        print('  Camera Matrix: ',mtx)
        print('  Distortion Coefficients: ',dist)
        return self.mtx,self.dist
    def undistort(self,ind):
        tmpimg = self.imgsArr[ind].copy()
        dst = undistortImg(tmpimg,self.mtx,self.dist)
        
        plt.figure(figsize=(6,3))
        plt.subplot(1,2,1)
        plt.imshow(dst,cmap='gray')
        plt.axis('off')
        plt.title('after')
        
        plt.subplot(1,2,2)
        plt.imshow(self.imgsArr[ind],cmap='gray')
        plt.axis('off')
        plt.title('before')
        plt.show()
        time.sleep(3)
        cv2.destroyAllWindows()
        
    def save(self,name='CameraCalibrate'):
        np.savez(name,mtx=self.mtx,dist=self.dist)

    def load(self,name='CameraCalibrate'):
        data = np.load(name+'.npz')
        try:
            self.mtx,self.dist = data['mtx'],data['dist']
        except:
            self.mtx,self.dist = data[0],data[1]
        print('  Camera Matrix: ',self.mtx)
        print('  Distortion Coefficients: ',self.dist)
        return self.mtx,self.dist

class ArmCalibrate:
    def __init__(self,streamload,mtx=[],dist=[]):
        self.refh = 20
        self.z_ = None
        self.H = []
        self.imaskG = []
        self.mtx = mtx
        self.dist = dist
        self.awaycoor = (-25,-230,50)
        self.cencoor = (254,8,13)
        self.armcp = np.array([[271.55,7.53],[272.76,91.28],[319.10,6.78],
                           [268.29,-163.73],[207.71,77.28],[304.95,-91.05]])
        self.streamload = streamload

        port = list_ports.comports()[0].device
        device = Dobot(port=port,verbose=False)
        self.device = device
        self.columns = os.get_terminal_size().columns

    def Initialize(self):
        self.device.home()
        (x,y,z,r,j1,j2,j3,j4) = self.device.pose()
        print(' Position (x,y,z) = ({:.2f},{:.2f},{:.2f})'.format(x,y,z))
        print(' Rotation Angles (r,j1,j2,j3,j4) = ({:.2f},{:.2f},{:.2f},{:.2f},{:.2f})'.format(r,j1,j2,j3,j4))

        print('Move Arm Away'.center(self.columns,'-'))
        self.device.wait_for_cmd(self.device.move_to(self.awaycoor[0],self.awaycoor[1],self.awaycoor[2],mode=0x00))
        (x,y,z,r,j1,j2,j3,j4) = self.device.pose()
        print(' Position (x,y,z) = ({:.2f},{:.2f},{:.2f})'.format(x,y,z))
        print(' Rotation Angles (r,j1,j2,j3,j4) = ({:.2f},{:.2f},{:.2f},{:.2f},{:.2f})'.format(r,j1,j2,j3,j4))          
    
    def DepthCalibrate(self):
        self.device.wait_for_cmd(self.device.move_to(self.cencoor[0],self.cencoor[1],self.cencoor[2],mode=0x00))
        print(' - Please move the chessboard away')
        print(' - Move the arm to align the suction cup with the white square')
        time.sleep(5)
        (x_,y_,z_,r_,j1_,j2_,j3_,j4_) = self.device.pose()
        self.z_ = z_
        print('   Depth h = {:.2f}'.format(z_))
        self.device.wait_for_cmd(self.device.move_to(self.cencoor[0],self.cencoor[1],self.cencoor[2],mode=0x00))
        self.device.wait_for_cmd(self.device.move_to(self.awaycoor[0],self.awaycoor[1],self.awaycoor[2],mode=0x00))

    def ArmCamHomography(self,mtx,dist):
        _, frame = self.streamload.get_frame()
        self.mtx = mtx
        self.dist = dist
        imaskG = colorMask(frame,mode='g')
        self.imaskG = imaskG
        pixelsArr, realsArr = [],[]
        for i in range(self.armcp.shape[0]):
            x,y = self.device.pose()[:2]
            self.device.wait_for_cmd(self.device.move_to(x,y,self.refh,mode=0x00))
            self.device.wait_for_cmd(self.device.move_to(self.armcp[i,0],self.armcp[i,1],self.z_+10,mode=0x00))
            time.sleep(5)
            _,frame = self.streamload.get_frame()
            green = MaskImg(frame,self.imaskG)
            dst = undistortImg(green,self.mtx,self.dist)
            imaskY = colorMask(frame,mode='y')
            yellow = MaskImg(frame,imaskY)
            dstY = undistortImg(yellow,self.mtx,self.dist)
            cv2.imshow('green',dst)
            cv2.waitKey(0) & 0xFF
            my,mx = detectColorCenter(dstY)
            realsArr.append((self.armcp[i,:]))
            pixelsArr.append((mx,my))

        cv2.destroyAllWindows()
        H = est_homography(realsArr,pixelsArr)
        self.H = H
        print('Homography: ',H)
        return H

    def Testing(self,mtx,dist,H):
        self.mtx,self.dist,self.H = mtx,dist,H
        print('  - Double left-click a pixel')
        print('  - Press s to align yellow marker with the mouse pointer')
        _, frame = self.streamload.get_frame()
        imaskG = colorMask(frame,mode='g')
        cv2.namedWindow('Testing')
        cv2.setMouseCallback('Testing',draw_circle)
        while True:
            _, frame = self.streamload.get_frame()
            green = MaskImg(frame,self.imaskG)
            dst = undistortImg(green,self.mtx,self.dist)
            cv2.imshow('Testing',dst)
            k = cv2.waitKey(2000) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('s'):
                cv2.setMouseCallback('Testing',draw_circle)
                tmpix = [mouseX,mouseY]
                tmpreal = self.device.pose()[:2]
                tmpix.append(1)
                pixels = np.array(tmpix)
                pred_xy = np.matmul(self.H,pixels)
                self.device.wait_for_cmd(self.device.move_to(tmpreal[0],tmpreal[1],self.refh,mode=0x00))
                self.device.wait_for_cmd(self.device.move_to(pred_xy[0]/pred_xy[-1],pred_xy[1]/pred_xy[-1],self.z_+10,mode=0x00))

                print('    (x,y): ',self.device.pose()[:2])
                print('    (pred x,pred y):',pred_xy[:2]/pred_xy[-1])
    def save(self,name='ArmCameraSync'):
        np.savez(name,mtx=self.mtx,dist=self.dist,H=self.H)

    def load(self,name='ArmCameraSync'):
        data = np.load(name+'.npz')
        try:
            self.mtx,self.dist,self.H = data['mtx'],data['dist'],data['H']
        except:
            self.mtx,self.dist,self.H = data[0],data[1],data[2]
        print('  Camera Matrix: ',self.mtx)
        print('  Distortion Coefficients: ',self.dist)
        print('  Homography: ',self.H)
        return self.mtx,self.dist,self.H



def vectorize(r,rp):
    x1,x2,x3 = r[0],r[1],1
    x1p,x2p = rp[0],rp[1]
    ax = [-x1,-x2,-1,0,0,0,x1*x1p,x2*x1p,x1p]
    ay = [0,0,0,-x1,-x2,-x3,x1*x2p,x2*x2p,x2p]
    return ax,ay

def est_homography(real_pts,pix_pts):
    A = []
    for i in range(len(pix_pts)):
        rp,r = real_pts[i],pix_pts[i]
        ax,ay = vectorize(r,rp)
        A.append(ax)
        A.append(ay)
    A = np.array(A)
    u,s,vh = np.linalg.svd(A,full_matrices=True)
    H = vh[-1,:].reshape((3,3))
    return H


def draw_circle(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouseX,mouseY = x,y

def colorMask(img,mode='g'): # green:g, yellow:y
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    if mode == 'g':
        mask = cv2.inRange(hsv,(36,25,25),(86,255,255))
    elif mode == 'y':
        mask = cv2.inRange(hsv,(22,60,200),(60,255,255))
    imask  = mask>0
    return imask

def undistortImg(img,mtx,dist):
    h,w = img.shape[:2]
    newmtx,roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newmtx,(w,h),5)
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
    x,y,w,h = roi
    dst = dst[y:y+h,x:x+w]
    return dst

def MaskImg(img,imask):
    color = np.zeros_like(img,np.uint8)
    color[imask] = img[imask]
    return color

def detectColorCenter(colormap):
    ix,iy,iz = np.where(colormap>0)
    return np.mean(ix),np.mean(iy)


