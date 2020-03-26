from Adafruit_MotorHAT import Adafruit_MotorHAT,Adafruit_DCMotor
import imutils
from imutils.video import VideoStream
import time
import atexit
import imagezmq
import socket
import cv2
import numpy as np

mh = Adafruit_MotorHAT(addr=0x60)

def turnOffMotors(mh):
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

def setUpMotors():
    mh = Adafruit_MotorHAT(addr=0x60)
    mm1 = mh.getMotor(1)
    mm2 = mh.getMotor(2)
    return mh,mm1,mm2

def setUpClient(ipadd):
    add = "tcp://{}:5555".format(ipadd)
    sender = imagezmq.ImageSender(connect_to=add)
    rpiName = socket.gethostname()
    vs = VideoStream(usePiCamera=True,resolution=(320,240),
            framerate=20).start()
    return sender,vs,rpiName

def w(mm1,mm2):
    mm1.run(Adafruit_MotorHAT.FORWARD)
    mm2.run(Adafruit_MotorHAT.FORWARD)
    mm1.setSpeed(100)
    mm2.setSpeed(100)
    time.sleep(.05)
    mm1.run(Adafruit_MotorHAT.RELEASE)
    mm2.run(Adafruit_MotorHAT.RELEASE)

def s(mm1,mm2):
    mm1.run(Adafruit_MotorHAT.BACKWARD)
    mm2.run(Adafruit_MotorHAT.BACKWARD)
    mm1.setSpeed(100)
    mm2.setSpeed(100)
    time.sleep(.05)
    mm1.run(Adafruit_MotorHAT.RELEASE)
    mm2.run(Adafruit_MotorHAT.RELEASE)

def a(mm1,mm2):
    mm1.run(Adafruit_MotorHAT.FORWARD)
    mm2.run(Adafruit_MotorHAT.FORWARD)
    mm1.setSpeed(200)
    mm2.setSpeed(100)
    time.sleep(.05)
    mm1.run(Adafruit_MotorHAT.RELEASE)
    mm2.run(Adafruit_MotorHAT.RELEASE)

def d(mm1,mm2):
    mm1.run(Adafruit_MotorHAT.FORWARD)
    mm2.run(Adafruit_MotorHAT.FORWARD)
    mm1.setSpeed(100)
    mm2.setSpeed(200)
    time.sleep(.05)
    mm1.run(Adafruit_MotorHAT.RELEASE)
    mm2.run(Adafruit_MotorHAT.RELEASE)

