'''
This script is to test some functionalities of Dobot Magician.

To see the command lines description in 
https://github.com/luismesas/pydobot/blob/master/pydobot/dobot.py
'''

from serial.tools import list_ports
from pydobot import Dobot
import os
import time
import tqdm


columns = os.get_terminal_size().columns
print('Device Open'.center(columns,'-'))
port = list_ports.comports()[0].device
device = Dobot(port=port,verbose=False)


print('Suction Cup Testing')
device.suck(enable=True)
with tqdm.tqdm(total=100) as pbar:
    for i in range(20):
        time.sleep(.1)
        pbar.update(5)
device.suck(enable=False)
pbar.close()
print('-'*columns)


print('Get Pose')
(x,y,z,r,j1,j2,j3,j4) = device.pose()
print('  Position (x,y,z) = ({:.2f},{:.2f},{:.2f})'.format(x,y,z))
print('  Rotation Angles (r,j1,j2,j3,j4) = ({:.2f},{:.2f},{:.2f},{:.2f},{:.2f})'.format(r,j1,j2,j3,j4))
print('-'*columns)



print('Translation Testing')
print('  along x')
device.wait_for_cmd(device.move_to(x+40,y,z))
(x_,y_,z_,_,_,_,_,_) = device.pose()
print('  Position (x,y,z) = ({:.2f},{:.2f},{:.2f})'.format(x_,y_,z_))
time.sleep(5)
device.wait_for_cmd(device.move_to(x,y,z))
print('  along y')
device.wait_for_cmd(device.move_to(x,y+40,z))
(x_,y_,z_,_,_,_,_,_) = device.pose()
print('  Position (x,y,z) = ({:.2f},{:.2f},{:.2f})'.format(x_,y_,z_))
time.sleep(5)
device.wait_for_cmd(device.move_to(x,y,z))
print('  along z')
device.wait_for_cmd(device.move_to(x,y,z+40))
(x_,y_,z_,_,_,_,_,_) = device.pose()
print('  Position (x,y,z) = ({:.2f},{:.2f},{:.2f})'.format(x_,y_,z_))
time.sleep(5)
device.wait_for_cmd(device.move_to(x,y,z))
print('-'*columns)


print('Translation Mode Testing')
print('  JUMP along x')
device.wait_for_cmd(device.move_to(x+20,y,z,mode=0x00))
device.wait_for_cmd(device.move_to(x,y,z,mode=0x00))
print('  MOVJ along x')
device.wait_for_cmd(device.move_to(x+20,y,z,mode=0x01))
device.wait_for_cmd(device.move_to(x,y,z,mode=0x01))
print('  MOVL along x')
device.wait_for_cmd(device.move_to(x+20,y,z,mode=0x02))
device.wait_for_cmd(device.move_to(x,y,z,mode=0x02))
print('-'*columns)


print('Return Home')
device.set_home(x,y,z,r=0)
device.wait_for_cmd(device.home())


device.close()
print('Device Close'.center(columns,'-'))
