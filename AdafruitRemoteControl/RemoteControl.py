from utils import *
import Tkinter as tk
import subprocess

mh,mm1,mm2 = setUpMotors()

def key_input(event):
    key_press = event.keysym.lower()
    print(key_press)
    
    if key_press == 'w':
        w(mm1,mm2)
    elif key_press == 's':
        s(mm1,mm2)
    elif key_press == 'a':
        a(mm1,mm2)
    elif key_press == 'd':
        d(mm1,mm2)
    elif key_press == 'q':
        turnOffMotors(mh)

command = tk.Tk()
command.bind_all('<Key>',key_input)
command.mainloop()
