#!/usr/bin/env python
class Jedi:
    def __init__(self,name):
        self.name = name
    def say_hi(self):
        print('Hello, my name is: ',self.name)

j = Jedi('ObiWan')
j.say_hi()