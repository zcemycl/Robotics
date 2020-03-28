import numpy as np
import imageio
import matplotlib.pyplot as plt
import random
import sys
import argparse
'''
Suceptible-Infected-Removed (SIR) [012]
'''
def press(event,obj):
    sys.stdout.flush()
    if event.key == 'q':
        imageio.mimsave('./SIR.gif',obj.images,fps=30)
        sys.exit(0)

class model:
    def __init__(self,sizGrid=10,spdK=.15,
            sizHuman=100,simT=1000000,
            infR=.2,infP=.05,rmT=50,rmP=.1):
        # Constant
        self.sizGrid = sizGrid
        self.spdConstant = spdK
        self.sizHuman = sizHuman
        self.simTime = simT
        self.infR,self.infP = infR,infP
        self.rmT,self.rmP = rmT,rmP
        # Location and Speed
        self.loc = self.sizGrid*np.random.uniform(0,1,
                (self.sizHuman,2))
        self.spd = self.spdConstant*np.random.normal(0,1,
                (self.sizHuman,2))
        # Infect and Remove Record
        # 1st: Infect (bol), 2nd: Remove (bol), 3rd: Infect Period
        self.state = np.zeros((self.sizHuman,3))
        self.state[np.random.randint(0,self.sizHuman),0] = 1
        # Plot attribute
        self.fig = plt.figure(figsize=(10,5))
        self.fig.canvas.mpl_connect('key_press_event',lambda event:press(event,self))
        plt.ion()
        self.t = 0
        # store SIR
        self.sS,self.sI,self.sR = [],[],[]
        # images
        self.images = []
    def updatePlot(self):
        ax = plt.gca(); ax.clear()
        
        ax1 = plt.subplot(1,2,1)
        ax1.clear()
        rmvBol = self.state[:,1]==1
        ax1.scatter(self.loc[rmvBol,0],self.loc[rmvBol,1],c='b')
        infectBol = [False if rmvBol[i]==True else (self.state[i,0]==1) for i in range(self.state.shape[0])]
        ax1.scatter(self.loc[infectBol,0],self.loc[infectBol,1],c='r')
        susceptBol = (self.state[:,0]==0)&(self.state[:,1]==0)
        ax1.scatter(self.loc[susceptBol==1,0],self.loc[susceptBol==1,1],c='g')
        plt.xlim([0,self.sizGrid])
        plt.ylim([0,self.sizGrid])
        plt.title('Iteration {}'.format(self.t))

        ax2 = plt.subplot(1,2,2)
        ax2.clear()
        S,I,R = np.sum(susceptBol),np.sum(infectBol),np.sum(rmvBol)
        self.sS.append(S); self.sI.append(I); self.sR.append(R)
        ax2.plot(self.sS,'g',label='susceptible')
        ax2.annotate('{}'.format(S),(self.t,S))
        ax2.plot(self.sI,'r',label='infectious')
        ax2.annotate('{}'.format(I),(self.t,I))
        ax2.plot(self.sR,'b',label='recovered')
        ax2.annotate('{}'.format(R),(self.t,R))
        plt.ylabel('Number')
        plt.xlabel('Iterations')
        ax2.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05),shadow=True, ncol=3)
        plt.show()
        self.fig.canvas.draw()
        image = np.frombuffer(self.fig.canvas.tostring_rgb(),dtype='uint8')
        image = image.reshape(self.fig.canvas.get_width_height()[::-1]+(3,))
        self.fig.canvas.flush_events()
        return image
    def updateLocSpd(self):
        self.t+=1
        self.loc += self.spd
        flipx = (self.loc[:,0]<=0) | (self.loc[:,0]>=self.sizGrid)
        flipy = (self.loc[:,1]<=0) | (self.loc[:,1]>=self.sizGrid)
        self.spd[:,0][flipx] *= -1
        self.spd[:,1][flipy] *= -1
    def updateInfect(self):
        infect = self.state[:,0]==1
        rmv = self.state[:,1]==1
        infectLoc = self.loc[infect,:]
        for i in range(infectLoc.shape[0]):
            tmpdist = np.linalg.norm(self.loc-infectLoc[i,:],axis=1)
            potential = (tmpdist<=self.infR)
            potentialP = np.random.uniform(0,1,self.sizHuman)
            newInfectbol = potential*potentialP >= (1-self.infP)
            infect[newInfectbol] = 1
        for i in range(len(infect)):
            if rmv[i] == True:  infect[i] = False
        self.state[:,0] = infect
        self.state[:,2][infect==1] += 1
    def updateRemove(self):
        infect = self.state[:,0]==1
        infectPeriodbol = self.state[:,2]*infect >= self.rmT
        potentialP = np.random.uniform(0,1,self.sizHuman)
        newRmvbol = potentialP*infectPeriodbol >= (1-self.rmP)
        self.state[:,0][newRmvbol] = 0
        self.state[:,1][newRmvbol] = 1
    def simulate(self):
        while self.t < self.simTime:
            if self.t%1 == 0:
                image = self.updatePlot()
                self.images.append(image)
            self.updateLocSpd()
            self.updateInfect()
            self.updateRemove()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Suceptible-Infectious-Removed Model')
    parser.add_argument('--infR',type=float,default=.2,help='Infectious Radius')
    parser.add_argument('--infP',type=float,default=.05,help='Infectious Probability')
    parser.add_argument('--rmT',type=int,default=50,help='Recovered Period')
    parser.add_argument('--rmP',type=float,default=.1,help='Recovered probability')
    parser.add_argument('--simT',type=int,default=100000,help='simulation epochs')
    args = parser.parse_args()
    model_kwargs = {
            'infR':args.infR,'infP':args.infP,
            'rmT':args.rmT,'rmP':args.rmP,'simT':args.simT}
    modelCov = model(**model_kwargs)
    modelCov.simulate()
