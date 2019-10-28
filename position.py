from bluepy.btle import Scanner, DefaultDelegate
import numpy as np
import matplotlib.pyplot as plt

####################
#
# Filter of Gaussian, use noise filtering in RSSI
# need enter your data array, e.g. arr=[-48,-47,-47,-47,-47,-46,-33,-60,-46,-44,-51,-49]
# and create a variable to hold the value, e.g. gfFunction = GFilter(arr)
# then, you can call getResult() function to get the filter value
#
####################
class GFilter:#GaussianFilter
    twoPiSqrt = np.sqrt(2*np.pi)
    def __init__(self,arr=[0,0]):#gfSigma=0,expeaction=0):
        #self.gfSigma=gfSigma
        #self.expeaction=expeaction
        self.arr = arr
        self.setExpecation(arr)
        self.setGfSigma(arr,self.expeaction)

    def setExpecation(self,rssi): #u
        self.expeaction = sum(rssi)/len(rssi)
    def getExpecation(self):
        return self.expeaction

    def setGfSigma(self,rssi,u):
        rssiSigma=[]
        for i in range(len(rssi)):
            rssiSigma += [(rssi[i]-u)**2]
        self.gfSigma = (sum(rssiSigma)/len(rssiSigma))**0.5
    def getGfSigma(self):
        return self.gfSigma

    def getGaussianFilter(self,rssi):
        if self.gfSigma != 0:
            val1 = (1/(self.gfSigma*self.twoPiSqrt))*np.exp((-1*pow(rssi-self.expeaction,2)) * 1/ (2*pow(self.gfSigma,2)))
            return val1
        else :
            return 0.01
    def getResult(self):
        if self.gfSigma !=0:
            self.gfArrayVal=[]
            gfVal=[]
            for i in self.arr:
                gfVal += [self.getGaussianFilter(i)]
            length=len(gfVal)
            average=(sum(gfVal)/length)*1.12
            print ("average ",average)
            print (gfVal)
            for i in range(length):
                if gfVal[i] >= average:
                    self.gfArrayVal += [self.arr[i]]
        else:
            self.gfArrayVal=[self.arr[1]]
        print(self.gfArrayVal)
        return (sum(self.gfArrayVal)/len(self.gfArrayVal))

####################
#
# least squares method, Solve the center point using three circular equations
# need enter three flag data then the data class is custom, please refer to class flagDev, it will be under this class
# you can call getPosition() to get the coordinate after create LSquare object
# or call getMatplot() to drawing a location sketch
#
####################
class LSquare:#LeastSquares
    def __init__(self, flagA, flagB, flagC):
        self.x1=flagA.x
        self.y1=flagA.y
        self.x2=flagB.x
        self.y2=flagB.y
        self.x3=flagC.x
        self.y3=flagC.y
    def getPosition(self,R1,R2,R3):
        A = np.array([[2*(self.x2-self.x1),2*(self.y2-self.y1)],[2*(self.x3-self.x2),2*(self.y3-self.y2)]])
        b = np.array([[R1*R1-R2*R2-self.x1*self.x1+self.x2*self.x2-self.y1*self.y1+self.y2*self.y2],[R2*R2-R3*R3-self.x2*self.x2+self.x3*self.x3-self.y2*self.y2+self.y3*self.y3]])
        return np.linalg.lstsq(A,b,rcond=None)[0]
    def getMatplot(self,x,y,R1,R2,R3):
        plt.clf()
        plt.plot(self.x1,self.y1,'bx')
        plt.plot(self.x2,self.y2,'bx')
        plt.plot(self.x3,self.y3,'bx')
        plt.plot(x,y,'rx')
        theta = np.arange(0, 2*np.pi, 0.01)
        x = self.x1 + R1 * np.cos(theta)
        y = self.y1 + R1 * np.sin(theta)
        plt.plot(x, y)
        x = self.x2 + R2 * np.cos(theta)
        y = self.y2 + R2 * np.sin(theta)
        plt.plot(x, y)
        x = self.x3 + R3 * np.cos(theta)
        y = self.y3 + R3 * np.sin(theta)
        plt.plot(x, y)
        plt.axis('equal')
        plt.pause(0.1)
        #plt.show()

####################
#
# Bluetooth position flag device data class
# Objectization Bluetooth locator for easy programming
#
####################

class flagDev:
    def __init__(self,x,y,addr,rssi=-33,N=3):
        self.x=x
        self.y=y
        self.addr=addr
        self.rssi=rssi
        self.N=N
    def getPosition(self):
        return self.x,self.y
    def setAddress(self,val):
        self.address=val
    def getDistance(self,rssi):
        powVal = (self.rssi - rssi) / (10*self.N)
        distance = 1*pow(10,powVal) #return m
        return distance

####################
#
# class of Position is the py-code file main function,
# it will call all the class in this file.
# need enter At least three flag object data in an flagArray,
# e.g. flagArray=[flagDev(0,0,"00:15:83:f7:54:87",-40,3),flagDev(1.80,1.55,"00:15:83:f7:59:51",-42.5,3),flagDev(3.63,0,"00:15:83:f7:5d:1a",-42,3.3)].
# than, you can call getCoordinate() to get x and y value, or call getMatplot() to draw sketch
# However, a problem in the getMatplot() function, you must first call getCoordinate() or an error will happen.
# because the farA、farB、farC etc will be zero, so the sketch position value is error
#
####################
class Position:
    farA=0
    farB=0
    farC=0
    def __init__(self):
        self.flagA = flagDev(0,0,"00:15:83:f7:54:87",-40,3)
        self.flagB = flagDev(1.80,1.55,"00:15:83:f7:59:51",-42.5,3)
        self.flagC = flagDev(3.63,0,"00:15:83:f7:5d:1a",-42,3.3)
        self.position = LSquare(self.flagA,self.flagB,self.flagC)
        self.scanner = Scanner()
    def getCoordinate(self):
        rssiA=None
        rssiB=None
        rssiC=None
        rssiABf=[]
        rssiBBf=[]
        rssiCBf=[]
        while(len(rssiABf)!=5 and len(rssiBBf)!=5 and len(rssiCBf)!=5):
            devices = self.scanner.scan(1.5)
            if len(devices) >= 3:
                print("get %d device" % ( len(devices)))
                for dev in devices:
                    if dev.rssi >= -100:
                        if dev.addr == self.flagA.addr:
                            print(dev.addr)
                            rssiA = dev.rssi
                        if dev.addr == self.flagB.addr:
                            print(dev.addr)
                            rssiB = dev.rssi
                        if dev.addr == self.flagC.addr:
                            print(dev.addr)
                            rssiC = dev.rssi
                if (rssiA!=None and rssiB!=None and rssiC!=None):
                    rssiABf += [rssiA]
                    rssiBBf += [rssiB]
                    rssiCBf += [rssiC]
                rssiA=None
                rssiB=None
                rssiC=None
        gfA = GFilter(rssiABf).getResult()
        gfB = GFilter(rssiBBf).getResult()
        gfC = GFilter(rssiCBf).getResult()
        self.farA=self.flagA.getDistance(gfA)
        self.farB=self.flagB.getDistance(gfB)
        self.farC=self.flagC.getDistance(gfC)
        # x, y=position.getPosition(farA,farB,farC)
        # print(rssiABf,'\n',rssiBBf,'\n',rssiCBf)
        # print('gfA= ',gfA,'\ngfB= ',gfB,'\ngfC= ',gfC)
        # print("farA= %.2f m \nfarB= %.2f m\nfarC= %.2f m" %(farA,farB,farC))
        return self.position.getPosition(self.farA,self.farB,self.farC)
    def getMatplot(self,x,y):
        self.position.getMatplot(x,y,self.farA,self.farB,self.farC)
if __name__ == "__main__":
    myLocat=Position()
    x,y=myLocat.getCoordinate()
    print('%.2f %.2f \n\n\n' %(x,y))
    myLocat.getMatplot(x,y)
