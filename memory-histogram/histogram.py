#!/usr/bin/env python
import time 
import numpy as np
from itertools import chain
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import rospy
import ros_numpy

defResolution = 5
VFOV, HFOV = 1.02974, (480/640)*1.02974 # radians
PI = 3.1415
def getCoor(glob, eps, tau, dist):
    x = glob[0]+dist*np.sin(eps)*np.sin(tau)
    y = glob[1]+dist*np.cos(eps)*np.sin(tau)
    z = glob[2]+dist*np.cos(tau)
    return (x,y,z)
xyz = np.zeros((480*640,3))
class histogram:
    
    def __init__(self, resolution):
        # initalize age,distance,b arrays to zero
        self.age = np.zeros((360/resolution,180/resolution))
        self.distance = np.zeros((360/resolution,180/resolution))
        self.bin = np.zeros((360/resolution,180/resolution))
        self.res = resolution
        self.xyzdistance = np.zeros((480*640,1)) 
        self.xyd = np.zeros((480*640,1))
        self.xyz = np.zeros((480*640,3))
        rospy.init_node('pclnode', anonymous=True)
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.callback)
    
    def callback(self, data):
        self.xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
        rospy.wait_for_message("/camera/depth/points", PointCloud2, timeout=None)
       

    def build(self):
       
        np.set_printoptions(threshold = np.inf)
        
        t = time.time()
        self.xyz = np.round(self.xyz, decimals=2)
        #print(self.xyz)
        self.xyzdistance = np.round(np.sqrt((self.xyz[:,0]*self.xyz[:,0])+(self.xyz[:,1]*self.xyz[:,1])+(self.xyz[:,2]*self.xyz[:,2])))
        self.xyzdistance = np.clip(self.xyzdistance, 0.1,10, out=None)
        self.xyd = np.sqrt((self.xyz[:,0]*self.xyz[:,0])+(self.xyz[:,1]*self.xyz[:,1]))
        self.xyd = np.clip(self.xyd, 0.1, 10, out=None)
        
        self.xyzdistance = np.round(self.xyzdistance, decimals=2)
        self.xyd = np.round(self.xyd, decimals=2)

        print('generated dxyz in:')
        print(time.time()-t)
        t =time.time()
        
        i = 0
        self.epsilon = np.arctan2(self.xyz[:,1],self.xyz[:,0]) + np.pi
        self.epsilon = np.floor((180*self.epsilon) / (self.res*np.pi))
        self.epsilon = ((self.epsilon + self.res - (self.epsilon%self.res))/self.res - 1).astype(int)
        self.tau = np.floor(180*np.arctan2(self.xyz[:,2],self.xyd[:]) / (self.res*np.pi)) +90
        self.tau = ((self.tau + self.res - (self.tau % self.res))/self.res - 1).astype(int)
        #for element in self.epsilon:
        #    if(element<0):
        #        print(':thumbsdown:')
        print('generated indices in:')
        print((time.time()-t))
        #self.index = self.index.astype(int)
        t = time.time()
        #print(self.index)
        maxEp   = np.int((np.max(self.epsilon)))
        minEp   = np.int((np.min(self.epsilon)))
        maxTau  = np.int((np.max(self.tau)))
        minTau  = np.int((np.min(self.tau)))
        
        print(minEp, maxEp, minTau, maxTau)
        x=0
        # build bin layer
        for epsIndex in range(maxEp+1):
            for tauIndex in range(maxTau+1):
                if(epsIndex in self.epsilon):
                    if(tauIndex in self.tau):
                        self.bin[epsIndex,tauIndex] = 1
                        x+=1
                        
        #print(self.bin)
        print('generated bin in:')
        print((time.time()-t))
        t = time.time()
        l = 0
        k = 0
        y=0
        # build distance layer

        for epsIndex in range(maxEp+1):
            for tauIndex in range(maxTau+1):
                if(self.bin[epsIndex,tauIndex] == 1):
                    for d in self.xyzdistance:
                        if((np.abs(self.epsilon[k]-epsIndex)<0.05) and (np.abs(self.tau[k] -  tauIndex))<0.05):
                            self.distance[epsIndex,tauIndex] = self.distance[epsIndex,tauIndex] + d
                            l += 1
                        k += 1
                    if(l!=0):
                        self.distance[epsIndex,tauIndex] = self.distance[epsIndex,tauIndex]/l
                        #print('Balle Balle')
                        y+=1
                    l = 0
                    k = 0
        print("generated distance in:")
        print(time.time()-t)
        #print(self.distance)
        #if(x==y):
        #    print('BASED')


    def update(self, posOld, posNew):
        resolution = self.res
        # now we will update the position of the points
        coordinateGlobal = []
        for epsilonInd in range(360/self.res):
            for tauInd in range(180/self.res):
                corners = [
                    [epsilonInd*self.res-self.res/2, tauInd*self.res-self.res/2],
                    [epsilonInd*self.res+self.res/2, tauInd*self.res-self.res/2],
                    [epsilonInd*self.res-self.res/2, tauInd*self.res+self.res/2],
                    [epsilonInd*self.res+self.res/2, tauInd*self.res+self.res/2]
                ]
                #fix this
                coordinateGlobal.append(getCoor(posOld, corners[0], corners[1] ,self.distance(epsilonInd,tauInd)))

        # half resolution build
        updatedAndDownSampled = histogram(2*self.res)
        coordinateGlobal = np.array(coordinateGlobal)
        updatedAndDownSampled_xyzdistance = np.sqrt((self.coordinateGlobal[:,0]-posNew[0])*(self.coordinateGlobal[:,0]-posNew[0])
        +(self.coordinateGlobal[:,1]-posNew[1])*(self.coordinateGlobal[:,1]-posNew[1])+
        (self.coordinateGlobal[:,2]-posNew[2])*(self.coordinateGlobal[:,2]-posNew[2]))

        updatedAndDownSampled_dxy = np.sqrt( (self.coordinateGlobal[:,0]-posNew[0])*(self.coordinateGlobal[:,0]-posNew[0])
        +(self.coordinateGlobal[:,1]-posNew[1])*(self.coordinateGlobal[:,1]-posNew[1]) )

        # the usual build with 6-point implementation
        updatedAndDownSampled_index = np.zeros(coordinateGlobal.shape[0])

        updatedAndDownSampled.epsilon = np.floor(np.arctan((updatedAndDownSampled.coordinateGlobal[:,0]-posNew[0])/(updatedAndDownSampled.coordinateGlobal[:,1]-posNew[1])) / updatedAndDownSampled.res)
        updatedAndDownSampled_index[:,0] = (updatedAndDownSampled.epsilon + updatedAndDownSampled.res - updatedAndDownSampled.epsilon%updatedAndDownSampled.res)/updatedAndDownSampled.res - 1

        updatedAndDownSampled.tau = np.floor(180*np.arctan((updatedAndDownSampled.xyz[:,2]-posNew[2])/updatedAndDownSampled_dxy[:]) / updatedAndDownSampled.res /PI)
        updatedAndDownSampled_index[:,1] = (updatedAndDownSampled.tau + updatedAndDownSampled.res - updatedAndDownSampled.tau%updatedAndDownSampled.res)/updatedAndDownSampled.res - 1

        # build bin layer
        for epsIndex in range(360/updatedAndDownSampled.res):
            for tauIndex in range(180/updatedAndDownSampled.res):
                if (np.count_nonzero(updatedAndDownSampled_index[:,0] == epsIndex) >= 6):
                    if (np.count_nonzero(updatedAndDownSampled_index[:,1] == tauIndex) >= 6):
                        updatedAndDownSampled.bin[epsIndex,tauIndex] = 1
        k = 0
        # build distance layer
        for epsIndex in range(360/updatedAndDownSampled.res):
            for tauIndex in range(180/updatedAndDownSampled.res):
                for d in updatedAndDownSampled_xyzdistance:
                    if(updatedAndDownSampled_index[k,:]== (epsIndex,tauIndex)):
                        updatedAndDownSampled.distance[epsIndex,tauIndex] = updatedAndDownSampled.distance[epsIndex,tauIndex] + d
                        k = k + 1
                updatedAndDownSampled.distance[epsIndex,tauIndex] = updatedAndDownSampled.distance[epsIndex,tauIndex]/k
                k = 0
        # change age
        self.age = self.age + 1

        upSampled = histogram(self.res)
        for epsilon in range(0,180/self.res-1):
            for tau in range(0,90/self.res-1):
                upSampled.distance[2*epsilon, 2*tau] = updatedAndDownSampled.distance[epsilon,tau]
                upSampled.distance[2*epsilon, 2*tau+1] = updatedAndDownSampled.distance[epsilon,tau]
                upSampled.distance[2*epsilon+1, 2*tau] = updatedAndDownSampled.distance[epsilon,tau]
                upSampled.distance[(2*epsilon+1), (2*tau+1)] = updatedAndDownSampled.distance[epsilon,tau]
                # do same for age and bin
                upSampled.age[2*epsilon, 2*tau] = updatedAndDownSampled.age[epsilon,tau]
                upSampled.age[2*epsilon, 2*tau+1] = updatedAndDownSampled.age[epsilon,tau]
                upSampled.age[2*epsilon+1, 2*tau] = updatedAndDownSampled.age[epsilon,tau]
                upSampled.age[(2*epsilon+1), (2*tau+1)] = updatedAndDownSampled.age[epsilon,tau]

                upSampled.bin[2*epsilon, 2*tau] = updatedAndDownSampled.bin[epsilon,tau]
                upSampled.bin[2*epsilon, 2*tau+1] = updatedAndDownSampled.bin[epsilon,tau]
                upSampled.bin[2*epsilon+1, 2*tau] = updatedAndDownSampled.bin[epsilon,tau]
                upSampled.bin[(2*epsilon+1), (2*tau+1)] = updatedAndDownSampled.bin[epsilon,tau]
        return upSampled

def merge(currentHistogram, memoryHistogram, HFOV, VFOV):
    # merge old and new histogram
    # assumed FOV to be a 2d array comprising of the two FOV angles
    # assumed the line of sight of camera to be the z-axis

    resolution = currentHistogram.res

    FOVEpsilon = HFOV*180/PI
    FOVTau     = VFOV*180/PI

    # create a new histogram called resultHistogram which is obtained upon merging current and memory hist
    resultHistogram = histogram(currentHistogram.res)

    # obtain index range for epsilon
    FOVEpsilonTemp =  FOVEpsilon + resolution - (FOVEpsilon % resolution)
    FOVEpsilonIndex = FOVEpsilonTemp/resolution - 1

    # obtain index range for elevation
    FOVTauTemp =    FOVTau + resolution - ( FOVTau % resolution)
    FOVTauIndex =   FOVTauTemp/resolution - 1

    # for angles within the FOV copy current hist data

    for epsilon in range(180/resolution - FOVEpsilonIndex/2, 180/resolution + FOVEpsilonIndex/2):
        for tau in range(90/resolution - FOVTauIndex/2, 90/resolution + FOVTauIndex/2):
            resultHistogram.age[epsilon,tau] = currentHistogram.age[epsilon,tau]
            resultHistogram.distance[epsilon,tau] = currentHistogram.distance[epsilon,tau]
            resultHistogram.bin[epsilon,tau] = currentHistogram.bin[epsilon,tau]

    # for angles outside FOV perform OR operation on binary layer
    # for angles outside FOV if bin=1, set distance = min(current, memory)
    # for angles outside FOV put age = age_corresponding_to_distance
   
    for epsilon in chain(range(0, 180/resolution - FOVEpsilonIndex/2-1), (180/resolution + FOVEpsilonIndex/2+1, 360/currentHistogram.res-1)):
        for tau in chain(range(0, 90/resolution - FOVTauIndex/2-1), (90/resolution + FOVTauIndex/2+1, 180/currentHistogram.res-1)):
            resultHistogram.bin[epsilon,tau] = max(currentHistogram.bin[epsilon,tau], memoryHistogram.bin[epsilon,tau]) 
            
            resultHistogram.distance[epsilon,tau] = min(currentHistogram.distance[epsilon,tau], memoryHistogram.distance[epsilon,tau])
             
            if(resultHistogram.distance[epsilon,tau] == currentHistogram.distance[epsilon,tau]):
                resultHistogram.age[epsilon,tau] = currentHistogram.age[epsilon,tau]
            elif(resultHistogram.distance[epsilon,tau] == memoryHistogram.distance[epsilon,tau]):
                resultHistogram.age[epsilon,tau] = memoryHistogram.age[epsilon,tau]

    return resultHistogram

memoryHistogram = histogram(defResolution)
time.sleep(5)
memoryHistogram.build()

prevPos = [0,0,0]
currPos = [0,0,0]

def callback(data):
    global vehicle_curr_pose
    vehicle_curr_pose = data
    rospy.wait_for_message("/camera/depth/points", PointCloud2, timeout=None)

def listener(memoryHistogram):

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback)
    currHist = histogram(defResolution)
    
    currHist.build()
    prevPos = currPos
    currPos = [vehicle_curr_pose.pose.position.x, vehicle_curr_pose.pose.position.y, vehicle_curr_pose.pose.position.z]
    upsampled = memoryHistogram.update(prevPos,currPos)
    memoryHistogram = merge(currHist, upsampled, HFOV, VFOV)  
    
    rospy.spin()

if __name__ == '__main__':
    listener(memoryHistogram)    
