#!/usr/bin/env python
from cmath import polar
import time
from turtle import color 
import numpy as np
from itertools import chain
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import rospy
import ros_numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
defResolution = 5
VFOV, HFOV = 1.02974, 1.02974 # radians
PI = 3.1415
def getCoor(glob, eps, tau, dist):
    x = glob[0]+dist*np.sin(eps)*np.sin(tau)
    y = glob[1]+dist*np.cos(eps)*np.sin(tau)
    z = glob[2]+dist*np.cos(tau)
    return (x,y,z)
xyz = np.zeros((640*480,3))
class histogram:
    
    def __init__(self, resolution):
        # initalize age,distance,b arrays to zero
        self.age = np.zeros((360/resolution,180/resolution))
        self.distance = np.zeros((360/resolution,180/resolution))
        self.bin = np.full((360/resolution,180/resolution), False)
        self.res = resolution
        self.xyz = np.zeros((640*480,3))
        rospy.init_node('pclnode', anonymous=True)
        rospy.Subscriber("/voxel_grid/output", PointCloud2, self.callback)
    
    def callback(self, data):
        self.xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
        rospy.wait_for_message("/voxel_grid/output", PointCloud2, timeout=None)
       

    def build(self):
       
        #np.set_printoptions(threshold = np.inf)
        
        t = time.time()
        #print(self.xyz)
        xyzdistance = np.sqrt((self.xyz[:,0]*self.xyz[:,0])+(self.xyz[:,1]*self.xyz[:,1])+(self.xyz[:,2]*self.xyz[:,2]))
        xyd = np.sqrt((self.xyz[:,0]*self.xyz[:,0])+(self.xyz[:,1]*self.xyz[:,1]))  
       

        print('generated dxyz in:')
        print(time.time()-t)
        t =time.time()
        polar = np.zeros((np.shape(self.xyz)[0],2))
        count = np.ones((360/self.res,180/self.res))
        polar[:,0] = np.arctan2(self.xyz[:,1],self.xyz[:,0]) + np.pi
        polar[:,0] = np.floor((180*polar[:,0]) / (self.res*np.pi))
        polar[:,0] = (polar[:,0] + self.res - (polar[:,0]%self.res))/self.res - 1
        polar[:,1] = np.floor((180*np.arctan2(self.xyz[:,2],xyd[:]) + np.pi/2 )/ (self.res*np.pi))
        polar[:,1] = (polar[:,1] + self.res - (polar[:,1] % self.res))/self.res - 1
        polar = polar.astype(int)
        print(np.shape(polar))

        print('generated indices in:')
        print((time.time()-t))
        t = time.time()
        for angle in np.unique(polar,axis = 0):
            if (np.count_nonzero(polar == angle) > 0):
                self.bin[angle[0],angle[1]] = True
                count[angle[0],angle[1]] = 0
        for index,angle in enumerate(polar):
            #self.bin[angle[0],angle[1]] = True
            count[angle[0],angle[1]] = count[angle[0],angle[1]] + 1
            self.distance[angle[0],angle[1]] = self.distance[angle[0],angle[1]] + xyzdistance[index]
        self.distance = self.distance/count
        # np.savetxt("ok.txt",np.extract(self.distance !=0,self.distance)) 
        #np.savetxt("ok2.txt",np.nonzero(self.distance))    
        # print(self.bin)
        print('generated in:')
        print((time.time()-t))

    def reproject(self):
        print("Reprojecting...")
        t=time.time()
        alpha = self.res/2
        zeta_init = np.arange(0,360/self.res)*self.res*np.pi/180 - np.pi # should you use // instead of /?
        eps_init = np.arange(0,180/self.res)*self.res*np.pi/180 - np.pi/2
        zeta, eps = np.meshgrid(zeta_init, eps_init, indexing = 'ij')
        px = np.concatenate((self.distance*np.cos(eps+alpha)*np.cos(zeta+alpha),self.distance*np.cos(eps-alpha)*np.cos(zeta+alpha),self.distance*np.cos(eps+alpha)*np.cos(zeta-alpha), self.distance*np.cos(eps-alpha)*np.cos(zeta-alpha)), axis = None)
        py = np.concatenate((self.distance*np.cos(eps+alpha)*np.sin(zeta+alpha),self.distance*np.cos(eps-alpha)*np.sin(zeta+alpha),self.distance*np.cos(eps+alpha)*np.sin(zeta-alpha),self.distance*np.cos(eps-alpha)*np.sin(zeta-alpha)), axis = None)
        pz = np.concatenate((self.distance*np.sin(eps+alpha),self.distance*np.sin(eps-alpha),self.distance*np.sin(eps+alpha),self.distance*np.sin(eps-alpha)), axis = None)
        points = np.stack((px,py,pz), axis=-1)
        points = points[~np.all(points==0, axis=1)] #nonzero elements only
        print(time.time()-t)
        # print(points)
        # plt.figure()
        # ax = plt.axes(projection = '3d')
        # ax.scatter(points[:,0], points[:,1], points[:,2], color ='b', marker= 'o')
        # ax.scatter(self.xyz[:,0], self.xyz[:,1], self.xyz[:,2],color = 'r' ,marker= 'o')
        # ax.scatter([0],[0],[0],color = 'g', marker = 'o')
        # plt.show()


    # def update(self, posOld, posNew):
    #     #resolution = self.res
    #     # now we will update the position of the points
    #     print("Running Update...")
    #     t=time.time()
    #     #reprojection
    #     alpha = self.res/2
    #     eps = np.arange(0,360/self.res)*self.res
    #     tau = np.arange(0,180/self.res)*self.res
    #     eps, zeta = np.meshgrid(eps, tau)
    #     points = self.distance

    #     for self.polar in range(360/self.res):
    #         for tauInd in range(180/self.res):
    #             corners = [
    #                 [self.polar*self.res-self.res/2, tauInd*self.res-self.res/2],
    #                 [self.polar*self.res+self.res/2, tauInd*self.res-self.res/2],
    #                 [self.polar*self.res-self.res/2, tauInd*self.res+self.res/2],
    #                 [self.polar*self.res+self.res/2, tauInd*self.res+self.res/2]
    #             ]
    #             corners=np.array(corners)
    #             print(np.shape(corners))
    #             self.coordinateGlobal.append(getCoor(posOld, corners[:,0], corners[:,1] ,self.distance[self.polar,tauInd]))
                
    #     # half resolution build
    #     updatedAndDownSampled = histogram(2*self.res)
    #     print(np.shape(self.coordinateGlobal))
    #     updatedAndDownSampled_xyzdistance = np.round(np.sqrt((self.coordinateGlobal[:,0]-posNew[0])*(self.coordinateGlobal[:,0]-posNew[0])
    #     +(self.coordinateGlobal[:,1]-posNew[1])*(self.coordinateGlobal[:,1]-posNew[1])+
    #     (self.coordinateGlobal[:,2]-posNew[2])*(self.coordinateGlobal[:,2]-posNew[2])), decimals=2)

    #     updatedAndDownSampled_dxy = np.round(np.sqrt( (self.coordinateGlobal[:,0]-posNew[0])*(self.coordinateGlobal[:,0]-posNew[0])
    #     +(self.coordinateGlobal[:,1]-posNew[1])*(self.coordinateGlobal[:,1]-posNew[1]) ), decimals =2)

    #     # the usual build with 6-point implementation
    #     n = np.array(self.coordinateGlobal.shape)
    #     updatedAndDownSampled_index = np.zeros((n[0],2))

    #     updatedAndDownSampled.epsi_initlon = np.floor(np.arctan2((self.coordinateGlobal[:,0]-posNew[0]),(self.coordinateGlobal[:,1]-posNew[1])) / updatedAndDownSampled.res)
    #     #print(np.shape(updatedAndDownSampled.epsi_initlon))
    #     updatedAndDownSampled_index[:,0] = ((updatedAndDownSampled.epsi_initlon + updatedAndDownSampled.res - updatedAndDownSampled.epsi_initlon%updatedAndDownSampled.res)/updatedAndDownSampled.res - 1).astype(int)

    #     updatedAndDownSampled.tau = np.floor(180*np.arctan2((updatedAndDownSampled.xyz[:,2]-posNew[2]),updatedAndDownSampled_dxy[:]) / updatedAndDownSampled.res /PI) + 90
    #     updatedAndDownSampled_index[:,1] = ((updatedAndDownSampled.tau + updatedAndDownSampled.res - updatedAndDownSampled.tau%updatedAndDownSampled.res)/updatedAndDownSampled.res - 1).astype(int)

    #     # build bin layer
    #     for epsi_initndex in range(360/updatedAndDownSampled.res):
    #         for tauIndex in range(180/updatedAndDownSampled.res):
    #             if (np.count_nonzero(updatedAndDownSampled_index[:,0] == epsi_initndex) >= 6):
    #                 if (np.count_nonzero(updatedAndDownSampled_index[:,1] == tauIndex) >= 6):
    #                     updatedAndDownSampled.bin[epsi_initndex,tauIndex] = 1
    #     k = 0
    #     # build distance layer
    #     for epsi_initndex in range(360/updatedAndDownSampled.res):
    #         for tauIndex in range(180/updatedAndDownSampled.res):
    #             if (np.count_nonzero(updatedAndDownSampled_index[:,0] == epsi_initndex) >= 6):
    #                 if (np.count_nonzero(updatedAndDownSampled_index[:,1] == tauIndex) >= 6):
    #                     for d in updatedAndDownSampled_xyzdistance:
    #                         if(updatedAndDownSampled_index[k,:]== (epsi_initndex,tauIndex)):
    #                             updatedAndDownSampled.distance[epsi_initndex,tauIndex] = updatedAndDownSampled.distance[epsi_initndex,tauIndex] + d
    #                             k = k + 1
    #                     updatedAndDownSampled.distance[epsi_initndex,tauIndex] = updatedAndDownSampled.distance[epsi_initndex,tauIndex]/k
    #                     k = 0
    #     # change age
    #     self.age = self.age + 1

    #     upSampled = histogram(self.res)
    #     for epsi_initlon in range(0,180/self.res-1):
    #         for tau in range(0,90/self.res-1):
    #             upSampled.distance[2*epsi_initlon, 2*tau] = updatedAndDownSampled.distance[epsi_initlon,tau]
    #             upSampled.distance[2*epsi_initlon, 2*tau+1] = updatedAndDownSampled.distance[epsi_initlon,tau]
    #             upSampled.distance[2*epsi_initlon+1, 2*tau] = updatedAndDownSampled.distance[epsi_initlon,tau]
    #             upSampled.distance[(2*epsi_initlon+1), (2*tau+1)] = updatedAndDownSampled.distance[epsi_initlon,tau]
    #             # do same for age and bin
    #             upSampled.age[2*epsi_initlon, 2*tau] = updatedAndDownSampled.age[epsi_initlon,tau]
    #             upSampled.age[2*epsi_initlon, 2*tau+1] = updatedAndDownSampled.age[epsi_initlon,tau]
    #             upSampled.age[2*epsi_initlon+1, 2*tau] = updatedAndDownSampled.age[epsi_initlon,tau]
    #             upSampled.age[(2*epsi_initlon+1), (2*tau+1)] = updatedAndDownSampled.age[epsi_initlon,tau]

    #             upSampled.bin[2*epsi_initlon, 2*tau] = updatedAndDownSampled.bin[epsi_initlon,tau]
    #             upSampled.bin[2*epsi_initlon, 2*tau+1] = updatedAndDownSampled.bin[epsi_initlon,tau]
    #             upSampled.bin[2*epsi_initlon+1, 2*tau] = updatedAndDownSampled.bin[epsi_initlon,tau]
    #             upSampled.bin[(2*epsi_initlon+1), (2*tau+1)] = updatedAndDownSampled.bin[epsi_initlon,tau]
    #     print("Updated")
    #     print("Process took", t-time.time())
    #     return upSampled


def merge(currentHistogram, memoryHistogram, HFOV, VFOV):
    # merge old and new histogram
    # assumed FOV to be a 2d array comprising of the two FOV angles
    # assumed the line of sight of camera to be the z-axis

    resolution = currentHistogram.res

    FOVepsi_initlon = HFOV*180/PI
    FOVTau     = VFOV*180/PI

    # create a new histogram called resultHistogram which is obtained upon merging current and memory hist
    resultHistogram = histogram(currentHistogram.res)

    # obtain index range for epsi_initlon
    FOVepsi_initlonTemp =  FOVepsi_initlon + resolution - (FOVepsi_initlon % resolution)
    FOVself.polarex = FOVepsi_initlonTemp/resolution - 1

    # obtain index range for elevation
    FOVTauTemp =    FOVTau + resolution - ( FOVTau % resolution)
    FOVTauIndex =   FOVTauTemp/resolution - 1

    # for angles within the FOV copy current hist data

    for epsi_initlon in range(180/resolution - FOVself.polarex/2, 180/resolution + FOVself.polarex/2):
        for tau in range(90/resolution - FOVTauIndex/2, 90/resolution + FOVTauIndex/2):
            resultHistogram.age[epsi_initlon,tau] = currentHistogram.age[epsi_initlon,tau]
            resultHistogram.distance[epsi_initlon,tau] = currentHistogram.distance[epsi_initlon,tau]
            resultHistogram.bin[epsi_initlon,tau] = currentHistogram.bin[epsi_initlon,tau]

    # for angles outside FOV perform OR operation on binary layer
    # for angles outside FOV if bin=1, set distance = min(current, memory)
    # for angles outside FOV put age = age_corresponding_to_distance
   
    for epsi_initlon in chain(range(0, 180/resolution - FOVself.polarex/2-1), (180/resolution + FOVself.polarex/2+1, 360/currentHistogram.res-1)):
        for tau in chain(range(0, 90/resolution - FOVTauIndex/2-1), (90/resolution + FOVTauIndex/2+1, 180/currentHistogram.res-1)):
            resultHistogram.bin[epsi_initlon,tau] = max(currentHistogram.bin[epsi_initlon,tau], memoryHistogram.bin[epsi_initlon,tau]) 
            
            resultHistogram.distance[epsi_initlon,tau] = min(currentHistogram.distance[epsi_initlon,tau], memoryHistogram.distance[epsi_initlon,tau])
             
            if(resultHistogram.distance[epsi_initlon,tau] == currentHistogram.distance[epsi_initlon,tau]):
                resultHistogram.age[epsi_initlon,tau] = currentHistogram.age[epsi_initlon,tau]
            elif(resultHistogram.distance[epsi_initlon,tau] == memoryHistogram.distance[epsi_initlon,tau]):
                resultHistogram.age[epsi_initlon,tau] = memoryHistogram.age[epsi_initlon,tau]

    return resultHistogram

memoryHistogram = histogram(defResolution)
time.sleep(5)
memoryHistogram.build()
memoryHistogram.reproject()

prevPos = [0,0,0]
currPos = [0,0,0]

def callback(data):
    global vehicle_curr_pose
    vehicle_curr_pose = data
    rospy.wait_for_message("/voxel_grid/output", PointCloud2, timeout=None)

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
