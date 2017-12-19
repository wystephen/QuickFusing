# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 26　17:10


import pygame

import numpy as np


class BeaconWithRange:
    '''

    '''

    def __init__(self,SCREEN_SIZE,OFFSET,ScaleFactor=1.0):
        '''

        :param SCREEN_SIZE:
        :param OFFSET: piexels
        :param ScaleFactor: Real(m) to piexels
        '''

        self.SCREEN_SIZE = SCREEN_SIZE
        self.OFFSET = OFFSET
        self.SCALEFACTOR = ScaleFactor

        self.Pose = [10,10]
        self.Dist  = 0

        self.IntPose = [1,1]
        self.IntDist = 0

        self.RangeMethond = "IDEAL"

    def SetRangeMethond(self,number):
        '''
        LIST:
        0   -  IDEAL
        1   -  SIMPLE_RANGE
        :param number:
        :return:
        '''

        if number == 0:
            self.RangeMethond = "IDEAL"
        elif number == 1:
            self.RangeMethond = "SIMPLE_RANGE"


    def SetPose(self,x,y):
        self.Pose = [x,y]
        for i in range(len(self.Pose)):
            self.IntPose[i] = int(self.Pose[i]*1.0 *self.SCALEFACTOR) + self.OFFSET[i]


    def SetRange(self,distance):
        self.Dist = distance
        self.IntDist = int(distance * 1.0 * self.SCALEFACTOR)

    def Draw(self,screen):
        pygame.draw.circle(screen,
                           [0,22,211],
                           self.IntPose,
                           self.IntDist+1,1)

        pygame.draw.rect(screen,[110,0,10],[self.IntPose[0]-5,self.IntPose[1]-5,10,10],10)

    def ComputeRange(self,the_pose):
        # tmp_pose =np.asarray(the_pose)
        # the_pose = tmp_pose
        # for i in range(len(the_pose)):
        #     print("tmp:",tmp_pose,type(tmp_pose),tmp_pose[i])
        #     the_pose[i] = int(tmp_pose[i] * self.SCALEFACTOR) + self.OFFSET[i]

        tmp_distance = 0.0
        for i in range(len(self.Pose)):
            tmp_distance += (self.IntPose[i]*1.0 - the_pose[i]*1.0) ** 2.0
        tmp_distance = tmp_distance ** 0.5

        self.Dist = tmp_distance * 1.0 / self.SCALEFACTOR
        self.IntDist = int(tmp_distance)

    def GetRange(self,the_pose,sigma):
        if self.RangeMethond == "IDEAL":
            tmp_distance = 0.0
            for i in range(len(self.Pose)):
                tmp_distance += (self.Pose[i]*1.0 - the_pose[i]*1.0) ** 2.0
            tmp_distance = tmp_distance ** 0.5

            return tmp_distance

        elif self.RangeMethond == "SIMPLE_RANGE":
            tmp_distance = 0.0
            # print("range ",the_pose,sigma,self.Pose)
            for i in range(len(self.Pose)):
                tmp_distance += (self.Pose[i] * 1.0 - the_pose[i] * 1.0) ** 2.0
            tmp_distance = tmp_distance ** 0.5

            return tmp_distance + np.random.normal(0.0, sigma)








