# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 27　9:09

import pygame

class Robo:
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


    def SetPose(self,pose):


        for i in range(len(self.Pose)):
            self.Pose[i] = pose[i] #* self.SCALEFACTOR
            self.IntPose[i] = int(self.Pose[i]*self.SCALEFACTOR +self.OFFSET[i] )

    def Draw(self,screen):
        '''

        :param screen:
        :return:
        '''
        pygame.draw.rect(screen,[100,0,100,100],[self.IntPose[0]-5,self.IntPose[1]-5,10,10],10)