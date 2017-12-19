# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 27　9:17

import pygame

import numpy as np
import scipy as sp


class PF_Frame:
    def __init__(self, SCREEN_SIZE, OFFSET, ScaleFactor, Particle_num):
        '''

        :param SCREEN_SIZE:
        :param OFFSET: piexels
        :param ScaleFactor: Real(m) to piexels
        '''

        self.SCREEN_SIZE = SCREEN_SIZE
        self.OFFSET = OFFSET
        self.SCALEFACTOR = ScaleFactor

        self.Pose = [10, 10]
        self.Dist = 0

        self.IntPose = [1, 1]
        self.IntDist = 0

        self.P_state = np.zeros([Particle_num, 2])
        self.Wight = np.ones(Particle_num)

        self.EstimatePose = self.Pose

        self.BeaconSet = np.zeros([3, 2])

        self.path = list()

    def SetBeaconSet(self, beaconset):
        self.BeaconSet = np.zeros_like(beaconset)
        self.BeaconSet = beaconset

    def InitialPose(self, pose):
        for i in range(len(pose)):
            self.Pose[i] = pose[i]

            self.IntPose[i] = int(self.Pose[i] * 1.0 * self.SCALEFACTOR) \
                              + self.OFFSET[i]

        for k in range(self.P_state.shape[0]):
            self.P_state[k, :] = self.Pose
        self.Sample(1.1)

    def Sample(self, sigma):
        rand_pose_offset = np.random.normal(0.0, sigma, self.P_state.shape)

        self.P_state += rand_pose_offset

    def OdometrySample(self, delta_pose, sigma):
        for i in range(self.P_state.shape[1]):
            self.P_state[:, i] += np.random.normal(delta_pose[i],
                                                   sigma,
                                                   self.P_state.shape[0])

    def Evaluated(self, Ranges, sigma=1.0):
        # print("P_state_shape [0] :",self.P_state.shape[0])
        # print("Ranges:",Ranges)
        for k in range(self.P_state.shape[0]):
            self.Wight[k] *= self.Score(Ranges, self.P_state[k, :], sigma)

    def Score(self, Ranges, pose, sigma=1.0):
        # Methond 1
        # dis_err = 0
        # for i in range((self.BeaconSet.shape[1])):
        #     dis_err += (np.linalg.norm(self.BeaconSet[i,:] - pose) - Ranges[i]) ** 2.0
        #
        # return 1/dis_err ** 0.5
        '''
        METHOnd 2
        '''
        # Methond 2 multiply
        dis = 0.0
        # ToDo: Chage it back to add.
        score = 1.0
        for i in range(self.BeaconSet.shape[0]):
            dis = np.linalg.norm(self.BeaconSet[i, :] - pose)
            score *= (self.NormPdf(Ranges[i], dis, sigma) + 1e-50)
        return score
        '''
        '''
        # Methond 3
        # dis = 0.0
        # score = 0.0
        # for i in range(self.BeaconSet.shape[0]):
        #
        #     dis = np.linalg.norm(self.BeaconSet[i,:]-pose)
        #     score += (4000 - np.abs(dis-Ranges[i]))/2000
        #     # print (dis)
        # return score

    def NormPdf(self, x, miu, sigma):
        para1 = 1 / np.sqrt(2.0 * np.pi) / sigma
        para2 = - (x - miu) ** 2.0 / sigma / sigma
        return para1 * np.exp(para2)

    def ReSample(self):
        self.Wight /= self.Wight.sum()
        self.Beta = self.Wight

        tmp_Wight = self.Wight
        tmp_P_state = self.P_state

        # for i in range(self.P_state.shape[0]):
        #     if i >0:
        #         self.Beta[i] = self.Beta[i-1] + self.Wight[i]
        #
        # for i in range(self.P_state.shape[0]):
        #     tmp_rnd = np.random.uniform(0.0, 1.0)
        #
        #     for j in range(self.P_state.shape[0]):
        #         if tmp_rnd < self.Beta[j]:
        #             tmp_P_state[i,:] = self.P_state[j,:]
        #             tmp_Wight[i] = self.Wight[j]
        #             # print("j:",j)
        #             break

        # RESAMPLE METHOND 2
        # print(self.Wight.sum())
        for i in range(self.P_state.shape[0]):
            # print("wight sum:",self.Wight.sum())
            if np.isnan(self.Wight.sum()):
                self.Wight = np.ones_like(self.Wight)
                self.Wight /= np.sum(self.Wight)
                print("some error ,so reset the wight.")
            tmp_rnd = np.random.uniform(0.0, self.Wight.sum())
            i_index = -1
            while (tmp_rnd > 0.0):
                i_index += 1
                tmp_rnd -= self.Wight[i_index]
                # if i_index == self.P_state.shape[0] - 1:
                # i_index =int( (np.random.uniform(0.0,0.9999)) * self.P_state.shape[0])
            tmp_P_state[i, :] = self.P_state[i_index, :]
            tmp_Wight[i] = self.Wight[i_index]

        self.P_state = tmp_P_state
        self.Wight = tmp_Wight

    def GetResult(self):
        self.Wight /= self.Wight.sum()
        tmp_result = np.zeros(self.P_state.shape[1])
        for i in range(self.Wight.shape[0]):
            tmp_result += self.P_state[i, :] * self.Wight[i]
        self.EstimatePose = tmp_result
        return tmp_result

    def Draw(self, screen):

        for k in range(self.P_state.shape[0]):
            IntPose = [0, 0]
            for i in range(self.P_state.shape[1]):
                IntPose[i] = int(self.P_state[k, i] * self.SCALEFACTOR) \
                             + self.OFFSET[i]
            # pygame.draw.circle(screen,[200,200,2,20],IntPose,int(self.Wight[k]*self.P_state.shape[0] * 10),int(self.Wight[k]*self.P_state.shape[0] * 10))
            pygame.draw.circle(screen,
                               [200, 200, 2, 20],
                               IntPose,
                               int(2), int(2))
            # print(k)
        IntPose = [0, 0]

        for i in range(self.P_state.shape[1]):
            IntPose[i] = int(self.EstimatePose[i] * self.SCALEFACTOR) \
                         + self.OFFSET[i]
        self.path.append([IntPose[0], IntPose[1]])
        if len(self.path) > 20:
            self.path.pop(0)
        if len(self.path) > 2:
            # print(self.path)
            pygame.draw.lines(screen, [100, 210, 100], False, self.path, 3)

        pygame.draw.circle(screen, [0, 210, 20, 30], IntPose, int(5), int(5))

    def DrawLikeliHood(self, screen, Ranges, RECT_POSE):

        # print("Beging")
        pixObj = pygame.PixelArray(screen)

        for i in range(RECT_POSE[0], RECT_POSE[1]):
            for j in range(RECT_POSE[2], RECT_POSE[3]):
                score = self.Score(Ranges,
                                   (np.asarray([i, j]) -
                                    np.asarray(self.OFFSET)) * 1.0 /
                                   self.SCALEFACTOR)
                score = int(score / 3.0 * 255.0)
                pixObj[i][j] = (score, score, score)
                # i += 20
                # j += 20
                if i >= RECT_POSE[1] or j >= RECT_POSE[3]:
                    break
        del pixObj
