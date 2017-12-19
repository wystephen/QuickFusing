# -*- coding:utf-8 -*-
# carete by steve at  2016 / 10 / 26　16:50


import pygame

import numpy as np

from Beacon import BeaconWithRange

from Agent import Robo

from PF_FRAME import PF_Frame

if __name__ == '__main__':
    # --- Clobals ---
    # Colors

    BLACK=(0,0,0)
    WHITE=(255,255,255)

    SCREEN_SIZE=[1680,980]

    OFFSET = [10,10] # piexels

    ScaleFactor = 1.0 #Real(m) to piexels


    pygame.init()

    screen = pygame.display.set_mode(SCREEN_SIZE)

    pygame.display.set_caption("Test All")

    allspriteslit = pygame.sprite.Group()

    clock = pygame.time.Clock()
    done = False

    tmp_beacon = BeaconWithRange(SCREEN_SIZE,OFFSET,ScaleFactor)
    tmp_beacon2 = BeaconWithRange(SCREEN_SIZE,OFFSET,ScaleFactor)
    tmp_beacon3 = BeaconWithRange(SCREEN_SIZE,OFFSET,ScaleFactor)

    tmp_beacon.SetPose(500,600)
    tmp_beacon2.SetPose(500,800)
    tmp_beacon3.SetPose(1300,900)

    tmp_beacon.SetRangeMethond(1)
    tmp_beacon2.SetRangeMethond(1)
    tmp_beacon3.SetRangeMethond(1)

    tmp_robo = Robo(SCREEN_SIZE,OFFSET,ScaleFactor)

    pf = PF_Frame(SCREEN_SIZE,OFFSET,ScaleFactor,1300)

    BeaconSet = np.zeros([3,2])

    BeaconSet[0,:] = tmp_beacon.Pose
    BeaconSet[1,:] = tmp_beacon2.Pose
    BeaconSet[2,:] = tmp_beacon3.Pose

    pf.SetBeaconSet(BeaconSet)

    pygame.mouse.set_visible(False)

    last_pose = np.zeros(2)

    IsStop = False
    while not done:
        pose = pygame.mouse.get_pos()
        print("dis:",np.linalg.norm(np.asarray(pose)-last_pose))
        last_pose = np.asarray(pose)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

            elif event.type == pygame.KEYDOWN:
                print(event.key)
                if event.key == 115:
                    pf.InitialPose([pose[0],pose[1]])
                    # IsStop = not IsStop
        if IsStop:
            continue




        screen.fill(BLACK)
        # pygame.draw.circle(screen,[110,10,155],pose,20,3)


        tmp_beacon.ComputeRange(pose)
        tmp_beacon2.ComputeRange(pose)
        tmp_beacon3.ComputeRange(pose)

        # tmp_beacon.SetRange(pose[1]+2)
        tmp_beacon.Draw(screen)
        tmp_beacon2.Draw(screen)
        tmp_beacon3.Draw(screen)

        #Robot draw
        tmp_robo.SetPose(pose)
        tmp_robo.Draw(screen)

        pf.Sample(52.1)
        # print("real range:" ,[tmp_beacon.GetRange(pose,0.1),
        #               tmp_beacon2.GetRange(pose,0.1),
        #               tmp_beacon3.GetRange(pose,0.1)])

        pf.Evaluated([tmp_beacon.GetRange(pose,21),
                      tmp_beacon2.GetRange(pose,21),
                      tmp_beacon3.GetRange(pose,21)])

        pf.ReSample()
        pf.Draw(screen)

        # pygame.draw.rect(screen,[0,100,100],[pose[0],pose[1],10,10],10)

        pygame.display.flip()

        # clock.tick(1)

    pygame.quit()


