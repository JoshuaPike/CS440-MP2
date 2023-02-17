
# transform.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
# 
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains the transform function that converts the robot arm map
to the maze.
"""
import copy
from arm import Arm
from maze import Maze
from search import *
from geometry import *
from const import *
from util import *

def transformToMaze(arm, goals, obstacles, window, granularity):
    """This function transforms the given 2D map to the maze in MP1.
    
        Args:
            arm (Arm): arm instance
            goals (list): [(x, y, r)] of goals
            obstacles (list): [(x, y, r)] of obstacles
            window (tuple): (width, height) of the window
            granularity (int): unit of increasing/decreasing degree for angles

        Return:
            Maze: the maze instance generated based on input arguments.

    """
    # TODO
    # GET THIS ALGORITHM WORKING FOR THE 3 ARM CASE // FOR AN N ARM CASE

    # Map is the alpha, beta of the tip
    angleLimit = arm.getArmLimit()
    if len(angleLimit) == 3:
        offsets = [angleLimit[0][0], angleLimit[1][0], angleLimit[2][0]]
        numRows = int((angleLimit[0][1] - angleLimit[0][0])/granularity) + 1
        numCols = int((angleLimit[1][1] - angleLimit[1][0])/granularity) + 1
        numDepth = int((angleLimit[2][1] - angleLimit[2][0])/granularity) + 1
    elif len(angleLimit) == 2:
        offsets = [angleLimit[0][0], angleLimit[1][0], 0]
        numRows = int((angleLimit[0][1] - angleLimit[0][0])/granularity) + 1
        numCols = int((angleLimit[1][1] - angleLimit[1][0])/granularity) + 1
        numDepth = 1
    else:
        offsets = [angleLimit[0][0], 0, 0]
        numRows = int((angleLimit[0][1] - angleLimit[0][0])/granularity) + 1
        numCols = 1
        numDepth = 1
    # # Number of positions in first dim, alpha (call it rows)
    # numRows = int((angleLimit[0][1] - angleLimit[0][0])/granularity) + 1
    # # Number of positions in second dim, beta (call it cols)
    # numCols = int((angleLimit[1][1] - angleLimit[1][0])/granularity) + 1
    # # Number of positions in third dim, gamma (call it depth)
    # numDepth = int((angleLimit[2][1] - angleLimit[2][0])/granularity) + 1

    # Told to loop thru output map coordinates and transform each pair into angles
    # Check if arm is:
    #   1. within window
    #   2. arm tip touch goals
    #   3. arm is hitting obstacle
    #   4. SET THE START POS

    # arm.setArmAngle([alpha, beta])
    # arm.getArmPosDist() gets list of all arms [start, end, padding]
    # arm.getArmPos() gets list of all arms [start, end]
    startAngles = arm.getArmAngle()
    if len(arm.getArmAngle()) == 3:
        pass
    elif len(arm.getArmAngle()) == 2:
        startAngles.append(0)
    else:
        startAngles.append(0)
        startAngles.append(0)
    mazeArray = []

    # print("Jumping into for loop, num rows: ", numRows)
    # ctr = 0
    for a in range(numRows): 
        # print("New Row: ", ctr, "\n")
        # ctr+=1
        rowToAdd = []
        for b in range(numCols):
            depthToAdd = []
            betaBreakFlag = False
            for c in range(numDepth):
                curAngle = idxToAngle([a, b, c], offsets, granularity)
                arm.setArmAngle(curAngle)
                armPos = arm.getArmPos()
                armPosDist = arm.getArmPosDist()

                # Check if tip has touched goal
                if doesArmTipTouchGoals(arm.getEnd(), goals):
                    depthToAdd.append('.')
                    continue
                
                # Check within window
                if not isArmWithinWindow([armPos[0]], window):
                    # Write wall to this alpha for all beta, gamma
                    for remainingBeta in range(0, numCols):
                        for remainingGamma in range(0, numDepth):
                            depthToAdd.append('%')
                        rowToAdd.append(depthToAdd)
                    betaBreakFlag = True
                    break
                elif not isArmWithinWindow([armPos[1]], window):
                    # Write wall to this (alpha, beta) for all gamma
                    for remainingGamma in range(0, numDepth):
                        depthToAdd.append('%')
                    break
                elif not isArmWithinWindow([armPos[2]], window):
                    # Write wall to this (alpha, beta, gamma)
                    depthToAdd.append('%')
                    continue

                # Check if arm is hitting obstacles
                if doesArmTouchObjects([armPosDist[0]], obstacles) or doesArmTouchObjects([armPosDist[0]], goals, True):
                    # Write wall to this alpha for all beta, gamma
                    for remainingBeta in range(0, numCols):
                        for remainingGamma in range(0, numDepth):
                            depthToAdd.append('%')
                        rowToAdd.append(depthToAdd)
                        betaBreakFlag = True
                    break
                elif doesArmTouchObjects([armPosDist[1]], obstacles) or doesArmTouchObjects([armPosDist[1]], goals, True):
                    # Write a wall to this (alpha, beta) for all gamma
                    for remainingGamma in range(0, numDepth):
                        depthToAdd.append('%')
                    break
                elif doesArmTouchObjects([armPosDist[2]], obstacles) or doesArmTouchObjects([armPosDist[2]], goals, True):
                    # Write a wall to this (alpha, beta, gamma)
                    depthToAdd.append('%')
                    continue
                depthToAdd.append(' ')
            if betaBreakFlag:
                # We want to also break out of the beta loop
                betaBreakFlag = False
                break
            else:
                rowToAdd.append(depthToAdd)
        mazeArray.append(rowToAdd)

    startIdx = angleToIdx(startAngles, offsets, granularity)
    mazeArray[startIdx[0]][startIdx[1]][startIdx[2]] = 'P'
    
    angleMaze = Maze(mazeArray, offsets, granularity)
    return angleMaze

    # for a in range(numRows): 
    #     rowToAdd = []
    #     for b in range(numCols):
    #         curAngle = idxToAngle([a, b], offsets, granularity)
    #         arm.setArmAngle(curAngle)
    #         armPos = arm.getArmPos()
    #         armPosDist = arm.getArmPosDist()
            
    #         # Check if tip has touched goal
    #         if doesArmTipTouchGoals(arm.getEnd(), goals):
    #             rowToAdd.append('.')
    #             continue

    #         # Check for first arm first so we can eliminate all betas for that alpha
    #         # Check within window
    #         if not isArmWithinWindow([armPos[0]], window):
    #             # Write a wall to this alpha for all beta
    #             for remainingBeta in range(b, numCols):
    #                 rowToAdd.append('%')
    #             break
    #         elif not isArmWithinWindow([armPos[1]], window):
    #             # Write a wall to this pos for only this a, b
    #             rowToAdd.append('%')
    #             continue

    #         # Check if arm is hitting obstacles
    #         if doesArmTouchObjects([armPosDist[0]], obstacles) or doesArmTouchObjects([armPosDist[0]], goals, True):
    #             # Write a wall to this alpha for all beta
    #             for remainingBeta in range(b, numCols):
    #                 rowToAdd.append('%')
    #             break
    #         elif doesArmTouchObjects([armPosDist[1]], obstacles) or doesArmTouchObjects([armPosDist[1]], goals, True):
    #             # Write a wall to this pos for only this a, b
    #             rowToAdd.append('%')
    #             continue
            
    #         rowToAdd.append(' ')
    #     mazeArray.append(rowToAdd)

    # startIdx = angleToIdx(startAngles, offsets, granularity)
    # mazeArray[startIdx[0]][startIdx[1]] = 'P'
    
    # angleMaze = Maze(mazeArray, offsets, granularity)
    # return angleMaze