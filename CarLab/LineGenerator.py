import numpy as np
import random
import math
import matplotlib.pyplot as plt


def intersects(s0,s1):
    dx0 = s0[1][0]-s0[0][0]
    dx1 = s1[1][0]-s1[0][0]
    dy0 = s0[1][1]-s0[0][1]
    dy1 = s1[1][1]-s1[0][1]
    p0 = dy1*(s1[1][0]-s0[0][0]) - dx1*(s1[1][1]-s0[0][1])
    p1 = dy1*(s1[1][0]-s0[1][0]) - dx1*(s1[1][1]-s0[1][1])
    p2 = dy0*(s0[1][0]-s1[0][0]) - dx0*(s0[1][1]-s1[0][1])
    p3 = dy0*(s0[1][0]-s1[1][0]) - dx0*(s0[1][1]-s1[1][1])
    return (p0*p1<=0) & (p2*p3<=0)
        
def randomPath():
    pathList = []
    StartCord = (0,0)
    startDir = 0
    ranStop = random.randint(0, 3)
    hasIntersect = False

    while ranStop > 0:
        ranLength = random.random() * 30 + 20
        newCord = (StartCord[0]+ranLength*(math.cos(math.radians(startDir))),StartCord[1]+ranLength*(math.sin(math.radians(startDir))))
        list2 = [StartCord,newCord]
  
        for i in pathList:
            list1 = [(pathList[i].startX, pathList[i].startY), (pathList[i].endX, pathList[i].endY)]
            if intersects(list1,list2):
                hasIntersect = True
                break
        if hasIntersect:
            continue

        pathList.append(Line(StartCord,newCord))
        Dir = bool(random.getrandbits(1))

        newAngle = random.random()*135+45
        newcurve = make_curve(newAngle, 10, start_point=[newCord[0],newCord[1]],n_lines=20,left=Dir,start_degree=startDir)
        if Dir:
            startDir = 360 % (startDir+newAngle)
        else:
            startDir = 360 % (startDir-newAngle)
        for i in newcurve:
          pathList.append(newcurve[i])
        StartCord = (newcurve[-1].endX,newcurve[-1].endY)
    return pathList