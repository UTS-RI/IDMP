'''
    IDMP - Interactive Distance Field Mapping and Planning to Enable Human-Robot Collaboration
    Copyright (C) 2024 Usama Ali

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License v3 as published by
    the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License v3 for more details.

    You should have received a copy of the GNU General Public License v3
    along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.

    Authors: Usama Ali <usama.ali@thws.de>
             Adrian Mueller <adrian.mueller@thws.de>
             Lan Wu <Lan.Wu-2@uts.edu.au>
'''

import rospy
from geometry_msgs.msg import Point

from idmp_ros.srv import GetDistanceGradient
from sensor_msgs.msg import PointCloud2
import numpy as np
import ros_numpy
import sys
import os

def getVoxelCenters(voxSize, xMin, xMax, yMin, yMax, zMin, zMax):
    xMinBound = round(int(xMin/voxSize) * voxSize + voxSize/2,4) #norm range to voxels and then shift by halflenght
    xMaxBound = round(int(xMax/voxSize) * voxSize + voxSize/2 - voxSize/4,4) 
    yMinBound = round(int(yMin/voxSize) * voxSize + voxSize/2,4)
    yMaxBound = round(int(yMax/voxSize) * voxSize + voxSize/2 - voxSize/4,4)
    zMinBound = round(int(zMin/voxSize) * voxSize + voxSize/2,4)
    zMaxBound = round(int(zMax/voxSize) * voxSize + voxSize/2 - voxSize/4,4)
    return np.mgrid[xMinBound:xMaxBound:voxSize, yMinBound:yMaxBound:voxSize, zMinBound:zMaxBound:voxSize].reshape(3,-1).T

def numpy_to_msg(arr, intensity, frame):
    data = np.zeros(len(arr), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),('Intensity', np.float32)])
    data['x'] = arr[:, 0]
    data['y'] = arr[:, 1]
    data['z'] = arr[:, 2]
    data['Intensity'] = intensity
    return ros_numpy.msgify(PointCloud2, data, stamp=rospy.Time.now(), frame_id=frame) # type:ignore

if __name__=="__main__":
    if(len(sys.argv) < 10): #storePath, testType, voxSize, xMin, xMax, yMin, yMax, zMin, zMax
        exit("Wrong parameters!")
    if(not os.path.isdir(sys.argv[1])):
        print(sys.argv[1])
        exit("Wrong directory!")

    rospy.init_node("dumpField")
    
    query = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)
    pcl_pub = rospy.Publisher("distfield", PointCloud2, queue_size=0)

    # print("Setup Grid")
    queryGrid = getVoxelCenters(float(sys.argv[3]),float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]))
    ser_grid = np.reshape(queryGrid, len(queryGrid)*3) #serialized points

    # print("Query")
    res = query(ser_grid)

    # print("Gradients")
    grads = np.array(res.gradients).reshape((len(res.gradients)//3,3)) #rearrange gradients so it is easier to index

    # print("Combine matrixes")
    dist = np.array(res.distances).reshape(len(res.distances),1)
    result = np.hstack((queryGrid,dist))
    result = np.hstack((result,grads))

    # print("Save to csv")
    lineDump = []
    lineDump.append("x,y,z,d,dx,dy,dz\n")
    for idx,p in enumerate(result):
        lineDump.append(str(round(p[0],4)) + "," + str(round(p[1],4)) + "," + str(round(p[2],4)) + "," + str(p[3]) + "," + str(p[4]) + "," + str(p[5]) + "," + str(p[6]) +"\n")
    
    if(float(sys.argv[3])>=0.1):
        name = str(int(float(sys.argv[3])*1000))
    else:
        name = "0"+str(int(float(sys.argv[3])*1000))

    file = open(sys.argv[1]+"/IDMP_"+name+".csv", "w")
    file.writelines(lineDump)
    file.close()
    # print("Done")