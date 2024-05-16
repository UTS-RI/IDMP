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

import subprocess
import os
import time
import sys
import yaml
from datetime import datetime, timedelta
import numpy as np

evalPath = "/home/usama/Workspace/ros/utsWs/eval/"

def readConfig(file):
    configDocument = yaml.emit(yaml.parse(file))
    return yaml.load(configDocument, Loader=yaml.FullLoader)

def writeConfig(config,file):
    file.seek(0)
    fileData = list(file)
    for i in range(len(fileData)):
        try:
            if ": " in fileData[i]:
                if (config[fileData[i].split(": ")[0]] is None):
                    continue
                elif(type(config[fileData[i].split(": ")[0]]) is str):
                    fileData[i] = fileData[i].split(": ")[0] + ': "' + config[fileData[i].split(": ")[0]] +'"\n'
                else:
                    fileData[i] = fileData[i].split(": ")[0] + ': ' + str(config[fileData[i].split(": ")[0]]) +'\n'
        except KeyError:
            pass
    file.seek(0)
    file.truncate(0)
    file.write("".join(fileData))

def runIDMP(testType):
    FNULL = open(os.devnull,'w')
    roscore = subprocess.Popen("roscore", stdout=FNULL)
    time.sleep(3)
    
    idmp = subprocess.Popen(["roslaunch", "--no-summary", "--skip-log-check", "idmp_ros", "run.launch"], stdout=subprocess.PIPE, universal_newlines=True)
    tf = subprocess.Popen(["rosrun", "idmp_ros", "datasetTFPublisher.py"])
    rviz = subprocess.Popen(["rosrun", "rviz", "rviz", "-d", "/home/usama/Workspace/ros/utsWs/src/LogGPIS_ROS/config/LogGPIS.rviz"], stdout=FNULL)
    time.sleep(1)
    if(getConfig("idmp_depth_input")):
        bag = subprocess.Popen(["rosbag","play","/home/usama/Workspace/ros/utsWs/bags/gazeboBall.bag"])
    else:
        bag = subprocess.Popen(["rosbag","play","/home/usama/Workspace/ros/utsWs/bags/ladyCow.bag", "--clock"])
    bag.wait()
    print("Bag finished...")
    time.sleep(1)

    dirPath = evalPath+"/"+"_".join(testType.split("_")[:-1])
    evalScript = subprocess.Popen(["rosrun", "idmp_ros", "evalTest.py", dirPath, testType])
    evalScript.wait()
    for i in [0.05,0.1,0.15,0.2,0.25]:
        dumpScript = subprocess.Popen(["rosrun", "idmp_ros", "dumpField.py", dirPath, testType,str(i),"-2","2","-2","0.8","0.2","1.5"])
        dumpScript.wait()

    tf.kill()
    idmp.terminate()
    text = idmp.communicate()[0]
    roscore.terminate()
    rviz.terminate()
    maxFrame = 0
    avgTime = 0
    for l in text.split("\n"):
        words = l.split(" ")
        if(len(words) == 2 and words[0].replace('.','',1).isdigit() and words[1].replace('.','',1).isdigit()):
            frame = int(words[0])
            avgTime += float(words[1])
            if frame > maxFrame:
                maxFrame = frame
    
    return maxFrame, avgTime/maxFrame


def setConfig(field,data):
    configFile = open("/home/usama/Workspace/ros/utsWs/src/LogGPIS_ROS/config/params.yaml","r+")
    configData = readConfig(configFile)
    configData[field] = data
    writeConfig(configData,configFile)
    configFile.close()

def getConfig(field):
    configFile = open("/home/usama/Workspace/ros/utsWs/src/LogGPIS_ROS/config/params.yaml","r+")
    configData = readConfig(configFile)
    return configData[field]

def runTest(param, vals, prefix = ""):
    print("Testing "+param+" with "+str(vals))
    print("Will approximately finish at ",datetime.now()+timedelta(0,150*len(vals)))
    oldParam = getConfig(param)
    testPath = evalPath+"/"+param+prefix
    if(not os.path.isdir(testPath)):
        os.mkdir(testPath)
        repFile = open(testPath+"/report.csv","a")
        repFile.write("parameter,rmse,frames,avgTime\n")
        repFile.close()
    
    for v in vals:
        print("Running "+param+" with "+str(v))
        setConfig(param, v)
        f,t = runIDMP(param+prefix+"_"+str(v))
        repFile = open(testPath+"/report.csv","a")
        repFile.write(","+str(f)+","+str(t)+"\n")
        repFile.close()

    setConfig(param,oldParam)

def benchmark():
    testPath = evalPath + "/benchmark"
    if(not os.path.isdir(testPath)):
        os.mkdir(testPath)
    repFile = open(testPath+"/report.csv","a")
    repFile.write("parameter,rmse,frames,avgTime\n")
    repFile.close()
    f,t = runIDMP("benchmark_Filt")
    repFile = open(testPath+"/report.csv","a")
    repFile.write(","+str(f)+","+str(t)+"\n")
    repFile.close()


if __name__ == "__main__":
    # runTest("idmp_filt_vox", np.array([0.01, 0.02,0.03,0.04, 0.05, 0.1, 0.15, 0.2]))
    # runTest("idmp_rleng", np.array([1.5,1.6,1.7,1.8,1.9,2.0,2.1,2.2,2.3]))
    # runTest("idmp_fusion", np.array([False, True]))
    # runTest("idmp_tree_hl_clust", np.array([0.025,0.05,0.1,0.2]))
    # runTest("idmp_tree_hl_min", np.array([0.0125,0.025,0.05]))


    # runTest("idmp_map_scale", np.array([5,10,15,20,25,30,35])**2)

    # runTest("idmp_fus_min", np.array([0.0018,0.0015,0.001,0.0005, 0.0003]))
    #Benchmark
    benchmark()
    