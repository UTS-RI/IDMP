import rospy
from visualization_msgs.msg import Marker, MarkerArray
from idmp_ros.srv import GetDistanceGradient
import numpy as np

shotThreshold = 0.3
oldDist = [0,0,0,0,0,0,0,0,0,0,0]
ballPose = np.array([0,0,0.15])
ballVel = 0
ballVelMax = 2
ballVelDamping = 0.95
ballDirection = np.array([0,0,0])
goalCounter = 0

updateRate = 0.05

goalPolePos = 0.2
fieldSize = [4,1.5,0.5]

# -------|     |-------
# |                   |
# |                   |
# |         x         |
# |         |->y      |
# |                   |
# |                   |

def createPole(x,y,z,y_scale,z_scale,idnum):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.id = idnum
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    m.scale.x = 0.05
    m.scale.y = y_scale
    m.scale.z = z_scale
    m.color.r = 0
    m.color.g = 0
    m.color.b = 0
    m.color.a = 1
    return m

def createBall(x,y,z,idnum):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.id = idnum
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    m.scale.x = 0.2
    m.scale.y = 0.2
    m.scale.z = 0.2
    m.color.r = 1
    m.color.g = 1
    m.color.b = 1
    m.color.a = 1
    return m

def setupGoal():
    print("SetupGoal")
    goal_pub = rospy.Publisher("goal", MarkerArray, queue_size=0)
    mArr = MarkerArray()
    mArr.markers.append(createPole(fieldSize[0],goalPolePos,fieldSize[2]/2,0.05,fieldSize[2],1))
    mArr.markers.append(createPole(fieldSize[0],-goalPolePos,fieldSize[2]/2,0.05,fieldSize[2],2))
    mArr.markers.append(createPole(fieldSize[0],0,fieldSize[2],goalPolePos*2,0.05,3))
    goal_pub.publish(mArr)

def calcBallParams(dist,grad):
    global ballVel, ballDirection, oldDist
    delta =0
    if ballVel < 0.01:
        ballVel = 0
    if ballVel > 0:
        # ToDO: Coole Funktion
        ballVel = ballVel*ballVelDamping

    # if(dist < 0.4):
    #     grad[2] = 0
    #     ballVel = 1/dist
    # else:
    #     grad = np.array([0,0,0])
    # ballDirection = grad / max(dist, 1e-5)
    if dist < shotThreshold:
        delta = oldDist[0]-dist
        if delta > 0.2:
            ballVel = min(ballVel+200*(delta),ballVelMax)
            ballDirection = grad
            ballDirection[2] = 0
    if(dist < 100):
        oldDist.append(dist)
        oldDist.pop(0)
    # print("Vell:", ballVel, "delta:",delta, oldDist)
    return


def calcNewBallPose():
    global ballPose
    ballPose = ballPose+(updateRate*ballVel)*ballDirection
    # ballPose = ballPose+updateRate*ballDirection
    return
    
def updateVis():
    ball_pub = rospy.Publisher("Ball", Marker, queue_size=0)
    ball_pub.publish(createBall(ballPose[0],ballPose[1],0.2,99))
    return

def goalQuery():
    global ballPose,ballVel,ballDirection
    #goal
    if ballPose[0] < -fieldSize[0] and ballPose[1] < goalPolePos and ballPose[1] > -goalPolePos:
        goalCounter+=1
        print("GOAL !!!!!")
        print(goalCounter)
        ballPose = np.array([0,0,0])
        ballVel = 0
        ballDirection = np.array([0,0,0])

    #out
    if ballPose[0] > fieldSize[0] or ballPose[0] < -fieldSize[0] or ballPose[1] > fieldSize[1] or ballPose[1] < -fieldSize[1]:
        print("OUT !!!!!")
        ballPose = np.array([0,0,0])
        ballVel = 0
        ballDirection = np.array([0,0,0])

    if(ballVel==0):
        ballPose = np.array([0,0,0])
        ballVel = 0
        ballDirection = np.array([0,0,0])
    return

if __name__=="__main__":
    rospy.init_node("soccer")
    query = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)
    goal_pub = rospy.Publisher("goal", Marker, queue_size=0)
    setupGoal()
    while not rospy.is_shutdown():
        response = query(ballPose)
        dist = response.distances[0]
        grad = np.array(response.gradients)
        # print("Dist: ", dist," Grad: ",grad)
        calcBallParams(dist,grad)

        calcNewBallPose()
        updateVis()
        goalQuery()
        rospy.sleep(updateRate)
