import rospy
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
import numpy as np
from idmp_ros.srv import GetDistanceGradient


class BallSimulation:
    def __init__(self):
        rospy.init_node('ball_simulation')

        self.ball_position = np.array([0.0, 0.0, 1.0])
        self.ball_velocity = np.array([0.0, 0.0, 0.0])
        self.goal_position = np.array([0.0, 0.0, 1.5])  # Change as needed

        self.marker_pub = rospy.Publisher('/ball_marker', Marker, queue_size=10)
        self.distance_service = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)

        rospy.Timer(rospy.Duration(0.1), self.update)

    def update(self, event):
        # Attract to goal
        attraction = self.goal_position - self.ball_position
        attraction_norm = np.linalg.norm(attraction)
        if attraction_norm > 0:
            attraction = attraction / attraction_norm
        if(attraction_norm < 0.05):
            attraction*=0

        # Repel from obstacles
        try:
            response = self.distance_service(self.ball_position)
            distance = response.distances[0]
            if(distance < 0.4):
                gradient = np.array(response.gradients)
                gradient[2] = 0
            else:
                gradient = np.array([0,0,0])
            repulsion = gradient / max(distance, 1e-5)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            repulsion = np.array([0.0, 0.0, 0.0])
        
        # Update velocity and position
        damping = 0.6
        self.ball_velocity += 0.8 * (attraction + repulsion)  # Adjust coefficients as needed
        self.ball_velocity*=damping
        self.ball_position += self.ball_velocity * 0.1

        # Publish marker
        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ball_simulation"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(self.ball_position[0], self.ball_position[1], self.ball_position[2])
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(0.3, 0.3, 0.3)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    BallSimulation()
    rospy.spin()
