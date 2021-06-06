#!/usr/bin/env python
from numpy import right_shift
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rospy.topics import Subscriber
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import tf
from gazebo_msgs.srv import GetModelState





class PoseMonitor():

    def __init__(self):

        self.x = 0.0
        self.y = 0.0 

        self.total = 1
        self.rights = 0

        rospy.init_node('pose_monitor', anonymous=True)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)


        self.report_pose = False

        print("Wait for service ....")
        rospy.wait_for_service("gazebo/get_model_state")
        
        print(" ... Got it!")
        
        self.get_ground_truth = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

        self.path = Path()

        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        self.final = rospy.Subscriber('/final', Int16, self.final_round)
        
        
    def callback_odometry(self, msg):
        global x
        global y
   
        global path

        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        

    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        #print ("Roll: %5.2f Pitch: %5.2f Yaw: %5.2f" % (roll, pitch, yaw))

    def final_round(self, msg):
        global x, y
        global rights, total
        
        print("reached final round")
        print("Calculating...")

        if round(self.x,1)==-1.5 and (round(self.y,1)>=-1.5 or round(self.y,1)<=1.5):
            self.rights = self.rights + 1
        if round(self.y,1)==1.5 and (round(self.x,1)>=-1.5 or round(self.x,1)<=1.5):
            self.rights = self.rights + 1
        if round(self.x,1)==1.5 and (round(self.y,1)>=-1.5 or round(self.y,1)<=1.5):
            self.rights = self.rights + 1
        if round(self.y,1)==-1.5 and (round(self.x,1)>=-1.5 or round(self.x,1)<=1.5):
            self.rights = self.rights + 1
        self.total += 1

        print("Error Rate: "+ str(self.rights/self.total *100))


        
if __name__ == '__main__':

    PoseMonitor()
    rospy.spin()
