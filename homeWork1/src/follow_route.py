#! /usr/bin/env python

# Import the Python library for ROS
from xmlrpc.client import TRANSPORT_ERROR
import rospy
import time

# Import the Twist message
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int16
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState


from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2

class MoveRobot():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        # Initiate a named node
        rospy.init_node('MoveRobot', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("CTRL + C to stop the turtlebot")
        
        # What function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        self.state_msg = ModelState()
        self.state_msg.model_name = 'turtlebot3_burger'
        self.state_msg.pose.position.x = -1.5
        self.state_msg.pose.position.y = 1.5
        self.state_msg.pose.position.z = 0
        self.state_msg.pose.orientation.x = 0
        self.state_msg.pose.orientation.y = 0
        self.state_msg.pose.orientation.z = 0
        self.state_msg.pose.orientation.w = 0

        # Create a Publisher object, will publish on cmd_vel_mux/input/teleop topic
        # to which the robot (real or simulated) is a subscriber
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Creates a var of msg type Twist for velocity
        self.vel = Twist()

        # Robot's initial place
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_state(self.state_msg)

        # Subscribe to topic /odom published by the robot base
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
    
        # publish a topic to notify node pose_monitor of the changed velocity
        self.new_velocity_sub = rospy.Publisher('/change', Twist, queue_size=1)

        self.notify_monitor = rospy.Publisher('/final', Int16, queue_size=1)

        self.reached = Int16()
        # Set a publish velocity rate of in Hz
        self.rate = rospy.Rate(5)

        

    def callback_odometry(self, msg):
        global x
        global y
        global theta
        global roll, pitch, yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
    

    def run(self):
        kp=1.2
        goal = Point()
        goal.x = -1.5
        goal.y = 1.5
        total_followed = 0
        lock = False
        rounds = 11
        while not rospy.is_shutdown():
            while total_followed != rounds+1:
                if total_followed == rounds :
                    self.reached.data = rounds
                    self.notify_monitor.publish(self.reached)
                    
                xdeff = goal.x -self.x
                ydeff = goal.y -self.y

                anglea = atan2(ydeff, xdeff)

                if abs(anglea - self.theta) > 0.15:
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = kp * (anglea-self.yaw)
                else:
                    self.vel.linear.x = 0.9
                    self.vel.angular.z = 0.0

                self.vel_pub.publish(self.vel)
                self.rate.sleep()
                # print("Robot is Moving in X: "+ str(round(self.x, 1))+ " and Y: "+ str(round(self.y, 1)))
                if round(self.x,1)==-1.5 and round(self.y,1)==1.5:
                    if lock is False:
                        total_followed += 1
                        print("Entering round: "+ str(total_followed))
                    lock = True
                    print("Side One Reached!")
                    goal.x = 1.5
                    goal.y = 1.5
                    continue
                if round(self.x, 1)==1.5 and round(self.y, 1)==1.5:
                    print("Side Two Reached!")
                    goal.x = 1.5
                    goal.y = -1.5
                    continue
                if round(self.x, 1)==1.5 and round(self.y, 1)==-1.5:
                    print("Side Three Reached!")
                    goal.x = -1.5
                    goal.y = -1.5
                    continue
                if round(self.x,1)==-1.5 and round(self.y,1)==-1.5:
                    lock = False
                    print("Side Four Reached!")
                    goal.x = -1.5
                    goal.y = 1.5
                    continue

            if total_followed == rounds+1:
                self.shutdown()
            
    
    def send_velocity_cmd(self):
        self.vel_pub.publish(self.vel)
    

    def shutdown(self):
        print ("Shutdown!")
        # stop TurtleBot
        rospy.loginfo("Stop TurtleBot")
        
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        
        self.vel_pub.publish(self.vel)
        
        # makes sure robot receives the stop command prior to shutting down
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        controller = MoveRobot()
        
        # keeping doing until ctrl+c
        while not rospy.is_shutdown():
            controller.run()
            # send velocity commands to the robots
            controller.send_velocity_cmd()
            
            # wait for the selected mseconds and publish velocity again
            controller.rate.sleep()
            
    except:
        rospy.loginfo("move_robot node terminated")