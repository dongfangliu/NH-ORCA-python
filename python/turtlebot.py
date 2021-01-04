#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from turtlesim.msg import Pose
from math import pow, atan2, sqrt,cos,sin
import numpy as np
class TurtleBot:
    def __init__(self,turtleId,botRadius,botPosition,botOrientation,botGoal,wheel_distance,effective_distance,maxRotSpeed,maxLinearSpeed ):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        self.turtleId = turtleId
        self.L = wheel_distance
        self.D = effective_distance
        self.initPos = botPosition
        self.initRot = botOrientation

        self.botRadius = botRadius
        self.botGoal = botGoal

        self.maxLinearSpeed = maxLinearSpeed
        self.maxRotSpeed = maxRotSpeed

        self.nodeName = 'turtle'+str(turtleId)
        self.vel_topic_name = "/"+self.nodeName+'/cmd_vel_mux/input/teleop'
        # self.vel_topic_name = '/cmd_vel_mux/input/teleop'
        # self.pose_topic_name = "/"+self.nodeName+'/odom'
        self.odom_topic_name = "/"+self.nodeName+'/odom'
        # self.odom_topic_name = '/odom'
        self.pose_topic_name = "/vrpn_client_node/"+self.nodeName+'/pose'

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(self.vel_topic_name,
                                                  Twist, queue_size=10)
        # self.pose_subscriber = rospy.Subscriber(self.pose_topic_name,
        #                                     Odometry, self.update_pose)
        self.odom_subscriber = rospy.Subscriber(self.odom_topic_name,
                                        Odometry, self.update_twist)
        self.pose_subscriber = rospy.Subscriber(self.pose_topic_name,
                                        PoseStamped, self.update_pose)
        self.initialized = False
        self.twist = Twist()

    def done(self):
        return  np.linalg.norm(self.getPos() - self.botGoal) > self.botRadius*2

    def update_twist(self,data):
        # print("Update twist")
        self.twist =  data.twist.twist

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        # self.pos = np.array([data.pose.pose.position.x,data.pose.pose.position.y])+self.initPos
        # self.theta = euler_from_quaternion( quaternion=( data.pose.pose.orientation.x,data.pose.pose.orientation.y, data.pose.pose.orientation.z,data.pose.pose.orientation.w))[2]+self.initRot
        #self.twist =  data.twist.twist
        self.pos = np.array([data.pose.position.x,data.pose.position.y])+self.initPos
        euler_angles = euler_from_quaternion( quaternion=( data.pose.orientation.x,data.pose.orientation.y, data.pose.orientation.z,data.pose.orientation.w))
        self.theta = euler_angles[2]+self.initRot

        if not self.initialized:
            print(self.nodeName+" initialized with pose ",self.pos, " orientation ",self.theta)
            self.initialized = True

    def cal_effective_cmd(self, pref_vel):
        A = 0.5*cos(self.theta)+self.D*sin(self.theta)/self.L
        B = 0.5*cos(self.theta)-self.D*sin(self.theta)/self.L
        C = 0.5*sin(self.theta)-self.D*cos(self.theta)/self.L
        D = 0.5*sin(self.theta)+self.D*cos(self.theta)/self.L

        # M = np.array([
        #     [A,B],
        #     [C,D],
        # ])
        # v = np.matmul(np.linalg.inv(M),pref_vel)
        vx = pref_vel[0]
        vy = pref_vel[1]
        vr = (vy-C/A*vx)/(D-B*C/A)
        vl = (vx-B*vr)/A

        # vl = v[0]
        # vr = v[1]
        vel_msg = Twist()
        vel_msg.angular.x = 0 
        vel_msg.angular.y = 0
        vel_msg.angular.z =   (vr-vl)/self.L
        # hand edition constraint
        vel_msg.angular.z= np.clip(-self.maxRotSpeed,vel_msg.angular.z,self.maxRotSpeed)
        vel_msg.linear.x =0.5*(vl+vr)
        # hand edition constraint
        vel_msg.linear.x = np.clip(-self.maxLinearSpeed,vel_msg.linear.x,self.maxLinearSpeed)
        vel_msg.linear.y = 0
        vel_msg.linear.z=0
        return vel_msg
    def getVelocity(self):
        pass

    def getPos(self):
        return self.pos

    def getEffectivePos(self):
        return self.getPos()+self.D*np.array([cos(self.theta),sin(self.theta)])

    def getEffectiveVel(self):
        w = self.twist.angular.z
        v = self.twist.linear.x
        vr = v+0.5*w*self.L
        vl = 2*v - vr
        x_vel = (0.5*cos(self.theta)+self.D*sin(self.theta)/self.L)*vl +  (0.5*cos(self.theta)-self.D*sin(self.theta)/self.L)*vr
        y_vel = (0.5*sin(self.theta)-self.D*cos(self.theta)/self.L)*vl +  (0.5*sin(self.theta)+self.D*cos(self.theta)/self.L)*vr
        return np.array([x_vel,y_vel])
        # A = 0.5*cos(self.theta)+self.D*sin(self.theta)/self.L
        # B = 0.5*cos(self.theta)-self.D*sin(self.theta)/self.L
        # C = 0.5*sin(self.theta)-self.D*cos(self.theta)/self.L
        # D = 0.5*sin(self.theta)+self.D*cos(self.theta)/self.L

        # M = np.array([
        #     [A,B],
        #     [C,D],
        # ])
        # v_wheel = np.transpose(np.array([vl,vr]))
        # return np.matmul(M,v_wheel)
    
    def cmd_vel(self,vel_msg):
        # Publishing our vel_msg
        self.velocity_publisher.publish(vel_msg)
        # If we press control + C, the node will stop.
        # rospy.spin()
