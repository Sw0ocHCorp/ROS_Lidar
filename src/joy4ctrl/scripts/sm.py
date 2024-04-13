#!/usr/bin/env python
""" ROS python pogramming with finite state machines to describe a robot's behaviors
    Vincent Hugel
    Seatech/SYSMER 2A Course
    free to use so long as the author and other contributers are credited.
"""
#############################################################################
# imports
#############################################################################
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from fsm import fsm
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# importation de fonctions utiles de transformations de repere (tf : transformation de repere)
import tf
from tf.transformations import *
from time import perf_counter
import math
import numpy as np

class GyroSensor:
    def __init__(self, default_rotation_angle= 180) -> None:
        self.default_rotation_angle = default_rotation_angle
        self.main_angle = 0.0
        self.rotation_target= 0
        self.angle_target= 0

    def OdoCallback(self, data):
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation
        # extraction de l'angle de lacet
        q = [self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w]
        euler = tf.transformations.euler_from_quaternion(q)
        self.main_angle = NormalizeAngle(Rad2Deg(euler[2]))

    
    def RegisterTargetAngle(self, rotation):
        self.rotation_target= rotation
        self.angle_target = self.main_angle + self.rotation_target
    
    def Reset(self):
        self.rotation_target= 0
        self.angle_target= 0
    
class LidarSensor:
    def __init__(self, gyro_sensor: GyroSensor, dist_detection= 2) -> None:
        self.obstacle_detection= False
        self.dist_detection= dist_detection
        self.interest_points= {}
        self.extract_points= False
        self.farest_obstacle= 0
        self.gyro_sensor= gyro_sensor

    def processScan(self, data):        
        self.scan_range = Rad2Deg(data.angle_max - data.angle_min)
        nb_values = len(data.ranges) # nombre de valeurs renvoyees par le lidar
        if self.extract_points:
            if np.nanmax(data.ranges) > self.farest_obstacle:
                self.farest_obstacle = np.nanmax(data.ranges)
                self.interest_points[self.farest_obstacle] = self.gyro_sensor.main_angle + ((self.scan_range / 2) - (self.scan_range * data.ranges.index(self.farest_obstacle)))
        if np.nanmin(data.ranges):
            self.obstacle_detection = True
        
class BumperSensor:
    def __init__(self) -> None:
        self.collision= False
        self.bumper_no= None

    def processBump(self,data):
        if ((not self.collision) and (data.state == BumperEvent.PRESSED)):
            self.collision=True
            self.bumper_no=data.bumper
        else:
            self.collision = False

#############################################################################
# class RobotBehavior
#############################################################################
class RobotBehavior(object):
    #############################################################################
    # constructor, called at creation of instance
    #############################################################################
    def __init__(self, handle_pub, T):
        self.gyro_sensor= GyroSensor()
        self.lidar_sensor = LidarSensor()
        self.bumper_sensor= BumperSensor()
        self.twist = Twist()
        self.twist_real = Twist()
        self.vreal = 0.0 # longitudial velocity
        self.wreal = 0.0 # angular velocity
        self.vmax = 1.5
        self.cpt = 0
        self.wmax = 4.0
        self.previous_signal = 0
        self.button_pressed = False
        self.joy_activated = False
        self.pub = handle_pub
        self.enough= True
        self.T = T
        self.time_stop= 0.33
        self.start_timer= True
        self.time_recul= 2.5
        self.time_rotate= 2.5
        self.dist_detect = 1.25 # 1 m, to be adjusted
        self.odo_rotate_angle= (2*math.pi)/3
        self.odo_target_angle = None
        
        self.last_obstacle = 0.0
        self.lidar_sensor.obstacle_detection= False
        # instance of fsm with source state, destination state, condition (transition), callback (defaut: None)
        self.fs = fsm([ ("Start","JoyControl", True ),
                ("JoyControl","AutonomousMode1", self.check_JoyControl_To_AutonomousMode1, self.DoAutonomousMode1),
                ("JoyControl","JoyControl", self.KeepJoyControl, self.DoJoyControl),
                ("AutonomousMode1","JoyControl", self.check_AutonomousMode1_To_JoyControl, self.DoJoyControl),
                ("AutonomousMode1","AutonomousMode1", self.KeepAutonomousMode1, self.DoAutonomousMode1),
                ("AutonomousMode1","rotate", self.check_AutonomousMode1_To_rotate, self.Dorotate),
                ("AutonomousMode1","stop1", self.check_AutonomousMode1_To_stop1, self.Dostop1),
                ("stop1","stop1", self.Keepstop1, self.Dostop1),
                ("stop1","recule", self.check_stop1_To_recule, self.Dorecule),
                ("recule","recule", self.Keeprecule, self.Dorecule),
                ("recule","stop2", self.check_recule_To_stop2, self.Dostop2),
                 ("stop2","stop2", self.Keepstop2, self.Dostop2),
                ("stop2","rotate", self.check_stop2_To_rotate, self.Dorotate),
                ("rotate","rotate", self.Keeprotate, self.Dorotate),
                    ("rotate","stop3", self.check_rotate_To_stop3, self.Dostop3),
                ("stop3","stop3", self.Keepstop3, self.Dostop3),
                ("stop3","AutonomousMode1", self.check_stop3_To_AutonomousMode1, self.DoAutonomousMode1),])
    #############################################################################
    # Bumper
    #############################################################################   
    

    #############################################################################
    # callback for joystick feedback
    #############################################################################
    def callback(self,data):
            #rospy.loginfo(rospy.get_name() + ": j'ai recu %f,%f", data.axes[1],data.axes[2])
        self.twist.linear.x = self.vmax * data.axes[1]
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = self.wmax*data.axes[2]
        #rospy.loginfo(rospy.get_name() + ": I publish linear=(%f,%f,%f), angular=(%f,%f,                            %f)",self.twist.linear.x,self.twist.linear.y,self.twist.linear.z,self.twist.angular.x,self.twist.angular.y,self.twist.angular.z)
    
        # for transition conditions of fsm
        if (not self.button_pressed):
            self.button_pressed = (self.previous_signal==0 and data.buttons[0]==1)       
        self.previous_signal = data.buttons[0];

        self.joy_activated = (abs(data.axes[1])>0.001 or abs(data.axes[2])>0.001)


    #############################################################################
    # smoothing velocity function to avoid brutal change of velocity
    #############################################################################
    def smooth_velocity(self):
        accmax = 0.01
        accwmax = 0.05
        vjoy = 0.0
        wjoy = 0.0
        vold = 0.0
        wold = 0.0   

        #filter twist
        vjoy = self.twist.linear.x
        vold = self.vreal
        deltav_max = accmax / self.T
    
        #vreal
        if abs(vjoy - self.vreal) < deltav_max:
            self.vreal = vjoy
        else:
            sign_ = 1.0
            if (vjoy < self.vreal):
                sign_ = -1.0
            else:
                sign_ = 1.0
            self.vreal = vold + sign_ * deltav_max
    
        #saturation
        if (self.vreal > self.vmax):
            self.vreal = self.vmax
        elif (self.vreal < -self.vmax):
            self.vreal = -self.vmax       
    
        #filter twist
        wjoy = self.twist.angular.z
        wold = self.wreal
        deltaw_max = accwmax / self.T
    
        #wreal
        if abs(wjoy - self.wreal) < deltaw_max:
            self.wreal = wjoy
        else:
            sign_ = 1.0
            if (wjoy < self.wreal):
                sign_ = -1.0
            else:
                sign_ = 1.0
            self.wreal = wold + sign_ * deltaw_max
        #saturation
        if (self.wreal > self.wmax):
            self.wreal = self.wmax
        elif (self.wreal < -self.wmax):
            self.wreal = -self.wmax       
           
        self.twist_real.linear.x = self.vreal   
        self.twist_real.angular.z = self.wreal
    
    #############################################################################
    # functions for fsm transitions
    #############################################################################
    def check_JoyControl_To_AutonomousMode1(self,fss):
        return self.button_pressed

    def check_AutonomousMode1_To_JoyControl(self,fss):
        #return self.button_pressed
        return self.joy_activated

    def check_AutonomousMode1_To_stop1(self,fss):
        return self.bumper_sensor.collision #or self.lidar_sensor.obstacle_detection
    
    def check_AutonomousMode1_To_recule(self,fss):
        return self.bumper_sensor.collision == False #and self.lidar_sensor.obstacle_detection

    def check_stop1_To_recule(self,fss):
        return self.cpt == 1

    def check_AutonomousMode1_To_rotate(self, fss):
        return self.lidar_sensor.obstacle_detection and self.bumper_sensor.collision == False

    def check_stop2_To_rotate(self,fss):
        return self.cpt == 2

    def check_stop3_To_AutonomousMode1(self,fss):
        return self.cpt == 3

    def check_recule_To_stop2(self,fss):
        return self.enough == True

    def check_rotate_To_stop3(self,fss):
        return self.enough == True

    def KeepJoyControl(self,fss):
        return (not self.check_JoyControl_To_AutonomousMode1(fss))

    def KeepAutonomousMode1(self,fss):
        return (not self.check_AutonomousMode1_To_JoyControl(fss) and not self.check_AutonomousMode1_To_stop1(fss)) and self.lidar_sensor.obstacle_detection == False

    def Keepstop1(self,fss):
        return not self.check_stop1_To_recule(fss)

    def Keeprecule(self,fss):
        return not self.check_recule_To_stop2(fss)

    def Keepstop2(self,fss):
        return not self.check_stop2_To_rotate(fss)

    def Keeprotate(self,fss):
        return not self.check_rotate_To_stop3(fss)

    def Keepstop3(self,fss):
        return (not self.check_stop3_To_AutonomousMode1(fss))





    #############################################################################
    # functions for instructions inside states of fsm
    #############################################################################
    def DoJoyControl(self,fss,value):
        self.cpt=0
        self.enough = False
        self.button_pressed =  False;
        self.smooth_velocity()
        self.pub.publish(self.twist_real)
        #print ('joy control : ',self.twist_real)
        pass

    def DoAutonomousMode1(self,fss,value):
        self.cpt=0
        self.enough = False
        self.button_pressed =  False;
        # go forward
        go_fwd = Twist()
        go_fwd.linear.x = self.vmax/2.0
        self.pub.publish(go_fwd)
        #print('autonomous : ',go_fwd)
        pass

    def Dostop1(self,fss,value):
        self.cpt=0
        #self.lidar_sensor.obstacle_detection = False
        self.button_pressed =  False;
        # do counter
        if self.start_timer == True:
            self.start_timer = False
            self.start_time= perf_counter()
        go_stop = Twist()
        go_stop.linear.x = 0
        go_stop.angular.z = 0
        self.pub.publish(go_stop)
        if self.start_timer == False and (perf_counter() - self.start_time) >= self.time_stop:
            self.start_timer= True
            self.cpt = 1

    def Dostop2(self,fss,value):
        self.cpt=0
        self.enough = False
        self.lidar_sensor.obstacle_detection = False
        self.button_pressed =  False;
        # do counter
        if self.start_timer == True:
            self.start_timer = False
            self.start_time= perf_counter()
        go_stop = Twist()
        go_stop.linear.x = 0
        go_stop.angular.z = 0
        self.pub.publish(go_stop)
        if self.start_timer == False and (perf_counter() - self.start_time) >= self.time_stop:
            self.start_timer= True
            self.cpt = 2

    def Dostop3(self,fss,value):
        self.cpt=0
        self.enough = False
        self.lidar_sensor.obstacle_detection = False
        self.button_pressed =  False;
        # do counter
        if self.start_timer == True:
            self.start_timer = False
            self.start_time= perf_counter()
        go_stop = Twist()
        go_stop.linear.x = 0
        go_stop.angular.z = 0
        self.pub.publish(go_stop)
        if self.start_timer == False and (perf_counter() - self.start_time) >= self.time_stop:
            self.start_timer= True
            self.cpt = 3

    def Dorecule(self,fss,value):
        self.cpt=0
        self.enough = False
        self.lidar_sensor.obstacle_detection = False
        self.button_pressed =  False;
        # do counter
        if self.start_timer == True:
            self.start_timer = False
            self.start_time= perf_counter()
        go_reculer = Twist()
        go_reculer.linear.x = -self.vmax/7.0
        self.pub.publish(go_reculer)
        if self.start_timer == False and (perf_counter() - self.start_time) >= self.time_recul:
            self.start_timer= True
            self.enough = True

    def Dorotate(self,fss,value):
        self.cpt=0
        self.enough = False
        self.lidar_sensor.obstacle_detection = False
        self.button_pressed =  False;

        
        if self.odo_target_angle == None:
            self.odo_target_angle = self.normalize_angle(self.angle_lacet + self.odo_rotate_angle)
        go_rotate = Twist()
        go_rotate.angular.z = self.vmax/2
        self.pub.publish(go_rotate)
        if self.angle_lacet >= self.odo_target_angle - 0.1 and self.angle_lacet <= self.odo_target_angle + 0.1:
            self.start_timer= True
            self.enough = True
            self.odo_target_angle = None
        pass

#############################################################################
# main function
#############################################################################
def Rad2Deg(angle):
    return angle * 180 / math.pi

def GetRotationNeeded(angle1, angle2):
    return angle1 - angle2

def NormalizeAngle(angle):
    if (abs(angle)):
        angle%=360
    elif angle < 0:
        angle += 360
    return angle

if __name__ == '__main__':
    try:
        rospy.init_node('joy4ctrl')
        # real turtlebot2
        pub = rospy.Publisher('mobile_base/commands/velocity', Twist,queue_size=1)
        # real turtlebot3
        #pub = rospy.Publisher('cmd_vel', Twist)
        # turtlesim   
        #pub = rospy.Publisher('turtle1/cmd_vel', Twist)
        Hz = 50
        rate = rospy.Rate(Hz)
        T = 1.0/Hz

        MyRobot = RobotBehavior(pub,T)
        rospy.Subscriber("joy", Joy, MyRobot.callback,queue_size=1)
        #lidar
        rospy.Subscriber("scan", LaserScan, MyRobot.processScan)
        #Odometrie
        rospy.Subscriber("odom", Odometry, MyRobot.OdoCallback,queue_size = 1)  
        MyRobot.fs.start("Start")
        rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,MyRobot.processBump,queue_size=1)
        # loop at rate Hz
        while (not rospy.is_shutdown()):
            ret = MyRobot.fs.event("")
            rate.sleep()

    except rospy.ROSInterruptException:
            pass