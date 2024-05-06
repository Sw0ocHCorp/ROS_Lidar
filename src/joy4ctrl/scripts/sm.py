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
import pandas as pd

class GyroSensor:
    def __init__(self, default_rotation_angle= 180) -> None:
        self.default_rotation_angle = default_rotation_angle
        self.main_angle = 0.0
        self.rotation_target= default_rotation_angle
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
        self.angle_target = NormalizeAngle(self.main_angle + self.rotation_target)
    
    def Reset(self):
        self.rotation_target= self.default_rotation_angle
        self.angle_target= 0
    
class LidarSensor:
    def __init__(self, gyro_sensor: GyroSensor, dist_detection= 2) -> None:
        self.obstacle_detection= False
        self.dist_detection= dist_detection
        self.interest_points= {"OOB": []}
        self.is_extract_points= None
        self.farest_obstacle= 0
        self.gyro_sensor= gyro_sensor
        self.avoid_obstacle= False
        self.avoid_dist = 0
        self.avoid_offset= 1.75


    def processScan(self, data):        
        self.scan_range = Rad2Deg(data.angle_max - data.angle_min)
        nb_values = len(data.ranges) # nombre de valeurs renvoyees par le lidar
        if self.is_extract_points:
            #Si full nan sur 10 valeurs en face du robot, on considère qu'on dépasse la range du lidar  donc point d'intérêt
            if np.isnan(np.array(data.ranges[nb_values//2 -35: nb_values//2 + 35])).sum() >= 50:
                self.interest_points["OOB"].append(self.gyro_sensor.main_angle)
            #Sinon si on a un objet plus loin que précédemment, en face du robot, on le considère comme un point d'intérêt
            elif (np.nanmax(data.ranges[nb_values//2 -5: nb_values//2 + 5]) > self.farest_obstacle):
                self.farest_obstacle = data.ranges[nb_values//2]
                self.interest_points[self.farest_obstacle] = self.gyro_sensor.main_angle
            
            """if np.nanmax(data.ranges) > self.farest_obstacle:
                self.farest_obstacle = np.nanmax(data.ranges)
                self.interest_points[self.farest_obstacle] = self.gyro_sensor.main_angle + ((self.scan_range * (data.ranges.index(self.farest_obstacle) / nb_values)) - (self.scan_range / 2))"""
        
        if np.nanmin(data.ranges) < (2*self.dist_detection):
            t= np.nanmax(data.ranges[nb_values//2 -20: nb_values//2 + 20])
            if np.nanmin(data.ranges) < (self.dist_detection):
                self.obstacle_detection = True
                
            elif np.nanmax(data.ranges[nb_values//2 -20: nb_values//2 + 20]) < (2*self.dist_detection) and self.avoid_obstacle == False:
                min_val= np.nanmax(data.ranges[nb_values//2 -20: nb_values//2 + 20])
                self.avoid_obstacle = True
                self.avoid_dist = np.nanmin(data.ranges)
                avoid_angle= Rad2Deg(math.atan(self.avoid_offset / self.avoid_dist))
                if (data.ranges.index(min_val) > nb_values//2):
                    self.gyro_sensor.RegisterTargetAngle(avoid_angle)
                else:
                    self.gyro_sensor.RegisterTargetAngle(-avoid_angle)
                

    def Reset(self, deep_reset= False):
        self.is_extract_points = False
        self.avoid_dist = 0
        if deep_reset:
            self.is_extract_points = None
        self.farest_obstacle = 0
        self.interest_points = {"OOB": []}
        self.obstacle_detection = False
        
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
        self.lidar_sensor = LidarSensor(self.gyro_sensor, dist_detection= 0.75)
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
        
        # instance of fsm with source state, destination state, condition (transition), callback (defaut: None)
        self.fs = fsm(states=[ ("Start","JoyControl", True ),
                ("JoyControl","AutonomousMode1", self.check_JoyControl_To_AutonomousMode1, self.DoAutonomousMode1),
                ("JoyControl","JoyControl", self.KeepJoyControl, self.DoJoyControl),
                ("AutonomousMode1","JoyControl", self.check_AutonomousMode1_To_JoyControl, self.DoJoyControl),
                ("AutonomousMode1","AutonomousMode1", self.KeepAutonomousMode1, self.DoAutonomousMode1),
                ("AutonomousMode1","rotate", self.check_AutonomousMode1_To_rotate, self.Dorotate),
                ("AutonomousMode1","stop1", self.check_AutonomousMode1_To_stop1, self.Dostop1),
                ("stop1","JoyControl", self.check_stop1_To_JoyControl, self.DoJoyControl),
                ("stop1","stop1", self.Keepstop1, self.Dostop1),
                ("stop1","recule", self.check_stop1_To_recule, self.Dorecule),
                ("recule","JoyControl", self.check_recule_To_JoyControl, self.DoJoyControl),
                ("recule","recule", self.Keeprecule, self.Dorecule),
                ("recule","stop2", self.check_recule_To_stop2, self.Dostop2),
                ("stop2","JoyControl", self.check_stop2_To_JoyControl, self.DoJoyControl),
                ("stop2","stop2", self.Keepstop2, self.Dostop2),
                ("stop2","rotate", self.check_stop2_To_rotate, self.Dorotate),
                ("rotate","JoyControl", self.check_rotate_To_JoyControl, self.DoJoyControl),
                ("rotate","rotate", self.Keeprotate, self.Dorotate),
                ("rotate","stop3", self.check_rotate_To_stop3, self.Dostop3),
                ("stop3","JoyControl", self.check_stop3_To_JoyControl, self.DoJoyControl),
                ("stop3","stop3", self.Keepstop3, self.Dostop3),
                ("stop3","AutonomousMode1", self.check_stop3_To_AutonomousMode1, self.DoAutonomousMode1),
                ("AutonomousMode1", "Virage", self.check_AutonomousMode1_To_Virage, self.DoVirage),
                ("Virage", "Virage", self.KeepVirage, self.DoVirage),
                ("Virage", "stop1", self.check_Virage_To_Stop1, self.Dostop1),
                ("Virage", "AutonomousMode1", self.check_Virage_To_Autonomous, self.DoAutonomousMode1)], robot=self)
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
        self.twist.angular.z = self.wmax*data.axes[0]
        #rospy.loginfo(rospy.get_name() + ": I publish linear=(%f,%f,%f), angular=(%f,%f,                            %f)",self.twist.linear.x,self.twist.linear.y,self.twist.linear.z,self.twist.angular.x,self.twist.angular.y,self.twist.angular.z)
    
        # for transition conditions of fsm
        if (not self.button_pressed):
            self.button_pressed = (self.previous_signal==0 and data.buttons[0]==1)       
        self.previous_signal = data.buttons[0];

        self.joy_activated = (abs(data.axes[1])>0.001 or abs(data.axes[0])>0.001)


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

    def check_AutonomousMode1_To_stop1(self,fss):
        return self.bumper_sensor.collision #or self.lidar_sensor.obstacle_detection
    
    def check_AutonomousMode1_To_recule(self,fss):
        return self.bumper_sensor.collision == False #and self.lidar_sensor.obstacle_detection

    def check_stop1_To_recule(self,fss):
        return self.cpt == 1

    def check_AutonomousMode1_To_rotate(self, fss):
        return  self.bumper_sensor.collision == False and self.lidar_sensor.obstacle_detection and self.lidar_sensor.avoid_obstacle == False

    def check_stop2_To_rotate(self,fss):
        return self.cpt == 2

    def check_stop3_To_AutonomousMode1(self,fss):
        return self.cpt == 3

    def check_recule_To_stop2(self,fss):
        return self.enough == True

    def check_rotate_To_stop3(self,fss):
        return self.enough == True
    
    def check_Virage_To_Autonomous(self,fss):
        return  self.enough == True
    
    def check_Virage_To_Stop1(self,fss):
        return self.bumper_sensor.collision == True
    
    def check_AutonomousMode1_To_Virage(self,fss):
        return self.lidar_sensor.avoid_obstacle == True
    
    #Le turtlebot doit pouvoir se controler au joystick peut importe l'état ou il est

    def check_AutonomousMode1_To_JoyControl(self,fss):
        return self.joy_activated or self.button_pressed
    
    def check_stop1_To_JoyControl(self,fss):
        return self.joy_activated or self.button_pressed
    
    def check_stop2_To_JoyControl(self,fss):
        return self.joy_activated or self.button_pressed
    
    def check_stop3_To_JoyControl(self,fss):
        return self.joy_activated or self.button_pressed
    
    def check_recule_To_JoyControl(self,fss):
        return self.joy_activated or self.button_pressed
    
    def check_rotate_To_JoyControl(self,fss):
        return self.joy_activated or self.button_pressed
    
    def check_Virage_To_JoyControl(self,fss):
        return self.joy_activated or self.button_pressed
    
    

    def KeepJoyControl(self,fss):
        return (not self.check_JoyControl_To_AutonomousMode1(fss))

    def KeepAutonomousMode1(self,fss):
        return (not self.check_AutonomousMode1_To_JoyControl(fss) and not self.check_AutonomousMode1_To_stop1(fss) and self.lidar_sensor.obstacle_detection == False  and not self.check_AutonomousMode1_To_Virage(fss))#and not self.check_AutonomousMode1_To_rotate(fss)

    def Keepstop1(self,fss):
        return not self.check_stop1_To_recule(fss) and not self.check_stop1_To_JoyControl(fss)

    def Keeprecule(self,fss):
        return not self.check_recule_To_stop2(fss) and not self.check_recule_To_JoyControl(fss)

    def Keepstop2(self,fss):
        return not self.check_stop2_To_rotate(fss) and not self.check_stop2_To_JoyControl(fss)

    def Keeprotate(self,fss):
        return not self.check_rotate_To_stop3(fss) and not self.check_rotate_To_JoyControl(fss)

    def Keepstop3(self,fss):
        return (not self.check_stop3_To_AutonomousMode1(fss) and not self.check_stop3_To_JoyControl(fss))
    
    def KeepVirage(self,fss):
        return (not self.check_Virage_To_Autonomous(fss) and not self.check_Virage_To_JoyControl(fss) and not self.check_Virage_To_Stop1(fss))





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
        self.lidar_sensor.Reset(True)
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
        self.button_pressed =  False
        #SCAN 1ere Etape: Scan de la Pièce pour récupérer les points les plus loins de cette scène
        if self.lidar_sensor.obstacle_detection and self.lidar_sensor.is_extract_points == None:
            self.lidar_sensor.is_extract_points = True
            self.gyro_sensor.RegisterTargetAngle(330)
        #COLLISION: Rotation de l'angle de rotation initial
        elif self.bumper_sensor.collision and self.lidar_sensor.obstacle_detection == False:
            self.gyro_sensor.RegisterTargetAngle(self.gyro_sensor.default_rotation_angle)
        go_rotate = Twist()
        go_rotate.angular.z = self.vmax
        if self.gyro_sensor.rotation_target < 0:
            go_rotate.angular.z *= -1

        self.pub.publish(go_rotate)
        if self.gyro_sensor.main_angle >= self.gyro_sensor.angle_target - 10 and self.gyro_sensor.main_angle <= self.gyro_sensor.angle_target + 10:
            #SCAN 2eme Etape: On à fini de Scanner la pièce, on va ensuite viser le point le plus loin de la scène
            if self.lidar_sensor.is_extract_points == True:
                #Récupération de la position du point le plus loin
                
                if len(self.lidar_sensor.interest_points["OOB"]) > 0:
                    farest_obstacle_angle = self.lidar_sensor.interest_points["OOB"][len(self.lidar_sensor.interest_points["OOB"]) // 2]
                else:
                    del self.lidar_sensor.interest_points["OOB"]
                    farest_obstacle= sorted(self.lidar_sensor.interest_points.keys(), reverse= True)[0]
                    farest_obstacle_angle= self.lidar_sensor.interest_points[farest_obstacle]
                target_rotation = farest_obstacle_angle - self.gyro_sensor.main_angle
                
                #RESET Capteurs
                self.gyro_sensor.Reset()
                self.lidar_sensor.Reset()
                self.gyro_sensor.RegisterTargetAngle(target_rotation)
            #SORTIE de l'Etat ROTATE
            elif self.lidar_sensor.is_extract_points == False or self.bumper_sensor.collision == False:
                self.start_timer= True
                self.enough = True
                self.gyro_sensor.Reset()
                self.lidar_sensor.Reset(True)
        pass

    def DoVirage(self,fss,value):
        go_avoid = Twist()
        go_avoid.angular.z = self.vmax/3.0
        if self.gyro_sensor.rotation_target < 0:
            go_avoid.angular.z *= -1
        go_avoid.linear.x = self.vmax/3.0
        self.pub.publish(go_avoid)
        if self.gyro_sensor.main_angle >= self.gyro_sensor.angle_target - 5 and self.gyro_sensor.main_angle <= self.gyro_sensor.angle_target + 5 :
            self.lidar_sensor.avoid_obstacle = False
            self.enough = True
            self.gyro_sensor.Reset()
            self.lidar_sensor.Reset()
        


#############################################################################
# main function
#############################################################################
def Rad2Deg(angle):
    return angle * 180 / math.pi

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
        rospy.Subscriber("scan", LaserScan, MyRobot.lidar_sensor.processScan)
        #Odometrie
        rospy.Subscriber("odom", Odometry, MyRobot.gyro_sensor.OdoCallback,queue_size = 1)  
        MyRobot.fs.start("Start")
        rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,MyRobot.bumper_sensor.processBump,queue_size=1)
        # loop at rate Hz
        while (not rospy.is_shutdown()):
            ret = MyRobot.fs.event("")
            rate.sleep()

    except rospy.ROSInterruptException:
            pass