#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
import time
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from ros_falcon.msg import falconForces




"""
haptic_interface.py
"""


class FalconNovintControl():

    def __init__(self):
        # variables
        self.base_orientation = 0.0
        self.yaw_angle = 0.0
        self.steer_angle = 0.0
        self.raw_pedal_data = 0
        self.base_linear_vel_fwd = 0.0
        self.base_linear_vel_bwd = 0.0
        self.base_angular_vel_z = 0.0
        
        self.raw_x_pos = 0.0
        self.raw_y_pos = 0.0
        self.raw_z_pos = 0.0

        self.z_max, self.z_min, self.z_mid = 0.075, 0.172, 0.125
        self.x_max, self.x_min, self.x_mid = 0.05, -0.05, 0.00
        self.z_pos_range = self.z_min - self.z_max
        self.x_pos_range = self.x_min - self.x_max

        self.x_buffer = 0.01
        self.z_buffer = 0.01

        self.kp = 0.02
        self.kf = 25 # default is 15
        self.cmd_vel = Twist()
        self.force = falconForces()
        self.pos_array = Float64MultiArray()
        self.pos_array.data = [] # [x, y, z]
        self.inactive = False
        self.button_pressed = 0
        self.centering_force = np.zeros(3)
        self.guidance_force = np.zeros(3)

        self.max_wheel_angle = 70 # degrees
        self.veh_length = 2.5 # 2.5
        self.steering_ratio = 1.2 
        self.max_linear_vel = 2.0
        self.max_angular_vel = 1.5

        # control mode (i.e. car-like control, pos-vel control, pos-vel-buffer control)
        # self.manual_mode = args[1]
        self.manual_mode = rospy.get_param("/manual_mode")

        # instantiate the node
        rospy.init_node('falcon_teleop_control')

        # print status
        rospy.loginfo("Falcon Control Interface Initiated...")

        # instantiate the services
            # odometry subscriber
        self.odom_subscriber = rospy.Subscriber('/base_controller/odom', 
            Odometry, self.getOrientation_callback)

            # publish to the mobile_base controller
        self.cmd_vel_pub = rospy.Publisher('/base_controller/cmd_vel', 
            Twist, queue_size=5)

            # subscribe to joystick inputs on topic "/falcon/joystick"
        self.falcon_joy_subscriber = rospy.Subscriber("/falcon/joystick", Joy, 
                                                    self.getFalconData_callback)
            
            # publish to falcon Force inputs on topic "/falconForce"
        self.force_pub = rospy.Publisher("/falconForce", falconForces, queue_size=5)

            # publish falcon position to topic
        self.pos_publisher = rospy.Publisher('/falcon_command_pos', 
                            Float64MultiArray, queue_size=5)

            # rosservice call to unpause simulation
        self.unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

            # rosservice call to unpause simulation
        self.pause_sim = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        # check if constant linear velocity is being set
        self.const_vel = [False, 0.0]
        # if rospy.get_param('const_velocity') != 0.0:
        #     self.const_vel[0] = True
        #     self.const_vel[1] = rospy.get_param('const_velocity')

        

    def getFalconData_callback(self, data):
        self.raw_x_pos = data.axes[0]
        self.raw_y_pos = data.axes[1]
        self.raw_z_pos = data.axes[2]
        self.button_pressed = data.buttons[0]

        # self.base_linear_vel_fwd = self.linear_velocity(self.raw_z_pos)
        # # self.steer_angle = self.transform_x_position(self.raw_x_pos)
        # self.base_angular_vel_z = self.angular_velocity(self.raw_x_pos)
        

    def getOrientation_callback(self, data):
        self.base_orientation = data.pose.pose.orientation
        orientation_quaternion = euler_from_quaternion([self.base_orientation.x,
                                            self.base_orientation.y, 
                                            self.base_orientation.z, 
                                            self.base_orientation.w])
        self.yaw_angle = np.degrees(orientation_quaternion[2])


    def compute_pos_vel_control(self):
        # compute linear x velocity
        if (self.raw_z_pos >= self.z_mid): 
            self.base_linear_vel_fwd = - (self.raw_z_pos - self.z_mid) * 2 * self.max_linear_vel/self.z_pos_range
                # negative sign is there because its backward motion
        else : 
            self.base_linear_vel_fwd = (self.z_mid - self.raw_z_pos) * 2 * self.max_linear_vel/self.z_pos_range

        # compute angular z velocity
        self.base_angular_vel_z = -self.raw_x_pos * self.max_angular_vel/self.x_max # since neutral point is zero
            # added -ve sign to address rotation direction

    
    def compute_pos_vel_buffer_control(self):
        # x_buffer = 0.01
        # # compute linear x velocity
        #     # z_max, z_min, z_mid = 0.073, 0.174, 0.125 
        # z_max, z_min, z_mid = 0.10, 0.172, 0.125 # modified z_max to accomodate lateral motion better
        # pos_range = z_min - z_max

        # self.base_linear_vel_fwd = (z_min - self.raw_z_pos) * self.max_linear_vel/pos_range

        # # compute angular z velocity
        #     # x_max, x_min, x_mid = 0.05, -0.05, 0.00
        # x_max, x_min, x_mid = (0.05 - x_buffer), (-0.05 + x_buffer), 0.00
        # pos_range = x_min - x_max

        # if abs(self.raw_x_pos) < 0.01:
        #      self.base_angular_vel_z = 0.0
        # else:
        #     self.base_angular_vel_z = -self.raw_x_pos * self.max_angular_vel/x_max # since neutral point is zero
        # # added -ve sign to address rotation direction

        # linear velocity
        if ( abs(self.raw_z_pos - self.z_mid) < 0.01): 
            self.base_linear_vel_fwd = 0.0
        else :
            if (self.raw_z_pos >= self.z_mid):
                self.base_linear_vel_fwd = - (self.raw_z_pos - self.z_mid) * 2 * self.max_linear_vel/(self.z_pos_range - self.z_buffer)
                    #  negative sign is there because its backward motion
            else :
                self.base_linear_vel_fwd = (self.z_mid - self.raw_z_pos) * 2 * self.max_linear_vel/(self.z_pos_range - self.z_buffer)

        # angular velocity
        if (self.raw_x_pos < 0.01) : 
            self.base_angular_vel_z = 0.0
        else :
            self.base_angular_vel_z = -self.raw_x_pos * self.max_angular_vel/(self.x_max - self.x_buffer)
                #  added -ve sign to address rotation direction
        
        
    def compute_car_like_control(self):
        # compute linear x velocity
            # z_max, z_min, z_mid = 0.073, 0.174, 0.125 
        z_max, z_min, z_mid = 0.10, 0.172, 0.125 # modified z_max to accomodate lateral motion better
        pos_range = z_min - z_max
        self.base_linear_vel_fwd = (z_min - self.raw_z_pos) * self.max_linear_vel/pos_range
        
        # compute angular z velocity
        x_max = 0.055
        max_yaw_angle = 120
        steer_angle = (self.raw_x_pos * max_yaw_angle) / x_max

        # transform steering angle to front wheel angle
        self.fw_angle = self.steering_ratio * steer_angle
        # bound the fw_angle
        if self.fw_angle > self.max_wheel_angle:
            self.fw_angle = self.max_wheel_angle
        if self.fw_angle < -self.max_wheel_angle:
            self.fw_angle = -self.max_wheel_angle

        # calculate angular velocity using car-like kinematics
        self.base_angular_vel_z = -1 * (self.base_linear_vel_fwd/self.veh_length) * \
                                    math.tan(math.radians(self.fw_angle))


    def command_velocity(self):

        #-------------------------------------------------------------------------------------
        # compute the velocity command values
        #-------------------------------------------------------------------------------------

        # add linear control:
            # check if constant velocity is set:
        # if self.const_vel[0] == True:
        #     self.cmd_vel.linear.x = self.const_vel[1]
        # else:
        #     self.cmd_vel.linear.x = self.base_linear_vel_fwd
        
        # select appropriate control mode
        if self.manual_mode == 'car-like':
            self.compute_car_like_control()
        elif self.manual_mode == 'pos-vel':
            self.compute_pos_vel_control()
        elif self.manual_mode == 'pos-vel-buffer':
            self.compute_pos_vel_buffer_control()
        
        # set to cmd_vel
        self.cmd_vel.linear.x = self.base_linear_vel_fwd
        self.cmd_vel.angular.z = self.base_angular_vel_z
        
        # set pos_array values
        self.pos_array.data = [self.raw_x_pos, self.raw_y_pos, self.raw_z_pos]

        # ros info
        # rospy.loginfo("Logging Data: Vx:[" + str(round(self.cmd_vel.linear.x, 2)) +
        #                         "], omega:[" + str(round(self.cmd_vel.angular.z, 2)) + "]")
        # rospy.loginfo("Engaged? : " + str(self.engaged) + " ")

        # manage engagement
        if self.button_pressed != 4:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            #--
            self.pos_array.data = [0.0, 0.0, 0.0]

        
        if self.button_pressed == 8:
            # initialize the simulation by unpausing and then pausing
            self.unpause_sim()
            rospy.loginfo_once("Simulation Initialized")
            self.pause_sim()

        # start up session
        if self.button_pressed == 2:
            # call the rosservice
            self.unpause_sim()
            rospy.set_param('/start_timer', True)

        # publish the cmd_vel
        self.cmd_vel_pub.publish(self.cmd_vel)

        # Publish array data
        self.pos_publisher.publish(self.pos_array)


    def command_force(self):

        #-------------------------------------------------------------------------------------
        # compute the force command values
        #-------------------------------------------------------------------------------------
        # centering_force, guidance_force = np.zeros(3), np.zeros(3)
        # compute centering force
        z_mid = 0.125
        self.centering_force[0] = self.kf * (-self.raw_x_pos)
        # centering_force[1] = self.kf * (-self.raw_y_pos)
        self.centering_force[2] = self.kf * -(z_mid - self.raw_z_pos)
        # compute guidance force
            ###
        # find resultant force
        self.force.X = self.centering_force[0] + self.guidance_force[0]
        self.force.Y = self.centering_force[1] + self.guidance_force[1]
        self.force.Z = self.centering_force[2] + self.guidance_force[2]
        # publish the force
        self.force_pub.publish(self.force)

        # rospy.loginfo("Logging Data: Fx:[" + str(round(self.force.X, 3)) + "], " +
        #                             "Fz:[" + str(round(self.force.Z, 3)) + "]")
        
        # rospy.loginfo("Logging Data: Fx:[" + str(round(self.force.X, 3)) + "], " + 
        #                             "Fy:[" + str(round(self.force.Y, 3)) + "], " +
        #                             "Fz:[" + str(round(self.force.Z, 3)) + "]")

        # rospy.loginfo("Speed Data: Vx:[" + str(self.cmd_vel.linear.x) + "], " + 
        #                           "Omz:[" + str(self.cmd_vel.angular.z) + "]")

        # rospy.loginfo("Logging Data: X:[" + str(round(self.raw_x_pos, 3)) + "], " + 
        #                             "Y:[" + str(round(self.raw_y_pos, 3)) + "], " +
        #                             "Z:[" + str(round(self.raw_z_pos, 3)) + "]")
        

if __name__ == "__main__":
    try:
        # get argument passed to node
        # args = rospy.myargv(argv=sys.argv)

        # instantiate haptic control object
        # base_control = FalconNovintControl(args)
        base_control = FalconNovintControl()

        # rospy.spin()
        while not rospy.is_shutdown():
            base_control.command_velocity()
            # base_control.command_force()
            # time.sleep(1)

    finally:
        pass