#!/usr/bin/env python


"""
pedestrian_gazebo.py

ROS node for obtaining pedestrian pose data from /gazebo/model_states and 
publishing to /spencer_tracking_msgs

"""

#####################################################################################
# Import packages
#####################################################################################

import sys
from gazebo_msgs import msg
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistWithCovariance, PoseWithCovariance
from tf.transformations import euler_from_quaternion
import time
from spencer_tracking_msgs.msg import TrackedPerson, TrackedPersons


class PedestrianGazebo():

    def __init__(self):

        # initialize ROS node
        rospy.init_node('group_tracking_helper_fcn')

        # initialize variables
        self.current_time = time.time()
        self.previous_time = time.time()
        model_states = rospy.wait_for_message('gazebo/model_states', ModelStates)
        self.actor_index = self.getActorIndices(model_states)
        self.prev_actor_poses = self.getActorPoses(model_states)

        # initialize ROS message services
        # self.actor_subscriber = rospy.Subscriber('/gazebo/model_states', 
        #     ModelStates, self.actorCallback)

        self.track_publisher = rospy.Publisher('/spencer/perception/tracked_persons', 
            TrackedPersons, queue_size=3)

        rospy.loginfo("Publishing tracked groups on /spencer/perception/tracked_persons")


    def getActorPoses(self, msg_data):
        # create empty pose list
        actor_poses = []
        # Loop to obtain actor model variables from data
        for idx in self.actor_index:
            actor_poses.append(msg_data.pose[idx].position)

        return actor_poses
                

    def getActorIndices(self, msg_data):
        # create empty index list
        actor_index = []
        # Loop to obtain actor model variables from data
        for idx in range(len(msg_data.name)):
            # check if current indexed model is an actor
            if msg_data.name[idx][0:5] == 'actor':
                actor_index.append(idx)
        
        return actor_index


    def getTrackPersons(self):

        # get one instance of message
        msg_data = None
        while msg_data is None:
            try:
                msg_data = rospy.wait_for_message('/gazebo/model_states', 
                                ModelStates, timeout=1)
            except:
                pass
        
        # set the track_id & trackedPersonsList
        track_id = 0
        trackedPersonsList = []

        # Loop to obtain actor model variables from data
        for idx in self.actor_index:

            # create trackedPerson and set variables
            trackedPerson = TrackedPerson()
            trackedPerson.track_id = track_id
            trackedPerson.is_matched = True
            trackedPerson.is_occluded = False
            trackedPerson.detection_id = 0
            trackedPerson.age = rospy.Duration(0)

            # set the pose information
            covariance = Float64MultiArray()
            covariance.data = [0] * 36
            trackedPerson.pose.pose = msg_data.pose[idx]
            trackedPerson.pose.covariance = covariance.data

            # set the velocity information
                ## compute dt
            self.current_time = time.time()
            dt = self.current_time - self.previous_time

                ## compute velocities
            actor_vel = TwistWithCovariance()
            actor_vel.covariance = covariance.data
            actor_vel.twist.angular.x = 0.0
            actor_vel.twist.angular.y = 0.0
            actor_vel.twist.angular.z = 0.0
            actor_vel.twist.linear.x = \
                (msg_data.pose[idx].position.x - self.prev_actor_poses[track_id].x)/dt
            actor_vel.twist.linear.y = \
                (msg_data.pose[idx].position.y - self.prev_actor_poses[track_id].y)/dt
            actor_vel.twist.linear.z = 0.0

            ## set twist to trackedPerson
            trackedPerson.twist = actor_vel

            # append trackedPerson to list
            trackedPersonsList.append(trackedPerson)

            # increment track_id
            track_id += 1

        # set previous values for next function call
        self.prev_actor_poses = self.getActorPoses(msg_data)
        self.previous_time = self.current_time
        
        return trackedPersonsList


    def publish(self):
        # call trackPersons
        tracks = self.getTrackPersons()

        # set trackedPersons
        trackedPersons = TrackedPersons()
        trackedPersons.header.seq = 1
        trackedPersons.header.stamp = rospy.get_rostime()
        trackedPersons.header.frame_id = 'world'
        trackedPersons.tracks = tracks

        # publish trackedPersons
        self.track_publisher.publish(trackedPersons)


#####################################################################################
# main
#####################################################################################

if __name__ == "__main__":
    try:
        
        #
        ped_gazebo = PedestrianGazebo()

        # rospy.spin()
        while not rospy.is_shutdown():
            ped_gazebo.publish()
            rospy.sleep(0.01)

    finally:
        pass