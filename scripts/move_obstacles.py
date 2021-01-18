#!/usr/bin/env python


import rospy
import time
from gazebo_msgs.msg import ModelState, ModelStates

class MoveObstacle():
    def __init__(self):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.moving()

    def moving(self):
        state, state2 = 0, 0
        step = 0.02
        
        while not rospy.is_shutdown():
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            for i in range(len(model.name)):
                if model.name[i] == 'dynamic_obstacle_1':
                    obstacle_1 = ModelState()
                    obstacle_1.model_name = model.name[i]
                    obstacle_1.pose = model.pose[i]
                    if abs(obstacle_1.pose.position.x + 4.95) < 0.5 and abs(obstacle_1.pose.position.y + 2.0) < 0.5:
                        state = 0

                    if state == 0:
                        obstacle_1.pose.position.x += 0.25
                        obstacle_1.pose.position.y -= 0.00
                        if abs(obstacle_1.pose.position.x - 9.15) < 0.5 and abs(obstacle_1.pose.position.y + 2.0) < 0.5:
                            state = 1

                    self.pub_model.publish(obstacle_1)
                    time.sleep(step)

                if model.name[i] == 'dynamic_obstacle_2':
                    obstacle_2 = ModelState()
                    obstacle_2.model_name = model.name[i]
                    obstacle_2.pose = model.pose[i]
                    if abs(obstacle_2.pose.position.x + 5.3) < 0.1 and abs(obstacle_2.pose.position.y - 1.1) < 0.1:
                        state2 = 0

                    if state2 == 0:
                        obstacle_2.pose.position.x += 0.15
                        obstacle_2.pose.position.y -= 0.05
                        if abs(obstacle_2.pose.position.x - 8.45) < 0.1:
                            state2 = 1

                    self.pub_model.publish(obstacle_2)
                    time.sleep(step)
                
                if model.name[i] == 'dynamic_obstacle_3':
                    obstacle_3 = ModelState()
                    obstacle_3.model_name = model.name[i]
                    obstacle_3.pose = model.pose[i]
                    if abs(obstacle_3.pose.position.x - 1.78) < 0.5 and abs(obstacle_3.pose.position.y - 7.64) < 0.5:
                        state3 = 0

                    if state3 == 0:
                        obstacle_3.pose.position.x += 0.00
                        obstacle_3.pose.position.y -= 0.25
                        if abs(obstacle_3.pose.position.y + 9.85) < 0.1:
                            state3 = 1

                    self.pub_model.publish(obstacle_3)
                    time.sleep(step)

                if model.name[i] == 'dynamic_obstacle_4':
                    obstacle_4 = ModelState()
                    obstacle_4.model_name = model.name[i]
                    obstacle_4.pose = model.pose[i]
                    if abs(obstacle_4.pose.position.x - 8.32) < 0.5 and abs(obstacle_4.pose.position.y - 6.81) < 0.5:
                        state4 = 0

                    if state4 == 0:
                        obstacle_4.pose.position.x -= 0.10
                        obstacle_4.pose.position.y -= 0.20
                        if obstacle_4.pose.position.y < -8.80:
                            state4 = 1

                    self.pub_model.publish(obstacle_4)
                    time.sleep(step)


def main():
    rospy.init_node('moving_obstacles_node')
    try:
        MoveObstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()