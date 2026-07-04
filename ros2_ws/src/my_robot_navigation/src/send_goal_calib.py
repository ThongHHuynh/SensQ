#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import math

def create_pose_stamped(navigator : BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x 
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose
    
def main():
    #Initialize
    rclpy.init()
    nav = BasicNavigator()

    # -- set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    # --INITIAL POSE SETTING SHOULD BE DISABLED IN CONFIG FILE
    nav.setInitialPose(initial_pose)

    # ---wait for nav2 
    nav.waitUntilNav2Active()

    # --Create goal pose
    goal_pose = create_pose_stamped(nav,1.0,0.0,math.radians(90.0))

    #-- Sending goal pose
    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        #print(feedback)



    
    print(nav.getResult())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
