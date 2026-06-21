#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

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
    nav.setInitialPose(initial_pose)

    # ---wait for nav2 
    nav.waitUntilNav2Active()

    # --Create goal pose
    goal_pose = create_pose_stamped(nav,2.5,4.5,0.0)
    goal_pose1 = create_pose_stamped(nav,-3.0,4.5,0.0)
    goal_pose2 = create_pose_stamped(nav,-3.0,-5.0,0.0)

    #-- Sending goal pose
    # nav.goToPose(goal_pose)

    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     #print(feedback)

    # -- Follow waypoints
    waypoints = [goal_pose, goal_pose1, goal_pose2]
    for i in range(3):

        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            #print(feedback)


    
    print(nav.getResult())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
