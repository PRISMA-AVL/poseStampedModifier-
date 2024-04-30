#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

class PoseRepublisher:
    def __init__(self):
        rospy.init_node('pose_republisher_node', anonymous=True)
        
        # Subscriber to the original PoseStamped topic
        self.subscriber = rospy.Subscriber('/original_pose', PoseStamped, self.pose_callback)

        # Publisher to republish modified PoseStamped messages
        self.publisher = rospy.Publisher('/modified_pose', PoseStamped, queue_size=10)

    def pose_callback(self, data):
        # Modify the received pose here (for example, increment position x by 1)
        modified_pose = self.swap_xyz(data.pose)

        # Publish the modified pose
        self.publisher.publish(modified_pose)

    def swap_xyz(self, pose):
        # Swap x, y, and z values of the pose
        modified_pose = PoseStamped()
        modified_pose.header = pose.header
        modified_pose.pose.position.x = pose.pose.position.y
        modified_pose.pose.position.y = pose.pose.position.z
        modified_pose.pose.position.z = pose.pose.position.x
        modified_pose.pose.orientation = pose.pose.orientation
        return modified_pose

if __name__ == '__main__':
    try:
        pose_republisher = PoseRepublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

