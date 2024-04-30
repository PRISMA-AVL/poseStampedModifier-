#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

class PoseRepublisher:
    def __init__(self):
        rospy.init_node('pose_republisher_node', anonymous=True)
        
        # Read the target frame_id from the parameter server
        self.target_frame_id = rospy.get_param('~target_frame_id', 'modified_frame')

        # Subscriber to the original PoseStamped topic
        self.subscriber = rospy.Subscriber('/original_pose', PoseStamped, self.pose_callback)

        # Publisher to republish modified PoseStamped messages
        self.publisher = rospy.Publisher('/modified_pose', PoseStamped, queue_size=10)

    def pose_callback(self, data):
        # Modify the received pose here (for example, swap x, y, and z values)
        modified_pose = self.modify_pose(data.pose)

        # Publish the modified pose
        self.publisher.publish(modified_pose)

    def modify_pose(self, pose):
        # Create a new PoseStamped message with modified values
        modified_pose = PoseStamped()
        modified_pose.header = pose.header
        modified_pose.header.frame_id = self.target_frame_id
        modified_pose.pose.position.x = pose.pose.position.x  # Swapping x and y
        modified_pose.pose.position.y = pose.pose.position.y  # Swapping y and z
        modified_pose.pose.position.z = pose.pose.position.z  # Swapping z and x
        modified_pose.pose.orientation = pose.pose.orientation
        return modified_pose

if __name__ == '__main__':
    try:
        pose_republisher = PoseRepublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

