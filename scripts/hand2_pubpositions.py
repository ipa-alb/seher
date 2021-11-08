#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def callback(msg):
    pub = rospy.Publisher('Hand_Position', Float32, queue_size=1)

    # text = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]
    # rospy.loginfo('%s\n' % text)


if __name__ == '__main__':
    rospy.init_node('hand_pose')
    
    # read the parameter from ROS parameter server
    frame_topic = rospy.get_param('~pub_topic')
    rospy.Subscriber(frame_topic, Frame, callback)

    rospy.spin()
