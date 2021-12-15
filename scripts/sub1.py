#!/usr/bin/env python2
import rospy 
from std_msgs.msg import String

def callback(data, pub):
    pub.publish(data)
    return

def listener():
    rospy.init_node('caller_republisher')
    pub = rospy.Publisher("republisher", String, queue_size=1)
    rospy.Subscriber("chatter", String, callback, callback_args=(pub))
    rospy.spin()

if __name__ == '__main__':
    listener()


# #!/usr/bin/env python2
# import rospy
# from std_msgs.msg import String

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)  
# def listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("chatter", String, callback)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()