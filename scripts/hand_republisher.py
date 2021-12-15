#!/usr/bin/env python2
from ros_openpose import msg
import rospy 
from ros_openpose.msg import Frame
from ros_openpose.msg import Pixel
from std_msgs.msg import Float32
from std_msgs.msg import String
import math

def isValid(bodyPart):
    '''
    When should we consider a body part as a valid entity?
    We make sure that the score and z coordinate is a positive number.
     Notice that the z coordinate denotes the distance of the object located
     in front of the camera. Therefore it must be a positive number always.
     '''
    return bodyPart.score > 0 and not math.isnan(bodyPart.point.x) and not math.isnan(bodyPart.point.y) and not math.isnan(bodyPart.point.z) and bodyPart.point.z > 0
        
        

def callback(data, pub):
    
    


    pub.publish(data)
    text = str(data)
    rospy.loginfo('%s\n' % text)
    
    return




def listener():
    rospy.init_node('position_republisher')
    pub = rospy.Publisher("republisher", Frame, queue_size=1)
    rospy.Subscriber("frame", Frame, callback, callback_args=(pub))
    rospy.spin()

if __name__ == '__main__':
    listener()



            # if isValid(handPart):
            #     x_pos.append(handPart.pixel.x)
            #     y_pos.append(handPart.pixel.y)

            # x_max = max(x_pos)
            # y_max = max(y_pos)
            # x_min = min(x_pos)
            # y_min = min(y_pos)

            # x_MP = x_max - x_min
            # y_MP = y_max - y_min
    # for person in data.persons:

        # if isValid(person.leftHandParts[0]):
        #     position0 = 1
        #     position0x =person.leftHandParts[0].point.x
        #     position0y =person.leftHandParts[0].point.y
        #     position0z =person.leftHandParts[0].point.z
        # else:
        #     position0 =float("nan")
           


            
        # if isValid(person.leftHandParts[9]):
        #     position9 = 1
        #     position9x =person.leftHandParts[9].point.x
        #     position9y =person.leftHandParts[9].point.y
        #     position9z =person.leftHandParts[9].point.z
            
        # else:    
        #     position9 =float("nan")
        
        # if (math.isnan(position0) or math.isnan(position9)):
        #     MP = float("nan")
        # else:
        #     distance = math.sqrt((position9x-position0x)**2+(position9y-position0y)**2+(position9z-position0z)**2) 
        #     MP = distance/2

        # pub.publish(MP)
        # rospy.loginfo('%s\n' % str(MP))


# #!/usr/bin/env python

# import rospy
# from std_msgs.msg import Float32

# def callback(msg):
#     pub = rospy.Publisher('Hand_Position', Float32, queue_size=1)

#     # text = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]
#     # rospy.loginfo('%s\n' % text)


# if __name__ == '__main__':
#     rospy.init_node('hand_pose')
    
#     # read the parameter from ROS parameter server
#     frame_topic = rospy.get_param('~pub_topic')
#     rospy.Subscriber(frame_topic, Frame, callback)

#     rospy.spin()
