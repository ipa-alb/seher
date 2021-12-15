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
    
    for person in data.persons:
        MP = Pixel()
        MP.x = float("nan")
        MP.y = float("nan")
        if isValid(person.rightHandParts[0]) and isValid(person.rightHandParts[9]):
            
            d_xl = abs(person.rightHandParts[9].pixel.x - person.rightHandParts[0].pixel.x) 
            MP_d_xl = d_xl/2

            if (person.rightHandParts[9].pixel.x >= person.rightHandParts[0].pixel.x):
                MP.x = person.rightHandParts[0].pixel.x + MP_d_xl

            else:
                MP.x = person.rightHandParts[9].pixel.x + MP_d_xl

            
            d_yl = abs(person.rightHandParts[9].pixel.y - person.rightHandParts[0].pixel.y) 
            MP_d_yl = d_yl/2

            if (person.rightHandParts[9].pixel.y >= person.rightHandParts[0].pixel.y):
                MP.y = person.rightHandParts[0].pixel.y + MP_d_yl

            else:
                MP.y = person.rightHandParts[9].pixel.y + MP_d_yl
            
            # der erste Eintrag is der Mittelpunkt von linker und der zweite Eintrag der Mittelpunkt von rechter Hand






        pub.publish(MP)
        

        text = "\n x0: " + str(person.rightHandParts[0].pixel.x) + ";\n x9: " + str(person.rightHandParts[9].pixel.x) + ";\n y0: " + str(person.rightHandParts[0].pixel.y) + ";\n y9: " + str(person.rightHandParts[9].pixel.y) + ";\n MP_x: " + str(MP.x) + ";\n MP_y: " + str(MP.y)
        rospy.loginfo('%s\n' % text)


    return


def listener():
    rospy.init_node('right_position_republisher')
    pub = rospy.Publisher("right_republisher", Pixel, queue_size=1)
    rospy.Subscriber("frame", Frame, callback, callback_args=(pub))
    rospy.spin()

if __name__ == '__main__':
    listener()



