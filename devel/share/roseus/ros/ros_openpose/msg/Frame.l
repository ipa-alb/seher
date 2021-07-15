;; Auto-generated. Do not edit!


(when (boundp 'ros_openpose::Frame)
  (if (not (find-package "ROS_OPENPOSE"))
    (make-package "ROS_OPENPOSE"))
  (shadow 'Frame (find-package "ROS_OPENPOSE")))
(unless (find-package "ROS_OPENPOSE::FRAME")
  (make-package "ROS_OPENPOSE::FRAME"))

(in-package "ROS")
;;//! \htmlinclude Frame.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ros_openpose::Frame
  :super ros::object
  :slots (_header _persons ))

(defmethod ros_openpose::Frame
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:persons __persons) (let (r) (dotimes (i 0) (push (instance ros_openpose::Person :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _persons __persons)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:persons
   (&rest __persons)
   (if (keywordp (car __persons))
       (send* _persons __persons)
     (progn
       (if __persons (setq _persons (car __persons)))
       _persons)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ros_openpose/Person[] _persons
    (apply #'+ (send-all _persons :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ros_openpose/Person[] _persons
     (write-long (length _persons) s)
     (dolist (elem _persons)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ros_openpose/Person[] _persons
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _persons (let (r) (dotimes (i n) (push (instance ros_openpose::Person :init) r)) r))
     (dolist (elem- _persons)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ros_openpose::Frame :md5sum-) "ca39f366a48028a3d7704368e0e8827d")
(setf (get ros_openpose::Frame :datatype-) "ros_openpose/Frame")
(setf (get ros_openpose::Frame :definition-)
      "# The standard ROS message contains a header.
# There can be multiple people in a frame.
# Hence we created an array of a person.
# We should name this array as people.
# However, for me, while coding persons
# make much more sense as it resembles
# the data structure array.
Header header
Person[] persons

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: ros_openpose/Person
# A person has some body parts. That is why we have created
# an array of body parts.
BodyPart[] bodyParts
BodyPart[] leftHandParts
BodyPart[] rightHandParts

================================================================================
MSG: ros_openpose/BodyPart
# The location and score of body parts are stored in a float array.
# Therefore we are using 32 bits instead of 64 bits.
# src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/include/openpose/core/datum.hpp
float32 score
Pixel pixel
geometry_msgs/Point32 point

================================================================================
MSG: ros_openpose/Pixel
# The location and score of body parts are stored in a float array.
# Therefore we are using 32 bits instead of 64 bits.
# src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/include/openpose/core/datum.hpp
# The location has been resized to the desired output
# resolution (e.g., `resolution` flag in the demo).
float32 x
float32 y

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
")



(provide :ros_openpose/Frame "ca39f366a48028a3d7704368e0e8827d")


