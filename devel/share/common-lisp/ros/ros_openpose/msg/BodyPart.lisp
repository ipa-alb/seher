; Auto-generated. Do not edit!


(cl:in-package ros_openpose-msg)


;//! \htmlinclude BodyPart.msg.html

(cl:defclass <BodyPart> (roslisp-msg-protocol:ros-message)
  ((score
    :reader score
    :initarg :score
    :type cl:float
    :initform 0.0)
   (pixel
    :reader pixel
    :initarg :pixel
    :type ros_openpose-msg:Pixel
    :initform (cl:make-instance 'ros_openpose-msg:Pixel))
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass BodyPart (<BodyPart>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyPart>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyPart)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_openpose-msg:<BodyPart> is deprecated: use ros_openpose-msg:BodyPart instead.")))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <BodyPart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_openpose-msg:score-val is deprecated.  Use ros_openpose-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'pixel-val :lambda-list '(m))
(cl:defmethod pixel-val ((m <BodyPart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_openpose-msg:pixel-val is deprecated.  Use ros_openpose-msg:pixel instead.")
  (pixel m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <BodyPart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_openpose-msg:point-val is deprecated.  Use ros_openpose-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyPart>) ostream)
  "Serializes a message object of type '<BodyPart>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pixel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyPart>) istream)
  "Deserializes a message object of type '<BodyPart>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'score) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pixel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyPart>)))
  "Returns string type for a message object of type '<BodyPart>"
  "ros_openpose/BodyPart")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyPart)))
  "Returns string type for a message object of type 'BodyPart"
  "ros_openpose/BodyPart")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyPart>)))
  "Returns md5sum for a message object of type '<BodyPart>"
  "243edb92e04d1e621047e9d565b8e64f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyPart)))
  "Returns md5sum for a message object of type 'BodyPart"
  "243edb92e04d1e621047e9d565b8e64f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyPart>)))
  "Returns full string definition for message of type '<BodyPart>"
  (cl:format cl:nil "# The location and score of body parts are stored in a float array.~%# Therefore we are using 32 bits instead of 64 bits.~%# src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/include/openpose/core/datum.hpp~%float32 score~%Pixel pixel~%geometry_msgs/Point32 point~%~%================================================================================~%MSG: ros_openpose/Pixel~%# The location and score of body parts are stored in a float array.~%# Therefore we are using 32 bits instead of 64 bits.~%# src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/include/openpose/core/datum.hpp~%# The location has been resized to the desired output~%# resolution (e.g., `resolution` flag in the demo).~%float32 x~%float32 y~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyPart)))
  "Returns full string definition for message of type 'BodyPart"
  (cl:format cl:nil "# The location and score of body parts are stored in a float array.~%# Therefore we are using 32 bits instead of 64 bits.~%# src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/include/openpose/core/datum.hpp~%float32 score~%Pixel pixel~%geometry_msgs/Point32 point~%~%================================================================================~%MSG: ros_openpose/Pixel~%# The location and score of body parts are stored in a float array.~%# Therefore we are using 32 bits instead of 64 bits.~%# src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/include/openpose/core/datum.hpp~%# The location has been resized to the desired output~%# resolution (e.g., `resolution` flag in the demo).~%float32 x~%float32 y~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyPart>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pixel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyPart>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyPart
    (cl:cons ':score (score msg))
    (cl:cons ':pixel (pixel msg))
    (cl:cons ':point (point msg))
))
