;; Auto-generated. Do not edit!


(when (boundp 'ros_openpose::BodyPart)
  (if (not (find-package "ROS_OPENPOSE"))
    (make-package "ROS_OPENPOSE"))
  (shadow 'BodyPart (find-package "ROS_OPENPOSE")))
(unless (find-package "ROS_OPENPOSE::BODYPART")
  (make-package "ROS_OPENPOSE::BODYPART"))

(in-package "ROS")
;;//! \htmlinclude BodyPart.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ros_openpose::BodyPart
  :super ros::object
  :slots (_score _pixel _point ))

(defmethod ros_openpose::BodyPart
  (:init
   (&key
    ((:score __score) 0.0)
    ((:pixel __pixel) (instance ros_openpose::Pixel :init))
    ((:point __point) (instance geometry_msgs::Point32 :init))
    )
   (send-super :init)
   (setq _score (float __score))
   (setq _pixel __pixel)
   (setq _point __point)
   self)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:pixel
   (&rest __pixel)
   (if (keywordp (car __pixel))
       (send* _pixel __pixel)
     (progn
       (if __pixel (setq _pixel (car __pixel)))
       _pixel)))
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; float32 _score
    4
    ;; ros_openpose/Pixel _pixel
    (send _pixel :serialization-length)
    ;; geometry_msgs/Point32 _point
    (send _point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _score
       (sys::poke _score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; ros_openpose/Pixel _pixel
       (send _pixel :serialize s)
     ;; geometry_msgs/Point32 _point
       (send _point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _score
     (setq _score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; ros_openpose/Pixel _pixel
     (send _pixel :deserialize buf ptr-) (incf ptr- (send _pixel :serialization-length))
   ;; geometry_msgs/Point32 _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;;
   self)
  )

(setf (get ros_openpose::BodyPart :md5sum-) "243edb92e04d1e621047e9d565b8e64f")
(setf (get ros_openpose::BodyPart :datatype-) "ros_openpose/BodyPart")
(setf (get ros_openpose::BodyPart :definition-)
      "# The location and score of body parts are stored in a float array.
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



(provide :ros_openpose/BodyPart "243edb92e04d1e621047e9d565b8e64f")


