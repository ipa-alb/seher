
(cl:in-package :asdf)

(defsystem "ros_openpose-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BodyPart" :depends-on ("_package_BodyPart"))
    (:file "_package_BodyPart" :depends-on ("_package"))
    (:file "Frame" :depends-on ("_package_Frame"))
    (:file "_package_Frame" :depends-on ("_package"))
    (:file "Person" :depends-on ("_package_Person"))
    (:file "_package_Person" :depends-on ("_package"))
    (:file "Pixel" :depends-on ("_package_Pixel"))
    (:file "_package_Pixel" :depends-on ("_package"))
  ))