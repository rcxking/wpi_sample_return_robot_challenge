
(cl:in-package :asdf)

(defsystem "fovis_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FovisInfo" :depends-on ("_package_FovisInfo"))
    (:file "_package_FovisInfo" :depends-on ("_package"))
  ))