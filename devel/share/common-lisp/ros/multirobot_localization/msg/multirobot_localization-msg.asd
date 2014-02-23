
(cl:in-package :asdf)

(defsystem "multirobot_localization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "motor_ctr" :depends-on ("_package_motor_ctr"))
    (:file "_package_motor_ctr" :depends-on ("_package"))
  ))