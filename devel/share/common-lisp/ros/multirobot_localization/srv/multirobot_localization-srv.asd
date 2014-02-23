
(cl:in-package :asdf)

(defsystem "multirobot_localization-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Occupancy2D" :depends-on ("_package_Occupancy2D"))
    (:file "_package_Occupancy2D" :depends-on ("_package"))
  ))