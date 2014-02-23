; Auto-generated. Do not edit!


(cl:in-package multirobot_localization-msg)


;//! \htmlinclude motor_ctr.msg.html

(cl:defclass <motor_ctr> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_wheel
    :reader left_wheel
    :initarg :left_wheel
    :type cl:float
    :initform 0.0)
   (right_wheel
    :reader right_wheel
    :initarg :right_wheel
    :type cl:float
    :initform 0.0))
)

(cl:defclass motor_ctr (<motor_ctr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_ctr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_ctr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multirobot_localization-msg:<motor_ctr> is deprecated: use multirobot_localization-msg:motor_ctr instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <motor_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_localization-msg:header-val is deprecated.  Use multirobot_localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_wheel-val :lambda-list '(m))
(cl:defmethod left_wheel-val ((m <motor_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_localization-msg:left_wheel-val is deprecated.  Use multirobot_localization-msg:left_wheel instead.")
  (left_wheel m))

(cl:ensure-generic-function 'right_wheel-val :lambda-list '(m))
(cl:defmethod right_wheel-val ((m <motor_ctr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_localization-msg:right_wheel-val is deprecated.  Use multirobot_localization-msg:right_wheel instead.")
  (right_wheel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_ctr>) ostream)
  "Serializes a message object of type '<motor_ctr>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_wheel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_wheel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_ctr>) istream)
  "Deserializes a message object of type '<motor_ctr>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_wheel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_wheel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_ctr>)))
  "Returns string type for a message object of type '<motor_ctr>"
  "multirobot_localization/motor_ctr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_ctr)))
  "Returns string type for a message object of type 'motor_ctr"
  "multirobot_localization/motor_ctr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_ctr>)))
  "Returns md5sum for a message object of type '<motor_ctr>"
  "3b8f6c33f8aa9ab11781a2a0e65b64dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_ctr)))
  "Returns md5sum for a message object of type 'motor_ctr"
  "3b8f6c33f8aa9ab11781a2a0e65b64dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_ctr>)))
  "Returns full string definition for message of type '<motor_ctr>"
  (cl:format cl:nil "Header header~%float64 left_wheel~%float64 right_wheel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_ctr)))
  "Returns full string definition for message of type 'motor_ctr"
  (cl:format cl:nil "Header header~%float64 left_wheel~%float64 right_wheel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_ctr>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_ctr>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_ctr
    (cl:cons ':header (header msg))
    (cl:cons ':left_wheel (left_wheel msg))
    (cl:cons ':right_wheel (right_wheel msg))
))
