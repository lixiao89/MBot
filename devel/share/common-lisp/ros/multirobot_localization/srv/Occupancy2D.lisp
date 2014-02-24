; Auto-generated. Do not edit!


(cl:in-package multirobot_localization-srv)


;//! \htmlinclude Occupancy2D-request.msg.html

(cl:defclass <Occupancy2D-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass Occupancy2D-request (<Occupancy2D-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Occupancy2D-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Occupancy2D-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multirobot_localization-srv:<Occupancy2D-request> is deprecated: use multirobot_localization-srv:Occupancy2D-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Occupancy2D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_localization-srv:x-val is deprecated.  Use multirobot_localization-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Occupancy2D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_localization-srv:y-val is deprecated.  Use multirobot_localization-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Occupancy2D-request>) ostream)
  "Serializes a message object of type '<Occupancy2D-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Occupancy2D-request>) istream)
  "Deserializes a message object of type '<Occupancy2D-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Occupancy2D-request>)))
  "Returns string type for a service object of type '<Occupancy2D-request>"
  "multirobot_localization/Occupancy2DRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Occupancy2D-request)))
  "Returns string type for a service object of type 'Occupancy2D-request"
  "multirobot_localization/Occupancy2DRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Occupancy2D-request>)))
  "Returns md5sum for a message object of type '<Occupancy2D-request>"
  "b4270992b09b8bca7b60c19349ac5c98")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Occupancy2D-request)))
  "Returns md5sum for a message object of type 'Occupancy2D-request"
  "b4270992b09b8bca7b60c19349ac5c98")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Occupancy2D-request>)))
  "Returns full string definition for message of type '<Occupancy2D-request>"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Occupancy2D-request)))
  "Returns full string definition for message of type 'Occupancy2D-request"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Occupancy2D-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Occupancy2D-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Occupancy2D-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
;//! \htmlinclude Occupancy2D-response.msg.html

(cl:defclass <Occupancy2D-response> (roslisp-msg-protocol:ros-message)
  ((prob
    :reader prob
    :initarg :prob
    :type cl:float
    :initform 0.0))
)

(cl:defclass Occupancy2D-response (<Occupancy2D-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Occupancy2D-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Occupancy2D-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multirobot_localization-srv:<Occupancy2D-response> is deprecated: use multirobot_localization-srv:Occupancy2D-response instead.")))

(cl:ensure-generic-function 'prob-val :lambda-list '(m))
(cl:defmethod prob-val ((m <Occupancy2D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multirobot_localization-srv:prob-val is deprecated.  Use multirobot_localization-srv:prob instead.")
  (prob m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Occupancy2D-response>) ostream)
  "Serializes a message object of type '<Occupancy2D-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'prob))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Occupancy2D-response>) istream)
  "Deserializes a message object of type '<Occupancy2D-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'prob) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Occupancy2D-response>)))
  "Returns string type for a service object of type '<Occupancy2D-response>"
  "multirobot_localization/Occupancy2DResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Occupancy2D-response)))
  "Returns string type for a service object of type 'Occupancy2D-response"
  "multirobot_localization/Occupancy2DResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Occupancy2D-response>)))
  "Returns md5sum for a message object of type '<Occupancy2D-response>"
  "b4270992b09b8bca7b60c19349ac5c98")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Occupancy2D-response)))
  "Returns md5sum for a message object of type 'Occupancy2D-response"
  "b4270992b09b8bca7b60c19349ac5c98")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Occupancy2D-response>)))
  "Returns full string definition for message of type '<Occupancy2D-response>"
  (cl:format cl:nil "float32 prob~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Occupancy2D-response)))
  "Returns full string definition for message of type 'Occupancy2D-response"
  (cl:format cl:nil "float32 prob~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Occupancy2D-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Occupancy2D-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Occupancy2D-response
    (cl:cons ':prob (prob msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Occupancy2D)))
  'Occupancy2D-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Occupancy2D)))
  'Occupancy2D-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Occupancy2D)))
  "Returns string type for a service object of type '<Occupancy2D>"
  "multirobot_localization/Occupancy2D")