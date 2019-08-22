; Auto-generated. Do not edit!


(cl:in-package arm_control-msg)


;//! \htmlinclude RoverGoal.msg.html

(cl:defclass <RoverGoal> (roslisp-msg-protocol:ros-message)
  ((desiredDir
    :reader desiredDir
    :initarg :desiredDir
    :type cl:float
    :initform 0.0)
   (distToGoal
    :reader distToGoal
    :initarg :distToGoal
    :type cl:float
    :initform 0.0))
)

(cl:defclass RoverGoal (<RoverGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoverGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoverGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_control-msg:<RoverGoal> is deprecated: use arm_control-msg:RoverGoal instead.")))

(cl:ensure-generic-function 'desiredDir-val :lambda-list '(m))
(cl:defmethod desiredDir-val ((m <RoverGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_control-msg:desiredDir-val is deprecated.  Use arm_control-msg:desiredDir instead.")
  (desiredDir m))

(cl:ensure-generic-function 'distToGoal-val :lambda-list '(m))
(cl:defmethod distToGoal-val ((m <RoverGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_control-msg:distToGoal-val is deprecated.  Use arm_control-msg:distToGoal instead.")
  (distToGoal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoverGoal>) ostream)
  "Serializes a message object of type '<RoverGoal>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'desiredDir))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distToGoal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoverGoal>) istream)
  "Deserializes a message object of type '<RoverGoal>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desiredDir) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distToGoal) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RoverGoal>)))
  "Returns string type for a message object of type '<RoverGoal>"
  "arm_control/RoverGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoverGoal)))
  "Returns string type for a message object of type 'RoverGoal"
  "arm_control/RoverGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RoverGoal>)))
  "Returns md5sum for a message object of type '<RoverGoal>"
  "4aa12af78877a5824d18dc450fa46f2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoverGoal)))
  "Returns md5sum for a message object of type 'RoverGoal"
  "4aa12af78877a5824d18dc450fa46f2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoverGoal>)))
  "Returns full string definition for message of type '<RoverGoal>"
  (cl:format cl:nil "float64 desiredDir~%float64 distToGoal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoverGoal)))
  "Returns full string definition for message of type 'RoverGoal"
  (cl:format cl:nil "float64 desiredDir~%float64 distToGoal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoverGoal>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoverGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'RoverGoal
    (cl:cons ':desiredDir (desiredDir msg))
    (cl:cons ':distToGoal (distToGoal msg))
))
