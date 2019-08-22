; Auto-generated. Do not edit!


(cl:in-package arm_control-msg)


;//! \htmlinclude AntennaGoal.msg.html

(cl:defclass <AntennaGoal> (roslisp-msg-protocol:ros-message)
  ((desiredDir
    :reader desiredDir
    :initarg :desiredDir
    :type cl:float
    :initform 0.0)
   (distFromBase
    :reader distFromBase
    :initarg :distFromBase
    :type cl:float
    :initform 0.0))
)

(cl:defclass AntennaGoal (<AntennaGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AntennaGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AntennaGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_control-msg:<AntennaGoal> is deprecated: use arm_control-msg:AntennaGoal instead.")))

(cl:ensure-generic-function 'desiredDir-val :lambda-list '(m))
(cl:defmethod desiredDir-val ((m <AntennaGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_control-msg:desiredDir-val is deprecated.  Use arm_control-msg:desiredDir instead.")
  (desiredDir m))

(cl:ensure-generic-function 'distFromBase-val :lambda-list '(m))
(cl:defmethod distFromBase-val ((m <AntennaGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_control-msg:distFromBase-val is deprecated.  Use arm_control-msg:distFromBase instead.")
  (distFromBase m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AntennaGoal>) ostream)
  "Serializes a message object of type '<AntennaGoal>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'desiredDir))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distFromBase))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AntennaGoal>) istream)
  "Deserializes a message object of type '<AntennaGoal>"
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
    (cl:setf (cl:slot-value msg 'distFromBase) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AntennaGoal>)))
  "Returns string type for a message object of type '<AntennaGoal>"
  "arm_control/AntennaGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AntennaGoal)))
  "Returns string type for a message object of type 'AntennaGoal"
  "arm_control/AntennaGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AntennaGoal>)))
  "Returns md5sum for a message object of type '<AntennaGoal>"
  "3e3ff4c1f14859f1d5d7315b6ab697b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AntennaGoal)))
  "Returns md5sum for a message object of type 'AntennaGoal"
  "3e3ff4c1f14859f1d5d7315b6ab697b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AntennaGoal>)))
  "Returns full string definition for message of type '<AntennaGoal>"
  (cl:format cl:nil "float64 desiredDir~%float64 distFromBase~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AntennaGoal)))
  "Returns full string definition for message of type 'AntennaGoal"
  (cl:format cl:nil "float64 desiredDir~%float64 distFromBase~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AntennaGoal>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AntennaGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'AntennaGoal
    (cl:cons ':desiredDir (desiredDir msg))
    (cl:cons ':distFromBase (distFromBase msg))
))
