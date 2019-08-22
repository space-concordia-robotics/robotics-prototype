; Auto-generated. Do not edit!


(cl:in-package mcu_control-srv)


;//! \htmlinclude ArmRequest-request.msg.html

(cl:defclass <ArmRequest-request> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass ArmRequest-request (<ArmRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mcu_control-srv:<ArmRequest-request> is deprecated: use mcu_control-srv:ArmRequest-request instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <ArmRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mcu_control-srv:msg-val is deprecated.  Use mcu_control-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmRequest-request>) ostream)
  "Serializes a message object of type '<ArmRequest-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmRequest-request>) istream)
  "Deserializes a message object of type '<ArmRequest-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmRequest-request>)))
  "Returns string type for a service object of type '<ArmRequest-request>"
  "mcu_control/ArmRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmRequest-request)))
  "Returns string type for a service object of type 'ArmRequest-request"
  "mcu_control/ArmRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmRequest-request>)))
  "Returns md5sum for a message object of type '<ArmRequest-request>"
  "694b24ecd6b00b470b19b9b137151d30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmRequest-request)))
  "Returns md5sum for a message object of type 'ArmRequest-request"
  "694b24ecd6b00b470b19b9b137151d30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmRequest-request>)))
  "Returns full string definition for message of type '<ArmRequest-request>"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmRequest-request)))
  "Returns full string definition for message of type 'ArmRequest-request"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmRequest-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmRequest-request
    (cl:cons ':msg (msg msg))
))
;//! \htmlinclude ArmRequest-response.msg.html

(cl:defclass <ArmRequest-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform "")
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ArmRequest-response (<ArmRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mcu_control-srv:<ArmRequest-response> is deprecated: use mcu_control-srv:ArmRequest-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <ArmRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mcu_control-srv:response-val is deprecated.  Use mcu_control-srv:response instead.")
  (response m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ArmRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mcu_control-srv:success-val is deprecated.  Use mcu_control-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmRequest-response>) ostream)
  "Serializes a message object of type '<ArmRequest-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmRequest-response>) istream)
  "Deserializes a message object of type '<ArmRequest-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmRequest-response>)))
  "Returns string type for a service object of type '<ArmRequest-response>"
  "mcu_control/ArmRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmRequest-response)))
  "Returns string type for a service object of type 'ArmRequest-response"
  "mcu_control/ArmRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmRequest-response>)))
  "Returns md5sum for a message object of type '<ArmRequest-response>"
  "694b24ecd6b00b470b19b9b137151d30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmRequest-response)))
  "Returns md5sum for a message object of type 'ArmRequest-response"
  "694b24ecd6b00b470b19b9b137151d30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmRequest-response>)))
  "Returns full string definition for message of type '<ArmRequest-response>"
  (cl:format cl:nil "string response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmRequest-response)))
  "Returns full string definition for message of type 'ArmRequest-response"
  (cl:format cl:nil "string response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmRequest-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmRequest-response
    (cl:cons ':response (response msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ArmRequest)))
  'ArmRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ArmRequest)))
  'ArmRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmRequest)))
  "Returns string type for a service object of type '<ArmRequest>"
  "mcu_control/ArmRequest")