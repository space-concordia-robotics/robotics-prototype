; Auto-generated. Do not edit!


(cl:in-package arm_control-srv)


;//! \htmlinclude ScienceRequest-request.msg.html

(cl:defclass <ScienceRequest-request> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass ScienceRequest-request (<ScienceRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ScienceRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ScienceRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_control-srv:<ScienceRequest-request> is deprecated: use arm_control-srv:ScienceRequest-request instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <ScienceRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_control-srv:msg-val is deprecated.  Use arm_control-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ScienceRequest-request>) ostream)
  "Serializes a message object of type '<ScienceRequest-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ScienceRequest-request>) istream)
  "Deserializes a message object of type '<ScienceRequest-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ScienceRequest-request>)))
  "Returns string type for a service object of type '<ScienceRequest-request>"
  "arm_control/ScienceRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ScienceRequest-request)))
  "Returns string type for a service object of type 'ScienceRequest-request"
  "arm_control/ScienceRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ScienceRequest-request>)))
  "Returns md5sum for a message object of type '<ScienceRequest-request>"
  "694b24ecd6b00b470b19b9b137151d30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ScienceRequest-request)))
  "Returns md5sum for a message object of type 'ScienceRequest-request"
  "694b24ecd6b00b470b19b9b137151d30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ScienceRequest-request>)))
  "Returns full string definition for message of type '<ScienceRequest-request>"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ScienceRequest-request)))
  "Returns full string definition for message of type 'ScienceRequest-request"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ScienceRequest-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ScienceRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ScienceRequest-request
    (cl:cons ':msg (msg msg))
))
;//! \htmlinclude ScienceRequest-response.msg.html

(cl:defclass <ScienceRequest-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ScienceRequest-response (<ScienceRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ScienceRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ScienceRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_control-srv:<ScienceRequest-response> is deprecated: use arm_control-srv:ScienceRequest-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <ScienceRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_control-srv:response-val is deprecated.  Use arm_control-srv:response instead.")
  (response m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ScienceRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_control-srv:success-val is deprecated.  Use arm_control-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ScienceRequest-response>) ostream)
  "Serializes a message object of type '<ScienceRequest-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ScienceRequest-response>) istream)
  "Deserializes a message object of type '<ScienceRequest-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ScienceRequest-response>)))
  "Returns string type for a service object of type '<ScienceRequest-response>"
  "arm_control/ScienceRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ScienceRequest-response)))
  "Returns string type for a service object of type 'ScienceRequest-response"
  "arm_control/ScienceRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ScienceRequest-response>)))
  "Returns md5sum for a message object of type '<ScienceRequest-response>"
  "694b24ecd6b00b470b19b9b137151d30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ScienceRequest-response)))
  "Returns md5sum for a message object of type 'ScienceRequest-response"
  "694b24ecd6b00b470b19b9b137151d30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ScienceRequest-response>)))
  "Returns full string definition for message of type '<ScienceRequest-response>"
  (cl:format cl:nil "string response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ScienceRequest-response)))
  "Returns full string definition for message of type 'ScienceRequest-response"
  (cl:format cl:nil "string response~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ScienceRequest-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ScienceRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ScienceRequest-response
    (cl:cons ':response (response msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ScienceRequest)))
  'ScienceRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ScienceRequest)))
  'ScienceRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ScienceRequest)))
  "Returns string type for a service object of type '<ScienceRequest>"
  "arm_control/ScienceRequest")