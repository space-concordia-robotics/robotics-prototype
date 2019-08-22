; Auto-generated. Do not edit!


(cl:in-package serial_cmd-srv)


;//! \htmlinclude SerialCmd-request.msg.html

(cl:defclass <SerialCmd-request> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass SerialCmd-request (<SerialCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SerialCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SerialCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serial_cmd-srv:<SerialCmd-request> is deprecated: use serial_cmd-srv:SerialCmd-request instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <SerialCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_cmd-srv:msg-val is deprecated.  Use serial_cmd-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SerialCmd-request>) ostream)
  "Serializes a message object of type '<SerialCmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SerialCmd-request>) istream)
  "Deserializes a message object of type '<SerialCmd-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SerialCmd-request>)))
  "Returns string type for a service object of type '<SerialCmd-request>"
  "serial_cmd/SerialCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SerialCmd-request)))
  "Returns string type for a service object of type 'SerialCmd-request"
  "serial_cmd/SerialCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SerialCmd-request>)))
  "Returns md5sum for a message object of type '<SerialCmd-request>"
  "44a43a3c9ed832dd024eb4e68053d686")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SerialCmd-request)))
  "Returns md5sum for a message object of type 'SerialCmd-request"
  "44a43a3c9ed832dd024eb4e68053d686")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SerialCmd-request>)))
  "Returns full string definition for message of type '<SerialCmd-request>"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SerialCmd-request)))
  "Returns full string definition for message of type 'SerialCmd-request"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SerialCmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SerialCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SerialCmd-request
    (cl:cons ':msg (msg msg))
))
;//! \htmlinclude SerialCmd-response.msg.html

(cl:defclass <SerialCmd-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass SerialCmd-response (<SerialCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SerialCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SerialCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serial_cmd-srv:<SerialCmd-response> is deprecated: use serial_cmd-srv:SerialCmd-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <SerialCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_cmd-srv:response-val is deprecated.  Use serial_cmd-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SerialCmd-response>) ostream)
  "Serializes a message object of type '<SerialCmd-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SerialCmd-response>) istream)
  "Deserializes a message object of type '<SerialCmd-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SerialCmd-response>)))
  "Returns string type for a service object of type '<SerialCmd-response>"
  "serial_cmd/SerialCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SerialCmd-response)))
  "Returns string type for a service object of type 'SerialCmd-response"
  "serial_cmd/SerialCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SerialCmd-response>)))
  "Returns md5sum for a message object of type '<SerialCmd-response>"
  "44a43a3c9ed832dd024eb4e68053d686")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SerialCmd-response)))
  "Returns md5sum for a message object of type 'SerialCmd-response"
  "44a43a3c9ed832dd024eb4e68053d686")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SerialCmd-response>)))
  "Returns full string definition for message of type '<SerialCmd-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SerialCmd-response)))
  "Returns full string definition for message of type 'SerialCmd-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SerialCmd-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SerialCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SerialCmd-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SerialCmd)))
  'SerialCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SerialCmd)))
  'SerialCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SerialCmd)))
  "Returns string type for a service object of type '<SerialCmd>"
  "serial_cmd/SerialCmd")