; Auto-generated. Do not edit!


(cl:in-package ping_acknowledgment-srv)


;//! \htmlinclude PingResponse-request.msg.html

(cl:defclass <PingResponse-request> (roslisp-msg-protocol:ros-message)
  ((ping
    :reader ping
    :initarg :ping
    :type cl:string
    :initform ""))
)

(cl:defclass PingResponse-request (<PingResponse-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PingResponse-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PingResponse-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ping_acknowledgment-srv:<PingResponse-request> is deprecated: use ping_acknowledgment-srv:PingResponse-request instead.")))

(cl:ensure-generic-function 'ping-val :lambda-list '(m))
(cl:defmethod ping-val ((m <PingResponse-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ping_acknowledgment-srv:ping-val is deprecated.  Use ping_acknowledgment-srv:ping instead.")
  (ping m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PingResponse-request>) ostream)
  "Serializes a message object of type '<PingResponse-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ping))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ping))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PingResponse-request>) istream)
  "Deserializes a message object of type '<PingResponse-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ping) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ping) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PingResponse-request>)))
  "Returns string type for a service object of type '<PingResponse-request>"
  "ping_acknowledgment/PingResponseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingResponse-request)))
  "Returns string type for a service object of type 'PingResponse-request"
  "ping_acknowledgment/PingResponseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PingResponse-request>)))
  "Returns md5sum for a message object of type '<PingResponse-request>"
  "2c33f5a2bff59c326ae8656e9ffa7758")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PingResponse-request)))
  "Returns md5sum for a message object of type 'PingResponse-request"
  "2c33f5a2bff59c326ae8656e9ffa7758")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PingResponse-request>)))
  "Returns full string definition for message of type '<PingResponse-request>"
  (cl:format cl:nil "string ping~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PingResponse-request)))
  "Returns full string definition for message of type 'PingResponse-request"
  (cl:format cl:nil "string ping~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PingResponse-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ping))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PingResponse-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PingResponse-request
    (cl:cons ':ping (ping msg))
))
;//! \htmlinclude PingResponse-response.msg.html

(cl:defclass <PingResponse-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass PingResponse-response (<PingResponse-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PingResponse-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PingResponse-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ping_acknowledgment-srv:<PingResponse-response> is deprecated: use ping_acknowledgment-srv:PingResponse-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <PingResponse-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ping_acknowledgment-srv:response-val is deprecated.  Use ping_acknowledgment-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PingResponse-response>) ostream)
  "Serializes a message object of type '<PingResponse-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PingResponse-response>) istream)
  "Deserializes a message object of type '<PingResponse-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PingResponse-response>)))
  "Returns string type for a service object of type '<PingResponse-response>"
  "ping_acknowledgment/PingResponseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingResponse-response)))
  "Returns string type for a service object of type 'PingResponse-response"
  "ping_acknowledgment/PingResponseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PingResponse-response>)))
  "Returns md5sum for a message object of type '<PingResponse-response>"
  "2c33f5a2bff59c326ae8656e9ffa7758")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PingResponse-response)))
  "Returns md5sum for a message object of type 'PingResponse-response"
  "2c33f5a2bff59c326ae8656e9ffa7758")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PingResponse-response>)))
  "Returns full string definition for message of type '<PingResponse-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PingResponse-response)))
  "Returns full string definition for message of type 'PingResponse-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PingResponse-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PingResponse-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PingResponse-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PingResponse)))
  'PingResponse-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PingResponse)))
  'PingResponse-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingResponse)))
  "Returns string type for a service object of type '<PingResponse>"
  "ping_acknowledgment/PingResponse")