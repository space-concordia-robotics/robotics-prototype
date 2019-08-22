; Auto-generated. Do not edit!


(cl:in-package mux_selector-srv)


;//! \htmlinclude SelectMux-request.msg.html

(cl:defclass <SelectMux-request> (roslisp-msg-protocol:ros-message)
  ((device
    :reader device
    :initarg :device
    :type cl:string
    :initform ""))
)

(cl:defclass SelectMux-request (<SelectMux-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectMux-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectMux-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mux_selector-srv:<SelectMux-request> is deprecated: use mux_selector-srv:SelectMux-request instead.")))

(cl:ensure-generic-function 'device-val :lambda-list '(m))
(cl:defmethod device-val ((m <SelectMux-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mux_selector-srv:device-val is deprecated.  Use mux_selector-srv:device instead.")
  (device m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectMux-request>) ostream)
  "Serializes a message object of type '<SelectMux-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'device))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'device))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectMux-request>) istream)
  "Deserializes a message object of type '<SelectMux-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'device) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'device) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectMux-request>)))
  "Returns string type for a service object of type '<SelectMux-request>"
  "mux_selector/SelectMuxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectMux-request)))
  "Returns string type for a service object of type 'SelectMux-request"
  "mux_selector/SelectMuxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectMux-request>)))
  "Returns md5sum for a message object of type '<SelectMux-request>"
  "ab715037a08c21b05b5bea5be576ffea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectMux-request)))
  "Returns md5sum for a message object of type 'SelectMux-request"
  "ab715037a08c21b05b5bea5be576ffea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectMux-request>)))
  "Returns full string definition for message of type '<SelectMux-request>"
  (cl:format cl:nil "string device~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectMux-request)))
  "Returns full string definition for message of type 'SelectMux-request"
  (cl:format cl:nil "string device~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectMux-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'device))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectMux-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectMux-request
    (cl:cons ':device (device msg))
))
;//! \htmlinclude SelectMux-response.msg.html

(cl:defclass <SelectMux-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass SelectMux-response (<SelectMux-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectMux-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectMux-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mux_selector-srv:<SelectMux-response> is deprecated: use mux_selector-srv:SelectMux-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <SelectMux-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mux_selector-srv:response-val is deprecated.  Use mux_selector-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectMux-response>) ostream)
  "Serializes a message object of type '<SelectMux-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectMux-response>) istream)
  "Deserializes a message object of type '<SelectMux-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectMux-response>)))
  "Returns string type for a service object of type '<SelectMux-response>"
  "mux_selector/SelectMuxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectMux-response)))
  "Returns string type for a service object of type 'SelectMux-response"
  "mux_selector/SelectMuxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectMux-response>)))
  "Returns md5sum for a message object of type '<SelectMux-response>"
  "ab715037a08c21b05b5bea5be576ffea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectMux-response)))
  "Returns md5sum for a message object of type 'SelectMux-response"
  "ab715037a08c21b05b5bea5be576ffea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectMux-response>)))
  "Returns full string definition for message of type '<SelectMux-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectMux-response)))
  "Returns full string definition for message of type 'SelectMux-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectMux-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectMux-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectMux-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SelectMux)))
  'SelectMux-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SelectMux)))
  'SelectMux-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectMux)))
  "Returns string type for a service object of type '<SelectMux>"
  "mux_selector/SelectMux")