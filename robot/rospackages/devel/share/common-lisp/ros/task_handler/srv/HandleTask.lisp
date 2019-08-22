; Auto-generated. Do not edit!


(cl:in-package task_handler-srv)


;//! \htmlinclude HandleTask-request.msg.html

(cl:defclass <HandleTask-request> (roslisp-msg-protocol:ros-message)
  ((task
    :reader task
    :initarg :task
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0)
   (args
    :reader args
    :initarg :args
    :type cl:string
    :initform ""))
)

(cl:defclass HandleTask-request (<HandleTask-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandleTask-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandleTask-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name task_handler-srv:<HandleTask-request> is deprecated: use task_handler-srv:HandleTask-request instead.")))

(cl:ensure-generic-function 'task-val :lambda-list '(m))
(cl:defmethod task-val ((m <HandleTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_handler-srv:task-val is deprecated.  Use task_handler-srv:task instead.")
  (task m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <HandleTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_handler-srv:status-val is deprecated.  Use task_handler-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'args-val :lambda-list '(m))
(cl:defmethod args-val ((m <HandleTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_handler-srv:args-val is deprecated.  Use task_handler-srv:args instead.")
  (args m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandleTask-request>) ostream)
  "Serializes a message object of type '<HandleTask-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'task))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'task))
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'args))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'args))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandleTask-request>) istream)
  "Deserializes a message object of type '<HandleTask-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'task) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'args) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'args) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandleTask-request>)))
  "Returns string type for a service object of type '<HandleTask-request>"
  "task_handler/HandleTaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandleTask-request)))
  "Returns string type for a service object of type 'HandleTask-request"
  "task_handler/HandleTaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandleTask-request>)))
  "Returns md5sum for a message object of type '<HandleTask-request>"
  "300264c411f28987cadaa5540174e12c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandleTask-request)))
  "Returns md5sum for a message object of type 'HandleTask-request"
  "300264c411f28987cadaa5540174e12c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandleTask-request>)))
  "Returns full string definition for message of type '<HandleTask-request>"
  (cl:format cl:nil "string task~%int64 status~%string args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandleTask-request)))
  "Returns full string definition for message of type 'HandleTask-request"
  (cl:format cl:nil "string task~%int64 status~%string args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandleTask-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'task))
     8
     4 (cl:length (cl:slot-value msg 'args))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandleTask-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HandleTask-request
    (cl:cons ':task (task msg))
    (cl:cons ':status (status msg))
    (cl:cons ':args (args msg))
))
;//! \htmlinclude HandleTask-response.msg.html

(cl:defclass <HandleTask-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass HandleTask-response (<HandleTask-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandleTask-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandleTask-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name task_handler-srv:<HandleTask-response> is deprecated: use task_handler-srv:HandleTask-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <HandleTask-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_handler-srv:response-val is deprecated.  Use task_handler-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandleTask-response>) ostream)
  "Serializes a message object of type '<HandleTask-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandleTask-response>) istream)
  "Deserializes a message object of type '<HandleTask-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandleTask-response>)))
  "Returns string type for a service object of type '<HandleTask-response>"
  "task_handler/HandleTaskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandleTask-response)))
  "Returns string type for a service object of type 'HandleTask-response"
  "task_handler/HandleTaskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandleTask-response>)))
  "Returns md5sum for a message object of type '<HandleTask-response>"
  "300264c411f28987cadaa5540174e12c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandleTask-response)))
  "Returns md5sum for a message object of type 'HandleTask-response"
  "300264c411f28987cadaa5540174e12c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandleTask-response>)))
  "Returns full string definition for message of type '<HandleTask-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandleTask-response)))
  "Returns full string definition for message of type 'HandleTask-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandleTask-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandleTask-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HandleTask-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HandleTask)))
  'HandleTask-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HandleTask)))
  'HandleTask-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandleTask)))
  "Returns string type for a service object of type '<HandleTask>"
  "task_handler/HandleTask")