; Auto-generated. Do not edit!


(cl:in-package tf2_web_republisher-srv)


;//! \htmlinclude RepublishTFs-request.msg.html

(cl:defclass <RepublishTFs-request> (roslisp-msg-protocol:ros-message)
  ((source_frames
    :reader source_frames
    :initarg :source_frames
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (target_frame
    :reader target_frame
    :initarg :target_frame
    :type cl:string
    :initform "")
   (angular_thres
    :reader angular_thres
    :initarg :angular_thres
    :type cl:float
    :initform 0.0)
   (trans_thres
    :reader trans_thres
    :initarg :trans_thres
    :type cl:float
    :initform 0.0)
   (rate
    :reader rate
    :initarg :rate
    :type cl:float
    :initform 0.0)
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:real
    :initform 0))
)

(cl:defclass RepublishTFs-request (<RepublishTFs-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RepublishTFs-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RepublishTFs-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tf2_web_republisher-srv:<RepublishTFs-request> is deprecated: use tf2_web_republisher-srv:RepublishTFs-request instead.")))

(cl:ensure-generic-function 'source_frames-val :lambda-list '(m))
(cl:defmethod source_frames-val ((m <RepublishTFs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-srv:source_frames-val is deprecated.  Use tf2_web_republisher-srv:source_frames instead.")
  (source_frames m))

(cl:ensure-generic-function 'target_frame-val :lambda-list '(m))
(cl:defmethod target_frame-val ((m <RepublishTFs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-srv:target_frame-val is deprecated.  Use tf2_web_republisher-srv:target_frame instead.")
  (target_frame m))

(cl:ensure-generic-function 'angular_thres-val :lambda-list '(m))
(cl:defmethod angular_thres-val ((m <RepublishTFs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-srv:angular_thres-val is deprecated.  Use tf2_web_republisher-srv:angular_thres instead.")
  (angular_thres m))

(cl:ensure-generic-function 'trans_thres-val :lambda-list '(m))
(cl:defmethod trans_thres-val ((m <RepublishTFs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-srv:trans_thres-val is deprecated.  Use tf2_web_republisher-srv:trans_thres instead.")
  (trans_thres m))

(cl:ensure-generic-function 'rate-val :lambda-list '(m))
(cl:defmethod rate-val ((m <RepublishTFs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-srv:rate-val is deprecated.  Use tf2_web_republisher-srv:rate instead.")
  (rate m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <RepublishTFs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-srv:timeout-val is deprecated.  Use tf2_web_republisher-srv:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RepublishTFs-request>) ostream)
  "Serializes a message object of type '<RepublishTFs-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'source_frames))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'source_frames))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target_frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target_frame))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_thres))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'trans_thres))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'timeout)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'timeout) (cl:floor (cl:slot-value msg 'timeout)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RepublishTFs-request>) istream)
  "Deserializes a message object of type '<RepublishTFs-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'source_frames) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'source_frames)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_frame) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target_frame) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_thres) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'trans_thres) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timeout) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RepublishTFs-request>)))
  "Returns string type for a service object of type '<RepublishTFs-request>"
  "tf2_web_republisher/RepublishTFsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RepublishTFs-request)))
  "Returns string type for a service object of type 'RepublishTFs-request"
  "tf2_web_republisher/RepublishTFsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RepublishTFs-request>)))
  "Returns md5sum for a message object of type '<RepublishTFs-request>"
  "ec8570dea2e6015c309eb6611d1a57d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RepublishTFs-request)))
  "Returns md5sum for a message object of type 'RepublishTFs-request"
  "ec8570dea2e6015c309eb6611d1a57d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RepublishTFs-request>)))
  "Returns full string definition for message of type '<RepublishTFs-request>"
  (cl:format cl:nil "~%~%string[] source_frames~%string target_frame~%float32 angular_thres~%float32 trans_thres~%float32 rate~%duration timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RepublishTFs-request)))
  "Returns full string definition for message of type 'RepublishTFs-request"
  (cl:format cl:nil "~%~%string[] source_frames~%string target_frame~%float32 angular_thres~%float32 trans_thres~%float32 rate~%duration timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RepublishTFs-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'source_frames) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'target_frame))
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RepublishTFs-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RepublishTFs-request
    (cl:cons ':source_frames (source_frames msg))
    (cl:cons ':target_frame (target_frame msg))
    (cl:cons ':angular_thres (angular_thres msg))
    (cl:cons ':trans_thres (trans_thres msg))
    (cl:cons ':rate (rate msg))
    (cl:cons ':timeout (timeout msg))
))
;//! \htmlinclude RepublishTFs-response.msg.html

(cl:defclass <RepublishTFs-response> (roslisp-msg-protocol:ros-message)
  ((topic_name
    :reader topic_name
    :initarg :topic_name
    :type cl:string
    :initform ""))
)

(cl:defclass RepublishTFs-response (<RepublishTFs-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RepublishTFs-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RepublishTFs-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tf2_web_republisher-srv:<RepublishTFs-response> is deprecated: use tf2_web_republisher-srv:RepublishTFs-response instead.")))

(cl:ensure-generic-function 'topic_name-val :lambda-list '(m))
(cl:defmethod topic_name-val ((m <RepublishTFs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-srv:topic_name-val is deprecated.  Use tf2_web_republisher-srv:topic_name instead.")
  (topic_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RepublishTFs-response>) ostream)
  "Serializes a message object of type '<RepublishTFs-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'topic_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'topic_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RepublishTFs-response>) istream)
  "Deserializes a message object of type '<RepublishTFs-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'topic_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'topic_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RepublishTFs-response>)))
  "Returns string type for a service object of type '<RepublishTFs-response>"
  "tf2_web_republisher/RepublishTFsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RepublishTFs-response)))
  "Returns string type for a service object of type 'RepublishTFs-response"
  "tf2_web_republisher/RepublishTFsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RepublishTFs-response>)))
  "Returns md5sum for a message object of type '<RepublishTFs-response>"
  "ec8570dea2e6015c309eb6611d1a57d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RepublishTFs-response)))
  "Returns md5sum for a message object of type 'RepublishTFs-response"
  "ec8570dea2e6015c309eb6611d1a57d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RepublishTFs-response>)))
  "Returns full string definition for message of type '<RepublishTFs-response>"
  (cl:format cl:nil "~%string topic_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RepublishTFs-response)))
  "Returns full string definition for message of type 'RepublishTFs-response"
  (cl:format cl:nil "~%string topic_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RepublishTFs-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'topic_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RepublishTFs-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RepublishTFs-response
    (cl:cons ':topic_name (topic_name msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RepublishTFs)))
  'RepublishTFs-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RepublishTFs)))
  'RepublishTFs-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RepublishTFs)))
  "Returns string type for a service object of type '<RepublishTFs>"
  "tf2_web_republisher/RepublishTFs")