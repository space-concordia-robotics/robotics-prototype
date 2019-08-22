; Auto-generated. Do not edit!


(cl:in-package tf2_web_republisher-msg)


;//! \htmlinclude TFSubscriptionGoal.msg.html

(cl:defclass <TFSubscriptionGoal> (roslisp-msg-protocol:ros-message)
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
    :initform 0.0))
)

(cl:defclass TFSubscriptionGoal (<TFSubscriptionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TFSubscriptionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TFSubscriptionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tf2_web_republisher-msg:<TFSubscriptionGoal> is deprecated: use tf2_web_republisher-msg:TFSubscriptionGoal instead.")))

(cl:ensure-generic-function 'source_frames-val :lambda-list '(m))
(cl:defmethod source_frames-val ((m <TFSubscriptionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-msg:source_frames-val is deprecated.  Use tf2_web_republisher-msg:source_frames instead.")
  (source_frames m))

(cl:ensure-generic-function 'target_frame-val :lambda-list '(m))
(cl:defmethod target_frame-val ((m <TFSubscriptionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-msg:target_frame-val is deprecated.  Use tf2_web_republisher-msg:target_frame instead.")
  (target_frame m))

(cl:ensure-generic-function 'angular_thres-val :lambda-list '(m))
(cl:defmethod angular_thres-val ((m <TFSubscriptionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-msg:angular_thres-val is deprecated.  Use tf2_web_republisher-msg:angular_thres instead.")
  (angular_thres m))

(cl:ensure-generic-function 'trans_thres-val :lambda-list '(m))
(cl:defmethod trans_thres-val ((m <TFSubscriptionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-msg:trans_thres-val is deprecated.  Use tf2_web_republisher-msg:trans_thres instead.")
  (trans_thres m))

(cl:ensure-generic-function 'rate-val :lambda-list '(m))
(cl:defmethod rate-val ((m <TFSubscriptionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tf2_web_republisher-msg:rate-val is deprecated.  Use tf2_web_republisher-msg:rate instead.")
  (rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TFSubscriptionGoal>) ostream)
  "Serializes a message object of type '<TFSubscriptionGoal>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TFSubscriptionGoal>) istream)
  "Deserializes a message object of type '<TFSubscriptionGoal>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TFSubscriptionGoal>)))
  "Returns string type for a message object of type '<TFSubscriptionGoal>"
  "tf2_web_republisher/TFSubscriptionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TFSubscriptionGoal)))
  "Returns string type for a message object of type 'TFSubscriptionGoal"
  "tf2_web_republisher/TFSubscriptionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TFSubscriptionGoal>)))
  "Returns md5sum for a message object of type '<TFSubscriptionGoal>"
  "b2dae39608227a5c1c4a91ad77023a27")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TFSubscriptionGoal)))
  "Returns md5sum for a message object of type 'TFSubscriptionGoal"
  "b2dae39608227a5c1c4a91ad77023a27")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TFSubscriptionGoal>)))
  "Returns full string definition for message of type '<TFSubscriptionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# goal~%string[] source_frames~%string target_frame~%float32 angular_thres~%float32 trans_thres~%float32 rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TFSubscriptionGoal)))
  "Returns full string definition for message of type 'TFSubscriptionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# goal~%string[] source_frames~%string target_frame~%float32 angular_thres~%float32 trans_thres~%float32 rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TFSubscriptionGoal>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'source_frames) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'target_frame))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TFSubscriptionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'TFSubscriptionGoal
    (cl:cons ':source_frames (source_frames msg))
    (cl:cons ':target_frame (target_frame msg))
    (cl:cons ':angular_thres (angular_thres msg))
    (cl:cons ':trans_thres (trans_thres msg))
    (cl:cons ':rate (rate msg))
))
