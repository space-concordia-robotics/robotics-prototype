;; Auto-generated. Do not edit!


(when (boundp 'tf2_web_republisher::RepublishTFs)
  (if (not (find-package "TF2_WEB_REPUBLISHER"))
    (make-package "TF2_WEB_REPUBLISHER"))
  (shadow 'RepublishTFs (find-package "TF2_WEB_REPUBLISHER")))
(unless (find-package "TF2_WEB_REPUBLISHER::REPUBLISHTFS")
  (make-package "TF2_WEB_REPUBLISHER::REPUBLISHTFS"))
(unless (find-package "TF2_WEB_REPUBLISHER::REPUBLISHTFSREQUEST")
  (make-package "TF2_WEB_REPUBLISHER::REPUBLISHTFSREQUEST"))
(unless (find-package "TF2_WEB_REPUBLISHER::REPUBLISHTFSRESPONSE")
  (make-package "TF2_WEB_REPUBLISHER::REPUBLISHTFSRESPONSE"))

(in-package "ROS")





(defclass tf2_web_republisher::RepublishTFsRequest
  :super ros::object
  :slots (_source_frames _target_frame _angular_thres _trans_thres _rate _timeout ))

(defmethod tf2_web_republisher::RepublishTFsRequest
  (:init
   (&key
    ((:source_frames __source_frames) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:target_frame __target_frame) "")
    ((:angular_thres __angular_thres) 0.0)
    ((:trans_thres __trans_thres) 0.0)
    ((:rate __rate) 0.0)
    ((:timeout __timeout) (instance ros::time :init))
    )
   (send-super :init)
   (setq _source_frames __source_frames)
   (setq _target_frame (string __target_frame))
   (setq _angular_thres (float __angular_thres))
   (setq _trans_thres (float __trans_thres))
   (setq _rate (float __rate))
   (setq _timeout __timeout)
   self)
  (:source_frames
   (&optional __source_frames)
   (if __source_frames (setq _source_frames __source_frames)) _source_frames)
  (:target_frame
   (&optional __target_frame)
   (if __target_frame (setq _target_frame __target_frame)) _target_frame)
  (:angular_thres
   (&optional __angular_thres)
   (if __angular_thres (setq _angular_thres __angular_thres)) _angular_thres)
  (:trans_thres
   (&optional __trans_thres)
   (if __trans_thres (setq _trans_thres __trans_thres)) _trans_thres)
  (:rate
   (&optional __rate)
   (if __rate (setq _rate __rate)) _rate)
  (:timeout
   (&optional __timeout)
   (if __timeout (setq _timeout __timeout)) _timeout)
  (:serialization-length
   ()
   (+
    ;; string[] _source_frames
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _source_frames)) 4
    ;; string _target_frame
    4 (length _target_frame)
    ;; float32 _angular_thres
    4
    ;; float32 _trans_thres
    4
    ;; float32 _rate
    4
    ;; duration _timeout
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _source_frames
     (write-long (length _source_frames) s)
     (dolist (elem _source_frames)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _target_frame
       (write-long (length _target_frame) s) (princ _target_frame s)
     ;; float32 _angular_thres
       (sys::poke _angular_thres (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _trans_thres
       (sys::poke _trans_thres (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rate
       (sys::poke _rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; duration _timeout
       (write-long (send _timeout :sec) s) (write-long (send _timeout :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _source_frames
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _source_frames (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _source_frames i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _target_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _angular_thres
     (setq _angular_thres (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _trans_thres
     (setq _trans_thres (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rate
     (setq _rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; duration _timeout
     (send _timeout :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timeout :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tf2_web_republisher::RepublishTFsResponse
  :super ros::object
  :slots (_topic_name ))

(defmethod tf2_web_republisher::RepublishTFsResponse
  (:init
   (&key
    ((:topic_name __topic_name) "")
    )
   (send-super :init)
   (setq _topic_name (string __topic_name))
   self)
  (:topic_name
   (&optional __topic_name)
   (if __topic_name (setq _topic_name __topic_name)) _topic_name)
  (:serialization-length
   ()
   (+
    ;; string _topic_name
    4 (length _topic_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _topic_name
       (write-long (length _topic_name) s) (princ _topic_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _topic_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _topic_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tf2_web_republisher::RepublishTFs
  :super ros::object
  :slots ())

(setf (get tf2_web_republisher::RepublishTFs :md5sum-) "ec8570dea2e6015c309eb6611d1a57d0")
(setf (get tf2_web_republisher::RepublishTFs :datatype-) "tf2_web_republisher/RepublishTFs")
(setf (get tf2_web_republisher::RepublishTFs :request) tf2_web_republisher::RepublishTFsRequest)
(setf (get tf2_web_republisher::RepublishTFs :response) tf2_web_republisher::RepublishTFsResponse)

(defmethod tf2_web_republisher::RepublishTFsRequest
  (:response () (instance tf2_web_republisher::RepublishTFsResponse :init)))

(setf (get tf2_web_republisher::RepublishTFsRequest :md5sum-) "ec8570dea2e6015c309eb6611d1a57d0")
(setf (get tf2_web_republisher::RepublishTFsRequest :datatype-) "tf2_web_republisher/RepublishTFsRequest")
(setf (get tf2_web_republisher::RepublishTFsRequest :definition-)
      "

string[] source_frames
string target_frame
float32 angular_thres
float32 trans_thres
float32 rate
duration timeout
---

string topic_name
")

(setf (get tf2_web_republisher::RepublishTFsResponse :md5sum-) "ec8570dea2e6015c309eb6611d1a57d0")
(setf (get tf2_web_republisher::RepublishTFsResponse :datatype-) "tf2_web_republisher/RepublishTFsResponse")
(setf (get tf2_web_republisher::RepublishTFsResponse :definition-)
      "

string[] source_frames
string target_frame
float32 angular_thres
float32 trans_thres
float32 rate
duration timeout
---

string topic_name
")



(provide :tf2_web_republisher/RepublishTFs "ec8570dea2e6015c309eb6611d1a57d0")


