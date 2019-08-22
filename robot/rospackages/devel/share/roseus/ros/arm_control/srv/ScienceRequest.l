;; Auto-generated. Do not edit!


(when (boundp 'arm_control::ScienceRequest)
  (if (not (find-package "ARM_CONTROL"))
    (make-package "ARM_CONTROL"))
  (shadow 'ScienceRequest (find-package "ARM_CONTROL")))
(unless (find-package "ARM_CONTROL::SCIENCEREQUEST")
  (make-package "ARM_CONTROL::SCIENCEREQUEST"))
(unless (find-package "ARM_CONTROL::SCIENCEREQUESTREQUEST")
  (make-package "ARM_CONTROL::SCIENCEREQUESTREQUEST"))
(unless (find-package "ARM_CONTROL::SCIENCEREQUESTRESPONSE")
  (make-package "ARM_CONTROL::SCIENCEREQUESTRESPONSE"))

(in-package "ROS")





(defclass arm_control::ScienceRequestRequest
  :super ros::object
  :slots (_msg ))

(defmethod arm_control::ScienceRequestRequest
  (:init
   (&key
    ((:msg __msg) "")
    )
   (send-super :init)
   (setq _msg (string __msg))
   self)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:serialization-length
   ()
   (+
    ;; string _msg
    4 (length _msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass arm_control::ScienceRequestResponse
  :super ros::object
  :slots (_response _success ))

(defmethod arm_control::ScienceRequestResponse
  (:init
   (&key
    ((:response __response) "")
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _response (string __response))
   (setq _success __success)
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass arm_control::ScienceRequest
  :super ros::object
  :slots ())

(setf (get arm_control::ScienceRequest :md5sum-) "694b24ecd6b00b470b19b9b137151d30")
(setf (get arm_control::ScienceRequest :datatype-) "arm_control/ScienceRequest")
(setf (get arm_control::ScienceRequest :request) arm_control::ScienceRequestRequest)
(setf (get arm_control::ScienceRequest :response) arm_control::ScienceRequestResponse)

(defmethod arm_control::ScienceRequestRequest
  (:response () (instance arm_control::ScienceRequestResponse :init)))

(setf (get arm_control::ScienceRequestRequest :md5sum-) "694b24ecd6b00b470b19b9b137151d30")
(setf (get arm_control::ScienceRequestRequest :datatype-) "arm_control/ScienceRequestRequest")
(setf (get arm_control::ScienceRequestRequest :definition-)
      "string msg
---
string response
bool success

")

(setf (get arm_control::ScienceRequestResponse :md5sum-) "694b24ecd6b00b470b19b9b137151d30")
(setf (get arm_control::ScienceRequestResponse :datatype-) "arm_control/ScienceRequestResponse")
(setf (get arm_control::ScienceRequestResponse :definition-)
      "string msg
---
string response
bool success

")



(provide :arm_control/ScienceRequest "694b24ecd6b00b470b19b9b137151d30")


