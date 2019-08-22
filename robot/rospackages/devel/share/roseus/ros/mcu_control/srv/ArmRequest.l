;; Auto-generated. Do not edit!


(when (boundp 'mcu_control::ArmRequest)
  (if (not (find-package "MCU_CONTROL"))
    (make-package "MCU_CONTROL"))
  (shadow 'ArmRequest (find-package "MCU_CONTROL")))
(unless (find-package "MCU_CONTROL::ARMREQUEST")
  (make-package "MCU_CONTROL::ARMREQUEST"))
(unless (find-package "MCU_CONTROL::ARMREQUESTREQUEST")
  (make-package "MCU_CONTROL::ARMREQUESTREQUEST"))
(unless (find-package "MCU_CONTROL::ARMREQUESTRESPONSE")
  (make-package "MCU_CONTROL::ARMREQUESTRESPONSE"))

(in-package "ROS")





(defclass mcu_control::ArmRequestRequest
  :super ros::object
  :slots (_msg ))

(defmethod mcu_control::ArmRequestRequest
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

(defclass mcu_control::ArmRequestResponse
  :super ros::object
  :slots (_response _success ))

(defmethod mcu_control::ArmRequestResponse
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

(defclass mcu_control::ArmRequest
  :super ros::object
  :slots ())

(setf (get mcu_control::ArmRequest :md5sum-) "694b24ecd6b00b470b19b9b137151d30")
(setf (get mcu_control::ArmRequest :datatype-) "mcu_control/ArmRequest")
(setf (get mcu_control::ArmRequest :request) mcu_control::ArmRequestRequest)
(setf (get mcu_control::ArmRequest :response) mcu_control::ArmRequestResponse)

(defmethod mcu_control::ArmRequestRequest
  (:response () (instance mcu_control::ArmRequestResponse :init)))

(setf (get mcu_control::ArmRequestRequest :md5sum-) "694b24ecd6b00b470b19b9b137151d30")
(setf (get mcu_control::ArmRequestRequest :datatype-) "mcu_control/ArmRequestRequest")
(setf (get mcu_control::ArmRequestRequest :definition-)
      "string msg
---
string response
bool success

")

(setf (get mcu_control::ArmRequestResponse :md5sum-) "694b24ecd6b00b470b19b9b137151d30")
(setf (get mcu_control::ArmRequestResponse :datatype-) "mcu_control/ArmRequestResponse")
(setf (get mcu_control::ArmRequestResponse :definition-)
      "string msg
---
string response
bool success

")



(provide :mcu_control/ArmRequest "694b24ecd6b00b470b19b9b137151d30")


