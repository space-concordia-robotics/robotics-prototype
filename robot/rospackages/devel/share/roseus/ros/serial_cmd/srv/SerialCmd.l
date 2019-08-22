;; Auto-generated. Do not edit!


(when (boundp 'serial_cmd::SerialCmd)
  (if (not (find-package "SERIAL_CMD"))
    (make-package "SERIAL_CMD"))
  (shadow 'SerialCmd (find-package "SERIAL_CMD")))
(unless (find-package "SERIAL_CMD::SERIALCMD")
  (make-package "SERIAL_CMD::SERIALCMD"))
(unless (find-package "SERIAL_CMD::SERIALCMDREQUEST")
  (make-package "SERIAL_CMD::SERIALCMDREQUEST"))
(unless (find-package "SERIAL_CMD::SERIALCMDRESPONSE")
  (make-package "SERIAL_CMD::SERIALCMDRESPONSE"))

(in-package "ROS")





(defclass serial_cmd::SerialCmdRequest
  :super ros::object
  :slots (_msg ))

(defmethod serial_cmd::SerialCmdRequest
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

(defclass serial_cmd::SerialCmdResponse
  :super ros::object
  :slots (_response ))

(defmethod serial_cmd::SerialCmdResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass serial_cmd::SerialCmd
  :super ros::object
  :slots ())

(setf (get serial_cmd::SerialCmd :md5sum-) "44a43a3c9ed832dd024eb4e68053d686")
(setf (get serial_cmd::SerialCmd :datatype-) "serial_cmd/SerialCmd")
(setf (get serial_cmd::SerialCmd :request) serial_cmd::SerialCmdRequest)
(setf (get serial_cmd::SerialCmd :response) serial_cmd::SerialCmdResponse)

(defmethod serial_cmd::SerialCmdRequest
  (:response () (instance serial_cmd::SerialCmdResponse :init)))

(setf (get serial_cmd::SerialCmdRequest :md5sum-) "44a43a3c9ed832dd024eb4e68053d686")
(setf (get serial_cmd::SerialCmdRequest :datatype-) "serial_cmd/SerialCmdRequest")
(setf (get serial_cmd::SerialCmdRequest :definition-)
      "string msg
---
string response

")

(setf (get serial_cmd::SerialCmdResponse :md5sum-) "44a43a3c9ed832dd024eb4e68053d686")
(setf (get serial_cmd::SerialCmdResponse :datatype-) "serial_cmd/SerialCmdResponse")
(setf (get serial_cmd::SerialCmdResponse :definition-)
      "string msg
---
string response

")



(provide :serial_cmd/SerialCmd "44a43a3c9ed832dd024eb4e68053d686")


