;; Auto-generated. Do not edit!


(when (boundp 'ping_acknowledgment::PingResponse)
  (if (not (find-package "PING_ACKNOWLEDGMENT"))
    (make-package "PING_ACKNOWLEDGMENT"))
  (shadow 'PingResponse (find-package "PING_ACKNOWLEDGMENT")))
(unless (find-package "PING_ACKNOWLEDGMENT::PINGRESPONSE")
  (make-package "PING_ACKNOWLEDGMENT::PINGRESPONSE"))
(unless (find-package "PING_ACKNOWLEDGMENT::PINGRESPONSEREQUEST")
  (make-package "PING_ACKNOWLEDGMENT::PINGRESPONSEREQUEST"))
(unless (find-package "PING_ACKNOWLEDGMENT::PINGRESPONSERESPONSE")
  (make-package "PING_ACKNOWLEDGMENT::PINGRESPONSERESPONSE"))

(in-package "ROS")





(defclass ping_acknowledgment::PingResponseRequest
  :super ros::object
  :slots (_ping ))

(defmethod ping_acknowledgment::PingResponseRequest
  (:init
   (&key
    ((:ping __ping) "")
    )
   (send-super :init)
   (setq _ping (string __ping))
   self)
  (:ping
   (&optional __ping)
   (if __ping (setq _ping __ping)) _ping)
  (:serialization-length
   ()
   (+
    ;; string _ping
    4 (length _ping)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _ping
       (write-long (length _ping) s) (princ _ping s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _ping
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ping (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ping_acknowledgment::PingResponseResponse
  :super ros::object
  :slots (_response ))

(defmethod ping_acknowledgment::PingResponseResponse
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

(defclass ping_acknowledgment::PingResponse
  :super ros::object
  :slots ())

(setf (get ping_acknowledgment::PingResponse :md5sum-) "2c33f5a2bff59c326ae8656e9ffa7758")
(setf (get ping_acknowledgment::PingResponse :datatype-) "ping_acknowledgment/PingResponse")
(setf (get ping_acknowledgment::PingResponse :request) ping_acknowledgment::PingResponseRequest)
(setf (get ping_acknowledgment::PingResponse :response) ping_acknowledgment::PingResponseResponse)

(defmethod ping_acknowledgment::PingResponseRequest
  (:response () (instance ping_acknowledgment::PingResponseResponse :init)))

(setf (get ping_acknowledgment::PingResponseRequest :md5sum-) "2c33f5a2bff59c326ae8656e9ffa7758")
(setf (get ping_acknowledgment::PingResponseRequest :datatype-) "ping_acknowledgment/PingResponseRequest")
(setf (get ping_acknowledgment::PingResponseRequest :definition-)
      "string ping
---
string response

")

(setf (get ping_acknowledgment::PingResponseResponse :md5sum-) "2c33f5a2bff59c326ae8656e9ffa7758")
(setf (get ping_acknowledgment::PingResponseResponse :datatype-) "ping_acknowledgment/PingResponseResponse")
(setf (get ping_acknowledgment::PingResponseResponse :definition-)
      "string ping
---
string response

")



(provide :ping_acknowledgment/PingResponse "2c33f5a2bff59c326ae8656e9ffa7758")


