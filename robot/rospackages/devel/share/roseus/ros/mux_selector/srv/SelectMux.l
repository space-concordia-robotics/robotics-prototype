;; Auto-generated. Do not edit!


(when (boundp 'mux_selector::SelectMux)
  (if (not (find-package "MUX_SELECTOR"))
    (make-package "MUX_SELECTOR"))
  (shadow 'SelectMux (find-package "MUX_SELECTOR")))
(unless (find-package "MUX_SELECTOR::SELECTMUX")
  (make-package "MUX_SELECTOR::SELECTMUX"))
(unless (find-package "MUX_SELECTOR::SELECTMUXREQUEST")
  (make-package "MUX_SELECTOR::SELECTMUXREQUEST"))
(unless (find-package "MUX_SELECTOR::SELECTMUXRESPONSE")
  (make-package "MUX_SELECTOR::SELECTMUXRESPONSE"))

(in-package "ROS")





(defclass mux_selector::SelectMuxRequest
  :super ros::object
  :slots (_device ))

(defmethod mux_selector::SelectMuxRequest
  (:init
   (&key
    ((:device __device) "")
    )
   (send-super :init)
   (setq _device (string __device))
   self)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:serialization-length
   ()
   (+
    ;; string _device
    4 (length _device)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _device
       (write-long (length _device) s) (princ _device s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _device
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _device (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mux_selector::SelectMuxResponse
  :super ros::object
  :slots (_response ))

(defmethod mux_selector::SelectMuxResponse
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

(defclass mux_selector::SelectMux
  :super ros::object
  :slots ())

(setf (get mux_selector::SelectMux :md5sum-) "ab715037a08c21b05b5bea5be576ffea")
(setf (get mux_selector::SelectMux :datatype-) "mux_selector/SelectMux")
(setf (get mux_selector::SelectMux :request) mux_selector::SelectMuxRequest)
(setf (get mux_selector::SelectMux :response) mux_selector::SelectMuxResponse)

(defmethod mux_selector::SelectMuxRequest
  (:response () (instance mux_selector::SelectMuxResponse :init)))

(setf (get mux_selector::SelectMuxRequest :md5sum-) "ab715037a08c21b05b5bea5be576ffea")
(setf (get mux_selector::SelectMuxRequest :datatype-) "mux_selector/SelectMuxRequest")
(setf (get mux_selector::SelectMuxRequest :definition-)
      "string device
---
string response

")

(setf (get mux_selector::SelectMuxResponse :md5sum-) "ab715037a08c21b05b5bea5be576ffea")
(setf (get mux_selector::SelectMuxResponse :datatype-) "mux_selector/SelectMuxResponse")
(setf (get mux_selector::SelectMuxResponse :definition-)
      "string device
---
string response

")



(provide :mux_selector/SelectMux "ab715037a08c21b05b5bea5be576ffea")


