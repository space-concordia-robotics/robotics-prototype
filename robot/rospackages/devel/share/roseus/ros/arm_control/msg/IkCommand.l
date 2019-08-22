;; Auto-generated. Do not edit!


(when (boundp 'arm_control::IkCommand)
  (if (not (find-package "ARM_CONTROL"))
    (make-package "ARM_CONTROL"))
  (shadow 'IkCommand (find-package "ARM_CONTROL")))
(unless (find-package "ARM_CONTROL::IKCOMMAND")
  (make-package "ARM_CONTROL::IKCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude IkCommand.msg.html


(defclass arm_control::IkCommand
  :super ros::object
  :slots (_x _y _z _wrist ))

(defmethod arm_control::IkCommand
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:wrist __wrist) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _wrist (float __wrist))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:wrist
   (&optional __wrist)
   (if __wrist (setq _wrist __wrist)) _wrist)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _wrist
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wrist
       (sys::poke _wrist (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wrist
     (setq _wrist (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get arm_control::IkCommand :md5sum-) "e56be75fc62b9fcc4a14e93c7d5229e8")
(setf (get arm_control::IkCommand :datatype-) "arm_control/IkCommand")
(setf (get arm_control::IkCommand :definition-)
      "float64 x
float64 y
float64 z
float64 wrist

")



(provide :arm_control/IkCommand "e56be75fc62b9fcc4a14e93c7d5229e8")


