;; Auto-generated. Do not edit!


(when (boundp 'arm_control::RoverPosition)
  (if (not (find-package "ARM_CONTROL"))
    (make-package "ARM_CONTROL"))
  (shadow 'RoverPosition (find-package "ARM_CONTROL")))
(unless (find-package "ARM_CONTROL::ROVERPOSITION")
  (make-package "ARM_CONTROL::ROVERPOSITION"))

(in-package "ROS")
;;//! \htmlinclude RoverPosition.msg.html


(defclass arm_control::RoverPosition
  :super ros::object
  :slots (_latitude _longitude _heading ))

(defmethod arm_control::RoverPosition
  (:init
   (&key
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:heading __heading) 0.0)
    )
   (send-super :init)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _heading (float __heading))
   self)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:serialization-length
   ()
   (+
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _heading
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading
     (setq _heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get arm_control::RoverPosition :md5sum-) "2a3396cc67514b0f3740edca07960ab1")
(setf (get arm_control::RoverPosition :datatype-) "arm_control/RoverPosition")
(setf (get arm_control::RoverPosition :definition-)
      "float64 latitude
float64 longitude
float64 heading

")



(provide :arm_control/RoverPosition "2a3396cc67514b0f3740edca07960ab1")


