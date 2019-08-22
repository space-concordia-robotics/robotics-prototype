;; Auto-generated. Do not edit!


(when (boundp 'mcu_control::AntennaGoal)
  (if (not (find-package "MCU_CONTROL"))
    (make-package "MCU_CONTROL"))
  (shadow 'AntennaGoal (find-package "MCU_CONTROL")))
(unless (find-package "MCU_CONTROL::ANTENNAGOAL")
  (make-package "MCU_CONTROL::ANTENNAGOAL"))

(in-package "ROS")
;;//! \htmlinclude AntennaGoal.msg.html


(defclass mcu_control::AntennaGoal
  :super ros::object
  :slots (_desiredDir _distFromBase ))

(defmethod mcu_control::AntennaGoal
  (:init
   (&key
    ((:desiredDir __desiredDir) 0.0)
    ((:distFromBase __distFromBase) 0.0)
    )
   (send-super :init)
   (setq _desiredDir (float __desiredDir))
   (setq _distFromBase (float __distFromBase))
   self)
  (:desiredDir
   (&optional __desiredDir)
   (if __desiredDir (setq _desiredDir __desiredDir)) _desiredDir)
  (:distFromBase
   (&optional __distFromBase)
   (if __distFromBase (setq _distFromBase __distFromBase)) _distFromBase)
  (:serialization-length
   ()
   (+
    ;; float64 _desiredDir
    8
    ;; float64 _distFromBase
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _desiredDir
       (sys::poke _desiredDir (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distFromBase
       (sys::poke _distFromBase (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _desiredDir
     (setq _desiredDir (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distFromBase
     (setq _distFromBase (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get mcu_control::AntennaGoal :md5sum-) "3e3ff4c1f14859f1d5d7315b6ab697b9")
(setf (get mcu_control::AntennaGoal :datatype-) "mcu_control/AntennaGoal")
(setf (get mcu_control::AntennaGoal :definition-)
      "float64 desiredDir
float64 distFromBase

")



(provide :mcu_control/AntennaGoal "3e3ff4c1f14859f1d5d7315b6ab697b9")


