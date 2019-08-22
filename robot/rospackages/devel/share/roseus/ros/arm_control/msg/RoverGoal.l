;; Auto-generated. Do not edit!


(when (boundp 'arm_control::RoverGoal)
  (if (not (find-package "ARM_CONTROL"))
    (make-package "ARM_CONTROL"))
  (shadow 'RoverGoal (find-package "ARM_CONTROL")))
(unless (find-package "ARM_CONTROL::ROVERGOAL")
  (make-package "ARM_CONTROL::ROVERGOAL"))

(in-package "ROS")
;;//! \htmlinclude RoverGoal.msg.html


(defclass arm_control::RoverGoal
  :super ros::object
  :slots (_desiredDir _distToGoal ))

(defmethod arm_control::RoverGoal
  (:init
   (&key
    ((:desiredDir __desiredDir) 0.0)
    ((:distToGoal __distToGoal) 0.0)
    )
   (send-super :init)
   (setq _desiredDir (float __desiredDir))
   (setq _distToGoal (float __distToGoal))
   self)
  (:desiredDir
   (&optional __desiredDir)
   (if __desiredDir (setq _desiredDir __desiredDir)) _desiredDir)
  (:distToGoal
   (&optional __distToGoal)
   (if __distToGoal (setq _distToGoal __distToGoal)) _distToGoal)
  (:serialization-length
   ()
   (+
    ;; float64 _desiredDir
    8
    ;; float64 _distToGoal
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _desiredDir
       (sys::poke _desiredDir (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distToGoal
       (sys::poke _distToGoal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _desiredDir
     (setq _desiredDir (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distToGoal
     (setq _distToGoal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get arm_control::RoverGoal :md5sum-) "4aa12af78877a5824d18dc450fa46f2d")
(setf (get arm_control::RoverGoal :datatype-) "arm_control/RoverGoal")
(setf (get arm_control::RoverGoal :definition-)
      "float64 desiredDir
float64 distToGoal

")



(provide :arm_control/RoverGoal "4aa12af78877a5824d18dc450fa46f2d")


