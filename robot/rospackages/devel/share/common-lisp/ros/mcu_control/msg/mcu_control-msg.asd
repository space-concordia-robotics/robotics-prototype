
(cl:in-package :asdf)

(defsystem "mcu_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AntennaGoal" :depends-on ("_package_AntennaGoal"))
    (:file "_package_AntennaGoal" :depends-on ("_package"))
    (:file "IkCommand" :depends-on ("_package_IkCommand"))
    (:file "_package_IkCommand" :depends-on ("_package"))
    (:file "RoverGoal" :depends-on ("_package_RoverGoal"))
    (:file "_package_RoverGoal" :depends-on ("_package"))
    (:file "RoverPosition" :depends-on ("_package_RoverPosition"))
    (:file "_package_RoverPosition" :depends-on ("_package"))
  ))