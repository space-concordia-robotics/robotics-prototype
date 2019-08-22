
(cl:in-package :asdf)

(defsystem "arm_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ArmRequest" :depends-on ("_package_ArmRequest"))
    (:file "_package_ArmRequest" :depends-on ("_package"))
    (:file "ScienceRequest" :depends-on ("_package_ScienceRequest"))
    (:file "_package_ScienceRequest" :depends-on ("_package"))
  ))