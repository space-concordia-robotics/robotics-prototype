
(cl:in-package :asdf)

(defsystem "serial_cmd-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SerialCmd" :depends-on ("_package_SerialCmd"))
    (:file "_package_SerialCmd" :depends-on ("_package"))
  ))