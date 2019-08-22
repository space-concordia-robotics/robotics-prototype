
(cl:in-package :asdf)

(defsystem "ping_acknowledgment-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PingResponse" :depends-on ("_package_PingResponse"))
    (:file "_package_PingResponse" :depends-on ("_package"))
  ))