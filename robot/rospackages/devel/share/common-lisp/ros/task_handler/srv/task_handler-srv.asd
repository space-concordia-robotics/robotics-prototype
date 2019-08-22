
(cl:in-package :asdf)

(defsystem "task_handler-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "HandleTask" :depends-on ("_package_HandleTask"))
    (:file "_package_HandleTask" :depends-on ("_package"))
  ))