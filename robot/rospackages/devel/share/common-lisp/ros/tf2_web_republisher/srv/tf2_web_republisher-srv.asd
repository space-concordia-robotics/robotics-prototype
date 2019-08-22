
(cl:in-package :asdf)

(defsystem "tf2_web_republisher-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RepublishTFs" :depends-on ("_package_RepublishTFs"))
    (:file "_package_RepublishTFs" :depends-on ("_package"))
  ))