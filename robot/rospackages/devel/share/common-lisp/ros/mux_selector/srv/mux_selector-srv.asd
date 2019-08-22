
(cl:in-package :asdf)

(defsystem "mux_selector-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SelectMux" :depends-on ("_package_SelectMux"))
    (:file "_package_SelectMux" :depends-on ("_package"))
  ))