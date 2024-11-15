
(cl:in-package :asdf)

(defsystem "rob599_basic-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Doubler" :depends-on ("_package_Doubler"))
    (:file "_package_Doubler" :depends-on ("_package"))
  ))