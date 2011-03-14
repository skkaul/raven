
(in-package :asdf)

(defsystem "roboteq-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "config" :depends-on ("_package"))
    (:file "_package_config" :depends-on ("_package"))
    ))
