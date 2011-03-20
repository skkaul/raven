
(in-package :asdf)

(defsystem "gpsd_viewer-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "cmd" :depends-on ("_package"))
    (:file "_package_cmd" :depends-on ("_package"))
    ))
