
(cl:in-package :asdf)

(defsystem "RobotCatheter-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "catheterState" :depends-on ("_package_catheterState"))
    (:file "_package_catheterState" :depends-on ("_package"))
  ))