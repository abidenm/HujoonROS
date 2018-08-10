
(cl:in-package :asdf)

(defsystem "SkillMate-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "HapticCommand" :depends-on ("_package_HapticCommand"))
    (:file "_package_HapticCommand" :depends-on ("_package"))
    (:file "Mouse3dCommand" :depends-on ("_package_Mouse3dCommand"))
    (:file "_package_Mouse3dCommand" :depends-on ("_package"))
  ))