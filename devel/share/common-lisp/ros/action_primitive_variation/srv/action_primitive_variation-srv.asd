
(cl:in-package :asdf)

(defsystem "action_primitive_variation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MoveArm" :depends-on ("_package_MoveArm"))
    (:file "_package_MoveArm" :depends-on ("_package"))
    (:file "PushButton" :depends-on ("_package_PushButton"))
    (:file "_package_PushButton" :depends-on ("_package"))
  ))