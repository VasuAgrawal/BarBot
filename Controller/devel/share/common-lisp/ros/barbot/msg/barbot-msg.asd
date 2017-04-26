
(cl:in-package :asdf)

(defsystem "barbot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Euler" :depends-on ("_package_Euler"))
    (:file "_package_Euler" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
    (:file "Thruster" :depends-on ("_package_Thruster"))
    (:file "_package_Thruster" :depends-on ("_package"))
  ))