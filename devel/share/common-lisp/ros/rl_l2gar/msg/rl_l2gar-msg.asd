
(cl:in-package :asdf)

(defsystem "rl_l2gar-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :unitree_legged_msgs-msg
)
  :components ((:file "_package")
    (:file "LowState_rl" :depends-on ("_package_LowState_rl"))
    (:file "_package_LowState_rl" :depends-on ("_package"))
    (:file "userValue_msg" :depends-on ("_package_userValue_msg"))
    (:file "_package_userValue_msg" :depends-on ("_package"))
  ))