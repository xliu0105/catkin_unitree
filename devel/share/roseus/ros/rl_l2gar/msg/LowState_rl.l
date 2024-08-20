;; Auto-generated. Do not edit!


(when (boundp 'rl_l2gar::LowState_rl)
  (if (not (find-package "RL_L2GAR"))
    (make-package "RL_L2GAR"))
  (shadow 'LowState_rl (find-package "RL_L2GAR")))
(unless (find-package "RL_L2GAR::LOWSTATE_RL")
  (make-package "RL_L2GAR::LOWSTATE_RL"))

(in-package "ROS")
;;//! \htmlinclude LowState_rl.msg.html
(if (not (find-package "UNITREE_LEGGED_MSGS"))
  (ros::roseus-add-msgs "unitree_legged_msgs"))


(defclass rl_l2gar::LowState_rl
  :super ros::object
  :slots (_imu _motorState _userValue _userCmd ))

(defmethod rl_l2gar::LowState_rl
  (:init
   (&key
    ((:imu __imu) (instance unitree_legged_msgs::IMU :init))
    ((:motorState __motorState) (let (r) (dotimes (i 20) (push (instance unitree_legged_msgs::MotorState :init) r)) r))
    ((:userValue __userValue) (instance rl_l2gar::userValue_msg :init))
    ((:userCmd __userCmd) 0)
    )
   (send-super :init)
   (setq _imu __imu)
   (setq _motorState __motorState)
   (setq _userValue __userValue)
   (setq _userCmd (round __userCmd))
   self)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:motorState
   (&rest __motorState)
   (if (keywordp (car __motorState))
       (send* _motorState __motorState)
     (progn
       (if __motorState (setq _motorState (car __motorState)))
       _motorState)))
  (:userValue
   (&rest __userValue)
   (if (keywordp (car __userValue))
       (send* _userValue __userValue)
     (progn
       (if __userValue (setq _userValue (car __userValue)))
       _userValue)))
  (:userCmd
   (&optional __userCmd)
   (if __userCmd (setq _userCmd __userCmd)) _userCmd)
  (:serialization-length
   ()
   (+
    ;; unitree_legged_msgs/IMU _imu
    (send _imu :serialization-length)
    ;; unitree_legged_msgs/MotorState[20] _motorState
    (apply #'+ (send-all _motorState :serialization-length))
    ;; rl_l2gar/userValue_msg _userValue
    (send _userValue :serialization-length)
    ;; int32 _userCmd
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; unitree_legged_msgs/IMU _imu
       (send _imu :serialize s)
     ;; unitree_legged_msgs/MotorState[20] _motorState
     (dolist (elem _motorState)
       (send elem :serialize s)
       )
     ;; rl_l2gar/userValue_msg _userValue
       (send _userValue :serialize s)
     ;; int32 _userCmd
       (write-long _userCmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; unitree_legged_msgs/IMU _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; unitree_legged_msgs/MotorState[20] _motorState
   (dotimes (i 20)
     (send (elt _motorState i) :deserialize buf ptr-) (incf ptr- (send (elt _motorState i) :serialization-length))
     )
   ;; rl_l2gar/userValue_msg _userValue
     (send _userValue :deserialize buf ptr-) (incf ptr- (send _userValue :serialization-length))
   ;; int32 _userCmd
     (setq _userCmd (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rl_l2gar::LowState_rl :md5sum-) "b77f6762640229fefac4685d9f05c4e0")
(setf (get rl_l2gar::LowState_rl :datatype-) "rl_l2gar/LowState_rl")
(setf (get rl_l2gar::LowState_rl :definition-)
      "unitree_legged_msgs/IMU imu
unitree_legged_msgs/MotorState[20] motorState
userValue_msg userValue
int32 userCmd
================================================================================
MSG: unitree_legged_msgs/IMU
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
int8 temperature
================================================================================
MSG: unitree_legged_msgs/MotorState
uint8 mode           # motor current mode 
float32 q            # motor current position（rad）
float32 dq           # motor current speed（rad/s）
float32 ddq          # motor current speed（rad/s）
float32 tauEst       # current estimated output torque（N*m）
float32 q_raw        # motor current position（rad）
float32 dq_raw       # motor current speed（rad/s）
float32 ddq_raw      # motor current speed（rad/s）
int8 temperature     # motor temperature（slow conduction of temperature leads to lag）
uint32[2] reserve
================================================================================
MSG: rl_l2gar/userValue_msg
float64 lx
float64 ly
float64 rx
float64 ry
float64 L2
")



(provide :rl_l2gar/LowState_rl "b77f6762640229fefac4685d9f05c4e0")


