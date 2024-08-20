; Auto-generated. Do not edit!


(cl:in-package rl_l2gar-msg)


;//! \htmlinclude LowState_rl.msg.html

(cl:defclass <LowState_rl> (roslisp-msg-protocol:ros-message)
  ((imu
    :reader imu
    :initarg :imu
    :type unitree_legged_msgs-msg:IMU
    :initform (cl:make-instance 'unitree_legged_msgs-msg:IMU))
   (motorState
    :reader motorState
    :initarg :motorState
    :type (cl:vector unitree_legged_msgs-msg:MotorState)
   :initform (cl:make-array 20 :element-type 'unitree_legged_msgs-msg:MotorState :initial-element (cl:make-instance 'unitree_legged_msgs-msg:MotorState)))
   (userValue
    :reader userValue
    :initarg :userValue
    :type rl_l2gar-msg:userValue_msg
    :initform (cl:make-instance 'rl_l2gar-msg:userValue_msg))
   (userCmd
    :reader userCmd
    :initarg :userCmd
    :type cl:integer
    :initform 0))
)

(cl:defclass LowState_rl (<LowState_rl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LowState_rl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LowState_rl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_l2gar-msg:<LowState_rl> is deprecated: use rl_l2gar-msg:LowState_rl instead.")))

(cl:ensure-generic-function 'imu-val :lambda-list '(m))
(cl:defmethod imu-val ((m <LowState_rl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_l2gar-msg:imu-val is deprecated.  Use rl_l2gar-msg:imu instead.")
  (imu m))

(cl:ensure-generic-function 'motorState-val :lambda-list '(m))
(cl:defmethod motorState-val ((m <LowState_rl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_l2gar-msg:motorState-val is deprecated.  Use rl_l2gar-msg:motorState instead.")
  (motorState m))

(cl:ensure-generic-function 'userValue-val :lambda-list '(m))
(cl:defmethod userValue-val ((m <LowState_rl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_l2gar-msg:userValue-val is deprecated.  Use rl_l2gar-msg:userValue instead.")
  (userValue m))

(cl:ensure-generic-function 'userCmd-val :lambda-list '(m))
(cl:defmethod userCmd-val ((m <LowState_rl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_l2gar-msg:userCmd-val is deprecated.  Use rl_l2gar-msg:userCmd instead.")
  (userCmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LowState_rl>) ostream)
  "Serializes a message object of type '<LowState_rl>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motorState))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'userValue) ostream)
  (cl:let* ((signed (cl:slot-value msg 'userCmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LowState_rl>) istream)
  "Deserializes a message object of type '<LowState_rl>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu) istream)
  (cl:setf (cl:slot-value msg 'motorState) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'motorState)))
    (cl:dotimes (i 20)
    (cl:setf (cl:aref vals i) (cl:make-instance 'unitree_legged_msgs-msg:MotorState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'userValue) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'userCmd) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LowState_rl>)))
  "Returns string type for a message object of type '<LowState_rl>"
  "rl_l2gar/LowState_rl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LowState_rl)))
  "Returns string type for a message object of type 'LowState_rl"
  "rl_l2gar/LowState_rl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LowState_rl>)))
  "Returns md5sum for a message object of type '<LowState_rl>"
  "b77f6762640229fefac4685d9f05c4e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LowState_rl)))
  "Returns md5sum for a message object of type 'LowState_rl"
  "b77f6762640229fefac4685d9f05c4e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LowState_rl>)))
  "Returns full string definition for message of type '<LowState_rl>"
  (cl:format cl:nil "unitree_legged_msgs/IMU imu~%unitree_legged_msgs/MotorState[20] motorState~%userValue_msg userValue~%int32 userCmd~%================================================================================~%MSG: unitree_legged_msgs/IMU~%float32[4] quaternion~%float32[3] gyroscope~%float32[3] accelerometer~%int8 temperature~%================================================================================~%MSG: unitree_legged_msgs/MotorState~%uint8 mode           # motor current mode ~%float32 q            # motor current position（rad）~%float32 dq           # motor current speed（rad/s）~%float32 ddq          # motor current speed（rad/s）~%float32 tauEst       # current estimated output torque（N*m）~%float32 q_raw        # motor current position（rad）~%float32 dq_raw       # motor current speed（rad/s）~%float32 ddq_raw      # motor current speed（rad/s）~%int8 temperature     # motor temperature（slow conduction of temperature leads to lag）~%uint32[2] reserve~%================================================================================~%MSG: rl_l2gar/userValue_msg~%float64 lx~%float64 ly~%float64 rx~%float64 ry~%float64 L2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LowState_rl)))
  "Returns full string definition for message of type 'LowState_rl"
  (cl:format cl:nil "unitree_legged_msgs/IMU imu~%unitree_legged_msgs/MotorState[20] motorState~%userValue_msg userValue~%int32 userCmd~%================================================================================~%MSG: unitree_legged_msgs/IMU~%float32[4] quaternion~%float32[3] gyroscope~%float32[3] accelerometer~%int8 temperature~%================================================================================~%MSG: unitree_legged_msgs/MotorState~%uint8 mode           # motor current mode ~%float32 q            # motor current position（rad）~%float32 dq           # motor current speed（rad/s）~%float32 ddq          # motor current speed（rad/s）~%float32 tauEst       # current estimated output torque（N*m）~%float32 q_raw        # motor current position（rad）~%float32 dq_raw       # motor current speed（rad/s）~%float32 ddq_raw      # motor current speed（rad/s）~%int8 temperature     # motor temperature（slow conduction of temperature leads to lag）~%uint32[2] reserve~%================================================================================~%MSG: rl_l2gar/userValue_msg~%float64 lx~%float64 ly~%float64 rx~%float64 ry~%float64 L2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LowState_rl>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motorState) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'userValue))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LowState_rl>))
  "Converts a ROS message object to a list"
  (cl:list 'LowState_rl
    (cl:cons ':imu (imu msg))
    (cl:cons ':motorState (motorState msg))
    (cl:cons ':userValue (userValue msg))
    (cl:cons ':userCmd (userCmd msg))
))
