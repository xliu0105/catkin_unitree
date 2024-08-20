; Auto-generated. Do not edit!


(cl:in-package rl_l2gar-msg)


;//! \htmlinclude userValue_msg.msg.html

(cl:defclass <userValue_msg> (roslisp-msg-protocol:ros-message)
  ((lx
    :reader lx
    :initarg :lx
    :type cl:float
    :initform 0.0)
   (ly
    :reader ly
    :initarg :ly
    :type cl:float
    :initform 0.0)
   (rx
    :reader rx
    :initarg :rx
    :type cl:float
    :initform 0.0)
   (ry
    :reader ry
    :initarg :ry
    :type cl:float
    :initform 0.0)
   (L2
    :reader L2
    :initarg :L2
    :type cl:float
    :initform 0.0))
)

(cl:defclass userValue_msg (<userValue_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <userValue_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'userValue_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_l2gar-msg:<userValue_msg> is deprecated: use rl_l2gar-msg:userValue_msg instead.")))

(cl:ensure-generic-function 'lx-val :lambda-list '(m))
(cl:defmethod lx-val ((m <userValue_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_l2gar-msg:lx-val is deprecated.  Use rl_l2gar-msg:lx instead.")
  (lx m))

(cl:ensure-generic-function 'ly-val :lambda-list '(m))
(cl:defmethod ly-val ((m <userValue_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_l2gar-msg:ly-val is deprecated.  Use rl_l2gar-msg:ly instead.")
  (ly m))

(cl:ensure-generic-function 'rx-val :lambda-list '(m))
(cl:defmethod rx-val ((m <userValue_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_l2gar-msg:rx-val is deprecated.  Use rl_l2gar-msg:rx instead.")
  (rx m))

(cl:ensure-generic-function 'ry-val :lambda-list '(m))
(cl:defmethod ry-val ((m <userValue_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_l2gar-msg:ry-val is deprecated.  Use rl_l2gar-msg:ry instead.")
  (ry m))

(cl:ensure-generic-function 'L2-val :lambda-list '(m))
(cl:defmethod L2-val ((m <userValue_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_l2gar-msg:L2-val is deprecated.  Use rl_l2gar-msg:L2 instead.")
  (L2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <userValue_msg>) ostream)
  "Serializes a message object of type '<userValue_msg>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ly))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'L2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <userValue_msg>) istream)
  "Deserializes a message object of type '<userValue_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ly) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ry) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'L2) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<userValue_msg>)))
  "Returns string type for a message object of type '<userValue_msg>"
  "rl_l2gar/userValue_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'userValue_msg)))
  "Returns string type for a message object of type 'userValue_msg"
  "rl_l2gar/userValue_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<userValue_msg>)))
  "Returns md5sum for a message object of type '<userValue_msg>"
  "c5f52d6674a24214f88c039b8afc2bfa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'userValue_msg)))
  "Returns md5sum for a message object of type 'userValue_msg"
  "c5f52d6674a24214f88c039b8afc2bfa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<userValue_msg>)))
  "Returns full string definition for message of type '<userValue_msg>"
  (cl:format cl:nil "float64 lx~%float64 ly~%float64 rx~%float64 ry~%float64 L2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'userValue_msg)))
  "Returns full string definition for message of type 'userValue_msg"
  (cl:format cl:nil "float64 lx~%float64 ly~%float64 rx~%float64 ry~%float64 L2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <userValue_msg>))
  (cl:+ 0
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <userValue_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'userValue_msg
    (cl:cons ':lx (lx msg))
    (cl:cons ':ly (ly msg))
    (cl:cons ':rx (rx msg))
    (cl:cons ':ry (ry msg))
    (cl:cons ':L2 (L2 msg))
))
