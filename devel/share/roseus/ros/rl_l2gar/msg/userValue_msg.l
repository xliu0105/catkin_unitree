;; Auto-generated. Do not edit!


(when (boundp 'rl_l2gar::userValue_msg)
  (if (not (find-package "RL_L2GAR"))
    (make-package "RL_L2GAR"))
  (shadow 'userValue_msg (find-package "RL_L2GAR")))
(unless (find-package "RL_L2GAR::USERVALUE_MSG")
  (make-package "RL_L2GAR::USERVALUE_MSG"))

(in-package "ROS")
;;//! \htmlinclude userValue_msg.msg.html


(defclass rl_l2gar::userValue_msg
  :super ros::object
  :slots (_lx _ly _rx _ry _L2 ))

(defmethod rl_l2gar::userValue_msg
  (:init
   (&key
    ((:lx __lx) 0.0)
    ((:ly __ly) 0.0)
    ((:rx __rx) 0.0)
    ((:ry __ry) 0.0)
    ((:L2 __L2) 0.0)
    )
   (send-super :init)
   (setq _lx (float __lx))
   (setq _ly (float __ly))
   (setq _rx (float __rx))
   (setq _ry (float __ry))
   (setq _L2 (float __L2))
   self)
  (:lx
   (&optional __lx)
   (if __lx (setq _lx __lx)) _lx)
  (:ly
   (&optional __ly)
   (if __ly (setq _ly __ly)) _ly)
  (:rx
   (&optional __rx)
   (if __rx (setq _rx __rx)) _rx)
  (:ry
   (&optional __ry)
   (if __ry (setq _ry __ry)) _ry)
  (:L2
   (&optional __L2)
   (if __L2 (setq _L2 __L2)) _L2)
  (:serialization-length
   ()
   (+
    ;; float64 _lx
    8
    ;; float64 _ly
    8
    ;; float64 _rx
    8
    ;; float64 _ry
    8
    ;; float64 _L2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _lx
       (sys::poke _lx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ly
       (sys::poke _ly (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rx
       (sys::poke _rx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ry
       (sys::poke _ry (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _L2
       (sys::poke _L2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _lx
     (setq _lx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ly
     (setq _ly (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rx
     (setq _rx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ry
     (setq _ry (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _L2
     (setq _L2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rl_l2gar::userValue_msg :md5sum-) "c5f52d6674a24214f88c039b8afc2bfa")
(setf (get rl_l2gar::userValue_msg :datatype-) "rl_l2gar/userValue_msg")
(setf (get rl_l2gar::userValue_msg :definition-)
      "float64 lx
float64 ly
float64 rx
float64 ry
float64 L2
")



(provide :rl_l2gar/userValue_msg "c5f52d6674a24214f88c039b8afc2bfa")


