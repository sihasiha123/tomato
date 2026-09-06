;; Auto-generated. Do not edit!


(when (boundp 'rover_msgs::roverRCStatus)
  (if (not (find-package "ROVER_MSGS"))
    (make-package "ROVER_MSGS"))
  (shadow 'roverRCStatus (find-package "ROVER_MSGS")))
(unless (find-package "ROVER_MSGS::ROVERRCSTATUS")
  (make-package "ROVER_MSGS::ROVERRCSTATUS"))

(in-package "ROS")
;;//! \htmlinclude roverRCStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rover_msgs::roverRCStatus
  :super ros::object
  :slots (_header _connect _CH1 _CH2 _CH3 _CH4 _CH5 _CH6 _CH7 _CH8 _CH9 _CH10 ))

(defmethod rover_msgs::roverRCStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:connect __connect) nil)
    ((:CH1 __CH1) 0)
    ((:CH2 __CH2) 0)
    ((:CH3 __CH3) 0)
    ((:CH4 __CH4) 0)
    ((:CH5 __CH5) 0)
    ((:CH6 __CH6) 0)
    ((:CH7 __CH7) 0)
    ((:CH8 __CH8) 0)
    ((:CH9 __CH9) 0)
    ((:CH10 __CH10) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _connect __connect)
   (setq _CH1 (round __CH1))
   (setq _CH2 (round __CH2))
   (setq _CH3 (round __CH3))
   (setq _CH4 (round __CH4))
   (setq _CH5 (round __CH5))
   (setq _CH6 (round __CH6))
   (setq _CH7 (round __CH7))
   (setq _CH8 (round __CH8))
   (setq _CH9 (round __CH9))
   (setq _CH10 (round __CH10))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:connect
   (&optional (__connect :null))
   (if (not (eq __connect :null)) (setq _connect __connect)) _connect)
  (:CH1
   (&optional __CH1)
   (if __CH1 (setq _CH1 __CH1)) _CH1)
  (:CH2
   (&optional __CH2)
   (if __CH2 (setq _CH2 __CH2)) _CH2)
  (:CH3
   (&optional __CH3)
   (if __CH3 (setq _CH3 __CH3)) _CH3)
  (:CH4
   (&optional __CH4)
   (if __CH4 (setq _CH4 __CH4)) _CH4)
  (:CH5
   (&optional __CH5)
   (if __CH5 (setq _CH5 __CH5)) _CH5)
  (:CH6
   (&optional __CH6)
   (if __CH6 (setq _CH6 __CH6)) _CH6)
  (:CH7
   (&optional __CH7)
   (if __CH7 (setq _CH7 __CH7)) _CH7)
  (:CH8
   (&optional __CH8)
   (if __CH8 (setq _CH8 __CH8)) _CH8)
  (:CH9
   (&optional __CH9)
   (if __CH9 (setq _CH9 __CH9)) _CH9)
  (:CH10
   (&optional __CH10)
   (if __CH10 (setq _CH10 __CH10)) _CH10)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _connect
    1
    ;; int8 _CH1
    1
    ;; int8 _CH2
    1
    ;; int8 _CH3
    1
    ;; int8 _CH4
    1
    ;; int8 _CH5
    1
    ;; int8 _CH6
    1
    ;; int8 _CH7
    1
    ;; int8 _CH8
    1
    ;; int8 _CH9
    1
    ;; int8 _CH10
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _connect
       (if _connect (write-byte -1 s) (write-byte 0 s))
     ;; int8 _CH1
       (write-byte _CH1 s)
     ;; int8 _CH2
       (write-byte _CH2 s)
     ;; int8 _CH3
       (write-byte _CH3 s)
     ;; int8 _CH4
       (write-byte _CH4 s)
     ;; int8 _CH5
       (write-byte _CH5 s)
     ;; int8 _CH6
       (write-byte _CH6 s)
     ;; int8 _CH7
       (write-byte _CH7 s)
     ;; int8 _CH8
       (write-byte _CH8 s)
     ;; int8 _CH9
       (write-byte _CH9 s)
     ;; int8 _CH10
       (write-byte _CH10 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _connect
     (setq _connect (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _CH1
     (setq _CH1 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH1 127) (setq _CH1 (- _CH1 256)))
   ;; int8 _CH2
     (setq _CH2 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH2 127) (setq _CH2 (- _CH2 256)))
   ;; int8 _CH3
     (setq _CH3 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH3 127) (setq _CH3 (- _CH3 256)))
   ;; int8 _CH4
     (setq _CH4 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH4 127) (setq _CH4 (- _CH4 256)))
   ;; int8 _CH5
     (setq _CH5 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH5 127) (setq _CH5 (- _CH5 256)))
   ;; int8 _CH6
     (setq _CH6 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH6 127) (setq _CH6 (- _CH6 256)))
   ;; int8 _CH7
     (setq _CH7 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH7 127) (setq _CH7 (- _CH7 256)))
   ;; int8 _CH8
     (setq _CH8 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH8 127) (setq _CH8 (- _CH8 256)))
   ;; int8 _CH9
     (setq _CH9 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH9 127) (setq _CH9 (- _CH9 256)))
   ;; int8 _CH10
     (setq _CH10 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _CH10 127) (setq _CH10 (- _CH10 256)))
   ;;
   self)
  )

(setf (get rover_msgs::roverRCStatus :md5sum-) "c06b6cb58b0a81562a8993101c944318")
(setf (get rover_msgs::roverRCStatus :datatype-) "rover_msgs/roverRCStatus")
(setf (get rover_msgs::roverRCStatus :definition-)
      "Header header

bool connect
int8 CH1
int8 CH2
int8 CH3
int8 CH4
int8 CH5
int8 CH6
int8 CH7
int8 CH8
int8 CH9
int8 CH10
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :rover_msgs/roverRCStatus "c06b6cb58b0a81562a8993101c944318")


