;; Auto-generated. Do not edit!


(when (boundp 'rover_msgs::roverBmsStatus)
  (if (not (find-package "ROVER_MSGS"))
    (make-package "ROVER_MSGS"))
  (shadow 'roverBmsStatus (find-package "ROVER_MSGS")))
(unless (find-package "ROVER_MSGS::ROVERBMSSTATUS")
  (make-package "ROVER_MSGS::ROVERBMSSTATUS"))

(in-package "ROS")
;;//! \htmlinclude roverBmsStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rover_msgs::roverBmsStatus
  :super ros::object
  :slots (_header _percentage _voltage _current _temperature ))

(defmethod rover_msgs::roverBmsStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:percentage __percentage) 0)
    ((:voltage __voltage) 0.0)
    ((:current __current) 0.0)
    ((:temperature __temperature) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _percentage (round __percentage))
   (setq _voltage (float __voltage))
   (setq _current (float __current))
   (setq _temperature (float __temperature))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:percentage
   (&optional __percentage)
   (if __percentage (setq _percentage __percentage)) _percentage)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; char _percentage
    1
    ;; float64 _voltage
    8
    ;; float64 _current
    8
    ;; float64 _temperature
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; char _percentage
       (write-byte _percentage s)
     ;; float64 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _current
       (sys::poke _current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; char _percentage
     (setq _percentage (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _voltage
     (setq _voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _current
     (setq _current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _temperature
     (setq _temperature (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rover_msgs::roverBmsStatus :md5sum-) "4e98861cb0e01a28a1d9239e1cc17878")
(setf (get rover_msgs::roverBmsStatus :datatype-) "rover_msgs/roverBmsStatus")
(setf (get rover_msgs::roverBmsStatus :definition-)
      "Header header

char percentage
float64 voltage
float64 current
float64 temperature
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



(provide :rover_msgs/roverBmsStatus "4e98861cb0e01a28a1d9239e1cc17878")


