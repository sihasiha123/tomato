; Auto-generated. Do not edit!


(cl:in-package rover_msgs-msg)


;//! \htmlinclude roverRCStatus.msg.html

(cl:defclass <roverRCStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (connect
    :reader connect
    :initarg :connect
    :type cl:boolean
    :initform cl:nil)
   (CH1
    :reader CH1
    :initarg :CH1
    :type cl:fixnum
    :initform 0)
   (CH2
    :reader CH2
    :initarg :CH2
    :type cl:fixnum
    :initform 0)
   (CH3
    :reader CH3
    :initarg :CH3
    :type cl:fixnum
    :initform 0)
   (CH4
    :reader CH4
    :initarg :CH4
    :type cl:fixnum
    :initform 0)
   (CH5
    :reader CH5
    :initarg :CH5
    :type cl:fixnum
    :initform 0)
   (CH6
    :reader CH6
    :initarg :CH6
    :type cl:fixnum
    :initform 0)
   (CH7
    :reader CH7
    :initarg :CH7
    :type cl:fixnum
    :initform 0)
   (CH8
    :reader CH8
    :initarg :CH8
    :type cl:fixnum
    :initform 0)
   (CH9
    :reader CH9
    :initarg :CH9
    :type cl:fixnum
    :initform 0)
   (CH10
    :reader CH10
    :initarg :CH10
    :type cl:fixnum
    :initform 0))
)

(cl:defclass roverRCStatus (<roverRCStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <roverRCStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'roverRCStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_msgs-msg:<roverRCStatus> is deprecated: use rover_msgs-msg:roverRCStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:header-val is deprecated.  Use rover_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'connect-val :lambda-list '(m))
(cl:defmethod connect-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:connect-val is deprecated.  Use rover_msgs-msg:connect instead.")
  (connect m))

(cl:ensure-generic-function 'CH1-val :lambda-list '(m))
(cl:defmethod CH1-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH1-val is deprecated.  Use rover_msgs-msg:CH1 instead.")
  (CH1 m))

(cl:ensure-generic-function 'CH2-val :lambda-list '(m))
(cl:defmethod CH2-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH2-val is deprecated.  Use rover_msgs-msg:CH2 instead.")
  (CH2 m))

(cl:ensure-generic-function 'CH3-val :lambda-list '(m))
(cl:defmethod CH3-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH3-val is deprecated.  Use rover_msgs-msg:CH3 instead.")
  (CH3 m))

(cl:ensure-generic-function 'CH4-val :lambda-list '(m))
(cl:defmethod CH4-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH4-val is deprecated.  Use rover_msgs-msg:CH4 instead.")
  (CH4 m))

(cl:ensure-generic-function 'CH5-val :lambda-list '(m))
(cl:defmethod CH5-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH5-val is deprecated.  Use rover_msgs-msg:CH5 instead.")
  (CH5 m))

(cl:ensure-generic-function 'CH6-val :lambda-list '(m))
(cl:defmethod CH6-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH6-val is deprecated.  Use rover_msgs-msg:CH6 instead.")
  (CH6 m))

(cl:ensure-generic-function 'CH7-val :lambda-list '(m))
(cl:defmethod CH7-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH7-val is deprecated.  Use rover_msgs-msg:CH7 instead.")
  (CH7 m))

(cl:ensure-generic-function 'CH8-val :lambda-list '(m))
(cl:defmethod CH8-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH8-val is deprecated.  Use rover_msgs-msg:CH8 instead.")
  (CH8 m))

(cl:ensure-generic-function 'CH9-val :lambda-list '(m))
(cl:defmethod CH9-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH9-val is deprecated.  Use rover_msgs-msg:CH9 instead.")
  (CH9 m))

(cl:ensure-generic-function 'CH10-val :lambda-list '(m))
(cl:defmethod CH10-val ((m <roverRCStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:CH10-val is deprecated.  Use rover_msgs-msg:CH10 instead.")
  (CH10 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <roverRCStatus>) ostream)
  "Serializes a message object of type '<roverRCStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'connect) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'CH1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'CH2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'CH3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'CH4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'CH5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'CH6)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'CH7)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'CH8)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'CH9)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'CH10)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <roverRCStatus>) istream)
  "Deserializes a message object of type '<roverRCStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'connect) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH1) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH2) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH3) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH4) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH5) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH6) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH7) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH8) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH9) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'CH10) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<roverRCStatus>)))
  "Returns string type for a message object of type '<roverRCStatus>"
  "rover_msgs/roverRCStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'roverRCStatus)))
  "Returns string type for a message object of type 'roverRCStatus"
  "rover_msgs/roverRCStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<roverRCStatus>)))
  "Returns md5sum for a message object of type '<roverRCStatus>"
  "c06b6cb58b0a81562a8993101c944318")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'roverRCStatus)))
  "Returns md5sum for a message object of type 'roverRCStatus"
  "c06b6cb58b0a81562a8993101c944318")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<roverRCStatus>)))
  "Returns full string definition for message of type '<roverRCStatus>"
  (cl:format cl:nil "Header header~%~%bool connect~%int8 CH1~%int8 CH2~%int8 CH3~%int8 CH4~%int8 CH5~%int8 CH6~%int8 CH7~%int8 CH8~%int8 CH9~%int8 CH10~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'roverRCStatus)))
  "Returns full string definition for message of type 'roverRCStatus"
  (cl:format cl:nil "Header header~%~%bool connect~%int8 CH1~%int8 CH2~%int8 CH3~%int8 CH4~%int8 CH5~%int8 CH6~%int8 CH7~%int8 CH8~%int8 CH9~%int8 CH10~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <roverRCStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <roverRCStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'roverRCStatus
    (cl:cons ':header (header msg))
    (cl:cons ':connect (connect msg))
    (cl:cons ':CH1 (CH1 msg))
    (cl:cons ':CH2 (CH2 msg))
    (cl:cons ':CH3 (CH3 msg))
    (cl:cons ':CH4 (CH4 msg))
    (cl:cons ':CH5 (CH5 msg))
    (cl:cons ':CH6 (CH6 msg))
    (cl:cons ':CH7 (CH7 msg))
    (cl:cons ':CH8 (CH8 msg))
    (cl:cons ':CH9 (CH9 msg))
    (cl:cons ':CH10 (CH10 msg))
))
