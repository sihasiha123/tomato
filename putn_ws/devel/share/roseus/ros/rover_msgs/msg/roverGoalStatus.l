;; Auto-generated. Do not edit!


(when (boundp 'rover_msgs::roverGoalStatus)
  (if (not (find-package "ROVER_MSGS"))
    (make-package "ROVER_MSGS"))
  (shadow 'roverGoalStatus (find-package "ROVER_MSGS")))
(unless (find-package "ROVER_MSGS::ROVERGOALSTATUS")
  (make-package "ROVER_MSGS::ROVERGOALSTATUS"))

(in-package "ROS")
;;//! \htmlinclude roverGoalStatus.msg.html


(intern "*PENDING*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*PENDING* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*PENDING* 0)
(intern "*ACTIVE*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*ACTIVE* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*ACTIVE* 1)
(intern "*PREEMPTED*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*PREEMPTED* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*PREEMPTED* 2)
(intern "*SUCCEEDED*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*SUCCEEDED* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*SUCCEEDED* 3)
(intern "*ABORTED*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*ABORTED* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*ABORTED* 4)
(intern "*REJECTED*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*REJECTED* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*REJECTED* 5)
(intern "*PREEMPTING*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*PREEMPTING* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*PREEMPTING* 6)
(intern "*RECALLING*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*RECALLING* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*RECALLING* 7)
(intern "*RECALLED*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*RECALLED* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*RECALLED* 8)
(intern "*LOST*" (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(shadow '*LOST* (find-package "ROVER_MSGS::ROVERGOALSTATUS"))
(defconstant rover_msgs::roverGoalStatus::*LOST* 9)

(defun rover_msgs::roverGoalStatus-to-symbol (const)
  (cond
        ((= const 0) 'rover_msgs::roverGoalStatus::*PENDING*)
        ((= const 1) 'rover_msgs::roverGoalStatus::*ACTIVE*)
        ((= const 2) 'rover_msgs::roverGoalStatus::*PREEMPTED*)
        ((= const 3) 'rover_msgs::roverGoalStatus::*SUCCEEDED*)
        ((= const 4) 'rover_msgs::roverGoalStatus::*ABORTED*)
        ((= const 5) 'rover_msgs::roverGoalStatus::*REJECTED*)
        ((= const 6) 'rover_msgs::roverGoalStatus::*PREEMPTING*)
        ((= const 7) 'rover_msgs::roverGoalStatus::*RECALLING*)
        ((= const 8) 'rover_msgs::roverGoalStatus::*RECALLED*)
        ((= const 9) 'rover_msgs::roverGoalStatus::*LOST*)
        (t nil)))

(defclass rover_msgs::roverGoalStatus
  :super ros::object
  :slots (_x _y _z _orientation_x _orientation_y _orientation_z _orientation_w _status _goal_id _text ))

(defmethod rover_msgs::roverGoalStatus
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:orientation_x __orientation_x) 0.0)
    ((:orientation_y __orientation_y) 0.0)
    ((:orientation_z __orientation_z) 0.0)
    ((:orientation_w __orientation_w) 0.0)
    ((:status __status) 0)
    ((:goal_id __goal_id) 0)
    ((:text __text) "")
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _orientation_x (float __orientation_x))
   (setq _orientation_y (float __orientation_y))
   (setq _orientation_z (float __orientation_z))
   (setq _orientation_w (float __orientation_w))
   (setq _status (round __status))
   (setq _goal_id (round __goal_id))
   (setq _text (string __text))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:orientation_x
   (&optional __orientation_x)
   (if __orientation_x (setq _orientation_x __orientation_x)) _orientation_x)
  (:orientation_y
   (&optional __orientation_y)
   (if __orientation_y (setq _orientation_y __orientation_y)) _orientation_y)
  (:orientation_z
   (&optional __orientation_z)
   (if __orientation_z (setq _orientation_z __orientation_z)) _orientation_z)
  (:orientation_w
   (&optional __orientation_w)
   (if __orientation_w (setq _orientation_w __orientation_w)) _orientation_w)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:goal_id
   (&optional __goal_id)
   (if __goal_id (setq _goal_id __goal_id)) _goal_id)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _orientation_x
    8
    ;; float64 _orientation_y
    8
    ;; float64 _orientation_z
    8
    ;; float64 _orientation_w
    8
    ;; uint8 _status
    1
    ;; uint8 _goal_id
    1
    ;; string _text
    4 (length _text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _orientation_x
       (sys::poke _orientation_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _orientation_y
       (sys::poke _orientation_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _orientation_z
       (sys::poke _orientation_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _orientation_w
       (sys::poke _orientation_w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _status
       (write-byte _status s)
     ;; uint8 _goal_id
       (write-byte _goal_id s)
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _orientation_x
     (setq _orientation_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _orientation_y
     (setq _orientation_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _orientation_z
     (setq _orientation_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _orientation_w
     (setq _orientation_w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _goal_id
     (setq _goal_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get rover_msgs::roverGoalStatus :md5sum-) "06fdaffd0dd12907d78dce66dec8b35a")
(setf (get rover_msgs::roverGoalStatus :datatype-) "rover_msgs/roverGoalStatus")
(setf (get rover_msgs::roverGoalStatus :definition-)
      "uint8 PENDING=0
uint8 ACTIVE=1
uint8 PREEMPTED=2
uint8 SUCCEEDED=3
uint8 ABORTED=4
uint8 REJECTED=5
uint8 PREEMPTING=6
uint8 RECALLING=7
uint8 RECALLED=8
uint8 LOST=9


float64 x
float64 y
float64 z
float64 orientation_x
float64 orientation_y
float64 orientation_z
float64 orientation_w
uint8 status
uint8 goal_id
string text
")



(provide :rover_msgs/roverGoalStatus "06fdaffd0dd12907d78dce66dec8b35a")


