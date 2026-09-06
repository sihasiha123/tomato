
(cl:in-package :asdf)

(defsystem "rover_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RoverBmsStatus" :depends-on ("_package_RoverBmsStatus"))
    (:file "_package_RoverBmsStatus" :depends-on ("_package"))
    (:file "RoverGoalStatus" :depends-on ("_package_RoverGoalStatus"))
    (:file "_package_RoverGoalStatus" :depends-on ("_package"))
    (:file "RoverRCStatus" :depends-on ("_package_RoverRCStatus"))
    (:file "_package_RoverRCStatus" :depends-on ("_package"))
    (:file "roverBmsStatus" :depends-on ("_package_roverBmsStatus"))
    (:file "_package_roverBmsStatus" :depends-on ("_package"))
    (:file "roverGoalStatus" :depends-on ("_package_roverGoalStatus"))
    (:file "_package_roverGoalStatus" :depends-on ("_package"))
    (:file "roverRCStatus" :depends-on ("_package_roverRCStatus"))
    (:file "_package_roverRCStatus" :depends-on ("_package"))
  ))