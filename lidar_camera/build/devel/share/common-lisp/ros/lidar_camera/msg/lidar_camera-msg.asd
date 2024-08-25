
(cl:in-package :asdf)

(defsystem "lidar_camera-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "calib_envluate" :depends-on ("_package_calib_envluate"))
    (:file "_package_calib_envluate" :depends-on ("_package"))
  ))