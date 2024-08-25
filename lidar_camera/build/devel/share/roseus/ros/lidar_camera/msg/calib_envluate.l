;; Auto-generated. Do not edit!


(when (boundp 'lidar_camera::calib_envluate)
  (if (not (find-package "LIDAR_CAMERA"))
    (make-package "LIDAR_CAMERA"))
  (shadow 'calib_envluate (find-package "LIDAR_CAMERA")))
(unless (find-package "LIDAR_CAMERA::CALIB_ENVLUATE")
  (make-package "LIDAR_CAMERA::CALIB_ENVLUATE"))

(in-package "ROS")
;;//! \htmlinclude calib_envluate.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lidar_camera::calib_envluate
  :super ros::object
  :slots (_header _ave_deviation_z _ave_horizontal_reprojecterr _ave_vertical_reprojecterr ))

(defmethod lidar_camera::calib_envluate
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ave_deviation_z __ave_deviation_z) 0.0)
    ((:ave_horizontal_reprojecterr __ave_horizontal_reprojecterr) 0.0)
    ((:ave_vertical_reprojecterr __ave_vertical_reprojecterr) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ave_deviation_z (float __ave_deviation_z))
   (setq _ave_horizontal_reprojecterr (float __ave_horizontal_reprojecterr))
   (setq _ave_vertical_reprojecterr (float __ave_vertical_reprojecterr))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ave_deviation_z
   (&optional __ave_deviation_z)
   (if __ave_deviation_z (setq _ave_deviation_z __ave_deviation_z)) _ave_deviation_z)
  (:ave_horizontal_reprojecterr
   (&optional __ave_horizontal_reprojecterr)
   (if __ave_horizontal_reprojecterr (setq _ave_horizontal_reprojecterr __ave_horizontal_reprojecterr)) _ave_horizontal_reprojecterr)
  (:ave_vertical_reprojecterr
   (&optional __ave_vertical_reprojecterr)
   (if __ave_vertical_reprojecterr (setq _ave_vertical_reprojecterr __ave_vertical_reprojecterr)) _ave_vertical_reprojecterr)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _ave_deviation_z
    4
    ;; float32 _ave_horizontal_reprojecterr
    4
    ;; float32 _ave_vertical_reprojecterr
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _ave_deviation_z
       (sys::poke _ave_deviation_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ave_horizontal_reprojecterr
       (sys::poke _ave_horizontal_reprojecterr (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ave_vertical_reprojecterr
       (sys::poke _ave_vertical_reprojecterr (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _ave_deviation_z
     (setq _ave_deviation_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ave_horizontal_reprojecterr
     (setq _ave_horizontal_reprojecterr (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ave_vertical_reprojecterr
     (setq _ave_vertical_reprojecterr (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get lidar_camera::calib_envluate :md5sum-) "1be80c75fd6a554f8f54217e7f0d9349")
(setf (get lidar_camera::calib_envluate :datatype-) "lidar_camera/calib_envluate")
(setf (get lidar_camera::calib_envluate :definition-)
      "Header header

float32 ave_deviation_z
float32 ave_horizontal_reprojecterr
float32 ave_vertical_reprojecterr
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
# 0: no frame
# 1: global frame
string frame_id

")



(provide :lidar_camera/calib_envluate "1be80c75fd6a554f8f54217e7f0d9349")


