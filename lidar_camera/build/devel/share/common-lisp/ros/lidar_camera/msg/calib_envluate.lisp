; Auto-generated. Do not edit!


(cl:in-package lidar_camera-msg)


;//! \htmlinclude calib_envluate.msg.html

(cl:defclass <calib_envluate> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ave_deviation_z
    :reader ave_deviation_z
    :initarg :ave_deviation_z
    :type cl:float
    :initform 0.0)
   (ave_horizontal_reprojecterr
    :reader ave_horizontal_reprojecterr
    :initarg :ave_horizontal_reprojecterr
    :type cl:float
    :initform 0.0)
   (ave_vertical_reprojecterr
    :reader ave_vertical_reprojecterr
    :initarg :ave_vertical_reprojecterr
    :type cl:float
    :initform 0.0))
)

(cl:defclass calib_envluate (<calib_envluate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <calib_envluate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'calib_envluate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_camera-msg:<calib_envluate> is deprecated: use lidar_camera-msg:calib_envluate instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <calib_envluate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_camera-msg:header-val is deprecated.  Use lidar_camera-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ave_deviation_z-val :lambda-list '(m))
(cl:defmethod ave_deviation_z-val ((m <calib_envluate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_camera-msg:ave_deviation_z-val is deprecated.  Use lidar_camera-msg:ave_deviation_z instead.")
  (ave_deviation_z m))

(cl:ensure-generic-function 'ave_horizontal_reprojecterr-val :lambda-list '(m))
(cl:defmethod ave_horizontal_reprojecterr-val ((m <calib_envluate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_camera-msg:ave_horizontal_reprojecterr-val is deprecated.  Use lidar_camera-msg:ave_horizontal_reprojecterr instead.")
  (ave_horizontal_reprojecterr m))

(cl:ensure-generic-function 'ave_vertical_reprojecterr-val :lambda-list '(m))
(cl:defmethod ave_vertical_reprojecterr-val ((m <calib_envluate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_camera-msg:ave_vertical_reprojecterr-val is deprecated.  Use lidar_camera-msg:ave_vertical_reprojecterr instead.")
  (ave_vertical_reprojecterr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <calib_envluate>) ostream)
  "Serializes a message object of type '<calib_envluate>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ave_deviation_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ave_horizontal_reprojecterr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ave_vertical_reprojecterr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <calib_envluate>) istream)
  "Deserializes a message object of type '<calib_envluate>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ave_deviation_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ave_horizontal_reprojecterr) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ave_vertical_reprojecterr) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<calib_envluate>)))
  "Returns string type for a message object of type '<calib_envluate>"
  "lidar_camera/calib_envluate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calib_envluate)))
  "Returns string type for a message object of type 'calib_envluate"
  "lidar_camera/calib_envluate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<calib_envluate>)))
  "Returns md5sum for a message object of type '<calib_envluate>"
  "1be80c75fd6a554f8f54217e7f0d9349")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'calib_envluate)))
  "Returns md5sum for a message object of type 'calib_envluate"
  "1be80c75fd6a554f8f54217e7f0d9349")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<calib_envluate>)))
  "Returns full string definition for message of type '<calib_envluate>"
  (cl:format cl:nil "Header header~%~%float32 ave_deviation_z~%float32 ave_horizontal_reprojecterr~%float32 ave_vertical_reprojecterr~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'calib_envluate)))
  "Returns full string definition for message of type 'calib_envluate"
  (cl:format cl:nil "Header header~%~%float32 ave_deviation_z~%float32 ave_horizontal_reprojecterr~%float32 ave_vertical_reprojecterr~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <calib_envluate>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <calib_envluate>))
  "Converts a ROS message object to a list"
  (cl:list 'calib_envluate
    (cl:cons ':header (header msg))
    (cl:cons ':ave_deviation_z (ave_deviation_z msg))
    (cl:cons ':ave_horizontal_reprojecterr (ave_horizontal_reprojecterr msg))
    (cl:cons ':ave_vertical_reprojecterr (ave_vertical_reprojecterr msg))
))
