; Auto-generated. Do not edit!


(in-package roboteq-msg)


;//! \htmlinclude config.msg.html

(defclass <config> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (MotorAmps1
    :reader MotorAmps1-val
    :initarg :MotorAmps1
    :type integer
    :initform 0)
   (MotorAmps2
    :reader MotorAmps2-val
    :initarg :MotorAmps2
    :type integer
    :initform 0)
   (BatteryAmps1
    :reader BatteryAmps1-val
    :initarg :BatteryAmps1
    :type integer
    :initform 0)
   (BatteryAmps2
    :reader BatteryAmps2-val
    :initarg :BatteryAmps2
    :type integer
    :initform 0)
   (motorpower1
    :reader motorpower1-val
    :initarg :motorpower1
    :type integer
    :initform 0)
   (motorpower2
    :reader motorpower2-val
    :initarg :motorpower2
    :type integer
    :initform 0)
   (drivervoltage
    :reader drivervoltage-val
    :initarg :drivervoltage
    :type integer
    :initform 0)
   (BatteryVoltage
    :reader BatteryVoltage-val
    :initarg :BatteryVoltage
    :type integer
    :initform 0)
   (DSUBvoltage
    :reader DSUBvoltage-val
    :initarg :DSUBvoltage
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <config>) ostream)
  "Serializes a message object of type '<config>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'MotorAmps1)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'MotorAmps1)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'MotorAmps1)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'MotorAmps1)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'MotorAmps2)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'MotorAmps2)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'MotorAmps2)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'MotorAmps2)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'BatteryAmps1)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'BatteryAmps1)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'BatteryAmps1)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'BatteryAmps1)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'BatteryAmps2)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'BatteryAmps2)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'BatteryAmps2)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'BatteryAmps2)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'motorpower1)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'motorpower1)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'motorpower1)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'motorpower1)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'motorpower2)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'motorpower2)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'motorpower2)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'motorpower2)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'drivervoltage)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'drivervoltage)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'drivervoltage)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'drivervoltage)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'BatteryVoltage)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'BatteryVoltage)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'BatteryVoltage)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'BatteryVoltage)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'DSUBvoltage)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'DSUBvoltage)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'DSUBvoltage)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'DSUBvoltage)) ostream)
)
(defmethod deserialize ((msg <config>) istream)
  "Deserializes a message object of type '<config>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'MotorAmps1)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'MotorAmps1)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'MotorAmps1)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'MotorAmps1)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'MotorAmps2)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'MotorAmps2)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'MotorAmps2)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'MotorAmps2)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'BatteryAmps1)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'BatteryAmps1)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'BatteryAmps1)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'BatteryAmps1)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'BatteryAmps2)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'BatteryAmps2)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'BatteryAmps2)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'BatteryAmps2)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'motorpower1)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'motorpower1)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'motorpower1)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'motorpower1)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'motorpower2)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'motorpower2)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'motorpower2)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'motorpower2)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'drivervoltage)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'drivervoltage)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'drivervoltage)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'drivervoltage)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'BatteryVoltage)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'BatteryVoltage)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'BatteryVoltage)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'BatteryVoltage)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'DSUBvoltage)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'DSUBvoltage)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'DSUBvoltage)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'DSUBvoltage)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<config>)))
  "Returns string type for a message object of type '<config>"
  "roboteq/config")
(defmethod md5sum ((type (eql '<config>)))
  "Returns md5sum for a message object of type '<config>"
  "8886c8836227fe18873a3b2abc061efe")
(defmethod message-definition ((type (eql '<config>)))
  "Returns full string definition for message of type '<config>"
  (format nil "Header header~%int32 MotorAmps1~%int32 MotorAmps2~%int32 BatteryAmps1~%int32 BatteryAmps2~%int32 motorpower1~%int32 motorpower2~%int32 drivervoltage~%int32 BatteryVoltage~%int32 DSUBvoltage~%~%~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <config>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <config>))
  "Converts a ROS message object to a list"
  (list '<config>
    (cons ':header (header-val msg))
    (cons ':MotorAmps1 (MotorAmps1-val msg))
    (cons ':MotorAmps2 (MotorAmps2-val msg))
    (cons ':BatteryAmps1 (BatteryAmps1-val msg))
    (cons ':BatteryAmps2 (BatteryAmps2-val msg))
    (cons ':motorpower1 (motorpower1-val msg))
    (cons ':motorpower2 (motorpower2-val msg))
    (cons ':drivervoltage (drivervoltage-val msg))
    (cons ':BatteryVoltage (BatteryVoltage-val msg))
    (cons ':DSUBvoltage (DSUBvoltage-val msg))
))
