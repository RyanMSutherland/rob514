; Auto-generated. Do not edit!


(cl:in-package rob599_basic-msg)


;//! \htmlinclude Rectangle.msg.html

(cl:defclass <Rectangle> (roslisp-msg-protocol:ros-message)
  ((height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0))
)

(cl:defclass Rectangle (<Rectangle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rectangle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rectangle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rob599_basic-msg:<Rectangle> is deprecated: use rob599_basic-msg:Rectangle instead.")))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <Rectangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rob599_basic-msg:height-val is deprecated.  Use rob599_basic-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <Rectangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rob599_basic-msg:width-val is deprecated.  Use rob599_basic-msg:width instead.")
  (width m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rectangle>) ostream)
  "Serializes a message object of type '<Rectangle>"
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rectangle>) istream)
  "Deserializes a message object of type '<Rectangle>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rectangle>)))
  "Returns string type for a message object of type '<Rectangle>"
  "rob599_basic/Rectangle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rectangle)))
  "Returns string type for a message object of type 'Rectangle"
  "rob599_basic/Rectangle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rectangle>)))
  "Returns md5sum for a message object of type '<Rectangle>"
  "f2cea01c6068f229f4d26f3c1c2bd39c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rectangle)))
  "Returns md5sum for a message object of type 'Rectangle"
  "f2cea01c6068f229f4d26f3c1c2bd39c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rectangle>)))
  "Returns full string definition for message of type '<Rectangle>"
  (cl:format cl:nil "int64 height~%int64 width~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rectangle)))
  "Returns full string definition for message of type 'Rectangle"
  (cl:format cl:nil "int64 height~%int64 width~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rectangle>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rectangle>))
  "Converts a ROS message object to a list"
  (cl:list 'Rectangle
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
))
