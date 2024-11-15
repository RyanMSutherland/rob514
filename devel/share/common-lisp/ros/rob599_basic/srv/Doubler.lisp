; Auto-generated. Do not edit!


(cl:in-package rob599_basic-srv)


;//! \htmlinclude Doubler-request.msg.html

(cl:defclass <Doubler-request> (roslisp-msg-protocol:ros-message)
  ((number
    :reader number
    :initarg :number
    :type cl:integer
    :initform 0))
)

(cl:defclass Doubler-request (<Doubler-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Doubler-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Doubler-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rob599_basic-srv:<Doubler-request> is deprecated: use rob599_basic-srv:Doubler-request instead.")))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <Doubler-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rob599_basic-srv:number-val is deprecated.  Use rob599_basic-srv:number instead.")
  (number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Doubler-request>) ostream)
  "Serializes a message object of type '<Doubler-request>"
  (cl:let* ((signed (cl:slot-value msg 'number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Doubler-request>) istream)
  "Deserializes a message object of type '<Doubler-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Doubler-request>)))
  "Returns string type for a service object of type '<Doubler-request>"
  "rob599_basic/DoublerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Doubler-request)))
  "Returns string type for a service object of type 'Doubler-request"
  "rob599_basic/DoublerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Doubler-request>)))
  "Returns md5sum for a message object of type '<Doubler-request>"
  "cd11c21fe194ec0173733fddd2cd457b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Doubler-request)))
  "Returns md5sum for a message object of type 'Doubler-request"
  "cd11c21fe194ec0173733fddd2cd457b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Doubler-request>)))
  "Returns full string definition for message of type '<Doubler-request>"
  (cl:format cl:nil "# inputs~%int64 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Doubler-request)))
  "Returns full string definition for message of type 'Doubler-request"
  (cl:format cl:nil "# inputs~%int64 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Doubler-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Doubler-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Doubler-request
    (cl:cons ':number (number msg))
))
;//! \htmlinclude Doubler-response.msg.html

(cl:defclass <Doubler-response> (roslisp-msg-protocol:ros-message)
  ((doubled
    :reader doubled
    :initarg :doubled
    :type cl:integer
    :initform 0))
)

(cl:defclass Doubler-response (<Doubler-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Doubler-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Doubler-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rob599_basic-srv:<Doubler-response> is deprecated: use rob599_basic-srv:Doubler-response instead.")))

(cl:ensure-generic-function 'doubled-val :lambda-list '(m))
(cl:defmethod doubled-val ((m <Doubler-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rob599_basic-srv:doubled-val is deprecated.  Use rob599_basic-srv:doubled instead.")
  (doubled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Doubler-response>) ostream)
  "Serializes a message object of type '<Doubler-response>"
  (cl:let* ((signed (cl:slot-value msg 'doubled)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Doubler-response>) istream)
  "Deserializes a message object of type '<Doubler-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'doubled) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Doubler-response>)))
  "Returns string type for a service object of type '<Doubler-response>"
  "rob599_basic/DoublerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Doubler-response)))
  "Returns string type for a service object of type 'Doubler-response"
  "rob599_basic/DoublerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Doubler-response>)))
  "Returns md5sum for a message object of type '<Doubler-response>"
  "cd11c21fe194ec0173733fddd2cd457b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Doubler-response)))
  "Returns md5sum for a message object of type 'Doubler-response"
  "cd11c21fe194ec0173733fddd2cd457b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Doubler-response>)))
  "Returns full string definition for message of type '<Doubler-response>"
  (cl:format cl:nil "# outputs~%int64 doubled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Doubler-response)))
  "Returns full string definition for message of type 'Doubler-response"
  (cl:format cl:nil "# outputs~%int64 doubled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Doubler-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Doubler-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Doubler-response
    (cl:cons ':doubled (doubled msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Doubler)))
  'Doubler-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Doubler)))
  'Doubler-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Doubler)))
  "Returns string type for a service object of type '<Doubler>"
  "rob599_basic/Doubler")