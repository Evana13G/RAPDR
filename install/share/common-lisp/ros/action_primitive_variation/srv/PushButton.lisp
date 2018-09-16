; Auto-generated. Do not edit!


(cl:in-package action_primitive_variation-srv)


;//! \htmlinclude PushButton-request.msg.html

(cl:defclass <PushButton-request> (roslisp-msg-protocol:ros-message)
  ((button_name
    :reader button_name
    :initarg :button_name
    :type cl:string
    :initform ""))
)

(cl:defclass PushButton-request (<PushButton-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PushButton-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PushButton-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name action_primitive_variation-srv:<PushButton-request> is deprecated: use action_primitive_variation-srv:PushButton-request instead.")))

(cl:ensure-generic-function 'button_name-val :lambda-list '(m))
(cl:defmethod button_name-val ((m <PushButton-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader action_primitive_variation-srv:button_name-val is deprecated.  Use action_primitive_variation-srv:button_name instead.")
  (button_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PushButton-request>) ostream)
  "Serializes a message object of type '<PushButton-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'button_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'button_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PushButton-request>) istream)
  "Deserializes a message object of type '<PushButton-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'button_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'button_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PushButton-request>)))
  "Returns string type for a service object of type '<PushButton-request>"
  "action_primitive_variation/PushButtonRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PushButton-request)))
  "Returns string type for a service object of type 'PushButton-request"
  "action_primitive_variation/PushButtonRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PushButton-request>)))
  "Returns md5sum for a message object of type '<PushButton-request>"
  "06b0fa47749790f71d50d61e72fea1cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PushButton-request)))
  "Returns md5sum for a message object of type 'PushButton-request"
  "06b0fa47749790f71d50d61e72fea1cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PushButton-request>)))
  "Returns full string definition for message of type '<PushButton-request>"
  (cl:format cl:nil "string button_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PushButton-request)))
  "Returns full string definition for message of type 'PushButton-request"
  (cl:format cl:nil "string button_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PushButton-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'button_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PushButton-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PushButton-request
    (cl:cons ':button_name (button_name msg))
))
;//! \htmlinclude PushButton-response.msg.html

(cl:defclass <PushButton-response> (roslisp-msg-protocol:ros-message)
  ((success_bool
    :reader success_bool
    :initarg :success_bool
    :type cl:integer
    :initform 0))
)

(cl:defclass PushButton-response (<PushButton-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PushButton-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PushButton-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name action_primitive_variation-srv:<PushButton-response> is deprecated: use action_primitive_variation-srv:PushButton-response instead.")))

(cl:ensure-generic-function 'success_bool-val :lambda-list '(m))
(cl:defmethod success_bool-val ((m <PushButton-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader action_primitive_variation-srv:success_bool-val is deprecated.  Use action_primitive_variation-srv:success_bool instead.")
  (success_bool m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PushButton-response>) ostream)
  "Serializes a message object of type '<PushButton-response>"
  (cl:let* ((signed (cl:slot-value msg 'success_bool)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PushButton-response>) istream)
  "Deserializes a message object of type '<PushButton-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'success_bool) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PushButton-response>)))
  "Returns string type for a service object of type '<PushButton-response>"
  "action_primitive_variation/PushButtonResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PushButton-response)))
  "Returns string type for a service object of type 'PushButton-response"
  "action_primitive_variation/PushButtonResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PushButton-response>)))
  "Returns md5sum for a message object of type '<PushButton-response>"
  "06b0fa47749790f71d50d61e72fea1cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PushButton-response)))
  "Returns md5sum for a message object of type 'PushButton-response"
  "06b0fa47749790f71d50d61e72fea1cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PushButton-response>)))
  "Returns full string definition for message of type '<PushButton-response>"
  (cl:format cl:nil "int64 success_bool~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PushButton-response)))
  "Returns full string definition for message of type 'PushButton-response"
  (cl:format cl:nil "int64 success_bool~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PushButton-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PushButton-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PushButton-response
    (cl:cons ':success_bool (success_bool msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PushButton)))
  'PushButton-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PushButton)))
  'PushButton-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PushButton)))
  "Returns string type for a service object of type '<PushButton>"
  "action_primitive_variation/PushButton")