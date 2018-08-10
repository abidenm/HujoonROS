; Auto-generated. Do not edit!


(cl:in-package SkillMate-msg)


;//! \htmlinclude HapticCommand.msg.html

(cl:defclass <HapticCommand> (roslisp-msg-protocol:ros-message)
  ((array
    :reader array
    :initarg :array
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (btn
    :reader btn
    :initarg :btn
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass HapticCommand (<HapticCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HapticCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HapticCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SkillMate-msg:<HapticCommand> is deprecated: use SkillMate-msg:HapticCommand instead.")))

(cl:ensure-generic-function 'array-val :lambda-list '(m))
(cl:defmethod array-val ((m <HapticCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:array-val is deprecated.  Use SkillMate-msg:array instead.")
  (array m))

(cl:ensure-generic-function 'btn-val :lambda-list '(m))
(cl:defmethod btn-val ((m <HapticCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:btn-val is deprecated.  Use SkillMate-msg:btn instead.")
  (btn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HapticCommand>) ostream)
  "Serializes a message object of type '<HapticCommand>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'array))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'btn))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HapticCommand>) istream)
  "Deserializes a message object of type '<HapticCommand>"
  (cl:setf (cl:slot-value msg 'array) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'array)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'btn) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'btn)))
    (cl:dotimes (i 2)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HapticCommand>)))
  "Returns string type for a message object of type '<HapticCommand>"
  "SkillMate/HapticCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HapticCommand)))
  "Returns string type for a message object of type 'HapticCommand"
  "SkillMate/HapticCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HapticCommand>)))
  "Returns md5sum for a message object of type '<HapticCommand>"
  "56596198094ce2af05806d426c471047")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HapticCommand)))
  "Returns md5sum for a message object of type 'HapticCommand"
  "56596198094ce2af05806d426c471047")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HapticCommand>)))
  "Returns full string definition for message of type '<HapticCommand>"
  (cl:format cl:nil "float64[6] array~%int32[2] btn~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HapticCommand)))
  "Returns full string definition for message of type 'HapticCommand"
  (cl:format cl:nil "float64[6] array~%int32[2] btn~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HapticCommand>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'btn) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HapticCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'HapticCommand
    (cl:cons ':array (array msg))
    (cl:cons ':btn (btn msg))
))
