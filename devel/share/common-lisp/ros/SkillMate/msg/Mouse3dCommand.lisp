; Auto-generated. Do not edit!


(cl:in-package SkillMate-msg)


;//! \htmlinclude Mouse3dCommand.msg.html

(cl:defclass <Mouse3dCommand> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (z
    :reader z
    :initarg :z
    :type cl:integer
    :initform 0)
   (rx
    :reader rx
    :initarg :rx
    :type cl:integer
    :initform 0)
   (ry
    :reader ry
    :initarg :ry
    :type cl:integer
    :initform 0)
   (rz
    :reader rz
    :initarg :rz
    :type cl:integer
    :initform 0)
   (press
    :reader press
    :initarg :press
    :type cl:integer
    :initform 0)
   (button
    :reader button
    :initarg :button
    :type cl:integer
    :initform 0))
)

(cl:defclass Mouse3dCommand (<Mouse3dCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Mouse3dCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Mouse3dCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SkillMate-msg:<Mouse3dCommand> is deprecated: use SkillMate-msg:Mouse3dCommand instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Mouse3dCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:type-val is deprecated.  Use SkillMate-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Mouse3dCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:x-val is deprecated.  Use SkillMate-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Mouse3dCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:y-val is deprecated.  Use SkillMate-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Mouse3dCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:z-val is deprecated.  Use SkillMate-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'rx-val :lambda-list '(m))
(cl:defmethod rx-val ((m <Mouse3dCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:rx-val is deprecated.  Use SkillMate-msg:rx instead.")
  (rx m))

(cl:ensure-generic-function 'ry-val :lambda-list '(m))
(cl:defmethod ry-val ((m <Mouse3dCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:ry-val is deprecated.  Use SkillMate-msg:ry instead.")
  (ry m))

(cl:ensure-generic-function 'rz-val :lambda-list '(m))
(cl:defmethod rz-val ((m <Mouse3dCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:rz-val is deprecated.  Use SkillMate-msg:rz instead.")
  (rz m))

(cl:ensure-generic-function 'press-val :lambda-list '(m))
(cl:defmethod press-val ((m <Mouse3dCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:press-val is deprecated.  Use SkillMate-msg:press instead.")
  (press m))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <Mouse3dCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SkillMate-msg:button-val is deprecated.  Use SkillMate-msg:button instead.")
  (button m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Mouse3dCommand>) ostream)
  "Serializes a message object of type '<Mouse3dCommand>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'z)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ry)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rz)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'press)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'button)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Mouse3dCommand>) istream)
  "Deserializes a message object of type '<Mouse3dCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'z) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rx) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ry) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rz) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'press) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'button) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Mouse3dCommand>)))
  "Returns string type for a message object of type '<Mouse3dCommand>"
  "SkillMate/Mouse3dCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Mouse3dCommand)))
  "Returns string type for a message object of type 'Mouse3dCommand"
  "SkillMate/Mouse3dCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Mouse3dCommand>)))
  "Returns md5sum for a message object of type '<Mouse3dCommand>"
  "691d3678a4135784240deeb965f6f776")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Mouse3dCommand)))
  "Returns md5sum for a message object of type 'Mouse3dCommand"
  "691d3678a4135784240deeb965f6f776")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Mouse3dCommand>)))
  "Returns full string definition for message of type '<Mouse3dCommand>"
  (cl:format cl:nil "int32 type~%~%int32 x~%int32 y~%int32 z~%~%int32 rx~%int32 ry~%int32 rz~%~%int32 press~%int32 button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Mouse3dCommand)))
  "Returns full string definition for message of type 'Mouse3dCommand"
  (cl:format cl:nil "int32 type~%~%int32 x~%int32 y~%int32 z~%~%int32 rx~%int32 ry~%int32 rz~%~%int32 press~%int32 button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Mouse3dCommand>))
  (cl:+ 0
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Mouse3dCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'Mouse3dCommand
    (cl:cons ':type (type msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':rx (rx msg))
    (cl:cons ':ry (ry msg))
    (cl:cons ':rz (rz msg))
    (cl:cons ':press (press msg))
    (cl:cons ':button (button msg))
))
