; Auto-generated. Do not edit!


(cl:in-package RobotCatheter-msg)


;//! \htmlinclude catheterState.msg.html

(cl:defclass <catheterState> (roslisp-msg-protocol:ros-message)
  ((Displacement
    :reader Displacement
    :initarg :Displacement
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (Tension
    :reader Tension
    :initarg :Tension
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (Current
    :reader Current
    :initarg :Current
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (LightPower
    :reader LightPower
    :initarg :LightPower
    :type cl:fixnum
    :initform 0))
)

(cl:defclass catheterState (<catheterState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <catheterState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'catheterState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name RobotCatheter-msg:<catheterState> is deprecated: use RobotCatheter-msg:catheterState instead.")))

(cl:ensure-generic-function 'Displacement-val :lambda-list '(m))
(cl:defmethod Displacement-val ((m <catheterState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobotCatheter-msg:Displacement-val is deprecated.  Use RobotCatheter-msg:Displacement instead.")
  (Displacement m))

(cl:ensure-generic-function 'Tension-val :lambda-list '(m))
(cl:defmethod Tension-val ((m <catheterState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobotCatheter-msg:Tension-val is deprecated.  Use RobotCatheter-msg:Tension instead.")
  (Tension m))

(cl:ensure-generic-function 'Current-val :lambda-list '(m))
(cl:defmethod Current-val ((m <catheterState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobotCatheter-msg:Current-val is deprecated.  Use RobotCatheter-msg:Current instead.")
  (Current m))

(cl:ensure-generic-function 'LightPower-val :lambda-list '(m))
(cl:defmethod LightPower-val ((m <catheterState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobotCatheter-msg:LightPower-val is deprecated.  Use RobotCatheter-msg:LightPower instead.")
  (LightPower m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <catheterState>) ostream)
  "Serializes a message object of type '<catheterState>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Displacement))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'Tension))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'Current))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'LightPower)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'LightPower)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <catheterState>) istream)
  "Deserializes a message object of type '<catheterState>"
  (cl:setf (cl:slot-value msg 'Displacement) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'Displacement)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'Tension) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'Tension)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'Current) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'Current)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'LightPower)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'LightPower)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<catheterState>)))
  "Returns string type for a message object of type '<catheterState>"
  "RobotCatheter/catheterState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'catheterState)))
  "Returns string type for a message object of type 'catheterState"
  "RobotCatheter/catheterState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<catheterState>)))
  "Returns md5sum for a message object of type '<catheterState>"
  "f3c53653df893edbb0c4f4974144a507")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'catheterState)))
  "Returns md5sum for a message object of type 'catheterState"
  "f3c53653df893edbb0c4f4974144a507")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<catheterState>)))
  "Returns full string definition for message of type '<catheterState>"
  (cl:format cl:nil "float32[4] Displacement~%uint16[4] Tension~%uint16[4] Current~%uint16 LightPower~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'catheterState)))
  "Returns full string definition for message of type 'catheterState"
  (cl:format cl:nil "float32[4] Displacement~%uint16[4] Tension~%uint16[4] Current~%uint16 LightPower~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <catheterState>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'Displacement) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'Tension) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'Current) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <catheterState>))
  "Converts a ROS message object to a list"
  (cl:list 'catheterState
    (cl:cons ':Displacement (Displacement msg))
    (cl:cons ':Tension (Tension msg))
    (cl:cons ':Current (Current msg))
    (cl:cons ':LightPower (LightPower msg))
))
