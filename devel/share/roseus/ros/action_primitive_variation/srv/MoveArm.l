;; Auto-generated. Do not edit!


(when (boundp 'action_primitive_variation::MoveArm)
  (if (not (find-package "ACTION_PRIMITIVE_VARIATION"))
    (make-package "ACTION_PRIMITIVE_VARIATION"))
  (shadow 'MoveArm (find-package "ACTION_PRIMITIVE_VARIATION")))
(unless (find-package "ACTION_PRIMITIVE_VARIATION::MOVEARM")
  (make-package "ACTION_PRIMITIVE_VARIATION::MOVEARM"))
(unless (find-package "ACTION_PRIMITIVE_VARIATION::MOVEARMREQUEST")
  (make-package "ACTION_PRIMITIVE_VARIATION::MOVEARMREQUEST"))
(unless (find-package "ACTION_PRIMITIVE_VARIATION::MOVEARMRESPONSE")
  (make-package "ACTION_PRIMITIVE_VARIATION::MOVEARMRESPONSE"))

(in-package "ROS")





(defclass action_primitive_variation::MoveArmRequest
  :super ros::object
  :slots (_location ))

(defmethod action_primitive_variation::MoveArmRequest
  (:init
   (&key
    ((:location __location) "")
    )
   (send-super :init)
   (setq _location (string __location))
   self)
  (:location
   (&optional __location)
   (if __location (setq _location __location)) _location)
  (:serialization-length
   ()
   (+
    ;; string _location
    4 (length _location)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _location
       (write-long (length _location) s) (princ _location s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _location
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _location (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass action_primitive_variation::MoveArmResponse
  :super ros::object
  :slots (_success_bool ))

(defmethod action_primitive_variation::MoveArmResponse
  (:init
   (&key
    ((:success_bool __success_bool) 0)
    )
   (send-super :init)
   (setq _success_bool (round __success_bool))
   self)
  (:success_bool
   (&optional __success_bool)
   (if __success_bool (setq _success_bool __success_bool)) _success_bool)
  (:serialization-length
   ()
   (+
    ;; int64 _success_bool
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _success_bool
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _success_bool (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _success_bool) (= (length (_success_bool . bv)) 2)) ;; bignum
              (write-long (ash (elt (_success_bool . bv) 0) 0) s)
              (write-long (ash (elt (_success_bool . bv) 1) -1) s))
             ((and (class _success_bool) (= (length (_success_bool . bv)) 1)) ;; big1
              (write-long (elt (_success_bool . bv) 0) s)
              (write-long (if (>= _success_bool 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _success_bool s)(write-long (if (>= _success_bool 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _success_bool
#+(or :alpha :irix6 :x86_64)
      (setf _success_bool (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _success_bool (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass action_primitive_variation::MoveArm
  :super ros::object
  :slots ())

(setf (get action_primitive_variation::MoveArm :md5sum-) "dabfe91d481ed6e7759f42648158bcae")
(setf (get action_primitive_variation::MoveArm :datatype-) "action_primitive_variation/MoveArm")
(setf (get action_primitive_variation::MoveArm :request) action_primitive_variation::MoveArmRequest)
(setf (get action_primitive_variation::MoveArm :response) action_primitive_variation::MoveArmResponse)

(defmethod action_primitive_variation::MoveArmRequest
  (:response () (instance action_primitive_variation::MoveArmResponse :init)))

(setf (get action_primitive_variation::MoveArmRequest :md5sum-) "dabfe91d481ed6e7759f42648158bcae")
(setf (get action_primitive_variation::MoveArmRequest :datatype-) "action_primitive_variation/MoveArmRequest")
(setf (get action_primitive_variation::MoveArmRequest :definition-)
      "string location
---
int64 success_bool

")

(setf (get action_primitive_variation::MoveArmResponse :md5sum-) "dabfe91d481ed6e7759f42648158bcae")
(setf (get action_primitive_variation::MoveArmResponse :datatype-) "action_primitive_variation/MoveArmResponse")
(setf (get action_primitive_variation::MoveArmResponse :definition-)
      "string location
---
int64 success_bool

")



(provide :action_primitive_variation/MoveArm "dabfe91d481ed6e7759f42648158bcae")


