;; Auto-generated. Do not edit!


(when (boundp 'action_primitive_variation::PushButton)
  (if (not (find-package "ACTION_PRIMITIVE_VARIATION"))
    (make-package "ACTION_PRIMITIVE_VARIATION"))
  (shadow 'PushButton (find-package "ACTION_PRIMITIVE_VARIATION")))
(unless (find-package "ACTION_PRIMITIVE_VARIATION::PUSHBUTTON")
  (make-package "ACTION_PRIMITIVE_VARIATION::PUSHBUTTON"))
(unless (find-package "ACTION_PRIMITIVE_VARIATION::PUSHBUTTONREQUEST")
  (make-package "ACTION_PRIMITIVE_VARIATION::PUSHBUTTONREQUEST"))
(unless (find-package "ACTION_PRIMITIVE_VARIATION::PUSHBUTTONRESPONSE")
  (make-package "ACTION_PRIMITIVE_VARIATION::PUSHBUTTONRESPONSE"))

(in-package "ROS")





(defclass action_primitive_variation::PushButtonRequest
  :super ros::object
  :slots (_button_name ))

(defmethod action_primitive_variation::PushButtonRequest
  (:init
   (&key
    ((:button_name __button_name) "")
    )
   (send-super :init)
   (setq _button_name (string __button_name))
   self)
  (:button_name
   (&optional __button_name)
   (if __button_name (setq _button_name __button_name)) _button_name)
  (:serialization-length
   ()
   (+
    ;; string _button_name
    4 (length _button_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _button_name
       (write-long (length _button_name) s) (princ _button_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _button_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _button_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass action_primitive_variation::PushButtonResponse
  :super ros::object
  :slots (_success_bool ))

(defmethod action_primitive_variation::PushButtonResponse
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

(defclass action_primitive_variation::PushButton
  :super ros::object
  :slots ())

(setf (get action_primitive_variation::PushButton :md5sum-) "06b0fa47749790f71d50d61e72fea1cc")
(setf (get action_primitive_variation::PushButton :datatype-) "action_primitive_variation/PushButton")
(setf (get action_primitive_variation::PushButton :request) action_primitive_variation::PushButtonRequest)
(setf (get action_primitive_variation::PushButton :response) action_primitive_variation::PushButtonResponse)

(defmethod action_primitive_variation::PushButtonRequest
  (:response () (instance action_primitive_variation::PushButtonResponse :init)))

(setf (get action_primitive_variation::PushButtonRequest :md5sum-) "06b0fa47749790f71d50d61e72fea1cc")
(setf (get action_primitive_variation::PushButtonRequest :datatype-) "action_primitive_variation/PushButtonRequest")
(setf (get action_primitive_variation::PushButtonRequest :definition-)
      "string button_name
---
int64 success_bool

")

(setf (get action_primitive_variation::PushButtonResponse :md5sum-) "06b0fa47749790f71d50d61e72fea1cc")
(setf (get action_primitive_variation::PushButtonResponse :datatype-) "action_primitive_variation/PushButtonResponse")
(setf (get action_primitive_variation::PushButtonResponse :definition-)
      "string button_name
---
int64 success_bool

")



(provide :action_primitive_variation/PushButton "06b0fa47749790f71d50d61e72fea1cc")


