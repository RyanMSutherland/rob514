;; Auto-generated. Do not edit!


(when (boundp 'rob599_basic::Doubler)
  (if (not (find-package "ROB599_BASIC"))
    (make-package "ROB599_BASIC"))
  (shadow 'Doubler (find-package "ROB599_BASIC")))
(unless (find-package "ROB599_BASIC::DOUBLER")
  (make-package "ROB599_BASIC::DOUBLER"))
(unless (find-package "ROB599_BASIC::DOUBLERREQUEST")
  (make-package "ROB599_BASIC::DOUBLERREQUEST"))
(unless (find-package "ROB599_BASIC::DOUBLERRESPONSE")
  (make-package "ROB599_BASIC::DOUBLERRESPONSE"))

(in-package "ROS")





(defclass rob599_basic::DoublerRequest
  :super ros::object
  :slots (_number ))

(defmethod rob599_basic::DoublerRequest
  (:init
   (&key
    ((:number __number) 0)
    )
   (send-super :init)
   (setq _number (round __number))
   self)
  (:number
   (&optional __number)
   (if __number (setq _number __number)) _number)
  (:serialization-length
   ()
   (+
    ;; int64 _number
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _number
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _number (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _number) (= (length (_number . bv)) 2)) ;; bignum
              (write-long (ash (elt (_number . bv) 0) 0) s)
              (write-long (ash (elt (_number . bv) 1) -1) s))
             ((and (class _number) (= (length (_number . bv)) 1)) ;; big1
              (write-long (elt (_number . bv) 0) s)
              (write-long (if (>= _number 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _number s)(write-long (if (>= _number 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _number
#+(or :alpha :irix6 :x86_64)
      (setf _number (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _number (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(defclass rob599_basic::DoublerResponse
  :super ros::object
  :slots (_doubled ))

(defmethod rob599_basic::DoublerResponse
  (:init
   (&key
    ((:doubled __doubled) 0)
    )
   (send-super :init)
   (setq _doubled (round __doubled))
   self)
  (:doubled
   (&optional __doubled)
   (if __doubled (setq _doubled __doubled)) _doubled)
  (:serialization-length
   ()
   (+
    ;; int64 _doubled
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _doubled
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _doubled (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _doubled) (= (length (_doubled . bv)) 2)) ;; bignum
              (write-long (ash (elt (_doubled . bv) 0) 0) s)
              (write-long (ash (elt (_doubled . bv) 1) -1) s))
             ((and (class _doubled) (= (length (_doubled . bv)) 1)) ;; big1
              (write-long (elt (_doubled . bv) 0) s)
              (write-long (if (>= _doubled 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _doubled s)(write-long (if (>= _doubled 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _doubled
#+(or :alpha :irix6 :x86_64)
      (setf _doubled (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _doubled (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(defclass rob599_basic::Doubler
  :super ros::object
  :slots ())

(setf (get rob599_basic::Doubler :md5sum-) "cd11c21fe194ec0173733fddd2cd457b")
(setf (get rob599_basic::Doubler :datatype-) "rob599_basic/Doubler")
(setf (get rob599_basic::Doubler :request) rob599_basic::DoublerRequest)
(setf (get rob599_basic::Doubler :response) rob599_basic::DoublerResponse)

(defmethod rob599_basic::DoublerRequest
  (:response () (instance rob599_basic::DoublerResponse :init)))

(setf (get rob599_basic::DoublerRequest :md5sum-) "cd11c21fe194ec0173733fddd2cd457b")
(setf (get rob599_basic::DoublerRequest :datatype-) "rob599_basic/DoublerRequest")
(setf (get rob599_basic::DoublerRequest :definition-)
      "# inputs
int64 number
---
# outputs
int64 doubled
")

(setf (get rob599_basic::DoublerResponse :md5sum-) "cd11c21fe194ec0173733fddd2cd457b")
(setf (get rob599_basic::DoublerResponse :datatype-) "rob599_basic/DoublerResponse")
(setf (get rob599_basic::DoublerResponse :definition-)
      "# inputs
int64 number
---
# outputs
int64 doubled
")



(provide :rob599_basic/Doubler "cd11c21fe194ec0173733fddd2cd457b")


