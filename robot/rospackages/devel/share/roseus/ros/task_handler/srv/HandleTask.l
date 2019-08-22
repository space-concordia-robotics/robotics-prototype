;; Auto-generated. Do not edit!


(when (boundp 'task_handler::HandleTask)
  (if (not (find-package "TASK_HANDLER"))
    (make-package "TASK_HANDLER"))
  (shadow 'HandleTask (find-package "TASK_HANDLER")))
(unless (find-package "TASK_HANDLER::HANDLETASK")
  (make-package "TASK_HANDLER::HANDLETASK"))
(unless (find-package "TASK_HANDLER::HANDLETASKREQUEST")
  (make-package "TASK_HANDLER::HANDLETASKREQUEST"))
(unless (find-package "TASK_HANDLER::HANDLETASKRESPONSE")
  (make-package "TASK_HANDLER::HANDLETASKRESPONSE"))

(in-package "ROS")





(defclass task_handler::HandleTaskRequest
  :super ros::object
  :slots (_task _status _args ))

(defmethod task_handler::HandleTaskRequest
  (:init
   (&key
    ((:task __task) "")
    ((:status __status) 0)
    ((:args __args) "")
    )
   (send-super :init)
   (setq _task (string __task))
   (setq _status (round __status))
   (setq _args (string __args))
   self)
  (:task
   (&optional __task)
   (if __task (setq _task __task)) _task)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:args
   (&optional __args)
   (if __args (setq _args __args)) _args)
  (:serialization-length
   ()
   (+
    ;; string _task
    4 (length _task)
    ;; int64 _status
    8
    ;; string _args
    4 (length _args)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _task
       (write-long (length _task) s) (princ _task s)
     ;; int64 _status
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _status (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _status) (= (length (_status . bv)) 2)) ;; bignum
              (write-long (ash (elt (_status . bv) 0) 0) s)
              (write-long (ash (elt (_status . bv) 1) -1) s))
             ((and (class _status) (= (length (_status . bv)) 1)) ;; big1
              (write-long (elt (_status . bv) 0) s)
              (write-long (if (>= _status 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _status s)(write-long (if (>= _status 0) 0 #xffffffff) s)))
     ;; string _args
       (write-long (length _args) s) (princ _args s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _task
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _task (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _status
#+(or :alpha :irix6 :x86_64)
      (setf _status (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _status (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _args
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _args (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass task_handler::HandleTaskResponse
  :super ros::object
  :slots (_response ))

(defmethod task_handler::HandleTaskResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass task_handler::HandleTask
  :super ros::object
  :slots ())

(setf (get task_handler::HandleTask :md5sum-) "300264c411f28987cadaa5540174e12c")
(setf (get task_handler::HandleTask :datatype-) "task_handler/HandleTask")
(setf (get task_handler::HandleTask :request) task_handler::HandleTaskRequest)
(setf (get task_handler::HandleTask :response) task_handler::HandleTaskResponse)

(defmethod task_handler::HandleTaskRequest
  (:response () (instance task_handler::HandleTaskResponse :init)))

(setf (get task_handler::HandleTaskRequest :md5sum-) "300264c411f28987cadaa5540174e12c")
(setf (get task_handler::HandleTaskRequest :datatype-) "task_handler/HandleTaskRequest")
(setf (get task_handler::HandleTaskRequest :definition-)
      "string task
int64 status
string args
---
string response

")

(setf (get task_handler::HandleTaskResponse :md5sum-) "300264c411f28987cadaa5540174e12c")
(setf (get task_handler::HandleTaskResponse :datatype-) "task_handler/HandleTaskResponse")
(setf (get task_handler::HandleTaskResponse :definition-)
      "string task
int64 status
string args
---
string response

")



(provide :task_handler/HandleTask "300264c411f28987cadaa5540174e12c")


