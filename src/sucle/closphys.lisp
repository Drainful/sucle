(defpackage #:physics
  (:use #:cl)
  (:import-from #:utility
                #:dobox
                #:with-vec
                #:floatify
                #:etouq
                #:once-only
                #:dohash
                #:%list
                #:toggle)
  (:export))

(in-package #:physics)


;;;; Utilities

;; Access nsb-cga:vec through wrapper macros in case the
;; implementation of vectors changes
(deftype vec () 'nsb-cga:vec) 

(defparameter *temp-vec* (nsb-cga:vec 0.0 0.0 0.0))

(defmacro modify (fun a &rest rest)
  (once-only (a)
    `(,fun ,a ,a ,@rest)))

(defmacro vec-incf (place delta)
  `(modify nsb-cga:%vec+
           ,place
           ,delta))

(defmacro vec-decf (place delta)
  `(modify nsb-cga:%vec-
           ,place
           ,delta))

(defun set-temp-vec (x y z &optional (temp-vec *temp-vec*))
  (with-vec (a b c) (temp-vec symbol-macrolet)
    (setf a x
          b y
          c z))
  temp-vec)

(defmacro vec* (a f &optional (temp-vec '*temp-vec*))
  `(nsb-cga:%vec* ,temp-vec ,a ,f))

(defun set-neck-values (neck yaw pitch)
  (setf (necking-yaw neck) yaw
	(necking-pitch neck) pitch))

;;;; Classes

(defclass has-position ()
  ((position :type vec
             :initarg :pos
             :accessor pos))
  (:documentation "An object with a world position"))

(defclass has-physics (has-position)
  ((velocity :type vec
             :initarg :velocity
             :accessor velocity)
   (acceleration :type vec
                 :initarg :acceleration
                 :accessor acceleration))
  (:documentation "An object that moves through the world"))

(defclass has-drag (has-physics)
  ((drag-coefficient :type float
                     :initform 0.0003
                     :initarg :drag-coefficient
                     :accessor drag))
  (:documentation "An object which experiences air friction"))

(defclass has-mass (has-physics)
  ((mass :type float
         :initarg :mass
         :accessor mass)
   (gravity-p :type boolean
              :initform t
              :accessor :gravity-p))
  (:documentation "An object with a mass"))

(defun apply-force (object force)
  (assert (typep object 'has-mass))
  (vec-incf (acceleration object)
            (vec* force
                  (mass object))))

(defclass has-aabb ()
  ((aabb :type aabb
         :initarg :aabb
         :accessor aabb))
  (:documentation "An object with an axis alligned bounding box"))

(defun create-aabb (&optional (maxx 1.0) (maxy maxx) (maxz maxx)
		      (minx (- maxx)) (miny (- maxy)) (minz (- maxz)))
  (aabbcc:make-aabb
   :minx minx
   :maxx maxx
   :miny miny
   :maxy maxy
   :minz minz
   :maxz maxz))

(defparameter *block-aabb*
  ;;;;1x1x1 cube
  (create-aabb 1.0 1.0 1.0 0.0 0.0 0.0))

(defun block-to-block-aabb (blockid)
  (declare (ignore blockid))
  ;;FIXME :use defmethod on objects?
  *block-aabb*)


(defclass has-world-collision (has-aabb has-physics)
  ((world-contact :type (integer 0 63)
                  :initform 0
                  :accessor world-contact)
   (clip-p :type boolean
           :initform t
           :accessor clip-p))
  (:documentation "An object whose motion is interrupted by blocks in
  the world"))

;; (defclass point-projectile (has-physics has-drag) ()
;;   (:documentation "An object "))

(defclass entity (has-physics
                  has-drag
                  has-mass
                  has-world-collision)
  ())

(defclass living-entity (entity)
  ((neck-yaw :type float
             :initform 0.0
             :acessor neck-yaw)
   (neck-pitch :type float
               :initform 0.0
               :acessor neck-pitch)
   (hips :type (or nil float)
         :initform nil
         :accessor hips)))

(defclass player-entity (living-entity)
  ((fly-p :type boolean
          :initform nil
          :accessor fly-p)
   (sneak-p :type boolean
            :initform nil
            :accessor sneak-p)))

;;;; Physics system
(defgeneric step-physics (entity dt)
  (:documentation "Entry point to the physics system. Modify ENTITY to
  have experienced DT seconds"))

(defmethod step-physics (entity dt))

(defmethod step-physics :before ((entity has-world-collision))
  "Before physics is calculated, find how ENTITY is in contact
with the world and store that information in ENTITY"
  (setf (world-contact entity) (find-world-contact entity)))

(defmethod step-physics :around ((entity has-physics) dt)
  "After all other physics operations, step velocity and positon"
  (call-next-method)
  (step-velocity entity (vec* (acceleration entity) dt))
  (step-position entity (vec* (velocity entity) dt)))

(defgeneric step-velocity (entity delta)
  (:documentation "Increase ENTITY's velocity by DX DY and DZ, taking
  whether ENTITY is in contact with any surfaces in to account."))

(defmethod step-velocity ((entity has-physics) delta)
  (vec-incf (velocity entity) delta))

;; TODO remove mutation and change in to an around method?
(defmethod step-velocity :before ((entity has-world-collision) delta)
  "Nullify ENTITY's velocity where it is in contact with the world."
  (let ((contact-state (world-contact entity)))
    (flet ((nullify-velocity-where-obstructed (velocity i+ i- j+ j- k+ k-)
             ;;velocity is a 3 float array.
             ;;If we are moving along an axis, but the contact
             ;;state says that direction is obstructed, then
             ;;set the physical motion along that direction to 0
             (with-vec (xvel yvel zvel) (velocity symbol-macrolet)
               (when (or (and (plusp xvel) i+)
                         (and (minusp xvel) i-))
                 (setf xvel 0.0))
               (when (or (and (plusp yvel) j+)
                         (and (minusp yvel) j-))
                 (setf yvel 0.0))
               (when (or (and (plusp zvel) k+)
                         (and (minusp zvel) k-))
                 (setf zvel 0.0)))))
      (nullify-velocity-where-obstructed
       (velocity entity)
       (logtest contact-state #b100000)
       (logtest contact-state #b010000)
       (logtest contact-state #b001000)
       (logtest contact-state #b000100)
       (logtest contact-state #b000010)
       (logtest contact-state #b000001)))))

(defgeneric step-position (entity delta)
  (:documentation "Translate ENTITY's position by DX DY and DZ, taking
  collision in to account."))

(defmethod step-position ((entity has-physics) delta)
  (vec-incf (pos entity) delta))

(defmethod step-positon :around ((entity has-world-collision) delta)
  "If the entity has noclip, continue to the next method. Otherwise,
clamp motion to prevent the entity from clipping."
  (if (not (clip-p entity))
      (call-next-method)
      ()))

(defgeneric find-world-contact (entity)
  (:documentation "Return a value which represents whether ENTITY is
  in contact with the world on each face of its bounding box."))

(defmethod find-world-contact ((entity has-world-collision))
  (with-vec (px py pz) ((pos entity))
    (with-vec (vx vy vz) ((velocity entity))
      (let ((aabb (aabb entity)))
        (aabbcc:with-touch-collector (collect-touch collapse-touch min-ratio)
          ;;[FIXME] aabb-collect-blocks does not check slabs, only blocks upon entering.
          ;;also check "intersecting shell blocks?"
          (aabbcc:aabb-collect-blocks (px py pz vx vy vz aabb)
              (x y z contact)
            (declare (ignorable contact))
            (let ((blockid (world:getblock x y z)))
              (when (block-data:data blockid :hard)
                #+nil
                (when *dirtying2*
                  (world:plain-setblock x y z (1+ (random 5)) 0))
                (let ((blockaabb (block-to-block-aabb blockid)))
                  (#+nil
                   let
                   #+nil((args
                          (list
                           aabb
                           px py pz
                           blockaabb
                           x y z
                           vx vy vz)))
                   progn
                   (multiple-value-bind (minimum type)
                       ;;(apply 'aabbcc:aabb-collide args)
                       ;;#+nil
                       (aabbcc:aabb-collide
                        aabb
                        px py pz
                        blockaabb
                        x y z
                        vx vy vz)
                     ;;(print (list minimum type (cons 'aabbcc:aabb-collide args)))
                     (collect-touch minimum type)))))))
          (values
           (collapse-touch vx vy vz)
           min-ratio))))))
