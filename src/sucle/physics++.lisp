(in-package #:sucle)

;; (defmacro vec (x y z &optional (temp-vec-sym 'temp-vec))
;;   (assert (symbolp temp-vec-sym))
;;   (alexandria:with-gensyms
;;    (a b c)
;;    `(progn
;;       (with-vec (,a ,b ,c) (,temp-vec-sym symbol-macrolet)
;;         (setf ,a ,x
;;               ,b ,y
;;               ,c ,z))
;;       ,temp-vec-sym)))

(defun set-temp-vec (temp-vec x y z)
  (with-vec (a b c) (temp-vec symbol-macrolet)
    (setf a x
          b y
          c z))
  temp-vec)

(defun physics (yaw dir fly is-jumping is-sneaking
                
                entity
                pointmass
                noclip gravity
                contact-handler
                world-collision-fun
                aabb
                &optional (temp-vec *temp-vec*))
  ;;[FIXME] This function is a total mess, a nightmare?
  (declare (optimize (debug 3))
           (ignorable is-sneaking))
  (step-pointmass pointmass)
  (let ((vel (pointmass-velocity pointmass))
        (pos (pointmass-position pointmass))
        (mass (pointmass-mass pointmass))
        (force (fill (pointmass-force pointmass) 0.0))
        (old-onground (logtest (entity-contact entity) 4)))
    (dotimes (x (length force))
      (if (< 100 (aref force x))
          (break)))
    (let ((contact-state (if noclip ;;(and noclip (not *dirtying*))
                             0
                             (mvc contact-handler
                                  (spread pos)
                                  aabb))))
      (apply-player-movement force temp-vec
                             :vel vel
                             :direction dir
                             :is-flying fly
                             :is-jumping is-jumping
                             :is-sneaking is-sneaking
                             :yaw yaw
                             :onground (logtest contact-state 4)
                             :old-onground old-onground)
      
      (nullify-velocity-where-obstructed
       vel
       (logtest contact-state 32)
       (logtest contact-state 16)
       (logtest contact-state 8)
       (logtest contact-state 4)
       (logtest contact-state 2)
       (logtest contact-state 1))

      (setf (entity-contact entity) contact-state))


    ;;wind resistance
    (apply-drag force vel temp-vec
                :total-speed (* *ticks-per-second* (nsb-cga:vec-length vel))
                :drag-coefficient (if fly 0.005 0.0003))

    ;;to allow walking around block corners
    ;;we introduce a frame of gravity lag
    (when (and (not old-onground) gravity)
      (apply-gravity force))

    (modify nsb-cga:%vec/ force (* (* *ticks-per-second*
                                      *ticks-per-second* 0.5)
                                   mass))
    (modify nsb-cga:%vec+ vel force)
    (let ((aabb-gen-fnc
           (if noclip
               (lambda (&rest args)
                 (declare (ignore args))
                 (values 0 1.0))
               (progn
                 world-collision-fun))))
      (with-vec (vx vy vz) (vel symbol-macrolet)
        (with-vec (px py pz) (pos symbol-macrolet)
          (multiple-value-bind (new-x new-y new-z xyzclamp)
              (step-motion aabb-gen-fnc px py pz vx vy vz aabb)
            ;;Update the position and velocity, after taking into
            ;;account the collision data.
            (psetf px (floatify new-x)
                   py (floatify new-y)
                   pz (floatify new-z)
                   vx (floatify (if (logtest 4 xyzclamp) 0 vx))
                   vy (floatify (if (logtest 2 xyzclamp) 0 vy))
                   vz (floatify (if (logtest 1 xyzclamp) 0 vz)))))))))

(defun apply-gravity (force)
  (modify nsb-cga:%vec- force
          (load-time-value
           (nsb-cga:vec 0.0
                        13.0 ; 9.8
                        0.0))))

(defun apply-drag (force vel temp-vec
                   &key total-speed drag-coefficient)
  (modify nsb-cga:%vec-
          force
          (nsb-cga:%vec* temp-vec
                         vel
                         (* *ticks-per-second* (* total-speed
                                                  total-speed)
                            drag-coefficient)))
  force)

(defun apply-jump (force &optional (temp-vec *temp-vec*) (speed 4))
  (modify nsb-cga:%vec+ force
          (set-temp-vec temp-vec
                        0.0
                        (* speed *ticks-per-second*)
                        0.0))
  force)

(defun player-target-velocity (dir yaw
                               &key
                                 (flying t)
                                 (speed *player-walkspeed*))
  (let ((yvalue (case flying
                  (:up speed)
                  (:down (- speed))
                  (t 0.0))))
    (if dir
        (let ((diraux (+ dir yaw)))
          (nsb-cga:vec
           (* speed (- (sin diraux)))
           yvalue
           (* speed (cos diraux))))
        (nsb-cga:vec 0.0 yvalue 0.0))))

(defparameter *player-walkspeed* 4.317)

(defun apply-step-force (force &key current-velocity target-velocity step-power)
  (let* ((delta-velocity (nsb-cga:vec-
                          target-velocity
                          current-velocity))
         (dv-magnitude (nsb-cga:vec-length delta-velocity)))
    (unless (zerop dv-magnitude)
      (let ((bump-direction (nsb-cga:vec/
                             delta-velocity
                             dv-magnitude)))
        (let ((step-force (* 2.0 dv-magnitude)))
          (modify nsb-cga:%vec+ force
                  (nsb-cga:vec* bump-direction
                                (* step-power step-force)))))))
  force)

(defun apply-player-movement (force temp-vec
                              &key direction yaw vel onground
                                old-onground is-flying is-jumping
                                is-sneaking
                                (speed *player-walkspeed*)
                                (step-power 4.0))
  (dotimes (x (length force))
    (if (< 100 (aref force x))
        (break)))
  (if is-flying (*= speed 4.0) ; increase speed while flying
      (if onground
          (progn (when (and (not direction) old-onground)
                   (modify nsb-cga:%vec- force ; apply stopping force
                           (nsb-cga:%vec* temp-vec
                                          (nsb-cga:%vec* temp-vec
                                                         vel
                                                         *ticks-per-second*)
                                          4.0)))
                 (when is-jumping (apply-jump force temp-vec))) ; apply jumping force
          (*= step-power 0.6)))

  (when (or direction is-flying)
    (apply-step-force
     force
     :current-velocity (nsb-cga:vec (* (aref vel 0) *ticks-per-second*)
                                    (if is-flying
                                        (* (aref vel 1) *ticks-per-second*)
                                        0.0)
                                    (* (aref vel 2) *ticks-per-second*))
     :target-velocity (player-target-velocity
                       direction yaw
                       :flying (when is-flying
                                 (cond
                                   (is-jumping :up)
                                   (is-sneaking :down)))
                       :speed speed)
     :step-power (if (and (not direction)
                          is-flying)
                     1.0
                     step-power))))


;; ;; split up the entity class
;; (defpackage #:physics
;;   (:use #:cl)
;;   (:import-from #:utility
;;                 #:dobox
;;                 #:with-vec
;;                 #:floatify
;;                 #:etouq
;;                 #:once-only
;;                 #:dohash
;;                 #:%list
;;                 #:toggle)
;;   (:export))

;; (in-package #:physics)

;; (defgeneric entity-collision-fun (entity))
;; (defgeneric entity-contact-fun (entity))

;; (defgeneric apply-entity-movement (entity))

;; ;; (defgeneric apply-entity-movement (entity &optional temp-vec))
;; (defgeneric entity-run-physics (entity))

;; (defclass simple-entity ()
;;   ((center-of-mass :initarg center-of-mass
;;                    :type pointmass
;;                    :getter entity-center-of-mass)
;;    (aabb :initarg aabb
;;          :type aabb
;;          :getter entity-aabb)
;;    (contact :initarg contact
;;             :getter entity-contact)
;;    (gravity? :initarg gravity
;;              :type boolean
;;              :accessor entity-gravity?)
;;    (clip? :initarg clip
;;           :type boolean
;;           :accessor entity-clip?)))

;; (defmethod entity-collision-fun ((entity simple-entity))
;;   (declare (ignore entity))
;;   #'entity-collision)

;; (defmethod entity-contact-fun ((entity simple-entity))
;;   (declare (ignore entity))
;;   #'find-blocks-in-contact-with)

;; ;; (defmethod apply-entity-movement ((entity simple-entity)
;; ;;                                   &optional temp-vec)
;; ;;   (declare (ignore entity force)))

;; (defclass entity (simple-entity)
;;   ((neck)
;;    (hips)
;;    (jump?)))

;; (defclass player-entity (entity)
;;   ((fly?)
;;    ))

;; ;; (defgeneric step-entity (entity))

