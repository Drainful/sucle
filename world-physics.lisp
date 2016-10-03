(in-package :sandbox)

(defparameter onground nil)
(defparameter cameraVelocity (mat:onebyfour '(0.0 0.0 0.0 0)))
(defparameter wow nil)
(defparameter worldhash (make-hash-table :test #'equal))
(defparameter isneaking nil)

(defun getworld (name)
  (gethash name worldhash))

(defun setworld (name newval)
  (setf (gethash name worldhash) newval))

(defparameter daytime 1.0)
(defparameter ourcam (make-simplecam))

(defun physinnit ()
  (setf (simplecam-pos ourcam) (mat:onebyfour '(0 128 0 0)))
  (setf cameraVelocity (mat:onebyfour '(0 0 0 0)))
  (setworld "player" ourcam)
  (setf isprinting nil)
  (setf wprev most-negative-fixnum))

(defun physics ()
  "a messy function for the bare bones physics"
  (let ((camera (getworld "player")))
    (setf wow camera)
    (let ((wowzer nil)
	  (collisiondata nil)
	  (velclamp nil))
      (multiple-value-bind (a b c)
	  (aabbcc::finish-clamps #'myafunc
				 (mat-vec (simplecam-pos camera))
				 (mat-vec cameraVelocity))
	(setf wowzer a)
	(setf collisiondata b)
	(setf velclamp c))
      (setf onground (if (numberp (elt collisiondata 2))
			 (= 0 (elt collisiondata 2))
			 (eq :contact (elt collisiondata 2))))

      (setf (simplecam-pos camera)
	    (vec-mat
	     wowzer))
      (setf cameraVelocity
	    (mat-clamper cameraVelocity velclamp))

      
      (if (not onground)
	  (mat:add! cameraVelocity (mat:onebyfour (list 0 (* -0.08 (expt tickscale 2)) 0 0))))
      (let ((airscaled
	     (mat:onebyfour
	      (list
	       (row-major-aref cameraVelocity 0)
	       0
	       (row-major-aref cameraVelocity 2)
	       0))))
	(mat:scale! airscaled (* 0.9))
	(if onground (mat:scale! airscaled (* 0.6 0.91)))
	(if nil
	    (let ((speed (hypot (mat-lis airscaled))))
	      (print speed)
	      (if (> 0.05 speed)
		  (setf isprinting nil))))
	(setf (row-major-aref cameraVelocity 0) (row-major-aref airscaled 0))
	(setf (row-major-aref cameraVelocity 2) (row-major-aref airscaled 2)))
      (setf (row-major-aref cameraVelocity 1)
	    (* (expt 0.98 tickscale)
	       (row-major-aref cameraVelocity 1)))
      (outofbounds camera))))

(defun outofbounds (camera)
  (if (> 0 (row-major-aref (simplecam-pos camera) 1))
      (progn	  
	(setf (row-major-aref cameraVelocity 1) 0)
	(setf (row-major-aref (simplecam-pos camera) 1) 0)
	(setf (simplecam-pos camera) (mat:onebyfour (list 0 128 0 1))))))

(defparameter lastw nil)
(defparameter wprev most-negative-fixnum)
(defparameter wpressprev nil)
(defparameter isprinting nil)

(defmacro toggle (var)
  `(setf ,var (not ,var)))

(defun controls ()
  (let ((camera (getworld "player")))
    (mouse-looking camera)
    (mat:add!
     cameraVelocity
     (keymovement camera))
    (progn
     (if (in::akeydown "w")
	 ;;if it was pressed last round
	 (progn
	   (if (not wpressprev)
	       (progn
		 (if (> 150 (- (get-internal-run-time) wprev))
		     (setf isprinting t))))
	   (setf wpressprev t))
	 (progn
	   (setf isprinting nil)
	   (if wpressprev
	       (progn
		 (setf wprev (get-internal-run-time))
		 (setf wpressprev nil))))))
    
    (if (in::akeydown "lshift")
	(progn
	  (setf isprinting nil)
	  (setf isneaking t))
	(setf isneaking nil))
    (if t
	(setf daytime (/ (+ 1 (cos (/ (get-internal-run-time) (* 20 100)))) 2))
	(setf daytime 1))

    (in:p+1 #\h (lambda () (someseq
			    (floor (row-major-aref (simplecam-pos camera) 0) 16)
			    (floor (row-major-aref (simplecam-pos camera) 2) 16))))

    (in:p+1 3 (lambda () (aplatform
			  (mat-world-pos (simplecam-pos camera))
			  2)))
    (in:p+1 2 (lambda ()
		(notaplatform (mat-world-pos (simplecam-pos camera)))
		(notaplatform (vecadd (mat-world-pos (simplecam-pos camera)) (vector 0 1 0)))))))

(defun mat-world-pos (mat)
  (vector
   (round (row-major-aref mat 0))
   (round (row-major-aref mat 1))
   (round (row-major-aref mat 2))))

(defun mouse-looking (camera)
  (let* ((change (in:delta))
	 (x (* 1/360 (aref change 0)))
	 (y (* 1/360 (aref change 1))))
    (setf
     (simplecam-yaw camera)
     (mod (+ (simplecam-yaw camera) x) (* 2 pi)))
    (setf (simplecam-pitch camera)
	  (anothershit
	   (+ (simplecam-pitch camera) y) (/ pi 2)))))

(defun anothershit (x whatthefuck)
  "used to clamp the pitch"
  (if (> x whatthefuck)
      whatthefuck
      (if (< x (- whatthefuck))
	  (- whatthefuck)
	  x)))

(defun good-func (some-number)
  "maps keys to vectors"
  (lambda (x)
    (if (in::akeydown (first x))
	(mat:add! some-number
		  (mat:onebyfour (second x))))))

(defun empty-vec4 ()
  (mat:onebyfour '(0 0 0 0)))

(defun key-legs ()
  "keys for walking"
  (let* ((delta (empty-vec4))
	 (lemod (good-func delta)))
    (mapcar
     lemod
     '(("s" ( 1  0  0  0))
       ("w" (-1  0  0  0))
       ("a" ( 0  0  1  0))
       ("d" ( 0  0 -1  0))))
    (mat:scale! (mat:normalize! delta) (* 0.4 (expt tickscale 2)))
    (if isneaking
	(mat:scale! delta 0.2))
    (if isprinting
	(mat:scale! delta 1.3))
    (if (not onground)
	(mat:scale! delta 0.2))
    delta))

(defun key-jumps ()
  "keys for jumping"
  (let* ((delta (empty-vec4))
	 (lemod (good-func delta)))
    (if onground
	(mapcar
	 lemod
	 `(("space" (0 ,(* 0.42 (expt tickscale 1)) 0 0)))))
    delta))

(defun keymovement (camera)
  "total keymovement"
  (mat:mmul! (mat:add (key-legs) (key-jumps))
	     (mat:rotation-matrix 0 1 0
				  (simplecam-yaw camera))))

(defun aplatform (pos blockid)
  (let ((i (elt pos 0))
	(j (elt pos 1))
	(k (elt pos 2)))
    (dotimes (a 3)
      (dotimes (b 3)
	(setblock-with-update (+ a i -1) (- j 1) (+ b k -1) blockid)))))

(defun notaplatform (pos)
  (let ((i (elt pos 0))
	(j (elt pos 1))
	(k (elt pos 2)))
    (dotimes (a 3)
      (dotimes (b 3)
	(setblock-with-update (+ a i -1) (+ j) (+ b k -1) 0)))))

(defun vec-mat (vec)
  (let ((newmat (mat:onebyfour '(0 0 0 0))))
    (dotimes (x (length vec))
      (setf (row-major-aref newmat x) (elt vec x)))
    newmat))

(defun mat-clamper (mat coldata)
  (vec-mat
   (aabbcc::clamp-vec (mat-vec mat) coldata)))

(defun mat-vec (mat)
  (vector
   (row-major-aref mat 0)
   (row-major-aref mat 1)
   (row-major-aref mat 2)))

(defun mat-pos (mat)
  (getblock
   (round (row-major-aref mat 0))
   (round (row-major-aref mat 1))
   (round (row-major-aref mat 2))))

(defun blocktocontact (blocklist vec3player vel)
  (mapcar
   (lambda (theplace)
     (aabbcc::%aabb-intersect
      (aabbcc::player-aabb)
      vec3player
      (aabbcc::block-aabb)
      theplace
      vel))
   blocklist))

(defun get-blocks-around-player (vec3player vel)
  (declare (ignore vel))
  (let ((places nil))
    (dotimes (x 3)
      (dotimes (y 4)
	(dotimes (z 3)
	  (let ((blockx (round (1- (+ x (elt vec3player 0)))))
		(blocky (round (1- (ceiling (+ y (elt vec3player 1))))))
		(blockz (round (1- (+ z (elt vec3player 2))))))
	    (let ((blockid (mat-pos (vector blockx blocky blockz))))
	      (if (eq t (aref mc-blocks::iscollidable blockid))
		  (push (vector blockx blocky blockz) places)))))))
    places))

(defun myafunc (vec3player vel)
  (let ((ourblocks (get-blocks-around-player vec3player vel)))
    (let ((ourcontacts (blocktocontact ourblocks vec3player vel)))
      (let ((totcollisoins (aabbcc::collapsecollisions ourcontacts)))
        totcollisoins))))

(defun insert-at (num vec place)
  (let* ((start (subseq vec 0 place))
	 (end (subseq vec place (length vec))))
    (concatenate 'vector start (vector num) end)))

(defun delete-at (vec place)
  (let* ((start (subseq vec 0 place))
	 (end (subseq vec (1+ place) (length vec))))
    (concatenate 'vector start end)))

(defun vecscale (vec3 scale)
  (vector
   (* scale (elt vec3 0))
   (* scale (elt vec3 1))
   (* scale (elt vec3 2))))

(defun vecsubtract (vec3 vec32)
  (vector
   (- (elt vec3 0) (elt vec32 0))
   (- (elt vec3 1) (elt vec32 1))
   (- (elt vec3 2) (elt vec32 2))))

(defun vecadd (vec3 vec32)
  (vector
   (+ (elt vec3 0) (elt vec32 0))
   (+ (elt vec3 1) (elt vec32 1))
   (+ (elt vec3 2) (elt vec32 2))))

(defun veczerop (vec)
  (dotimes (n (length vec))
    (if (zerop (elt vec n))
	nil
	(return-from veczerop nil)))
  t)
