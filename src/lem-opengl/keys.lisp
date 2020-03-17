(in-package :lem-sucle)

(defparameter *keycodes2* (make-hash-table))
(defun define-key-code (sym &optional glfw3-code)
  (setf (gethash glfw3-code *keycodes2*)
	sym))
(defun get-sym-from-glfw3-code (code)
  (gethash code *keycodes2*))

(defvar *keycode-table* (make-hash-table))
(defvar *keyname-table* (make-hash-table :test 'equal))
(defun defkeycode (name code &optional key)
  (setf (gethash name *keyname-table*) code)
  (when key (setf (gethash code *keycode-table*) key)))
(defun get-code (name)
  (let ((code (gethash name *keyname-table*)))
    (assert code)
    code))
(defun char-to-key (char)
  (or (gethash (char-code char) *keycode-table*)
      (lem:make-key :sym (string char))))
(defun code-to-key (code)
  (or (gethash code *keycode-table*)
      (lem:make-key :sym (string (code-char code)))))
(defun get-key-from-name (name)
  (char-to-key (code-char (get-code name))))

(progn
  (defkeycode "C-@" 0 (lem:make-key :ctrl t :sym "@"))
  (defkeycode "C-a" 1 (lem:make-key :ctrl t :sym "a"))
  (defkeycode "C-b" 2 (lem:make-key :ctrl t :sym "b"))
  (defkeycode "C-c" 3 (lem:make-key :ctrl t :sym "c"))
  (defkeycode "C-d" 4 (lem:make-key :ctrl t :sym "d"))
  (defkeycode "C-e" 5 (lem:make-key :ctrl t :sym "e"))
  (defkeycode "C-f" 6 (lem:make-key :ctrl t :sym "f"))
  (defkeycode "C-g" 7 (lem:make-key :ctrl t :sym "g"))
  (defkeycode "C-h" 8 (lem:make-key :ctrl t :sym "h")))
;;#+nil
(defkeycode "C-i" 9 (lem:make-key :sym "Tab"))
(define-key-code "Tab" :tab)
;;#+nil
(progn
  (defkeycode "C-j" 10 (lem:make-key :ctrl t :sym "j"))
  (defkeycode "C-k" 11 (lem:make-key :ctrl t :sym "k"))
  (defkeycode "C-l" 12 (lem:make-key :ctrl t :sym "l")))
;;#+nil
(defkeycode "C-m" 13 (lem:make-key :sym "Return"))
;;FIXME:: is enter 10 or 13? have multiple keys like keypad?
(define-key-code "Return" :enter)
;;#+nil
(progn
  (defkeycode "C-n" 14 (lem:make-key :ctrl t :sym "n"))
  (defkeycode "C-o" 15 (lem:make-key :ctrl t :sym "o"))
  (defkeycode "C-p" 16 (lem:make-key :ctrl t :sym "p"))
  (defkeycode "C-q" 17 (lem:make-key :ctrl t :sym "q"))
  (defkeycode "C-r" 18 (lem:make-key :ctrl t :sym "r"))
  (defkeycode "C-s" 19 (lem:make-key :ctrl t :sym "s"))
  (defkeycode "C-t" 20 (lem:make-key :ctrl t :sym "t"))
  (defkeycode "C-u" 21 (lem:make-key :ctrl t :sym "u"))
  (defkeycode "C-v" 22 (lem:make-key :ctrl t :sym "v"))
  (defkeycode "C-w" 23 (lem:make-key :ctrl t :sym "w"))
  (defkeycode "C-x" 24 (lem:make-key :ctrl t :sym "x"))
  (defkeycode "C-y" 25 (lem:make-key :ctrl t :sym "y"))
  (defkeycode "C-z" 26 (lem:make-key :ctrl t :sym "z")))
;;#+nil
(defkeycode "escape" 27 (lem:make-key :sym "Escape"))
(define-key-code "Escape" :escape) ;;fixme:: not found?
;;#+nil
(progn
  (defkeycode "C-\\" 28 (lem:make-key :ctrl t :sym "\\"))
  (defkeycode "C-]" 29 (lem:make-key :ctrl t :sym "]"))
  (defkeycode "C-^" 30 (lem:make-key :ctrl t :sym "^"))
  (defkeycode "C-_" 31 (lem:make-key :ctrl t :sym "_")))
#+nil
(defkeycode "Spc" #x20 (lem:make-key :sym "Space"))
(define-key-code "Space" :space)
#+nil
(defkeycode "[backspace]" #x7F (lem:make-key :sym "Backspace"))
(define-key-code "Backspace" :backspace)

;;#+nil ;;FIXME -> character keys
(loop :for code :from #x21 :below #x7F
   :do (let ((string (string (code-char code))))
            (defkeycode string code (lem:make-key :sym string))))
#+nil
(defkeycode "[down]" #o402 (lem:make-key :sym "Down"))
(define-key-code "Down" :down)
#+nil
(defkeycode "[up]" #o403 (lem:make-key :sym "Up"))
(define-key-code "Up" :up)
#+nil
(defkeycode "[left]" #o404 (lem:make-key :sym "Left"))
(define-key-code "Left" :left)
#+nil
(defkeycode "[right]" #o405 (lem:make-key :sym "Right"))
(define-key-code "Right" :right)
#+nil
(progn
  (defkeycode "C-down" 525 (lem:make-key :ctrl t :sym "Down"))
  (defkeycode "C-up" 566 (lem:make-key :ctrl t :sym "Up"))
  (defkeycode "C-left" 545 (lem:make-key :ctrl t :sym "Left"))
  (defkeycode "C-right" 560 (lem:make-key :ctrl t :sym "Right")))
#+nil
(defkeycode "[home]" #o406 (lem:make-key :sym "Home"))
(define-key-code "Home" :home)
#+nil
(defkeycode "[backspace]" #o407 (lem:make-key :sym "Backspace"))
#+nil
(defkeycode "[f0]" #o410 (lem:make-key :sym "F0"))
#+nil
(progn
  (defkeycode "[f1]" #o411 (lem:make-key :sym "F1"))
  (defkeycode "[f2]" #o412 (lem:make-key :sym "F2"))
  (defkeycode "[f3]" #o413 (lem:make-key :sym "F3"))
  (defkeycode "[f4]" #o414 (lem:make-key :sym "F4"))
  (defkeycode "[f5]" #o415 (lem:make-key :sym "F5"))
  (defkeycode "[f6]" #o416 (lem:make-key :sym "F6"))
  (defkeycode "[f7]" #o417 (lem:make-key :sym "F7"))
  (defkeycode "[f8]" #o420 (lem:make-key :sym "F8"))
  (defkeycode "[f9]" #o421 (lem:make-key :sym "F9"))
  (defkeycode "[f10]" #o422 (lem:make-key :sym "F10"))
  (defkeycode "[f11]" #o423 (lem:make-key :sym "F11"))
  (defkeycode "[f12]" #o424 (lem:make-key :sym "F12")))
(define-key-code "F1" :f1)
(define-key-code "F2" :f2)
(define-key-code "F3" :f3)
(define-key-code "F4" :f4)
(define-key-code "F5" :f5)
(define-key-code "F6" :f6)
(define-key-code "F7" :f7)
(define-key-code "F8" :f8)
(define-key-code "F9" :f9)
(define-key-code "F10" :f10)
(define-key-code "F11" :f11)
(define-key-code "F12" :f12)
#+nil
(progn
  (defkeycode "[sf1]" #o425 (lem:make-key :shift t :sym "F1"))
  (defkeycode "[sf2]" #o426 (lem:make-key :shift t :sym "F2"))
  (defkeycode "[sf3]" #o427 (lem:make-key :shift t :sym "F3"))
  (defkeycode "[sf4]" #o430 (lem:make-key :shift t :sym "F4"))
  (defkeycode "[sf5]" #o431 (lem:make-key :shift t :sym "F5"))
  (defkeycode "[sf6]" #o432 (lem:make-key :shift t :sym "F6"))
  (defkeycode "[sf7]" #o433 (lem:make-key :shift t :sym "F7"))
  (defkeycode "[sf8]" #o434 (lem:make-key :shift t :sym "F8"))
  (defkeycode "[sf9]" #o435 (lem:make-key :shift t :sym "F9"))
  (defkeycode "[sf10]" #o436 (lem:make-key :shift t :sym "F10"))
  (defkeycode "[sf11]" #o437 (lem:make-key :shift t :sym "F11"))
  (defkeycode "[sf12]" #o440 (lem:make-key :shift t :sym "F12"))
  (defkeycode "[dl]" #o510)
  (defkeycode "[il]" #o511))
#+nil
(defkeycode "[dc]" #o512 (lem:make-key :sym "Delete"))
(define-key-code "Delete" :delete)
#+nil
(progn
  (defkeycode "C-dc" 519 (lem:make-key :ctrl t :sym "Delete"))
  (defkeycode "[ic]" #o513)
  (defkeycode "[eic]" #o514)
  (defkeycode "[clear]" #o515)
  (defkeycode "[eos]" #o516)
  (defkeycode "[eol]" #o517)
  (defkeycode "[sf]" #o520 (lem:make-key :shift t :sym "Down"))
  (defkeycode "[sr]" #o521 (lem:make-key :shift t :sym "Up")))
#+nil
(defkeycode "[npage]" #o522 (lem:make-key :sym "PageDown"))
(define-key-code "PageDown" :page-down)
#+nil
(defkeycode "[ppage]" #o523 (lem:make-key :sym "PageUp"))
(define-key-code "PageUp" :page-up)
#+nil
(progn
  (defkeycode "[stab]" #o524)
  (defkeycode "[ctab]" #o525)
  (defkeycode "[catab]" #o526)
  (defkeycode "[enter]" #o527)
  (defkeycode "[print]" #o532)
  (defkeycode "[ll]" #o533)
  (defkeycode "[a1]" #o534)
  (defkeycode "[a3]" #o535)
  (defkeycode "[b2]" #o536)
  (defkeycode "[c1]" #o537)
  (defkeycode "[c3]" #o540)
  (defkeycode "[btab]" #o541  (lem:make-key :shift t :sym "Tab"))
  (defkeycode "[beg]" #o542)
  (defkeycode "[cancel]" #o543)
  (defkeycode "[close]" #o544)
  (defkeycode "[command]" #o545)
  (defkeycode "[copy]" #o546)
  (defkeycode "[create]" #o547))
#+nil
(defkeycode "[end]" #o550 (lem:make-key :sym "End"))
(define-key-code "End" :end)
#+nil
(progn
  (defkeycode "[exit]" #o551)
  (defkeycode "[find]" #o552)
  (defkeycode "[help]" #o553)
  (defkeycode "[mark]" #o554)
  (defkeycode "[message]" #o555)
  (defkeycode "[move]" #o556)
  (defkeycode "[next]" #o557)
  (defkeycode "[open]" #o560)
  (defkeycode "[options]" #o561)
  (defkeycode "[previous]" #o562)
  (defkeycode "[redo]" #o563)
  (defkeycode "[reference]" #o564)
  (defkeycode "[refresh]" #o565)
  (defkeycode "[replace]" #o566)
  (defkeycode "[restart]" #o567)
  (defkeycode "[resume]" #o570)
  (defkeycode "[save]" #o571)
  (defkeycode "[sbeg]" #o572)
  (defkeycode "[scancel]" #o573)
  (defkeycode "[scommand]" #o574)
  (defkeycode "[scopy]" #o575)
  (defkeycode "[screate]" #o576)
  (defkeycode "[sdc]" #o577 (lem:make-key :shift t :sym "Delete"))
  (defkeycode "[sdl]" #o600)
  (defkeycode "[select]" #o601)
  (defkeycode "[send]" #o602 (lem:make-key :shift t :sym "End"))
  (defkeycode "[seol]" #o603)
  (defkeycode "[sexit]" #o604)
  (defkeycode "[sfind]" #o605)
  (defkeycode "[shelp]" #o606)
  (defkeycode "[shome]" #o607 (lem:make-key :shift t :sym "Home"))
  (defkeycode "[sic]" #o610)
  (defkeycode "[sleft]" #o611 (lem:make-key :shift t :sym "Left"))
  (defkeycode "[smessage]" #o612)
  (defkeycode "[smove]" #o613)
  (defkeycode "[snext]" #o614 (lem:make-key :shift t :sym "PageDown"))
  (defkeycode "[soptions]" #o615)
  (defkeycode "[sprevious]" #o616 (lem:make-key :shift t :sym "PageUp"))
  (defkeycode "[sprint]" #o617)
  (defkeycode "[sredo]" #o620)
  (defkeycode "[sreplace]" #o621)
  (defkeycode "[sright]" #o622 (lem:make-key :shift t :sym "Right"))
  (defkeycode "[srsume]" #o623)
  (defkeycode "[ssave]" #o624)
  (defkeycode "[ssuspend]" #o625)
  (defkeycode "[sundo]" #o626)
  (defkeycode "[suspend]" #o627)
  (defkeycode "[undo]" #o630)
  (defkeycode "[mouse]" #o631)
  (defkeycode "[resize]" #o632)
  (defkeycode "[event]" #o633))
