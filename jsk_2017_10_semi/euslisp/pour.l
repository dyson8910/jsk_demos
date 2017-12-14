#!/usr/bin/env roseus

(defun pour ()
  (let ((cds)
	(cup)
	(cup-noodle)
	(hand-pos)
	(above-cup)
	(i)
	)

    (if *simulation*
	(progn
	  (setq cup-noodle (make-cascoords :pos #f(3100 8000 30702)))
	  (send cup-noodle :rotate (* 3 pi/2) :z)
	  )
      (progn
        (while (null (setq cds (send (check-detection :type "cup-noodle" :speak-name "cup-noodle" :timeout 10 :speak nil :single t) :copy-coords)))
	  (print "not detect cup-noodle")
	  )
	(setq cup (send (send *ri* :state :worldcoords) :copy-coords))
	(send cup :move-to (make-cascoords :coords (send cds :coords)))
	(setq cup-noodle (make-cascoords :pos (send (send cup :copy-coords) :pos)))
	(send cup-noodle :move-to (make-cascoords :pos (float-vector 90 90 0)))
	)
      )

    (setq hand-pos (make-cascoords :coords (send cup-noodle :copy-coords)))
    (send hand-pos :move-to (make-cascoords :pos (float-vector 0 120 100)))
    (send hand-pos :rotate pi/2 :x)
    (send hand-pos :rotate pi :z)
    (setq above-cup (make-cascoords :coords (send cup-noodle :copy-coords)))
    (send above-cup :move-to (make-cascoords :pos (float-vector 0 120 200)))
    (send above-cup :rotate pi/2 :x)
    (send above-cup :rotate pi :z)
    (send cup-noodle :assoc hand-pos)

    (objects (list *pr2* *scene* cup-noodle hand-pos above-cup))
    (send *irtviewer* :look-all *pr2*)
    (do-until-key
     (x::window-main-one)
     )
    (send *pr2* :rarm :inverse-kinematics hand-pos
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :darw-objects)
    (do-until-key
     (x::window-main-one)
     )
    
    (setq i 0)
    (while (< i 30)
      (progn
	(send cup-noodle :rotate (deg2rad 1) :x)
	(describe hand-pos)
	(send *pr2* :rarm :inverse-kinematics hand-pos
	      :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	      :move-target (send *pr2* :rarm :end-coords)
	      :rotation-axis t
	      :debug-view :no-message :p-limit 10 :r-limit 0.05
	      :cog-null-space t
	      )
	(send *ri* :angle-vector (send *pr2* :angle-vector) 100)
	(send *ri* :wait-interpolation)
	(send *irtviewer* :darw-objects)
	(unix::usleep (* 400 1000))
	(setq i (+ i 1))
	)
      )
    
    (send *pr2* :rarm :inverse-kinematics above-cup
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    )
  )
