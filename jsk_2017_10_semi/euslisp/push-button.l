#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(defun push-button ()
  (let ((cds)
	(IH-button-front)
	(IH-button))
    (setq *cook-spot* (make-coords :pos #f(1350 8200 30002)))
    (send *cook-spot* :rotate pi :z)
    (send *pr2* :move-to *cook-spot* :world)
    (send *ri* :move-to *cook-spot* :frame-id "eng2")    
    (send *ri* :wait-interpolation)
    
    (objects (list *pr2* *scene*))
    (send *irtviewer* :look-all *pr2*)  
    (send *pr2* :head :neck-p :joint-angle 40)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (if *simulation*
	(progn
	  (setq *kitchen-panel* (make-cascoords :pos #f(630 7815 30737)))
	  (send *kitchen-panel* :rotate pi/2 :y)
	  )
      (progn
	(while (null (setq cds (send (check-detection :type "button" :speak-name "IHbutton" :timeout 10 :speak nil :single t) :copy-coords)))
	  (print "not detect button")
	  )
	(setq *kitchen-panel* (send (send *ri* :state :worldcoords) :copy-coords))
	(send *kitchen-panel* :move-to (make-coords :pos (send cds :pos)))
	(setq *kitchen-panel* (make-cascoords :pos (send *kitchen-panel* :pos)))
	)
      )
    (setq IH-button (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (setq IH-button-front (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send IH-button :locate (float-vector 0 655 -35))
    (send IH-button :rotate pi :z)
    (send IH-button-front :locate (float-vector 20 655 -35))
    (send IH-button-front :rotate pi :z)
    
    (send *pr2* :reset-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *irtviewer* :draw-objects)
    (send *ri* :wait-interpolation)
    
    (do-until-key
     (x::window-main-one)
     )
    
    (send *pr2* :rarm :inverse-kinematics IH-button-front
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :x
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    
    (do-until-key
     (x::window-main-one)
     )
    
    (send *pr2* :rarm :inverse-kinematics IH-button
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :x
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    
    (do-until-key
     (x::window-main-one)
     )
    
    (send *pr2* :rarm :inverse-kinematics IH-button-front
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :x
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    
    (do-until-key
     (x::window-main-one)
     )
    
    (send *pr2* :reset-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *irtviewer* :draw-objects)
    (send *ri* :wait-interpolation)
    )
  )
