#!/usr/bin/env roseus

(defun hang-kettle ()
  (let ((kettle-handle)
	(kettle-handle-front)
	(kettle-handle-above))

    (if *simulation*
	(progn
	  (setq kettle-handle (make-cascoords :pos #f(500 8470 31020)))
	  (send kettle-handle :rotate pi/2 :x)
	  (send kettle-handle :rotate pi :z)
	  
	  (setq kettle-handle-front (make-cascoords :pos #f(660 8470 31020)))
	  (send kettle-handle-front :rotate pi/2 :x)
	  (send kettle-handle-front :rotate pi :z)
	  
	  (setq kettle-handle-above (make-cascoords :pos #f(500 8470 31120)))
	  (send kettle-handle-above :rotate pi/2 :x)
	  (send kettle-handle-above :rotate pi :z)
	  )
      (progn
	(setq kettle-handle (make-cascoords :pos (send *kitchen-panel* :pos)))
	(send kettle-handle :locate (float-vector -200 590 280))
	(send kettle-handle :rotate pi/2 :x)
	(send kettle-handle :rotate pi :z)
	
	(setq kettle-handle-front (make-cascoords :pos (send *kitchen-panel* :pos)))
	(send kettle-handle-front :locate (float-vector -100 590 280))
	(send kettle-handle-front :rotate pi/2 :x)
	(send kettle-handle-front :rotate pi :z)
	)
      )
    
    (objects (list *pr2* *scene* kettle-handle kettle-handle-front kettle-handle-above))
    (send *irtviewer* :look-all *pr2*)
    (do-until-key
     (x::window-main-one)
     )
    
    (send *pr2* :rarm :inverse-kinematics kettle-handle-front
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :translation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :use-base t
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :stop-grasp :rarm)
    (send *ri* :wait-interpolation)
    
    (do-until-key
     (x::window-main-one)
     )
    
    (send *pr2* :rarm :inverse-kinematics kettle-handle
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :no-message
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :use-base t
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    
    (send *ri* :start-grasp :rarm)
    
    (do-until-key
     (x::window-main-one)
     )
        
    (send *pr2* :rarm :inverse-kinematics kettle-handle-above
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :use-base t
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    
    (do-until-key
     (x::window-main-one)
     )

    (setq *table-spot* (make-cascoords :pos #f(3000 7300 30002) :rpy (float-vector pi/2 0 0)))
    (send *pr2* :move-to *table-spot* :world)
    (send *ri* :move-to *table-spot* :fram-id "/map")
    (send *irtviewer* :look-all *pr2*)
    (send *irtviewer* :look-all *pr2*)
    )
  )
