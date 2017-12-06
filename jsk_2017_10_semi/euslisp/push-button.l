#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(load "package://jsk_maps/src/eng2-scene.l")
(setq *scene* (make-eng2-scene))

(defun push-button ()
    (setq *cook-spot* (make-coords :pos #f(1450 8140 30002)))
    (send *cook-spot* :rotate pi :z)
    
    (send *pr2* :move-to *cook-spot* :world)
    (send *ri* :move-to *cook-spot* :frame-id "eng2")
    
    (send *ri* :wait-interpolation)
    (objects (list *pr2* *scene*))
    (send *irtviewer* :look-all *pr2*)  
    (send *ri* :wait-interpolation)
    (send *pr2* :head :neck-p :joint-angle 40)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    
    (setq *IH-button* (send (check-detection :type "button" :speak-name "IHbutton" :timeout 10 :speak nil) :copy-coords));;TODO
    (setq *IH-button-front* (send *IH-button* :copy-coords))
    (send *IH-button-front* :locate);;TODO
    
    (send *pr2* :reset-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *irtviewer* :draw-objects)
  (send *ri* :wait-interpolation)
  
  (do-until-key
   (x::window-main-one)
   )
  
  (send *pr2* :rarm :inverse-kinematics *IH-button-front*
	:link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	:move-target (send *pr2* :rarm :end-coords)
	:rotation-axis :x
	:debug-view :no-message :p-limit 10 :r-limit 0.05;;
	:cog-null-space t
	)
  
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  
  (do-until-key
   (x::window-main-one)
   )
  
  (send *pr2* :rarm :inverse-kinematics *IH-button*
	:link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	:move-target (send *pr2* :rarm :end-coords)
	:rotation-axis :x
	:debug-view :no-message :p-limit 10 :r-limit 0.05;;
	:cog-null-space t
	)
  
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  
  (do-until-key
   (x::window-main-one)
   )
  
  (send *pr2* :rarm :inverse-kinematics *IH-button-front*
	:link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	:move-target (send *pr2* :rarm :end-coords)
	:rotation-axis :x
	:debug-view :no-message :p-limit 10 :r-limit 0.05;;
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

;(push-button)
