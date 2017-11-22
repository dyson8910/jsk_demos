#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(load "package://jsk_maps/src/eng2-scene.l")
(setq *scene* (make-eng2-scene))

(send *pr2* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-door-front") :worldcoords) :world)
(send *ri* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-door-front") :worldcoords))
(objects (list *pr2* *scene*))
(send *irtviewer* :look-all *pr2*)

(do-until-key
 (x::window-main-one)
 )

(send *pr2* :move-to (send (send *scene* :spot "cook-spot") :worldcoords) :world)

(send *ri* :move-to (send (send *scene* :spot "cook-spot") :worldcoords))
(send *ri* :wait-interpolation)
(objects (list *pr2* *scene*))
(send *irtviewer* :look-all *pr2*)

(send *ri* :wait-interpolation)

(setq *IH-button* (make-coords :pos #f(660 8470 30702)))
(send *IH-button* :rotate pi :z)
(setq *IH-button-front* (make-coords :pos #f(695 8470 30702)))
(send *IH-button-front* :rotate pi :z)

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
      :debug-view t :p-limit 10 :r-limit 0.05;;
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
      :debug-view t :p-limit 10 :r-limit 0.05;;
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
      :debug-view t :p-limit 10 :r-limit 0.05;;
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
