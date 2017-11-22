#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(load "package://jsk_maps/src/eng2-scene.l")
(setq *scene* (make-eng2-scene))

(setq *cook-spot-* (make-coords :pos #f(1200 8500 30002)))
(send *cook-spot-* :rotate pi :z)

(send *pr2* :move-to *cook-spot-* :world)
(send *ri* :move-to *cook-spot-* :fram-id "/map")

(objects (list *pr2* *scene*))
(send *irtviewer* :look-all *pr2*)
(send *ri* :wait-interpolation)

(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *irtviewer* :draw-objects)
(send *ri* :wait-interpolation)

(setq *kettle-handle* (make-coords :pos #f(500 8470 31020)))
(send *kettle-handle* :rotate pi/2 :x)
(send *kettle-handle* :rotate pi :z)

(setq *kettle-handle-front* (make-coords :pos #f(660 8470 31020)))
(send *kettle-handle-front* :rotate pi/2 :x)
(send *kettle-handle-front* :rotate pi :z)

(do-until-key
 (x::window-main-one)
 )

(send *pr2* :rarm :inverse-kinematics *kettle-handle-front*
      :stop 200
      :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
      :move-target (send *pr2* :rarm :end-coords)
      :rotation-axis t
      :translation-axis :z
      :debug-view t
      :p-limit 10 :r-limit 0.05
      :cog-null-space t)

(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :stop-grasp)
(send *ri* :wait-interpolation)

(do-until-key
 (x::window-main-one)
 )

(send *pr2* :rarm :inverse-kinematics *kettle-handle*
      :stop 200
      :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
      :move-target (send *pr2* :rarm :end-coords)
      :rotation-axis t
      :debug-view t
      :p-limit 10 :r-limit 0.05
      :cog-null-space t)

(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)

(do-until-key
 (x::window-main-one)
 )

(setq *kettle-handle-above* (make-cascoords :pos #f(500 8470 31102)))
(send *kettle-handle-above* :rotate pi/2 :x)
(send *kettle-handle-above* :rotate pi :z)

(send *pr2* :rarm :inverse-kinematics *kettle-handle-above*
      :stop 200
      :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
      :move-target (send *pr2* :rarm :end-coords)
      :rotation-axis t
      :debug-view t
      :p-limit 10 :r-limit 0.05
      :cog-null-space t)

(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(do-until-key
 (x::window-main-one)
 )

(setq *table-spot* (make-cascoords :pos #f(3000 7000 30002) :rpy (float-vector pi/2 0 0)))
(send *pr2* :move-to *table-spot* :world)
(send *irtviewer* :look-all *pr2*)
(send *ri* :move-to *table-spot* :fram-id "/map")
