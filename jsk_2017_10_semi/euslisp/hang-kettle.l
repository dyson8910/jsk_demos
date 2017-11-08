#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")

(load "package://fetcheus/fetch-interface.l")
(fetch-init)
(load "models/room73b2-scene.l")
(setq *room* (room73b2))

(setq *cook-spot-* (make-cascoords :pos #f(1300 1600 0) :rpy (float-vector pi 0 0)))
(send *fetch* :move-to *cook-spot-* :world)
(send *ri* :move-to *cook-spot-* :fram-id "/map")

(objects (list *fetch* *room*))
(send *irtviewer* :look-all *fetch*)
(send *ri* :wait-interpolation)

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
(send *irtviewer* :draw-objects)
(send *ri* :wait-interpolation)

(setq *kettle* (send *room* :object "room73b2-kettle"))
(if (send *kettle* :parent) (send (send *kettle* :parent) :dissoc *kettle*))

(setq *kettle-handle* (car (send *kettle* :handle)))
(send *kettle-handle* :rotate pi :z)
(send *kettle-handle* :rotate pi/2 :y)
(setq *kettle-handle-front* (make-cascoords :pos #f(650 1500 1020)))
(send *kettle-handle-front* :rotate pi/2 :x)
(send *kettle-handle-front* :rotate pi :z)
      
(send *fetch* :rarm :inverse-kinematics *kettle-handle-front*
      :stop 200
      :link-list (send *fetch* :link-list (send *fetch* :rarm :end-coords :parent))
      :move-target (send *fetch* :rarm :end-coords)
      :rotation-axis t
      :debug-view t
      :p-limit 10 :r-limit 0.05
      :cog-null-space t)

(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :stop-grasp)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics *kettle-handle*
      :stop 200
      :link-list (send *fetch* :link-list (send *fetch* :rarm :end-coords :parent))
      :move-target (send *fetch* :rarm :end-coords)
      :rotation-axis t
      :debug-view t
      :p-limit 10 :r-limit 0.05
      :cog-null-space t)

(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp)
(send (send *fetch* :rarm :end-coords) :assoc *kettle*)

(setq *kettle-handle-above* (make-cascoords :pos #f(500 1500 1200)))
(send *kettle-handle-above* :rotate pi/2 :x)
(send *kettle-handle-above* :rotate pi :z)

(send *fetch* :rarm :inverse-kinematics *kettle-handle-above*
      :stop 200
      :link-list (send *fetch* :link-list (send *fetch* :rarm :end-coords :parent))
      :move-target (send *fetch* :rarm :end-coords)
      :rotation-axis t
      :debug-view t
      :p-limit 10 :r-limit 0.05
      :cog-null-space t)

(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(setq *table-spot* (make-cascoords :pos #f(3200 500 0) :rpy (float-vector pi/2 0 0)))
(send *fetch* :move-to *table-spot* :world)
(send *irtviewer* :look-all *fetch*)
(send *ri* :move-to *table-spot* :fram-id "/map")
