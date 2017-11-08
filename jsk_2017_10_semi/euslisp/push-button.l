#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://fetcheus/fetch-interface.l")
(fetch-init)

(load "models/room73b2-scene.l")
(setq *room* (room73b2))

(objects (list *fetch* *room*))
(send *irtviewer* :look-all *fetch*)

(send *fetch* :move-to (send *room* :spot "cook-spot") :world)
(send *ri* :move-to (send *room* :spot "cook-spot") :frame-id "/map")
(send *ri* :wait-interpolation)
(objects (list *fetch* *room*))
(send *irtviewer* :look-all *fetch*)

(send *ri* :wait-interpolation)

(setq *kettle* (send *room* :object "room73b2-kettle"))
(if (send *kettle* :parent) (send (send *kettle* :parent) :dissoc *kettle*))

(setq *IH-button* (make-coords :pos #f(685 1570 700)))
(send *IH-button* :rotate pi :z)
(setq *IH-button-front* (make-coords :pos #f(695 1570 700)))
(send *IH-button-front* :rotate pi :z)

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
(send *irtviewer* :draw-objects)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics *IH-button-front*
      :link-list (send *fetch* :link-list (send *fetch* :rarm :end-coords :parent))
      :move-target (send *fetch* :rarm :end-coords)
      :rotation-axis :x
      :debug-view t :p-limit 10 :r-limit 0.05;;
      :cog-null-space t
      )

(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics *IH-button*
      :link-list (send *fetch* :link-list (send *fetch* :rarm :end-coords :parent))
      :move-target (send *fetch* :rarm :end-coords)
      :rotation-axis :x
      :debug-view t :p-limit 10 :r-limit 0.05;;
      :cog-null-space t
      )

(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics *IH-button-front*
      :link-list (send *fetch* :link-list (send *fetch* :rarm :end-coords :parent))
      :move-target (send *fetch* :rarm :end-coords)
      :rotation-axis :x
      :debug-view t :p-limit 10 :r-limit 0.05;;
      :cog-null-space t
      )

(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
(send *irtviewer* :draw-objects)
(send *ri* :wait-interpolation)
