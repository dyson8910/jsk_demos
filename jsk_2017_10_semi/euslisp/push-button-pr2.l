#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(load "models/room73b2-scene.l")
(setq *room* (room73b2))

(objects (list *pr2* *room*))
(send *irtviewer* :look-all *pr2*)

(send *pr2* :move-to (send *room* :spot "cook-spot") :world)
(send *ri* :move-to (send *room* :spot "cook-spot") :frame-id "/map")
(send *ri* :wait-interpolation)
(objects (list *pr2* *room*))
(send *irtviewer* :look-all *pr2*)

(send *ri* :wait-interpolation)

(setq *kettle* (send *room* :object "room73b2-kettle"))
(if (send *kettle* :parent) (send (send *kettle* :parent) :dissoc *kettle*))

(setq *IH-button* (make-coords :pos #f(685 1570 700)))
(send *IH-button* :rotate pi :z)
(setq *IH-button-front* (make-coords :pos #f(695 1570 700)))
(send *IH-button-front* :rotate pi :z)

(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *irtviewer* :draw-objects)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *IH-button-front*
      :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
      :move-target (send *pr2* :rarm :end-coords)
      :rotation-axis :x
      :debug-view t :p-limit 10 :r-limit 0.05;;
      :cog-null-space t
      )

(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *IH-button*
      :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
      :move-target (send *pr2* :rarm :end-coords)
      :rotation-axis :x
      :debug-view t :p-limit 10 :r-limit 0.05;;
      :cog-null-space t
      )

(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics *IH-button-front*
      :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
      :move-target (send *pr2* :rarm :end-coords)
      :rotation-axis :x
      :debug-view t :p-limit 10 :r-limit 0.05;;
      :cog-null-space t
      )

(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *irtviewer* :draw-objects)
(send *ri* :wait-interpolation)
