#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://fetcheus/fetch-interface.l")
(fetch-init)

(load "models/room73b2-scene.l")
(setq *room* (room73b2))
(send *fetch* :move-to (send *room* :spot "cook-spot") :world)

(objects (list *fetch* *room*))
(send *irtviewer* :look-all *fetch*)
(do-until-key
 (x::window-main-one)
 )

(setq *kettle* (send *room* :object "room73b2-kettle"))
(if (send *kettle* :parent) (send (send *kettle* :parent) :dissoc *kettle*))

(setq *IH-button* (make-coords :pos #f(685 1570 700)))
(send *IH-button* :rotate pi :z)
(setq *IH-button-front* (make-coords :pos #f(695 1570 700)))
(send *IH-button-front* :rotate pi :z)


(send *fetch* :rarm :inverse-kinematics *IH-button-front*
      :link-list (send *fetch* :link-list (send *fetch* :rarm :end-coords :parent))
      :move-target (send *fetch* :rarm :end-coords)
      :rotation-axis :x
      :debug-view t :p-limit 10 :r-limit 0.05;;
      :cog-null-space t
      )

(send *fetch* :rarm :inverse-kinematics *IH-button*
      :link-list (send *fetch* :link-list (send *fetch* :rarm :end-coords :parent))
      :move-target (send *fetch* :rarm :end-coords)
      :rotation-axis :x
      :debug-view t :p-limit 10 :r-limit 0.05;;
      :cog-null-space t
      )

(send *fetch* :rarm :inverse-kinematics *IH-button-front*
      :link-list (send *fetch* :link-list (send *fetch* :rarm :end-coords :parent))
      :move-target (send *fetch* :rarm :end-coords)
      :rotation-axis :x
      :debug-view t :p-limit 10 :r-limit 0.05;;
      :cog-null-space t
      )


(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
