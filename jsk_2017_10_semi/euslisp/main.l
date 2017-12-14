#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(load "package://jsk_maps/src/eng2-scene.l")
(setq *scene* (make-eng2-scene))
(load "package://jsk_perception/euslisp/detection_interface.l")

(load "function.l")
(setq *simulation* nil)
(unless *simulation*
    (defparameter *detection-topic* "/ObjectDetection")
    )

(push-button)
#|
(do-until-key
 (x::window-main-one)
 )

(push-button)
(hang-kettle)
(do-until-key
 (x::window-main-one)
 )

(pour)
(do-until-key
 (x::window-main-one)
 )
(return-kettle)
|#
