#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(load "package://jsk_maps/src/eng2-scene.l")
(setq *scene* (make-eng2-scene))
(load "package://jsk_perception/euslisp/detection_interface.l")
(load "function.l")

(setq *simulation* nil)
(defparameter *detection-topic* "/ObjectDetection")

(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(detect-button)
(push-button)
#|
(do-until-key
 (x::window-main-one)
 )
|#
(push-button)
(hang-kettle)
(detect-marker)
(pour)
(detect-button)
(return-kettle)

