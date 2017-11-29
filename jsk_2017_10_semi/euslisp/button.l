(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(defparameter *detection-topic* "/ObjectDetection")
(pr2-init)
(setq cds (check-detection :type "button" :speak-name "IHbutton" :timeout 10 :speak nil))

(send *pr2* :rarm :inverse-kinematics cds :rotation-axis nil)
