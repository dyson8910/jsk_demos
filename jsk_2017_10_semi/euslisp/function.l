#!/usr/bin/env roseus

(defun detect-button ()
  (let ((cds))
    ;;キッチンへの移動
    (setq *cook-spot* (make-coords :pos #f(1350 8040 30002)))
    (send *cook-spot* :rotate pi :z)
    (send *pr2* :move-to *cook-spot* :world)
    (send *ri* :move-to *cook-spot* :frame-id "eng2")
    (send *ri* :wait-interpolation)
    (objects (list *pr2* *scene*))
    (send *irtviewer* :look-all *pr2*)
    (send *pr2* :head :neck-p :joint-angle 40)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (ros::ros-info "arrived at kitchen")
    ;;画像の座標を認識
    (if *simulation*
	(progn
	  (setq *kitchen-panel* (make-coords :pos #f(630 7950 30737)))
	  (send *kitchen-panel* :rotate pi/2 :y)
	  )
      (progn
	(while (null (setq cds (check-detection :type "button" :speak-name "IHbutton" :timeout 10 :speak nil :single t))) 
	  (print "not detect button")
	  )
	(setq *kitchen-panel* (make-coords :coords (send (send *pr2* :worldcoords) :copy-coords)))
	(send *kitchen-panel* :move-to (make-cascoords :pos (send cds :pos)))
	(setq *kitchen-panel* (make-coords :pos (send *kitchen-panel* :pos)))
	)
      )
    ;;コンロに近づく
    (send *pr2* :move-to (make-cascoords :pos #f(200 0 0)))
    (send *ri* :go-pos-unsafe 0.2 0 0)
    (send *ri* :wait-interpolation)
    )
  )

(defun push-button ()
  (let ((IH-button-front)
	(IH-button))
    ;;画像の座標からボタンの位置を決定
    (setq IH-button (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send IH-button :locate (float-vector 0 525 -25))
    (send IH-button :rotate pi :z)
    (setq IH-button-front (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send IH-button-front :locate (float-vector 100 525 -25))
    (send IH-button-front :rotate pi :z)
    (send *ri* :start-grasp)
    ;;irtviewerで表示
    (objects (list *pr2* *scene* IH-button IH-button-front *kitchen-panel*))
    (send *irtviewer* :look-all *pr2*)
    (ros::ros-info "push button")
    ;;ボタンの前まで手を持っていく
    (send *pr2* :rarm :inverse-kinematics IH-button-front
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :x
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :look-all *pr2*)
    ;;ボタンを押す
    (send *pr2* :rarm :inverse-kinematics IH-button
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :x
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :look-all *pr2*)
    (unix::usleep (* 500 1000))
    ;;手を引く
    (send *pr2* :rarm :inverse-kinematics IH-button-front
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :x
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    ;;reset-poseにして終了        
    (send *pr2* :reset-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :look-all *pr2*)
    )
  )

(defun hang-kettle ()
  (let ((kettle-handle)
	(kettle-handle-front)
	(kettle-handle-above)
	(hold-pos)
	)
    ;;detect-buttonで得た座標からやかんの位置を決定
    (setq kettle-handle (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send kettle-handle :move-to (make-cascoords :pos (float-vector -220 450 350)))
    (send kettle-handle :rotate pi/2 :x)
    (send kettle-handle :rotate pi :z)
    
    (setq kettle-handle-front (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send kettle-handle-front :move-to (make-cascoords :pos (float-vector 0 450 350)))
    (send kettle-handle-front :rotate pi/2 :x)
    (send kettle-handle-front :rotate pi :z)
    
    (setq kettle-handle-above (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send kettle-handle-above :move-to (make-cascoords :pos (float-vector -220 450 450)))
    (send kettle-handle-above :rotate pi/2 :x)
    (send kettle-handle-above :rotate pi :z)
        
    (setq hold-pos (make-cascoords :coords (send (send *pr2* :worldcoords) :copy-coords)))
    (send hold-pos :move-to (make-cascoords :pos #f(400 -400 1300)))
    (send hold-pos :rotate (* 3 pi/2) :x)

    (send *pr2* :move-to (make-cascoords :pos #f(100 0 0)))
    (send *ri* :go-pos-unsafe 0.1 0 0)
    (send *ri* :wait-interpolation)

    
    (objects (list *pr2* *scene* kettle-handle kettle-handle-front kettle-handle-above hold-pos))
    (send *irtviewer* :look-all *pr2*)
    (ros::ros-info "hang-kettle")
    ;;やかんの取っ手の前まで手を移動
    (send *pr2* :rarm :inverse-kinematics kettle-handle-front
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :translation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)    
    ;;やかんの取っ手に手を開いて移動
    (send *ri* :stop-grasp :rarm)
    (send *ri* :wait-interpolation)    
    (send *pr2* :rarm :inverse-kinematics kettle-handle
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :no-message
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :look-all *pr2*)
    ;;やかんの取っ手を掴んで上げる
    (send *ri* :start-grasp :rarm)
    (send *ri* :wait-interpolation)
    (send *pr2* :rarm :inverse-kinematics kettle-handle-above
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :look-all *pr2*)
    ;;手を引っ込める
    (send *pr2* :rarm :inverse-kinematics hold-pos
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :look-all *pr2*)
    )
  )

(defun detect-marker ()
  (let ((cds))
    ;;テーブルの前まで移動
    (setq *table-spot* (make-cascoords :pos #f(1200 8600 30002) :rpy (float-vector pi 0 0)));;TODO
    (send *pr2* :move-to *table-spot* :world)
    (send *ri* :move-to *table-spot* :fram-id "/map")
    (send *ri* :wait-interpolation)
    (objects (list *pr2* *scene*))
    (send *irtviewer* :look-all *pr2*)
    (send *pr2* :head :neck-p :joint-angle 40)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (ros::ros-info "arrived at table")
    
    (if *simulation*
	(progn
	  (setq *cup-noodle* (make-cascoords :pos #f(500 8600 31002)))
	  )
      (progn
        (while (null (setq cds (check-detection :type "noodle" :speak-name "cup-noodle" :timeout 10 :speak nil :single t))))
	  (print "not detect cup-noodle")
	  )
	(setq *cup-noodle* (make-cascoords :coords (send (send *pr2* :worldcoords) :copy-coords)))
	(send *cup-noodle* :move-to (make-cascoords :coords (send cds :coords)))
	(send *cup-noodle* :move-to (make-cascoords :pos (float-vector -100 40 10)))
	)
    )
  )

(defun pour ()
  (let ((hand-pos)
	(above-cup)
	(hold-pos)
	(i)
	)
    ;;手を置く位置を決定
    (setq hand-pos (make-cascoords :coords (send *cup-noodle* :copy-coords)))
    (send hand-pos :move-to (make-cascoords :pos (float-vector 0 120 100)))
    (send hand-pos :rotate pi/2 :x)
    (send hand-pos :rotate pi :z)
    (setq above-cup (make-cascoords :coords (send *cup-noodle* :copy-coords)))
    (send above-cup :move-to (make-cascoords :pos (float-vector 0 120 200)))
    (send above-cup :rotate pi/2 :x)
    (send above-cup :rotate pi :z)
    (send *cup-noodle* :assoc hand-pos)

    (objects (list *pr2* *scene* *cup-noodle* hand-pos above-cup))
    (send *irtviewer* :look-all *pr2*)
    ;;最初に手を置く位置に移動
    (send *pr2* :rarm :inverse-kinematics hand-pos
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :darw-objects)
    (send *irtviewer* :look-all *pr2*)
    ;;30度までやかんを傾ける
    (setq i 0)
    (while (< i 30)
      (progn
	(send *cup-noodle* :rotate (deg2rad 1) :x)
	(describe hand-pos)
	(send *pr2* :rarm :inverse-kinematics hand-pos
	      :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	      :move-target (send *pr2* :rarm :end-coords)
	      :rotation-axis t
	      :debug-view :no-message :p-limit 10 :r-limit 0.05
	      :cog-null-space t
	      )
	(send *ri* :angle-vector (send *pr2* :angle-vector) 100)
	(send *ri* :wait-interpolation)
	(send *irtviewer* :darw-objects)
	(send *irtviewer* :look-all *pr2*)
	(unix::usleep (* 500 1000))
	(setq i (+ i 2))
	)
      )
    ;;やかんを上に持っていく
    (send *pr2* :rarm :inverse-kinematics above-cup
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :darw-objects)
    (send *irtviewer* :look-all *pr2*)
    ;;手を引っ込める
    (setq hold-pos (make-cascoords :coords (send (send *pr2* :worldcoords) :copy-coords)))
    (send hold-pos :move-to (make-cascoords :pos #f(400 -400 1300)))
    (send hold-pos :rotate (* 3 pi/2) :x)

    (send *pr2* :rarm :inverse-kinematics hold-pos
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :look-all *pr2*)
    )
  )

(defun return-kettle ()
  (let ((kettle-handle)
	(kettle-handle-front)
	(kettle-handle-above)
	(kettle-handle-front-above)
	)
    ;;detect-buttonで認識した座標から置く場所を計算
    (setq kettle-handle (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send kettle-handle :locate (float-vector -220 440 380))
    (send kettle-handle :rotate pi/2 :x)
    (send kettle-handle :rotate pi :z)
    
    (setq kettle-handle-front (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send kettle-handle-front :locate (float-vector 0 440 380))
    (send kettle-handle-front :rotate pi/2 :x)
    (send kettle-handle-front :rotate pi :z)
    
    (setq kettle-handle-above (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send kettle-handle-above :locate (float-vector -220 440 480))
    (send kettle-handle-above :rotate pi/2 :x)
    (send kettle-handle-above :rotate pi :z)

    (setq kettle-handle-front-above (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send kettle-handle-front-above :locate (float-vector 0 440 480))
    (send kettle-handle-front-above :rotate pi/2 :x)
    (send kettle-handle-front-above :rotate pi :z)
    	
    (objects (list *pr2* *scene* kettle-handle kettle-handle-front kettle-handle-above))
    (send *irtviewer* :look-all *pr2*)
    ;;コンロの上にやかんを持っていく
    (send *pr2* :rarm :inverse-kinematics kettle-handle-front-above
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :translation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :darw-objects)
    (send *irtviewer* :look-all *pr2*)    

    (send *pr2* :rarm :inverse-kinematics kettle-handle-above
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :translation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :darw-objects)
    (send *irtviewer* :look-all *pr2*)    
    ;;コンロにやかんを置く
    (send *pr2* :rarm :inverse-kinematics kettle-handle
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :no-message
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :darw-objects)
    (send *irtviewer* :look-all *pr2*)
    ;;離して手を引く
    (send *ri* :stop-grasp :rarm)
    (send *ri* :wait-interpolation)    
    (send *pr2* :rarm :inverse-kinematics kettle-handle-front
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :darw-objects)
    (send *irtviewer* :look-all *pr2*)
    ;;reset-poseにして終了        
    (send *pr2* :reset-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :look-all *pr2*)
    )
  )
