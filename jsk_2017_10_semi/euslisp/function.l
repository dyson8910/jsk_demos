#!/usr/bin/env roseus

(defun push-button ()
  (let ((cds)
	(IH-button-front)
	(IH-button))
    ;;キッチンへの移動
    (setq *cook-spot* (make-coords :pos #f(1450 8040 30002)))
    (send *cook-spot* :rotate pi :z)
    (send *pr2* :move-to *cook-spot* :world)
    (send *ri* :move-to *cook-spot* :frame-id "eng2")
    (send *ri* :wait-interpolation)
    
    (objects (list *pr2* *scene*))
    (send *irtviewer* :look-all *pr2*)
    (send *pr2* :reset-pose)
    (send *pr2* :head :neck-p :joint-angle 40)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;;画像の座標を認識
    (if *simulation*
	(progn
	  (setq *kitchen-panel* (make-cascoords :pos #f(630 7950 30737)))
	  (send *kitchen-panel* :rotate pi/2 :y)
	  )
      (progn
	(while (null (setq cds (check-detection :type "button" :speak-name "IHbutton" :timeout 10 :speak nil :single t))) 
	  (print "not detect button")
	  )
	(setq *kitchen-panel* (make-cascoords :coords (send (send *ri* :state :worldcoords) :copy-coords)))
	(send *kitchen-panel* :move-to (make-cascoords :pos (send cds :pos)))
	(setq *kitchen-panel* (make-cascoords :pos (send *kitchen-panel* :pos)))
	)
      )

    ;;画像の座標からボタンの位置を決定
    (setq IH-button (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send IH-button :locate (float-vector 250 510 -35))
    (send IH-button :rotate pi :z)
    (setq IH-button-front (make-cascoords :pos (send (send *kitchen-panel* :copy-coords) :pos)))
    (send IH-button-front :locate (float-vector 300 510 -35))
    (send IH-button-front :rotate pi :z)

    (send *pr2* :move-to (make-cascoords :pos #f(-100 0 0)))
    (send *ri* :go-pos-unsafe 0.1 0 0)

    (objects (list *pr2* *scene* IH-button IH-button-front *kitchen-panel*))
    (send *irtviewer* :look-all *pr2*)

    (do-until-key
     (x::window-main-one)
     )

    ;;ボタンの前まで手を持っていく
    (send *pr2* :rarm :inverse-kinematics IH-button-front
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :x
	  :use-base t
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    
    (do-until-key
     (x::window-main-one)
     )

    ;;ボタンを押す
    (send *pr2* :rarm :inverse-kinematics IH-button
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :x
	  :use-base t
	  :debug-view :no-message :p-limit 10 :r-limit 0.05
	  :cog-null-space t
	  )
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    (do-until-key
     (x::window-main-one)
     )

    ;;手を引く
    (send *pr2* :rarm :inverse-kinematics IH-button-front
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :x
	  :use-base t
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
    )
  )

(defun hang-kettle ()
  (let ((kettle-handle)
	(kettle-handle-front)
	(kettle-handle-above))

    ;;push-buttonで得た座標からやかんの位置を決定
    (if *simulation*
	(progn
	  (setq kettle-handle (make-cascoords :pos #f(500 8470 31020)))
	  (send kettle-handle :rotate pi/2 :x)
	  (send kettle-handle :rotate pi :z)
	  
	  (setq kettle-handle-front (make-cascoords :pos #f(660 8470 31020)))
	  (send kettle-handle-front :rotate pi/2 :x)
	  (send kettle-handle-front :rotate pi :z)
	  
	  (setq kettle-handle-above (make-cascoords :pos #f(500 8470 31120)))
	  (send kettle-handle-above :rotate pi/2 :x)
	  (send kettle-handle-above :rotate pi :z)
	  )
      (progn
	(setq kettle-handle (make-cascoords :pos (send *kitchen-panel* :pos)))
	(send kettle-handle :locate (float-vector 30 440 280))
	(send kettle-handle :rotate pi/2 :x)
	(send kettle-handle :rotate pi :z)
	
	(setq kettle-handle-front (make-cascoords :pos (send *kitchen-panel* :pos)))
	(send kettle-handle-front :locate (float-vector 150 440 280))
	(send kettle-handle-front :rotate pi/2 :x)
	(send kettle-handle-front :rotate pi :z)

	(setq kettle-handle-above (make-cascoords :pos (send *kitchen-panel* :pos)))
	(send kettle-handle-above :locate (float-vector 30 440 380))
	(send kettle-handle-above :rotate pi/2 :x)
	(send kettle-handle-above :rotate pi :z)
	)
      )
    
    (objects (list *pr2* *scene* kettle-handle kettle-handle-front kettle-handle-above))
    (send *irtviewer* :look-all *pr2*)
    (do-until-key
     (x::window-main-one)
     )

    ;;やかんの取っ手の前まで手を移動
    (send *pr2* :rarm :inverse-kinematics kettle-handle-front
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :translation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :use-base t
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    
    (do-until-key
     (x::window-main-one)
     )
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
	  :use-base t
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (do-until-key
     (x::window-main-one)
     )
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
	  :use-base t
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (do-until-key
     (x::window-main-one)
     )

    ;;テーブルの前まで移動
    (setq *table-spot* (make-cascoords :pos #f(3000 7300 30002) :rpy (float-vector pi/2 0 0)))
    (send *pr2* :move-to *table-spot* :world)
    (send *ri* :move-to *table-spot* :fram-id "/map")
    (send *irtviewer* :look-all *pr2*)
    (send *irtviewer* :look-all *pr2*)
    )
  )

(defun pour ()
  (let ((cds)
	(cup)
	(cup-noodle)
	(hand-pos)
	(above-cup)
	(i)
	)

    (send *pr2* :head :neck-p :joint-angle 40)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;;カップ麺の座標を認識
    (if *simulation*
	(progn
	  (setq cup-noodle (make-cascoords :pos #f(3100 8000 30702)))
	  (send cup-noodle :rotate (* 3 pi/2) :z)
	  )
      (progn
        (while (null (setq cds (send (check-detection :type "cup-noodle" :speak-name "cup-noodle" :timeout 10 :speak nil :single t) :copy-coords)))
	  (print "not detect cup-noodle")
	  )
	(setq cup (make-cascoords :coords (send *ri* :state :worldcoords)))
	(send cup :move-to (make-cascoords :coords (send cds :coords)))
	(setq cup-noodle (make-cascoords :pos (send (send cup :copy-coords) :pos)))
	(send cup-noodle :move-to (make-cascoords :pos (float-vector 90 90 0)))
	)
      )
    ;;手を置く位置を決定
    (setq hand-pos (make-cascoords :coords (send cup-noodle :copy-coords)))
    (send hand-pos :move-to (make-cascoords :pos (float-vector 0 120 100)))
    (send hand-pos :rotate pi/2 :x)
    (send hand-pos :rotate pi :z)
    (setq above-cup (make-cascoords :coords (send cup-noodle :copy-coords)))
    (send above-cup :move-to (make-cascoords :pos (float-vector 0 120 200)))
    (send above-cup :rotate pi/2 :x)
    (send above-cup :rotate pi :z)
    (send cup-noodle :assoc hand-pos)

    (objects (list *pr2* *scene* cup-noodle hand-pos above-cup))
    (send *irtviewer* :look-all *pr2*)
    (do-until-key
     (x::window-main-one)
     )
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
    (do-until-key
     (x::window-main-one)
     )

    ;;30度までやかんを傾ける
    (setq i 0)
    (while (< i 30)
      (progn
	(send cup-noodle :rotate (deg2rad 1) :x)
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
	(unix::usleep (* 400 1000))
	(setq i (+ i 1))
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
    )
  )

(defun return-kettle ()
  (let ((kettle-handle)
	(kettle-handle-front)
	(kettle-handle-above))

    ;;キッチンへ移動
    (send *pr2* :move-to *cook-spot* :world)
    (send *ri* :move-to *cook-spot* :fram-id "eng2")
    (send *ri* :wait-interpolation)

    ;;push-buttonで認識した座標から置く場所を計算
    (if *simulation*
	(progn
	  (setq kettle-handle (make-coords :pos #f(500 8470 31020)))
	  (send kettle-handle :rotate pi/2 :x)
	  (send kettle-handle :rotate pi :z)
	  
	  (setq kettle-handle-front (make-coords :pos #f(660 8470 31020)))
	  (send kettle-handle-front :rotate pi/2 :x)
	  (send kettle-handle-front :rotate pi :z)
	  
	  (setq kettle-handle-above (make-coords :pos #f(500 8470 31120)))
	  (send kettle-handle-above :rotate pi/2 :x)
	  (send kettle-handle-above :rotate pi :z)
	  )
      (progn
	(setq kettle-handle (make-cascoords :pos (send *kitchen-panel* :pos)))
	(send kettle-handle :locate (float-vector 30 440 280))
	(send kettle-handle :rotate pi/2 :x)
	(send kettle-handle :rotate pi :z)
	
	(setq kettle-handle-front (make-cascoords :pos (send *kitchen-panel* :pos)))
	(send kettle-handle-front :locate (float-vector 150 440 280))
	(send kettle-handle-front :rotate pi/2 :x)
	(send kettle-handle-front :rotate pi :z)

	(setq kettle-handle-above (make-cascoords :pos (send *kitchen-panel* :pos)))
	(send kettle-handle-above :locate (float-vector 30 440 380))
	(send kettle-handle-above :rotate pi/2 :x)
	(send kettle-handle-above :rotate pi :z)
	)
      )
    
    (objects (list *pr2* *scene* kettle-handle kettle-handle-front kettle-handle-above))
    (send *irtviewer* :look-all *pr2*)
    (do-until-key
     (x::window-main-one)
     )

    ;;コンロの上にやかんを持っていく
    (send *pr2* :rarm :inverse-kinematics kettle-handle-above
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :translation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :use-base t
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    
    (do-until-key
     (x::window-main-one)
     )

    ;;コンロに置いて
    (send *pr2* :rarm :inverse-kinematics kettle-handle
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis :no-message
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :use-base t
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    
    (send *ri* :stop-grasp :rarm)
    (do-until-key
     (x::window-main-one)
     )
    
    (send *pr2* :rarm :inverse-kinematics kettle-handle-front
	  :stop 200
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :move-target (send *pr2* :rarm :end-coords)
	  :rotation-axis t
	  :debug-view :no-message
	  :p-limit 10 :r-limit 0.05
	  :use-base t
	  :cog-null-space t)
    
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
     )
    )
  )
